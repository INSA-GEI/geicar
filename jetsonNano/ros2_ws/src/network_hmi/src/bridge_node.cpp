#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/twist.hpp>
#include "interfaces/msg/joystick_order.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp> // Nécessite la bibliothèque nlohmann/json

#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <errno.h>

// Pour la partie réseau
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

using json = nlohmann::json;
using namespace std::placeholders;

// Structure pour stocker les informations sur le client
struct ClientInfo {
    std::string ip;
    int recv_udp_port;
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};

int mode = 0;
bool start = false;
bool systemCheckPrintRequest = false;

class TcpUdpBridge : public rclcpp::Node
{
public:
    TcpUdpBridge() : Node("tcp_udp_bridge")
    {

        // Déclaration des paramètres ROS 2
        this->declare_parameter<int>("tcp_control_port", 5001);
        this->declare_parameter<int>("udp_data_port", 5000);
        
        tcp_control_port_ = this->get_parameter("tcp_control_port").as_int();
        udp_data_port_ = this->get_parameter("udp_data_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Démarrage du pont...");
        RCLCPP_INFO(this->get_logger(), "Port de contrôle TCP : %d", tcp_control_port_);
        RCLCPP_INFO(this->get_logger(), "Port de données UDP : %d", udp_data_port_);
        // Créer les topics fixes (toujours les mêmes) dès le démarrage
        // Topics globaux : /cmd_vel (publisher) et /odom (subscriber)
        //cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        publisher_joystick_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("joystick_order", 1);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TcpUdpBridge::odom_callback, this, std::placeholders::_1)
        );

        // Socket pour envoyer les données UDP aux clients
        udp_send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Lancement des threads de réseau
        tcp_thread_ = std::thread(&TcpUdpBridge::tcp_control_loop, this);
        udp_thread_ = std::thread(&TcpUdpBridge::udp_data_loop, this);
    }

    ~TcpUdpBridge()
    {
        // Arrêt propre
        close(udp_data_socket_);
        close(udp_send_socket_);
        if (tcp_thread_.joinable()) tcp_thread_.join();
        if (udp_thread_.joinable()) udp_thread_.join();
    }

private:
    // --- THREAD DE CONTRÔLE TCP ---
    void tcp_control_loop()
    {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(tcp_control_port_);

        bind(server_fd, (struct sockaddr*)&address, sizeof(address));
        listen(server_fd, 3);

        RCLCPP_INFO(this->get_logger(), "Serveur TCP en écoute sur le port %d", tcp_control_port_);

        while (rclcpp::ok())
        {
            new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
            if (new_socket < 0) {
                // accept erreur
                continue;
            }
            std::string client_ip = inet_ntoa(address.sin_addr);

            // Détacher un thread pour gérer la session client et garder la connexion ouverte
            std::thread(&TcpUdpBridge::client_session, this, new_socket, client_ip).detach();
        }
        close(server_fd);
    }

    // --- Gestion d'une session client TCP ---
    void client_session(int client_socket, std::string client_ip)
    {
        char buffer[2048];

        RCLCPP_INFO(this->get_logger(), "Nouvelle session client depuis %s (socket %d)", client_ip.c_str(), client_socket);

        bool registered_in_this_session = false;
        std::shared_ptr<std::atomic<bool>> session_alive = std::make_shared<std::atomic<bool>>(false);
        std::shared_ptr<std::atomic<long long>> last_activity_ms = std::make_shared<std::atomic<long long>>(0);

        auto now_ms = []() -> long long {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        };

        while (rclcpp::ok()) {
            ssize_t valread = read(client_socket, buffer, sizeof(buffer));
            if (valread <= 0) {
                // client déconnecté, erreur, or recv timeout
                if (valread < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    // recv timeout — vérifier dernière activité
                    long long last_ms = last_activity_ms->load();
                    long long now = now_ms();
                    const long long TIMEOUT_MS = 15000;
                    if (last_ms != 0 && (now - last_ms) > TIMEOUT_MS) {
                        RCLCPP_WARN(this->get_logger(), "Timeout d'activité (%lld ms) — fermeture de la session", now - last_ms);
                        break;
                    } else {
                        // pas encore timeout, continuer à attendre
                        continue;
                    }
                }
                break;
            }

            // Mettre à jour la dernière activité reçue
            last_activity_ms->store(now_ms());

            std::string payload(buffer, static_cast<size_t>(valread));

            try {
                json msg = json::parse(payload);

                if (msg.contains("type") && msg["type"] == "register") {
                    std::string client_id = msg.value("client_id", "client");
                    int recv_port = msg.value("recv_udp_port", 0);
                    RCLCPP_INFO(this->get_logger(), "Requête d'enregistrement '%s' depuis %s (port retour UDP: %d)",
                        client_id.c_str(), client_ip.c_str(), recv_port);

                    // Enregistrement pour le modèle à client unique
                    {
                        std::lock_guard<std::mutex> lock(single_client_mutex_);
                        if (single_client_connected_) {
                            // Un client est déjà connecté
                            json response = {{"ok", false}, {"error", "another client already connected"}};
                            std::string resp_str = response.dump();
                            send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                            continue;
                        }
                        single_client_connected_ = true;
                        single_client_ip_ = client_ip;
                        single_client_recv_port_ = recv_port;
                    }
                    registered_in_this_session = true;

                    // Marquer la session comme vivante et lancer le heartbeat
                    session_alive->store(true);
                    last_activity_ms->store(now_ms());
                    std::weak_ptr<std::atomic<bool>> weak_alive = session_alive;
                    std::weak_ptr<std::atomic<long long>> weak_last = last_activity_ms;
                    int hb_socket = client_socket; // capture copy
                    const long long TIMEOUT_MS = 15000; // 15s without response -> close
                    std::thread([this, weak_alive, weak_last, hb_socket, TIMEOUT_MS]() {
                        if (weak_alive.expired() || weak_last.expired()) return;
                        auto alive = weak_alive.lock();
                        auto last = weak_last.lock();
                        while (alive && alive->load() && rclcpp::ok()) {
                            json hb = {{"type", "heartbeat"}};
                            std::string hb_str = hb.dump();
                            // Ignorer les erreurs de send
                            send(hb_socket, hb_str.c_str(), hb_str.length(), 0);

                            // Attendre intervalle de heartbeat
                            std::this_thread::sleep_for(std::chrono::seconds(13));

                            long long last_ms = last->load();
                            long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                            if (last_ms != 0 && (now - last_ms) > TIMEOUT_MS) {
                                RCLCPP_WARN(rclcpp::get_logger("tcp_udp_bridge"), "Client non-réactif depuis %lld ms, fermeture de la session", now - last_ms);
                                // Indiquer fin de session ; attempt to shutdown the socket to wake reads
                                shutdown(hb_socket, SHUT_RDWR);
                                alive->store(false);
                                break;
                            }
                        }
                    }).detach();

                    // Réponse au client (on garde la connexion ouverte)
                    json response = {
                        {"ok", true},
                        {"udp_data_port", udp_data_port_}
                    };
                    std::string resp_str = response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);

                } else if (msg.contains("type") && msg["type"] == "ping") {
                    // Répondre au ping pour indiquer que le serveur est vivant
                    json response = {{"type", "pong"}};
                    std::string resp_str = response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                } else if (msg.contains("type") && msg["type"] == "emergency_stop") {
                    // Gérer l'arrêt d'urgence (par exemple, publier une commande d'arrêt)
                    RCLCPP_WARN(this->get_logger(), "Arrêt d'urgence reçu de %s", client_ip.c_str());
                    
                    // Ici, on pourrait publier une commande d'arrêt sur un topic ROS 2 si nécessaire
                    start = false;

                    json response = {{"ok", true}, {"message", "Emergency stop acknowledged"}};
                    std::string resp_str = response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                } else if (msg.contains("type") && msg["type"] == "close") {

                    // Le client demande la fermeture de la session
                    break;
                } else if (msg.contains("type") && msg["type"] == "start") {
                    // Gérer la commande de démarrage
                    RCLCPP_INFO(this->get_logger(), "Commande de démarrage reçue de %s", client_ip.c_str());

                    if (mode != 2) {
                        start = true;
                    }

                    json response = {{"ok", true}, {"message", "Start command acknowledged"}};
                    std::string resp_str = response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                } else if (msg.contains("type") && msg["type"] == "set_mode") {
                    // Gérer le changement de mode
                    int new_mode = msg.value("mode", 0);
                    RCLCPP_INFO(this->get_logger(), "Changement de mode reçu de %s : %d", client_ip.c_str(), new_mode);
                    
                    mode = new_mode;
                    if (mode == 2) {
                        start = false; 
                    }

                    json response = {{"ok", true}, {"message", "Mode change acknowledged"}};
                    std::string resp_str = response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                } else if (msg.contains("type") && msg["type"] == "heartbeat_ack") {
                    // Gérer l'accusé de réception du heartbeat
                    RCLCPP_INFO(this->get_logger(), "Accusé de réception du heartbeat de %s", client_ip.c_str());
                } else {
                    // Messages non gérés : log
                    RCLCPP_WARN(this->get_logger(), "Message TCP inconnu de %s: %s", client_ip.c_str(), payload.c_str());
                    json error_response = {{"ok", false}, {"error", "Unknown message type"}};
                    std::string resp_str = error_response.dump();
                    send(client_socket, resp_str.c_str(), resp_str.length(), 0);
                }

            } catch (json::parse_error& e) {
                RCLCPP_WARN(this->get_logger(), "Échec du parsing JSON TCP (session %s) : %s", client_ip.c_str(), e.what());
                // On peut ignorer et attendre la suite des données
                json error_response = {{"ok", false}, {"error", "Invalid JSON"}};
                std::string resp_str = error_response.dump();
                send(client_socket, resp_str.c_str(), resp_str.length(), 0);
            }
        }

        // Fermeture de la socket lorsque la session se termine
        // Indiquer que la session n'est plus vivante (arrêt du heartbeat)
        session_alive->store(false);
        close(client_socket);

        // Si cette session avait enregistré le client, déconnecter au close
        if (registered_in_this_session) {
            std::lock_guard<std::mutex> lock(single_client_mutex_);
            single_client_connected_ = false;
            single_client_ip_.clear();
            single_client_recv_port_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Session client %s (socket %d) terminée", client_ip.c_str(), client_socket);
    }

    // --- THREAD DE DONNÉES UDP (Réception) ---
    void udp_data_loop()
    {
        struct sockaddr_in servaddr, cliaddr;
        char buffer[1024];

        udp_data_socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(udp_data_port_);

        bind(udp_data_socket_, (const struct sockaddr*)&servaddr, sizeof(servaddr));

        RCLCPP_INFO(this->get_logger(), "Serveur UDP en écoute sur le port %d", udp_data_port_);

        while (rclcpp::ok())
        {
            socklen_t len = sizeof(cliaddr);
            int n = recvfrom(udp_data_socket_, (char*)buffer, 1024, MSG_WAITALL, (struct sockaddr*)&cliaddr, &len);
            if (n <= 0) continue;
            buffer[n] = '\0';

            try {
                json data_msg = json::parse(buffer);
                if (data_msg["type"] == "cmd_vel")
                {
                    // Dans le modèle à client unique, on publie directement sur le topic global /cmd_vel
                    float linear_x = data_msg.value("linear_x", 0.0);
                    float angular_z = data_msg.value("angular_z", 0.0);

                    auto joystick_order = std::make_shared<interfaces::msg::JoystickOrder>();

                    if (linear_x < 0.0) {
                        linear_x = -linear_x; // Assurer que la vitesse est positive
                        joystick_order->reverse = true;
                    }else{
                        joystick_order->reverse = false;
                    }

                    joystick_order->throttle = linear_x;
                    joystick_order->steer = angular_z;
                    joystick_order->start = start ? mode != 2 : false;
                    joystick_order->mode = mode;
                    
                    if (publisher_joystick_order_) {
                        // rclcpp::Publisher expects a message object (const ref), not a shared_ptr.
                        // Dereference the shared_ptr and publish the message itself.
                        publisher_joystick_order_->publish(*joystick_order);
                    }
                }
            } catch (json::parse_error& e) {
                RCLCPP_WARN(this->get_logger(), "Échec du parsing JSON UDP : %s", e.what());
            }
        }
    }

    // --- Enregistrement du client (modèle à client unique) ---
    void register_client(const std::string& client_id, const std::string& ip, int recv_port)
    {
        // Dans le modèle à client unique, on ne crée pas de publishers/subscribers par client
        std::lock_guard<std::mutex> lock(single_client_mutex_);
        if (single_client_connected_) {
            RCLCPP_WARN(this->get_logger(), "Tentative d'enregistrer un second client '%s' — refusé", client_id.c_str());
            return;
        }
        single_client_connected_ = true;
        single_client_ip_ = ip;
        single_client_recv_port_ = recv_port;
        RCLCPP_INFO(this->get_logger(), "Client unique enregistré: %s (%s:%d)", client_id.c_str(), ip.c_str(), recv_port);
    }

    // --- Callback pour les messages Odom ---
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::string client_ip;
        int client_port = 0;

        {
            std::lock_guard<std::mutex> lock(single_client_mutex_);
            if (!single_client_connected_) {
                return; // Pas de client connecté
            }
            client_ip = single_client_ip_;
            client_port = single_client_recv_port_;
        }

        // Formatage du message de vitesse réelle
        json real_vel_msg = {
            {"type", "real_vel"},
            {"linear_x", msg->twist.twist.linear.x},
            {"angular_z", msg->twist.twist.angular.z}
        };
        std::string payload = real_vel_msg.dump();

        // Préparation de l'adresse de destination
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(client_port);
        inet_pton(AF_INET, client_ip.c_str(), &dest_addr.sin_addr);

        // Envoi du paquet UDP
        sendto(udp_send_socket_, payload.c_str(), payload.length(), 0,
               (const struct sockaddr*)&dest_addr, sizeof(dest_addr));
    }

    // Modèle à client unique
    bool single_client_connected_ = false;
    std::string single_client_ip_;
    int single_client_recv_port_ = 0;
    std::mutex single_client_mutex_;

    // Topics fixes initialisés au démarrage
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_joystick_order_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    int tcp_control_port_;
    int udp_data_port_;
    
    int udp_data_socket_;  // Socket pour recevoir les cmd_vel
    int udp_send_socket_; // Socket pour envoyer les real_vel

    std::thread tcp_thread_;
    std::thread udp_thread_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpUdpBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}