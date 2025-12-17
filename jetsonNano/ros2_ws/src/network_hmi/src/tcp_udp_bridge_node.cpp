#include "network_hmi/tcp_udp_bridge_node.hpp"
#include "network_hmi/h264_streamer.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <nlohmann/json.hpp>
#include <arpa/inet.h> // For htonl, htons
#include <cstring> // For memcpy

using json = nlohmann::json;
using std::placeholders::_1;

TcpUdpBridgeNode::TcpUdpBridgeNode()
: Node("tcp_udp_bridge"),
  image_packet_buffer_(HEADER_SIZE + IMAGE_PACKET_PAYLOAD_SIZE) // Pre-allocate buffer
{
    // Declare and get parameters
    this->declare_parameter<int>("tcp_control_port", 5001);
    this->declare_parameter<int>("udp_data_port", 5000);
    this->declare_parameter<std::string>("image_topic", "/usb_cam_left/image_raw/compressed");
    this->declare_parameter<std::string>("general_data_topic", "/general_data");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("control_topic", "/control_msg");

    tcp_control_port_ = this->get_parameter("tcp_control_port").as_int();
    udp_data_port_ = this->get_parameter("udp_data_port").as_int();
    image_topic_ = this->get_parameter("image_topic").as_string();
    general_data_topic_ = this->get_parameter("general_data_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    control_topic_ = this->get_parameter("control_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting bridge node...");
    RCLCPP_INFO(this->get_logger(), " - TCP Control Port: %d", tcp_control_port_);
    RCLCPP_INFO(this->get_logger(), " - UDP Data Port: %d", udp_data_port_);
    RCLCPP_INFO(this->get_logger(), " - Image Topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - General Data Topic: %s", general_data_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - Map Topic: %s", map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - Control Topic: %s", control_topic_.c_str());

    // --- Create core components ---
    vehicle_state_ = std::make_shared<SharedVehicleState>();
    client_info_ = std::make_shared<SharedClientInfo>(this->get_logger());
    udp_sender_ = std::make_unique<UdpDataSender>();
    
    // The receiver needs the node to create a publisher
    udp_receiver_ = std::make_unique<UdpDataReceiver>(
        this, 
        vehicle_state_, 
        udp_data_port_,
        "network_joystick_order"
    );
    
    // The server needs the shared state objects to pass to new sessions
    tcp_server_ = std::make_unique<TcpControlServer>(
        this,
        this->get_logger(),
        tcp_control_port_,
        client_info_,
        vehicle_state_,
        control_topic_
    );

    // --- Create ROS subscribers ---
    // Note: joystick_order_pub_ is created inside UdpDataReceiver
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TcpUdpBridgeNode::odom_callback, this, _1)
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, 1, // QoS 1, only need the latest
        std::bind(&TcpUdpBridgeNode::image_callback, this, _1)
    );

    general_data_sub_ = this->create_subscription<interfaces::msg::GeneralData>(
        general_data_topic_, 10,
        std::bind(&TcpUdpBridgeNode::general_data_callback, this, _1)
    );

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 1,
        std::bind(&TcpUdpBridgeNode::map_callback, this, _1)
    );


    // --- Start network threads ---
    udp_receiver_->start();
    tcp_server_->start();
    
    RCLCPP_INFO(this->get_logger(), "Bridge node started and network threads running.");
}

TcpUdpBridgeNode::~TcpUdpBridgeNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down bridge node...");

    // Stop threads in reverse order with timing logs to diagnose hangs
    {
        auto t0 = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Stopping TcpControlServer...");
        tcp_server_->stop();
        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        RCLCPP_INFO(this->get_logger(), "TcpControlServer stopped (%.1f ms)", static_cast<double>(ms));
    }

    {
        auto t0 = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Stopping UdpDataReceiver...");
        udp_receiver_->stop();
        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        RCLCPP_INFO(this->get_logger(), "UdpDataReceiver stopped (%.1f ms)", static_cast<double>(ms));
    }

    // udp_sender_ and state objects are auto-destroyed
    RCLCPP_INFO(this->get_logger(), "Bridge node shut down complete.");
}

void TcpUdpBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    json real_vel_msg = {
        {"type", "real_vel"},
        {"linear_x", msg->twist.twist.linear.x},
        {"angular_z", msg->twist.twist.angular.z}
    };
    
    udp_sender_->send_json(real_vel_msg.dump(), dest);
}

void TcpUdpBridgeNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        // 1. Convert ROS message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 2. Push the image to the streamer
        auto streamer = client_info_->get_h264_streamer();
        if (streamer) {
            streamer->push_image(cv_ptr->image);
        }

    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void TcpUdpBridgeNode::general_data_callback(const interfaces::msg::GeneralData::SharedPtr msg)
{
    static time_t last_log_time = 0;
    time_t current_time = time(nullptr);
    if (current_time - last_log_time < 5)
    { // Ignore logs more frequent than every 5 seconds
        return;    
    }

    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    json general_data_msg = {
        {"type", "general_data"},
        {"battery_level", msg->battery_level}
    };
    
    udp_sender_->send_json(general_data_msg.dump(), dest);
    last_log_time = current_time;
}

void TcpUdpBridgeNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    uint32_t w = msg->info.width;
    uint32_t h = msg->info.height;
    
    // Downsample map if too large
    float x_scale = 1.0f;
    float y_scale = 1.0f;
    if (w > MAP_SIZE_LIMIT) {
        x_scale = static_cast<float>(w) / MAP_SIZE_LIMIT;
    }
    if (h > MAP_SIZE_LIMIT) {
        y_scale = static_cast<float>(h) / MAP_SIZE_LIMIT;
    }
    float scale = std::max(x_scale, y_scale);
    uint32_t r_w = static_cast<uint32_t>(w / scale);
    uint32_t r_h = static_cast<uint32_t>(h / scale);
    std::vector<int8_t> r_map(r_w * r_h);
    for (uint32_t y = 0; y < r_h; ++y) {
        for (uint32_t x = 0; x < r_w; ++x) {
            uint32_t orig_x = static_cast<uint32_t>(x * scale);
            uint32_t orig_y = static_cast<uint32_t>(y * scale);
            r_map[y * r_w + x] = msg->data[orig_y * w + orig_x];
        }
    }

    json map_info_msg = {
        {"type", "occupancy_grid"},
        {"width", r_w},
        {"height", r_h},
        {"resolution", msg->info.resolution * scale},
        {"origin_position_x", msg->info.origin.position.x},
        {"origin_position_y", msg->info.origin.position.y},
        {"origin_position_z", msg->info.origin.position.z},
        {"origin_orientation_x", msg->info.origin.orientation.x},
        {"origin_orientation_y", msg->info.origin.orientation.y},
        {"origin_orientation_z", msg->info.origin.orientation.z},
        {"origin_orientation_w", msg->info.origin.orientation.w},
        {"data", r_map}
    };

    udp_sender_->send_json(map_info_msg.dump(), dest);
}