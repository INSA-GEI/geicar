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
    this->declare_parameter<std::string>("image_topic", "/out/compressed");

    tcp_control_port_ = this->get_parameter("tcp_control_port").as_int();
    udp_data_port_ = this->get_parameter("udp_data_port").as_int();
    image_topic_ = this->get_parameter("image_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting bridge node...");
    RCLCPP_INFO(this->get_logger(), " - TCP Control Port: %d", tcp_control_port_);
    RCLCPP_INFO(this->get_logger(), " - UDP Data Port: %d", udp_data_port_);
    RCLCPP_INFO(this->get_logger(), " - Image Topic: %s", image_topic_.c_str());

    // --- Create core components ---
    vehicle_state_ = std::make_shared<SharedVehicleState>();
    client_info_ = std::make_shared<SharedClientInfo>(this->get_logger());
    udp_sender_ = std::make_unique<UdpDataSender>();
    
    // The receiver needs the node to create a publisher
    udp_receiver_ = std::make_unique<UdpDataReceiver>(
        this, 
        vehicle_state_, 
        udp_data_port_
    );
    
    // The server needs the shared state objects to pass to new sessions
    tcp_server_ = std::make_unique<TcpControlServer>(
        this->get_logger(),
        tcp_control_port_,
        client_info_,
        vehicle_state_
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

    // --- Start network threads ---
    udp_receiver_->start();
    tcp_server_->start();
    
    RCLCPP_INFO(this->get_logger(), "Bridge node started and network threads running.");
}

TcpUdpBridgeNode::~TcpUdpBridgeNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down bridge node...");
    // Stop threads in reverse order
    tcp_server_->stop();
    udp_receiver_->stop();
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