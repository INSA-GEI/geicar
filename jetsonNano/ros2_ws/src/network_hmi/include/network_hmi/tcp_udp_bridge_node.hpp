#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "interfaces/msg/joystick_order.hpp"

#include "shared_client_info.hpp"
#include "shared_vehicle_state.hpp"
#include "udp_data_sender.hpp"
#include "udp_data_receiver.hpp"
#include "tcp_control_server.hpp"
#include "image_packet_header.hpp"

#include <memory>
#include <vector>

// The main ROS Node class, acts as the "conductor"
class TcpUdpBridgeNode : public rclcpp::Node
{
public:
    TcpUdpBridgeNode();
    ~TcpUdpBridgeNode();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // --- ROS Parameters ---
    int tcp_control_port_;
    int udp_data_port_;
    std::string image_topic_;

    // --- ROS Interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    // The UdpDataReceiver will publish to this, so it's not strictly "owned" by the node
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr joystick_order_pub_;

    // --- Core Components ---
    std::shared_ptr<SharedVehicleState> vehicle_state_;
    std::shared_ptr<SharedClientInfo> client_info_;
    std::unique_ptr<UdpDataSender> udp_sender_;
    std::unique_ptr<UdpDataReceiver> udp_receiver_;
    std::unique_ptr<TcpControlServer> tcp_server_;

    // --- Image fragmentation constants ---
    static constexpr int IMAGE_PACKET_PAYLOAD_SIZE = 1400; 
    static constexpr int HEADER_SIZE = sizeof(ImagePacketHeader);
    
    // Reusable buffer for image packets
    std::vector<uint8_t> image_packet_buffer_;
};