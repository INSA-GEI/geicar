#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <string>
#include "shared_client_info.hpp"
#include "shared_vehicle_state.hpp"

// Runs in its own thread to accept new TCP connections
class TcpControlServer
{
public:
    TcpControlServer(
        rclcpp::Logger logger,
        int port,
        std::shared_ptr<SharedClientInfo> client_info,
        std::shared_ptr<SharedVehicleState> vehicle_state
    );
    ~TcpControlServer();

    void start();
    void stop();

private:
    void accept_loop();

    rclcpp::Logger logger_;
    int port_;
    int server_fd_ = -1;
    std::shared_ptr<SharedClientInfo> client_info_;
    std::shared_ptr<SharedVehicleState> vehicle_state_;

    std::thread thread_;
    std::atomic<bool> running_{false};
};