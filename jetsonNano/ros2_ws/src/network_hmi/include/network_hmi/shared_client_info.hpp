#pragma once

#include <string>
#include <mutex>
#include <netinet/in.h> // For sockaddr_in

// Thread-safe class to hold the single client's connection info
class SharedClientInfo
{
public:
    bool is_connected();
    
    // Attempts to register. Returns false if a client is already connected.
    bool register_client(const std::string& ip, int data_port, int image_port);
    void deregister_client();

    // Struct to hold a pre-built UDP address for efficiency
    struct UdpAddress {
        sockaddr_in addr;
        bool valid = false;
    };

    UdpAddress get_data_address();
    UdpAddress get_image_address();

private:
    std::mutex mutex_;
    bool connected_ = false;
    std::string ip_;
    int data_port_ = 0;
    int image_port_ = 0;

    // Pre-computed address structures
    UdpAddress data_addr_;
    UdpAddress image_addr_;
};