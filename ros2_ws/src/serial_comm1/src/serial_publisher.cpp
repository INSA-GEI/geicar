// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>

using namespace std::chrono_literals;
using namespace boost::asio;

std::vector<uint8_t> readFrame(boost::asio::serial_port& serial) {
  std::vector<uint8_t> frame;
  uint8_t start_delimiter;

  RCLCPP_INFO(rclcpp::get_logger("serial_publisher"),"Waiting for start delimiter...");
  do {
    boost::asio::read(serial, boost::asio::buffer(&start_delimiter, 1));
  } while (start_delimiter != 0x7E);

  frame.push_back(start_delimiter);

  uint8_t vide;
  boost::asio::read(serial, boost::asio::buffer(&vide, 1)); //car deux 0 ont été ajouté à la trame alors que normalement le start delimiter est sur 1 octet 

  uint8_t length_bytes[2];
  boost::asio::read(serial, boost::asio::buffer(length_bytes, 2));
  uint16_t length = (length_bytes[1] << 8) | length_bytes[0];
  //std::swap(length_bytes[0],length_bytes[1]); //inverser les deux octets de longueur
  frame.push_back(length_bytes[1]);
  frame.push_back(length_bytes[0]);

  uint8_t frame_type;
  boost::asio::read(serial, boost::asio::buffer(&frame_type, 1));
  frame.push_back(frame_type);

  

  std::vector<uint8_t> frame_data(length+1);
  boost::asio::read(serial, boost::asio::buffer(frame_data.data(), length+1));

  frame.insert(frame.end(), frame_data.begin(), frame_data.end());
  
  switch (frame_type)
  {
    case 0x20 :
      RCLCPP_INFO(rclcpp::get_logger("serial_publisher"),"ID IMU 0x20");
      break;
    case 0x30 : 
      RCLCPP_INFO(rclcpp::get_logger("serial_publisher"),"ID GPS 0x30");
      break;
    case 0x40 : 
      RCLCPP_INFO(rclcpp::get_logger("serial_publisher"),"ID LIDAR 0x40");
      break;

  }
  
  return frame;
}

// Fonction pour convertir une trame en chaîne hexadécimale
std::string toHexString(const std::vector<uint8_t>& frame) {
  std::ostringstream hex_stream;
  hex_stream << std::hex << std::setfill('0');
  for (auto byte : frame) {
    hex_stream << std::setw(2) << static_cast<int>(byte);
  }
  return hex_stream.str();
}

void swapEndian(std::vector<uint8_t>& data) {
    size_t data_size = data.size();
    
    for (size_t i = 0; i + 4 <= data_size; i += 4) {
        std::swap(data[i], data[i + 3]);
        std::swap(data[i + 1], data[i + 2]);
    }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("serial_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("serial_out", 10);

  try {
    boost::asio::io_service io;
    boost::asio::serial_port serial(io, "/dev/ttyUSB0");
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      std::vector<uint8_t> frame = readFrame(serial);
      std::string frame_str(frame.begin(), frame.end());

      std::vector<uint8_t> data(frame.begin() + 4, frame.end()); // Extraire les données 
      //(après le start delimiter 1 octet la longueur 2 octets et le type frame 1 octet)
      swapEndian(data);

      // Réinsérer les données inversées dans la trame
      std::copy(data.begin(), data.end(), frame.begin() + 4);

      auto msg = std::make_shared<std_msgs::msg::String>();
      std::string frame_hex = toHexString(frame);
      //msg->data = frame_str;
      msg->data = frame_hex;
      publisher->publish(*msg);

      

      if(frame.empty()){
        RCLCPP_WARN(node->get_logger(),"Empty frame received");
      }
      else
      {
        RCLCPP_INFO(node->get_logger(), "Frame received with %zu bytes", frame.size());
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg->data.c_str());
      }
      

      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
  } catch (const boost::system::system_error& ex) {
    RCLCPP_ERROR(node->get_logger(), "System error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
