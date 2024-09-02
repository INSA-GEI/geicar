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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/asio.hpp>
#include <iostream>


using namespace std::chrono_literals;

using namespace boost::asio;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SerialPublisher : public rclcpp::Node
{
public:
  SerialPublisher()
  : Node("serial_publisher"), io(),serial(io,"COM8")
  {
    serial.set_option(serial_port_base::baud_rate(115200));
    publisher_ = this->create_publisher<std_msgs::msg::String>("serial_out", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SerialPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string line;
    boost::asio::read_until(serial, boost::asio::dynamic_buffer(line),'\n');
    auto message = std_msgs::msg::String();
    message.data = line;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  io_service io;
  serial_port serial;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialPublisher>());
  rclcpp::shutdown();
  return 0;
}
