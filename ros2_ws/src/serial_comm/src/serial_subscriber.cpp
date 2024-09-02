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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <string>

using std::placeholders::_1;

using namespace boost::asio;

class SerialSubscriber : public rclcpp::Node
{
public:
  SerialSubscriber()
  : Node("serial_subscriber"), io(), serial(io, "COM8")
  {
    serial.set_option(serial_port_base::baud_rate(115200));
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "serial_in", 10, std::bind(&SerialSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  io_service io;
  serial_port serial;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialSubscriber>());
  rclcpp::shutdown();
  return 0;
}
