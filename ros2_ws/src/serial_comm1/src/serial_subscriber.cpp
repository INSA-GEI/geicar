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

int main(int argc, char * argv[])
{
  // Initialisation de ROS
  rclcpp::init(argc, argv);
  
  // Création du Node ROS
  auto node = rclcpp::Node::make_shared("pub_ros");

  // Initialisation du port série
  io_service io;
  serial_port serial(io, "/dev/ttyUSB0");
  try {
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  } catch (const boost::system::system_error& ex) {
    RCLCPP_ERROR(node->get_logger(), "Erreur système lors de la configuration du port série : %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // Callback pour traiter les messages reçus
  auto topic_callback = [&](const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "Reçu : '%s'", msg->data.c_str());

    // Décomposer le message
    std::istringstream iss(msg->data);
    int pwm_number;
    int pwm_channel;
    float pwm_value;

    // Vérification du format du message
    if (!(iss >> pwm_number >> pwm_channel >> pwm_value)) {
      RCLCPP_ERROR(node->get_logger(), "Format d'entrée invalide");
      return;
    }

    // Vérification des limites du canal PWM
    if (pwm_channel < 0 || pwm_channel > 255) {
      RCLCPP_ERROR(node->get_logger(), "Le canal PWM doit être une valeur de 8 bits (0-255)");
      return;
    }

    // Vérification de la plage de la valeur PWM
    if (pwm_value < 1.0f || pwm_value > 2.0f) {
      RCLCPP_ERROR(node->get_logger(), "La valeur PWM est hors des limites (doit être entre 1.0 et 2.0)");
      return;
    }

    // Préparation des données à envoyer
    std::vector<uint8_t> buffer;
    buffer.push_back(static_cast<uint8_t>(pwm_number));   // Numéro PWM
    buffer.push_back(static_cast<uint8_t>(pwm_channel));  // Canal PWM (8 bits)

    // Conversion de la valeur float en bytes
    uint8_t* pwm_value_bytes = reinterpret_cast<uint8_t*>(&pwm_value);
    buffer.insert(buffer.end(), pwm_value_bytes, pwm_value_bytes + sizeof(float));

    // Envoi des données via le port série
    try {
      boost::asio::write(serial, boost::asio::buffer(buffer));
      RCLCPP_INFO(node->get_logger(), "Trame envoyée sur /dev/ttyUSB0");
    } catch (const boost::system::system_error& ex) {
      RCLCPP_ERROR(node->get_logger(), "Erreur d'envoi sur le port série : %s", ex.what());
    }
  };

  // Création de la souscription au topic "serial_out"
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "serial_in", 10, topic_callback);

  // Exécution du node ROS
  rclcpp::spin(node);
  
  // Fermeture propre du port série
  serial.close();

  // Arrêt de ROS
  rclcpp::shutdown();
  return 0;
}
