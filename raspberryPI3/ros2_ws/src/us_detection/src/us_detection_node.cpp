

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacles.hpp"


#include "../include/us_detection/us_detection_node.h"

using namespace std;
using placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class us_detection : public rclcpp::Node
{
public:
  us_detection()
  : Node("us_detection_node")
  {
    publisher_ = this->create_publisher<interfaces::msg::Obstacles>("Obstacles", 10);

    subscription_ultrasonic_sensor_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&us_detection::usDataCallback, this, _1));
    
    
    timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&us_detection::Object_detection, this));

    RCLCPP_INFO(this->get_logger(), "car_control_node READY");
  }

private:

  int a = 0;

  void usDataCallback(const interfaces::msg::Ultrasonic & ultrasonic){
    CenterObstacle = ultrasonic.front_center;
    RightObstacle = ultrasonic.front_right;
    LeftObstacle = ultrasonic.front_left;
  }

  void Object_detection() {

    auto Obstacles = interfaces::msg::Obstacles();

    /*
    if ((CenterObstacle <= 50.0)){
      if(a!=1){
          RCLCPP_INFO(this->get_logger(), "Front obstacle near = %f cm", CenterObstacle);
          a = 1;
      }
      
      RCLCPP_INFO(this->get_logger(), "Front obstacle near = %f cm", CenterObstacle);
      speed_order = 0;
    }

  //obstacle à gauche à moins de 20 cm

    else if((LeftObstacle <= 20.0)){
        if(a!=4){
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %f cm", LeftObstacle);
            a = 4;
        }
      speed_order = 0;
      RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %f cm", LeftObstacle);
    }

  //obstacle à droite à moins de 20 cm

    else if((RightObstacle <= 20.0)){
      RCLCPP_INFO(this->get_logger(), "Obstacle on the right = %f cm", RightObstacle);
      a = 5;
      speed_order = 0; 
    }

  //obstacle au centre entre 50cm et 1m : half speed   

    else if(CenterObstacle > 50.0 && CenterObstacle <= 100.0){
      if(a!=2){
          RCLCPP_INFO(this->get_logger(), "Front obstacle far = %f cm", CenterObstacle);
          a = 2;
      }
      RCLCPP_INFO(this->get_logger(), "Front obstacle far = %f cm", CenterObstacle);
      speed_order = 1;
    }

  //pas d'obstacle à moins d'1m

    else{
      if(a!=3){
          RCLCPP_INFO(this->get_logger(), "No obstacle");
          a = 3;
      }
      RCLCPP_INFO(this->get_logger(), "No obstacle");
      speed_order = 2;
    }
    */

    //RCLCPP_INFO(this->get_logger(), "a = %i", a);

  //Obstacles.speed_order = speed_order;

  Obstacles.speed_order = 2;

  publisher_->publish(Obstacles);

  }

    //Ultrasonic feedback variables
    float LeftObstacle;
    float RightObstacle;
    float CenterObstacle;

    //Speed variable
    uint8_t speed_order;

  rclcpp::TimerBase::SharedPtr timer_;

  //Publisher
  rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_;

  //Subscriber
  rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_sensor_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<us_detection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
