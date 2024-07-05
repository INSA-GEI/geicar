#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"


using namespace std;
using placeholders::_1;

class simulation : public rclcpp::Node {

public:
    simulation()
    : Node("simulation_node")
    {
        reverse = false;

        publisher_carla_= this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);

        subscription_motors_order_ = this->create_subscription<interfaces::msg::MotorsOrder>(
        "motors_order", 10, std::bind(&simulation::motorsOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&simulation::motorsFeedbackCallback, this, _1));

        timer_ = this->create_wall_timer(5ms, std::bind(&simulation::updateSimulation, this));

        
        RCLCPP_INFO(this->get_logger(), "simulation_node READY");
    }

    
private:

    //Update throttle from the car_control command
    void motorsOrderCallback(const interfaces::msg::MotorsOrder & order) {
        throttle = order.left_rear_pwm;   //PWM [0;100]

        //Convert throttle in [0;1] for the simulator
        if (throttle >=50 ){
            throttle = (throttle - 50.0)/50.0;
            reverse = false;

        } else if (throttle < 50){
            throttle = (50.0 - throttle)/50.0;
            reverse = true;
        } 
    }

    //Update steering angle from the can feedback 
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & feedback) {
        steeringAngle = feedback.steering_angle;
    }


    void updateSimulation(){
        auto simulationCmd = carla_msgs::msg::CarlaEgoVehicleControl();  //By default, motorsOrder stop the car (see interfaces/msg/MotorsOrder.msg)

        simulationCmd.throttle = throttle;
        simulationCmd.steer = steeringAngle;
        simulationCmd.reverse = reverse;

        publisher_carla_->publish(simulationCmd);   //Send cmd to carla simulator
    }



    
    // ---- Private variables ----
    bool reverse;

    //Control variables
    float steeringAngle, throttle; 

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr publisher_carla_;

    rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr subscription_motors_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simulation>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}