#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <functional>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;


using SetClaw = robot_arm_interfaces::srv::SetClaw;
using JointAngles = robot_arm_interfaces::msg::JointAngles;

//controls the servo motors and sends data for forward kinematics

class MotorController : public rclcpp::Node
{
public:

  MotorController(const rclcpp::NodeOptions & options= rclcpp::NodeOptions())
  : Node("motor_controller", options)
  {
    
    RCLCPP_INFO(this->get_logger(), "Motors ready.");

    this->service_ptr_ = this->create_service<SetClaw>(
      "set_claw",
      std::bind(&MotorController::set_claw, this, _1, _2));

    this->subscription_ptr_ = this->create_subscription<JointAngles>(
      "arm_angles",10, std::bind(&MotorController::topic_callback, this, std::placeholders::_1));

     auto subscriber_timer_callback =
            [this](){
              return std::bind(&MotorController::topic_callback, this,  std::placeholders::_1);
            };
     
     subscription_timer_ = this->create_wall_timer(500ms, subscriber_timer_callback);
  }

private:

  rclcpp::Service<SetClaw>::SharedPtr service_ptr_;
  rclcpp::Subscription<JointAngles>::SharedPtr subscription_ptr_;
  rclcpp::TimerBase::SharedPtr subscription_timer_;

  void topic_callback(const JointAngles::SharedPtr msg){
            for (size_t i=0; i<msg->joint_angles.size(); i++){
                RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_angles[i]);
            }
         }

  void set_claw(
    const std::shared_ptr<SetClaw::Request> request,
    std::shared_ptr<SetClaw::Response> response)
  {
    if (request->clamp_claw){
      RCLCPP_INFO(this->get_logger(), "Clamping claw.");
      // Code to clamp the claw
    } else {
      RCLCPP_INFO(this->get_logger(), "Releasing claw.");
      // Code to release the claw
    }
    response->success = true;
  }
};  


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =std::make_shared<MotorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}