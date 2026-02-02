#include <memory>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <sstream>
#include <iostream>
#include <optional>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"
#include "robot_arm_interfaces/srv/servo_angles.hpp"

/*
Set target node

1. fetches user input from the terminal: goal_type, desired position & orientation
2. if configuring the arm, sends service request to inverse_kin for joint angles
	if configuring the claw, sends service request to microros -- TO BE IMPLEMENTED
4. sends the claw state (as a server) over /claw_state
*/

using namespace std::chrono_literals;
using namespace std::placeholders;

using InverseKin = robot_arm_interfaces::srv::InverseKin;
using SetClaw = robot_arm_interfaces::srv::SetClaw;
using ServoAngles = robot_arm_interfaces::srv::ServoAngles;

enum class GoalType {RELEASE_CLAW, CLAMP_CLAW, CONFIG_ARM};

class ServiceClient : public rclcpp::Node{
public:
	 ServiceClient(const rclcpp::NodeOptions & options, geometry_msgs::msg::Transform target_, GoalType goal_type_) 
    : Node("service_client", options),  target(target_), goal_type(goal_type_)
  {	
		angles_client_ptr_ =this->create_client<InverseKin>("target_angles");
		claw_client_ptr_ =this->create_client<SetClaw>("set_claw");	
		servos_client_ptr_ =this->create_client<ServoAngles>("servo_angles");	
	}

	void get_angles(){

		auto request = std::make_shared<InverseKin::Request>();
		request->target.translation.x = target.translation.x;
		request->target.translation.y = target.translation.y;
		request->target.translation.z = target.translation.z;

		request->target.rotation.x = target.rotation.x;
		request->target.rotation.y = target.rotation.y;
		request->target.rotation.z = target.rotation.z;

		RCLCPP_INFO(this->get_logger(), "Sending inverse kin service request: %lf, %lf, %lf, %lf, %lf, %lf", 
		request->target.translation.x,  request->target.translation.y, request->target.translation.z,
		request->target.rotation.x, request->target.rotation.y, request->target.rotation.z);
		
		if (!angles_client_ptr_->service_is_ready()) {
			RCLCPP_WARN(this->get_logger(), "angles service not ready");
			return;
		}
		
        while (!angles_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        //send the request and execute the callback when the response is received. store in a shared_future
		auto future_result = angles_client_ptr_->async_send_request(request);

		// Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			joint_angles.clear();
			auto response = future_result.get()->angles;
           
			RCLCPP_INFO(this->get_logger(), "Inverse kinematics response received: ");
			for (auto angle: response){
				RCLCPP_INFO(this->get_logger(), "%lf ", angle);
				joint_angles.push_back(angle);
			}
			return;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_angles");
        }
    }

	void send_servos_request(){
		if (!this->servos_client_ptr_) {
			RCLCPP_ERROR(this->get_logger(), "Servos service client not initialized");
			return;
		}

		if (joint_angles.size() != 6) {
			RCLCPP_ERROR(this->get_logger(), "joint_angles incomplete, cannot send");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Request to configure servos sent.");

		auto request = std::make_shared<ServoAngles::Request>();
		request->angle_1 = joint_angles[0];
		request->angle_2 = joint_angles[1];
		request->angle_3 = joint_angles[2];
		request->angle_4 = joint_angles[3];
		request->angle_5 = joint_angles[4];
		request->angle_6 = joint_angles[5];

		while (!servos_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

       auto result = servos_client_ptr_->async_send_request(request);
    
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			bool response = result.get()->success;
           
			if (response){
				RCLCPP_INFO(this->get_logger(), "servo state set successfully");
			}else{
				RCLCPP_ERROR(this->get_logger(), "Failed to set servo state");
			}
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service servo_angles");
        }
	}

	void send_claw_request(){

		if (!this->claw_client_ptr_) {
			RCLCPP_ERROR(this->get_logger(), "Claw service client not initialized");
			return;
		}
		auto request = std::make_shared<SetClaw::Request>();
		
		if (goal_type == GoalType::CLAMP_CLAW){
			RCLCPP_INFO(this->get_logger(), "Request to clamp claw sent.");
			request->claw_state = 1;
		}else if (goal_type == GoalType::RELEASE_CLAW){
			RCLCPP_INFO(this->get_logger(), "Request to release claw sent.");
			request->claw_state = 0;
		}else{
			RCLCPP_ERROR(this->get_logger(), "Invalid claw command");
			return;
		}

		while (!claw_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

       auto result = claw_client_ptr_->async_send_request(request);
    
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			bool response = result.get()->success;
           
			if (response){
				RCLCPP_INFO(this->get_logger(), "claw state set successfully");
			}else{
				RCLCPP_ERROR(this->get_logger(), "Failed to set claw state");
			}
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service set_claw");
        }
	}
	private:
		rclcpp::Client<InverseKin>::SharedPtr angles_client_ptr_;
		rclcpp::Client<SetClaw>::SharedPtr claw_client_ptr_;
		rclcpp::Client<ServoAngles>::SharedPtr servos_client_ptr_;
		rclcpp::TimerBase::SharedPtr timer_;
		geometry_msgs::msg::Transform target;
		std::vector<double> joint_angles;
		GoalType goal_type;
};

//sends the target goal to the motors and is subsrcibed to the gyro sensors to get the current angles
class SetTargetClient : public rclcpp::Node{
public:
	
  SetTargetClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  	: Node("set_target_client", options)
  {	
	target = get_user_input();

	if (goal_type == GoalType::CONFIG_ARM){
		auto angles_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		angles_client->get_angles();
		angles_client->send_servos_request();
	}else{

		//create client
		auto claw_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		claw_client->send_claw_request();
	}
	
	}
private:

	rclcpp::TimerBase::SharedPtr timer_;

	std::optional<std::vector<double>> goal_angles;
	std::optional<geometry_msgs::msg::Transform> target;

	GoalType goal_type;
	int goal_type_param;


	std::optional<geometry_msgs::msg::Transform> get_user_input()
	{	
		std::string set_goal_type;
		geometry_msgs::msg::Transform target_;

		RCLCPP_INFO(this->get_logger(),
			"Enter separated by spaces:\n"
			"(1) Configure arm\n"
			"(2) Clamp claw\n"
			"(3) Release claw >> ");
		
		std::getline(std::cin, set_goal_type);  // read the whole line, including spaces
		std::stringstream str_set_goal_type(set_goal_type);     // put it into a stringstream
		char choice;

		if (!(str_set_goal_type >> choice)){
			RCLCPP_ERROR(this->get_logger(), "Invalid input");
			return std::nullopt;
		}     
		switch(static_cast<int>(choice)){
			case '1':
				RCLCPP_INFO(this->get_logger(), "Request received: Configure arm");
				goal_type = GoalType::CONFIG_ARM;
				break;
			case '2':
				RCLCPP_INFO(this->get_logger(), "Request received: Clamp claw");
				goal_type = GoalType::CLAMP_CLAW;
				break;
			case '3':
				RCLCPP_INFO(this->get_logger(), "Request received: Release claw");
				goal_type = GoalType::RELEASE_CLAW;
				break;
			default:
				RCLCPP_ERROR(this->get_logger(),"Request received: Invalid input");
				return std::nullopt;
		}

		if (goal_type== GoalType::CONFIG_ARM){
			RCLCPP_INFO(this->get_logger(), "Enter target arm position as x y z >> ");

				std::string str_target;
				std::getline(std::cin, str_target);  // read the whole line, including spaces
				std::stringstream ss(str_target);     // put it into a stringstream

				double x, y, z;

				if (!(ss >> x >> y >> z)){
					RCLCPP_ERROR(this->get_logger(), "Invalid arm target input");
					return std::nullopt;
				}

				target_.translation.x = x;
				target_.translation.y = y;
				target_.translation.z = z;

			RCLCPP_INFO(this->get_logger(), "Enter target wrist rotations as roll, pitch, yaw (in radians) >> ");

				std::string str_rotation;
				std::getline(std::cin, str_rotation);  // read the whole line, including spaces
				std::stringstream ss_(str_rotation);     // put it into a stringstream

				double roll, pitch, yaw;

				if (!(ss_ >> roll >>pitch >> yaw)){
					RCLCPP_ERROR(this->get_logger(), "Invalid wrist target input");
					return std::nullopt;
				}
				//i know this is supposed to be a quarternion but idk how to use it yet
				target_.rotation.x = roll;
				target_.rotation.y = pitch;
				target_.rotation.z = yaw;
		}

		return target_;
	
	}
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SetTargetClient>());
	rclcpp::shutdown();
	return 0;
}