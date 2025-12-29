#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "robot_arm_interfaces/action/set_target.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"
#include "robot_arm_interfaces/msg/joint_states.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

using JointStates = robot_arm_interfaces::msg::JointStates;
using SetTarget = robot_arm_interfaces::action::SetTarget;
using GoalHandleTarget = rclcpp_action::ClientGoalHandle<SetTarget>;
using InverseKin = robot_arm_interfaces::srv::InverseKin;


class SetTargetClient : public rclcpp::Node
{
public:
	
  SetTargetClient(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  	: Node("set_target_client", options)
  {	
	//a client needs 3 things:
	/*
	1.The templated action type name: SetTargetClient.
	2.A ROS 2 node to add the action client to: this.
	3. The action name: 'set_target'.
	*/

    subscription_ptr_ = this->create_subscription<JointStates>(
		"gyro_arm_angles", 10, std::bind(&SetTargetClient::topic_callback, this, _1));

	this->client_ptr_ = rclcpp_action::create_client<SetTarget>(
      this,
      "set_target");
	//callable object that will initiate the function send_goal
    auto client_timer_callback_lambda = [this](){ return this->send_goal(); };
    
	//create timer that will call the timer_callback every 500 ms
	this->client_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      client_timer_callback_lambda);

    //callable object that will initiate the function send_goal
	auto sub_timer_callback_lambda = [this](){ return std::bind(&SetTargetClient::topic_callback, this, _1);};
	
	//create timer that will call the timer_callback every 500 ms
	this->sub_timer_ = this->create_wall_timer(
	std::chrono::milliseconds(500),
	sub_timer_callback_lambda);
	}

    
	void topic_callback(const JointStates::SharedPtr msg)
	{
		for (size_t i=0; i<msg->joint_pos.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_pos[i]);
		}
    }
	
	void send_goal(){
		using namespace std::placeholders;

		//immediately cancel the timer (the goal should only be called once)
		this->client_timer_->cancel();

		//if time out
		if (!this->client_ptr_->wait_for_action_server()) {
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		}
		//fetches the goal from ArmAngles.action
		auto goal_msg = SetTarget::Goal();

        goal_msg.target.x = 10.0f;
        goal_msg.target.y = 20.0f;
        goal_msg.target.z = -5.0f;

		//configure what to do after sending the goal. Is sent to ROS to be managed later
		auto send_goal_options = rclcpp_action::Client<SetTarget>::SendGoalOptions();

		//when the goal is sent to the server and the server responds, ROS will fill the shared ptr of type
		//rclcpp_action::ClientGoalHandle
		send_goal_options.goal_response_callback = [this](const GoalHandleTarget::SharedPtr & goal_handle)
		{
		if (!goal_handle) {
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		} else {
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		}
		};

		//specify which goal this feedback is for, and also the feedback message
		send_goal_options.feedback_callback = [this](
		GoalHandleTarget::SharedPtr,
		const std::shared_ptr<const SetTarget::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(), "\nCurrent Position Recieved: %lf, %lf, %lf", 
            feedback->curr_pos.x, feedback->curr_pos.y, feedback->curr_pos.z);
		};

		//handle the response depending on the type of response
		send_goal_options.result_callback = [this](const GoalHandleTarget::WrappedResult & result)
		{
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
			break;
			case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
			return;
			case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
			return;
			default:
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			return;
		}
		auto res = result.result->target_reached;

		RCLCPP_INFO(this->get_logger(), "Result received: %lf, %lf, %lf: ", res.x, res.y, res.z);
		};
		this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
	}
	private:
		rclcpp_action::Client<SetTarget>::SharedPtr client_ptr_;
        rclcpp::Subscription<JointStates>::SharedPtr subscription_ptr_;
		rclcpp::TimerBase::SharedPtr client_timer_;
        rclcpp::TimerBase::SharedPtr sub_timer_;
};

class GetAnglesClient: public rclcpp::Node
{
public:
    GetAnglesClient(const rclcpp::NodeOptions & options, int argc_, char** argv_) 
    : Node("get_angles_client", options), argc(argc_), argv(argv_)
    {
        client_ptr_ =this->create_client<InverseKin>("arm_angles");
    }

    void get_angles(){

        if (argc != 4) {
            RCLCPP_INFO(this->get_logger(), "usage: add_two_ints_client X Y");
            return;
        }

        auto request = std::make_shared<InverseKin::Request>();
        
        request->target.x = std::stof(argv[1]);
        request->target.y = std::stof(argv[2]);
        request->target.z =std::stof(argv[3]);

        RCLCPP_INFO(this->get_logger(), "send request: %lf, %lf, %lf", 
            request->target.x,  request->target.y, request->target.z);

        while (!client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

       auto result = client_ptr_->async_send_request(request);
    
        // Wait for the result.
        //shared_from_this is a smart pointer that can point to this
         if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {   
            auto response = result.get()->angles;
           
            RCLCPP_INFO(this->get_logger(), "Angles Received:");
            for (auto angle : response){
                RCLCPP_INFO(this->get_logger(), "%f ", angle);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
     
    }
private:
    rclcpp::Client<InverseKin>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    int argc;
    char** argv;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto angles_client = std::make_shared<GetAnglesClient>(rclcpp::NodeOptions(), argc, argv);
  angles_client->get_angles();
  //rclcpp::spin(angles_client);
    rclcpp::spin(std::make_shared<SetTargetClient>());

  rclcpp::shutdown();
  return 0;
}