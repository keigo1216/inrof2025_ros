#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class BTNode: public rclcpp::Node {
    public:
        explicit BTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("bt_node", options) {
            // create service client
            srvGenRoute_ = this->create_client<inrof2025_ros_type::srv::GenRoute>("generate_route");
            while (!srvGenRoute_->wait_for_service(1s))
            {
                if (!rclcpp::ok()) {
                    break;
                }
                std::cout << "srvGenRoute not available" << std::endl;
            }
            std::cout << "srvGenRoute service available" << std::endl;

            actFollow_ = rclcpp_action::create_client<inrof2025_ros_type::action::Follow> (this, "follow");
            while (!actFollow_->wait_for_action_server(1s))
            {
                if (!rclcpp::ok()) {
                    break;
                }
                std::cout << "actFollow_ not available" << std::endl;
            }
            std::cout << "actFollow_ service available" << std::endl;
        }

        void send_pose(double x, double y) {
            // std::shared_ptr<::srv::GenerateRoute::Request> request(
            //     new bt_sample::srv::GenerateRoute::Request());
            auto request = std::make_shared<inrof2025_ros_type::srv::GenRoute::Request>();
            request->x = x;
            request->y = y;

            srvGenRoute_->async_send_request(request);
        }

        bool isRuning() {
            return isRun_;
        }

        void send_start_follow() {
            auto goal_msg = inrof2025_ros_type::action::Follow::Goal();
            auto send_goal_options = rclcpp_action::Client<inrof2025_ros_type::action::Follow>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&BTNode::goalResponseCallback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&BTNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&BTNode::resultCallback, this, std::placeholders::_1);

            RCLCPP_INFO(this->get_logger(), "send goal");
            actFollow_->async_send_goal(goal_msg, send_goal_options);
            this->isRun_ = true;
        }

        void goalResponseCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle){
            if (goal_handle) {
                RCLCPP_INFO(this->get_logger(), "get goal_handle");
            } else {
                RCLCPP_WARN(this->get_logger(), "empty goal_handle");
            }
        }

        void feedbackCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle, 
            const std::shared_ptr<const inrof2025_ros_type::action::Follow::Feedback> feedback)
        {
            (void) goal_handle;
        }

        void resultCallback(const rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::WrappedResult result) {
            this->isRun_ = false;
        }

    private:
        // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubGenRoute_;
        rclcpp::Client<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvGenRoute_;
        rclcpp_action::Client<inrof2025_ros_type::action::Follow>::SharedPtr actFollow_;
        rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr currentFollow_;
        bool isRun_{false};
};

std::shared_ptr<BTNode> ros_node;