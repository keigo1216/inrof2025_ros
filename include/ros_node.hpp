#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>

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
        }

        void send_pose(double x, double y) {
            // std::shared_ptr<::srv::GenerateRoute::Request> request(
            //     new bt_sample::srv::GenerateRoute::Request());
            auto request = std::make_shared<inrof2025_ros_type::srv::GenRoute::Request>();
            request->x = x;
            request->y = y;

            srvGenRoute_->async_send_request(request);
        }

    private:
        // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubGenRoute_;
        rclcpp::Client<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvGenRoute_;
};

std::shared_ptr<BTNode> ros_node;