#pragma once
#include <rclcpp/allocator/allocator_common.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include "ros_node.hpp"

using namespace BT;

namespace ActionNodes {
    class GenerateRoute: public SyncActionNode {
        public:
            GenerateRoute(const std::string& name, const NodeConfig& config): SyncActionNode(name, config) {};

            // port info
            static PortsList providedPorts() {
                return {
                    InputPort<double> ("x"),
                    InputPort<double> ("y")
                };
            }

            NodeStatus tick() override {
                std::cout << "call generate route" << std::endl;

                Expected<double> tmp_x = getInput<double>("x");
                Expected<double> tmp_y = getInput<double>("y");
                if (!tmp_x) {
                    throw BT::RuntimeError("missing required input x: ", tmp_x.error() );
                }
                if (!tmp_y) {
                    throw BT::RuntimeError("missing required input x: ", tmp_y.error() );
                }

                double x = tmp_x.value();
                double y = tmp_y.value();
                
                if (ros_node == nullptr) std::cerr << "null ptr" << std::endl;

                ros_node->send_pose(x, y);

                return NodeStatus::SUCCESS;
            }
        private:
    };

    class FollowRoute: public StatefulActionNode {
        public:
            FollowRoute(const std::string& name) : StatefulActionNode(name, {}){}

            NodeStatus onStart() override {
                ros_node->send_start_follow();
                return NodeStatus::RUNNING;
            }

            NodeStatus onRunning() override {
                
                if (ros_node->isRuning()) {
                    return NodeStatus::RUNNING;
                } else {
                    return NodeStatus::SUCCESS;
                }
            }

            void onHalted() override {
                // TODO
                std::cout << "interrupt SampleNode" << std::endl;
            }
    };

    class Rotate : public StatefulActionNode {
        public:
            Rotate(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

            static PortsList providedPorts() {
                return { InputPort<int>("theta") };
            }

            NodeStatus onStart() override {
                std::cout << "call SampleNode" << std::endl;
                
                // InputPortの値を受け取る
                Expected<double> msg = getInput<int>("theta");
                if (!msg) { // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [sample_input]: ", msg.error() );
                }
                double targetTheta = msg.value();
                ros_node->send_rotate_position(targetTheta);

                return NodeStatus::RUNNING;
            }

            NodeStatus onRunning() override {
            
                if (ros_node->isRotateRuning()) {
                    return NodeStatus::RUNNING;
                } else {
                    return NodeStatus::SUCCESS;
                }
                return NodeStatus::SUCCESS;
            }

            void onHalted() override {
                std::cout << "interrupt SampleNode" << std::endl;
            }
    };
}