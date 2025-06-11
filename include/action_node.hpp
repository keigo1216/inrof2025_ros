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
}