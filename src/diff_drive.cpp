#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace diff {
    class TeleopKeyboard: public rclcpp::Node {
        public:
            TeleopKeyboard(): Node("teleop_keyborad") {
                publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_base_controller/cmd_vel", 10);
                RCLCPP_INFO(this->get_logger(), "Teleop Keyboard Node Started.");
            }
            
            void keyboradLoop() {
                char c;

                while (rclcpp::ok()) {
                    std::cout << "コマンド入力 (w: 前進, s: 後退, a: 左旋回, d: 右旋回, x: 停止, q: 終了): ";
                    std::cin >> c;

                    if (c == 'q') {
                        RCLCPP_INFO(this->get_logger(), "finished");
                        break;
                    }

                    geometry_msgs::msg::TwistStamped twist_msg;

                    twist_msg.header.stamp = this->now();
                    twist_msg.header.frame_id = "base_link";

                    const double speed = 5;
                    const double turn  = 1.0;

                    switch (c) {
                        case 'w':
                            twist_msg.twist.linear.x = speed;
                            twist_msg.twist.angular.z = 0.0;
                            break;
                        case 's':
                            twist_msg.twist.linear.x = -speed;
                            twist_msg.twist.angular.z = 0.0;
                            break;
                        case 'a':
                            twist_msg.twist.linear.x = 0.0;
                            twist_msg.twist.angular.z = turn;
                            break;
                        case 'd':
                            twist_msg.twist.linear.x = 0.0;
                            twist_msg.twist.angular.z = -turn;
                            break;
                        case 'x':
                            twist_msg.twist.linear.x = 0.0;
                            twist_msg.twist.angular.z = 0.0;
                            break;
                        default:
                            RCLCPP_WARN(this->get_logger(), "無効なコマンドです: [%c]", c);
                            continue;
                    }
                

                    publisher_->publish(twist_msg);
                }
            }
        private:
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    };
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto teleop_node = std::make_shared<diff::TeleopKeyboard>();

    std::thread keyboard_thread(&diff::TeleopKeyboard::keyboradLoop, teleop_node);
    rclcpp::spin(teleop_node);

    if (keyboard_thread.joinable()) {
        keyboard_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}