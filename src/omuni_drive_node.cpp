#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> 
#include <cmath>

namespace omuni {
    class OmuniDriveNode: public rclcpp::Node {
        public:
            explicit OmuniDriveNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
                : Node("omuni_drive_node", options) {
                this->declare_parameter<std::float_t>("r", 0.03);
                this->declare_parameter<std::float_t>("R", 0.15);

                r_ = this->get_parameter("r").as_double();
                R_ = this->get_parameter("R").as_double();

                // cac each sin and cos
                std::float_t alpha1 = 60.0*M_PI/180.0;
                std::float_t alpha2 = 180.0*M_PI/180.0;
                std::float_t alpha3 = 300.0*M_PI/180.0;
                v1cos_ = std::cos(alpha1);
                v2cos_ = std::cos(alpha2);
                v3cos_ = std::cos(alpha3);
                v1sin_ = std::sin(alpha1);
                v2sin_ = std::sin(alpha2);
                v3sin_ = std::sin(alpha3);

                // node settings
                rclcpp::QoS qos(rclcpp::KeepLast(10));
                sub_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", qos, std::bind(&OmuniDriveNode::callback, this, std::placeholders::_1)
                );
                pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controllers/commands", qos);
            }
        private:
            void callback(const geometry_msgs::msg::Twist::UniquePtr msg) {
                std::float_t v_x = msg->linear.x;
                std::float_t v_y = msg->linear.y;
                std::float_t omega = msg->angular.z;

                std::float_t v1 = -v_x*v1sin_ + v_y*v1cos_ + R_*omega;
                std::float_t v2 = -v_x*v2sin_ + v_y*v2cos_ + R_*omega;
                std::float_t v3 = -v_x*v3sin_ + v_y*v3cos_ + R_*omega;

                std::float_t phi1 = v1/r_;
                std::float_t phi2 = v2/r_;
                std::float_t phi3 = v3/r_;

                auto msg_out = std_msgs::msg::Float64MultiArray();
                msg_out.data = {phi1, phi2, phi3};
                pub_->publish(msg_out);

                RCLCPP_DEBUG(this->get_logger(), "Published wheel speeds: v1=%.2f, v2=%.2f, v3=%.2f, phi1=%.2f, phi2=%.2f, phi3=%.2f", v1, v2, v3, phi1, phi2, phi3);
            }

            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
            std::float_t r_; // 車輪半径
            std::float_t R_; // 機体中心から車輪までの距離
            std::float_t v1cos_, v1sin_, v2cos_, v2sin_, v3cos_, v3sin_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<omuni::OmuniDriveNode>());
    rclcpp::shutdown();

    return 0;
}