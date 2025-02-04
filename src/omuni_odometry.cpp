#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace omuni {
    class Odometry: public rclcpp::Node {
        public:
            explicit Odometry(std::float_t r, std::float_t R): Node("omuni_odometry_frame_publisher") {
                r_ = r;
                R_ = R;

                // declare paramters, default set to base_footprint
                base_footprint_ = "base_footprint";

                // set up trigonometric functions
                float alpha0 = 60.0*M_PI/180.0;
                float alpha1 = 180.0*M_PI/180.0;
                float alpha2 = 300.0*M_PI/180.0;
                float cosa0 = std::cos(alpha0);
                float cosa1 = std::cos(alpha1);
                float cosa2 = std::cos(alpha2);
                float sina0 = std::sin(alpha0);
                float sina1 = std::sin(alpha1);
                float sina2 = std::sin(alpha2);

                sina1_a0 = std::sin(alpha1-alpha0);
                sina2_a1 = std::sin(alpha2-alpha1);
                sina0_a2 = std::sin(alpha0-alpha2);
                cosa0_cosa1 = cosa0 - cosa1;
                cosa1_cosa2 = cosa1 - cosa2;
                cosa2_cosa0 = cosa2 - cosa0;
                sina0_sina1 = sina0 - sina1;
                sina1_sina2 = sina1 - sina2;
                sina2_sina0 = sina2 - sina0;
                m = sina0_sina1*cosa2 + sina2_sina0*cosa1 + sina1_sina2*cosa0;

                // Initialize the transform broadcaster
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_states",
                    10,
                    std::bind(&Odometry::callback, this, std::placeholders::_1)
                );
            }
        private:
            void callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
                // reference: https://t-semi.esa.io/posts/191
                // それぞれの回転量からv_x, v_y, omegaを計算
                
                // angular velocity
                std::float_t wheel_0_omega = 0.0;
                std::float_t wheel_1_omega = 0.0;
                std::float_t wheel_2_omega = 0.0;
                for (size_t i=0; i<msg->name.size(); i++) {
                    if (msg->name[i] == "wheel_1_joint") {
                        wheel_0_omega = msg->velocity[i];
                    } else if (msg->name[i] == "wheel_2_joint") {
                        wheel_1_omega = msg->velocity[i];
                    } else if (msg->name[i] == "wheel_3_joint") {
                        wheel_2_omega = msg->velocity[i];
                    }
                }

                // convert to velocity
                std::float_t v[3] = {r_*wheel_0_omega, r_*wheel_1_omega, r_*wheel_2_omega};

                RCLCPP_INFO(this->get_logger(), "Subscribe wheel speeds: v1=%.2f, v2=%.2f, v3=%.2f", v[0], v[1], v[2]);

                // convet to v_1, v_2, v_3 to v_x, v_y, omega
                v_x_ = ((cosa1_cosa2)*v[0] + (cosa2_cosa0)*v[1] + (cosa0_cosa1)*v[2])/m;
                v_y_ = ((sina1_sina2)*v[0] + (sina2_sina0)*v[1] + (sina0_sina1)*v[2])/m;
                omega_ = ((sina2_a1)*v[0] + (sina0_a2)*v[1] + (sina1_a0)*v[2])/(R_*m);

                // RCLCPP_INFO(this->get_logger(), "Subscribe wheel speeds: v_x=%.2f, v_y=%.2f, omega=%.2f", v_x_, v_y_, omega_);
            }
            
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::string base_footprint_;

            std::double_t v_x_;
            std::double_t v_y_;
            std::double_t omega_;

            std::float_t r_, R_;
            std::float_t sina2_a1, sina0_a2, sina1_a0, cosa0_cosa1, cosa1_cosa2, cosa2_cosa0, sina0_sina1, sina1_sina2, sina2_sina0, m;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<omuni::Odometry>(0.03, 0.15));
    rclcpp::shutdown();
    return 0;
}