#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

namespace omuni {
    class Odometry: public rclcpp::Node {
        public:
            explicit Odometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                : Node("omuni_odometry_frame_publisher", options){
                this->declare_parameter<bool>("is_mapping", false);

                this->declare_parameter<double>("r", 0.03);
                this->declare_parameter<double>("R", 0.15);
                this->declare_parameter<double>("x", 0.0);
                this->declare_parameter<double>("y", 0.0);
                this->declare_parameter<double>("theata", 0.0);
                
                is_mapping_ = this->get_parameter("is_mapping").as_bool();
                r_ = this->get_parameter("r").as_double();
                R_ = this->get_parameter("R").as_double();
                x_ = this->get_parameter("x").as_double();
                y_ = this->get_parameter("y").as_double();
                theata_ = this->get_parameter("theata").as_double();

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

                if (is_mapping_) {
                    sub_for_mapping_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
                        "/gazebo/model_states",
                        10,
                        std::bind(&Odometry::callback_for_mapping, this, std::placeholders::_1)
                    );
                } else {
                    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        "/joint_states",
                        10,
                        std::bind(&Odometry::callback, this, std::placeholders::_1)
                    );
                }
            }
        private:
            void callback_for_mapping(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
                std::string target_model = "trolley";
                auto it = std::find(msg->name.begin(), msg->name.end(), target_model);
                if (it != msg->name.end()) {
                    size_t index = std::distance(msg->name.begin(), it);
                    rclcpp::Time cur_time = this->now();

                    // create tf message (odom -> base_footprint)
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = cur_time;
                    transformStamped.header.frame_id = "odom";
                    transformStamped.child_frame_id = base_footprint_;
                    transformStamped.transform.translation.x = msg->pose[index].position.x;
                    transformStamped.transform.translation.y = msg->pose[index].position.y;
                    transformStamped.transform.translation.z = 0.0;

                    transformStamped.transform.rotation.x = msg->pose[index].orientation.x;
                    transformStamped.transform.rotation.y = msg->pose[index].orientation.y;
                    transformStamped.transform.rotation.z = msg->pose[index].orientation.z;
                    transformStamped.transform.rotation.w = msg->pose[index].orientation.w;

                    // broadcast tf
                    tf_broadcaster_->sendTransform(transformStamped);
                }
            }

            void callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
                // reference: https://t-semi.esa.io/posts/191
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

                // convet to v_1, v_2, v_3 to v_x, v_y, omega
                v_x_ = ((cosa1_cosa2)*v[0] + (cosa2_cosa0)*v[1] + (cosa0_cosa1)*v[2])/m;
                v_y_ = ((sina1_sina2)*v[0] + (sina2_sina0)*v[1] + (sina0_sina1)*v[2])/m;
                omega_ = ((sina2_a1)*v[0] + (sina0_a2)*v[1] + (sina1_a0)*v[2])/(R_*m);

                rclcpp::Time cur_time = msg->header.stamp;
                float dt = 0.0;
                if (first_time_) {
                    last_time_ = cur_time;
                    first_time_ = false;
                }
                dt = (cur_time - last_time_).seconds();
                last_time_ = cur_time;

                float delta_x = (v_x_*std::cos(theata_) - v_y_*std::sin(theata_))*dt;
                float delta_y = (v_x_*std::sin(theata_) + v_y_*std::cos(theata_))*dt;
                float delta_theata = omega_*dt;

                // update
                x_ += delta_x;
                y_ += delta_y;
                theata_ += delta_theata;
                RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, theta=%.2f", x_, y_, theata_);

                // create tf message (odom -> base_footprint)
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = cur_time;
                transformStamped.header.frame_id = "odom";
                transformStamped.child_frame_id = base_footprint_;
                transformStamped.transform.translation.x = x_;
                transformStamped.transform.translation.y = y_;
                transformStamped.transform.translation.z = 0.0;

                // caculate quaternion
                tf2::Quaternion q;
                q.setRPY(0, 0, theata_);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();

                // broadcast tf
                tf_broadcaster_->sendTransform(transformStamped);
            }
            
            bool is_mapping_;

            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
            rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_for_mapping_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::string base_footprint_;

            std::double_t v_x_;
            std::double_t v_y_;
            std::double_t omega_;

            // For caculating odometry
            // TODO: initialize robot position
            std::float_t x_;
            std::float_t y_;
            std::float_t theata_;
            rclcpp::Time last_time_;
            bool first_time_{true};

            std::float_t r_, R_;
            std::float_t sina2_a1, sina0_a2, sina1_a0, cosa0_cosa1, cosa1_cosa2, cosa2_cosa0, sina0_sina1, sina1_sina2, sina2_sina0, m;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<omuni::Odometry>());
    rclcpp::shutdown();
    return 0;
}