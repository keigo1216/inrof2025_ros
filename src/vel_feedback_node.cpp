#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace cmd_vel {
    class CmdVelFeedBack: public rclcpp::Node {
        public:
            explicit CmdVelFeedBack(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("cmd_vel_feedback", options) {
                rclcpp::QoS twistQos(rclcpp::KeepLast(10));
                rclcpp::QoS callbackQos(rclcpp::KeepLast(10));
                
                lastOdom_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

                pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", twistQos);
                subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", callbackQos, std::bind(&CmdVelFeedBack::callback, this, std::placeholders::_1)
                );
            }
        
        private:
            void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                if (lastOdom_.header.stamp.sec==0 && lastOdom_.header.stamp.nanosec==0) {
                    lastOdom_ = *msg;
                    return;
                }

                rclcpp::Time now = msg->header.stamp;
                rclcpp::Duration dt = now - lastOdom_.header.stamp;
                double dt_sec = dt.seconds();

                if (dt_sec <= 0.0) {
                    return;
                }

                // フィールド座標
                double dxField = msg->pose.pose.position.x - lastOdom_.pose.pose.position.x;
                double dyField = msg->pose.pose.position.y - lastOdom_.pose.pose.position.y;
                

                geometry_msgs::msg::Quaternion &q0 = lastOdom_.pose.pose.orientation;
                geometry_msgs::msg::Quaternion &q1 = msg->pose.pose.orientation;

                double siny_cosp_0 = 2.0 * (q0.w * q0.z + q0.x * q0.y);
                double cosy_cosp_0 = 1.0 - 2.0 * (q0.y * q0.y + q0.z * q0.z);
                double yaw0 = std::atan2(siny_cosp_0, cosy_cosp_0);

                double siny_cosp_1 = 2.0 * (q1.w * q1.z + q1.x * q1.y);
                double cosy_cosp_1 = 1.0 - 2.0 * (q1.y * q1.y + q1.z * q1.z);
                double yaw1 = std::atan2(siny_cosp_1, cosy_cosp_1);
                
                double dyaw = yaw1 - yaw0;
                while (dyaw > M_PI) dyaw -= 2*M_PI;
                while (dyaw < -M_PI) dyaw += 2*M_PI;

                // ロボット座標系へ変換
                double dxRobot = std::cos(yaw0)*dxField + std::sin(yaw0)*dyField;
                double dyRobot = -std::sin(yaw0)*dxField + std::cos(yaw0)*dyField;

                geometry_msgs::msg::Twist twist;
                twist.linear.x = dxRobot / dt_sec;
                twist.linear.y = dyRobot / dt_sec;
                twist.angular.z = dyaw / dt_sec;
                pub_->publish(twist);

                lastOdom_ = *msg;
            }

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
            nav_msgs::msg::Odometry lastOdom_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cmd_vel::CmdVelFeedBack>());
    rclcpp::shutdown();
    return 0;
}