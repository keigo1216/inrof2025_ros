#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ExampleOmuniDrive: public rclcpp::Node {
    public:
        explicit ExampleOmuniDrive(): Node("ExampleOmuniDrive") {
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ExampleOmuniDrive::timer_callback, this));
        }
    
    private:
        void timer_callback() {
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = 10;
            pub_->publish(msg);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ExampleOmuniDrive>());
    rclcpp::shutdown();
    return 0;
}