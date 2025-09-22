#include <rclcpp/rclcpp.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>

namespace dummy {
    class Vacume: public rclcpp::Node {
        public:
            explicit Vacume(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("vacume", options) {
                srvVacume_ = this->create_service<inrof2025_ros_type::srv::Vacume> (
                    "/srv/vacume",
                    std::bind(&Vacume::vacumeCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
            }
        private:
            void vacumeCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Response> response
            ) {}

            rclcpp::Service<inrof2025_ros_type::srv::Vacume>::SharedPtr srvVacume_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dummy::Vacume>());
    rclcpp::shutdown();
    return 0;
}