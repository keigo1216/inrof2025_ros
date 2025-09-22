#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>

namespace raspi {
    class Vacume: public rclcpp::Node {
        public:
            explicit Vacume(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("vacume", options) {
                fd_vac_ = open_serial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2");

                rclcpp::QoS callbackVacQ(rclcpp::KeepLast(10));
                subVac_ = this->create_subscription<std_msgs::msg::Bool>(
                    "/vac", callbackVacQ, std::bind(&Vacume::sendVac, this, std::placeholders::_1)
                );

                srvVacume_ = this->create_service<inrof2025_ros_type::srv::Vacume> (
                    "/srv/vacume",
                    std::bind(&Vacume::vacumeCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
            }
        private:
            void vacumeCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Response> response
            ) {
                std_msgs::msg::Bool msg;
                msg.data = request->on;

                sendVac(msg);
            }
            void sendVac(std_msgs::msg::Bool msg) {
                uint8_t buf[3];
                memset(buf, 0x00, sizeof(buf));

                if (msg.data) {
                    buf[0] = 0x31;
                } else {
                    buf[0] = 0x30;
                }

                buf[1] = 0x0d;
                buf[2] = 0x0a;

                RCLCPP_INFO(this->get_logger(), "fjoerjfovejriofjer");

                ::write(fd_vac_, buf, 3);
            }

            int open_serial(const char *device_name)
            {
                // 1. オープン（ノンブロッキングで open → 後からブロッキングモードに切り替え）
                int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: could not open %s (%s)",
                                 device_name, std::strerror(errno));
                    return -1;
                }
                // ノンブロックをクリアしてブロッキングに
                fcntl(fd, F_SETFL, 0);
            
                // 2. 現在の端末設定を取得
                struct termios tty;
                if (tcgetattr(fd, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcgetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                // 3. ボーレート設定 (入力／出力ともに 115200)
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
            
                // 4. RAW モード設定
                cfmakeraw(&tty);
            
                // 5. フラグ設定
                //  - CS8: 8 ビットデータ
                //  - CLOCAL: ローカルライン (モデム制御なし)
                //  - CREAD: 受信有効
                tty.c_cflag &= ~PARENB;        // パリティなし
                tty.c_cflag &= ~CSTOPB;        // ストップビット 1
                tty.c_cflag &= ~CRTSCTS;       // ハードウェアフロー制御なし
                tty.c_cflag |= (CS8 | CLOCAL | CREAD);
            
                // 6. 非同期読み出し設定 (VMIN/VTIME)
                //    VMIN=0, VTIME=0 → read() が即リターン（バイトがなければ0を返す）
                tty.c_cc[VMIN]  = 0;
                tty.c_cc[VTIME] = 0;
            
                // 7. 設定を反映
                if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcsetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                // 8. 入出力バッファをクリア
                tcflush(fd, TCIOFLUSH);
            
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Serial opened: %s @ 115200, 8N1, raw", device_name);
                return fd;
            }

            int fd_vac_;
            std::vector<uint8_t> recev_buffer_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subVac_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
            rclcpp::Service<inrof2025_ros_type::srv::Vacume>::SharedPtr srvVacume_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raspi::Vacume>());
    rclcpp::shutdown();
    return 0;
}