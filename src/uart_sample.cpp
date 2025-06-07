#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using std::placeholders::_1;
int fd1;

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber()
        : Node("my_subscriber")
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MySubscriber::topic_callback, this)
        );
        receive_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MySubscriber::receive_callback, this));
    }

private:
    std::vector<uint8_t> recv_buffer_;

    // 受信：バッファに積んで 12 バイト揃ったらフレームとして取り出す
    void receive_callback()
    {
        uint8_t tmp[12];
        ssize_t n = read(fd1, tmp, sizeof(tmp));
        if (n > 0) {
        // 1) バッファに追加
        recv_buffer_.insert(recv_buffer_.end(), tmp, tmp + n);

        // 2) 12 バイト以上あれば何フレームでも処理
        while (recv_buffer_.size() >= 12) {
            uint8_t frame[12];
            std::copy(recv_buffer_.begin(),
                    recv_buffer_.begin() + 12,
                    frame);

            // バッファ先頭から削除
            recv_buffer_.erase(recv_buffer_.begin(),
                            recv_buffer_.begin() + 12);

            // フレーム内容を文字列化（バイナリなら別途バイナリ処理）
            std::string s(reinterpret_cast<char*>(frame), 12);
            RCLCPP_INFO(this->get_logger(),
                        "Serial frame recv: \"%s\"", s.c_str());
        }
        }
        // n == 0: データなし、n < 0: EAGAINなど。特に処理不要。
    }

    void topic_callback()
    {
        uint8_t buf[12];
        memset(buf, 0x00, sizeof(buf));

        buf[0] = 'H';

        // 常に 12 バイト送る
        const size_t bytes_to_write = sizeof(buf);  // 12

        // 書き込み
        ssize_t rec = write(fd1, buf, bytes_to_write);
        if (rec == (ssize_t)bytes_to_write) {
            RCLCPP_INFO(this->get_logger(),
                        "Serial send: %zu bytes", bytes_to_write);
        }
        else if (rec >= 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Serial send partial: %zd/%zu bytes", rec, bytes_to_write);
        }
        else {
            RCLCPP_ERROR(this->get_logger(),
                        "Serial Fail: could not write (%s)", strerror(errno));
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
};

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Serialport");

    char device_name[] = "/dev/ttyACM0";
    fd1 = open_serial(device_name);

    if (fd1 < 0) {
        printf("Error");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}