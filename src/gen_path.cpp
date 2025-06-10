#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>

namespace path {
    class PathGenerator: public rclcpp::Node {
        public:
            explicit PathGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("path_generator", options) {
                this->mapResolution_ = 0.01;
                this->mapWidth_ = 182;
                this->mapHeight_ = 232;
                this->mapDir_ = "src/inrof2025_ros/map/";

                pubPath_ = create_publisher<nav_msgs::msg::Path>("route", 10);

                readMap();

                // TODO: subscriber
                timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PathGenerator::generator, this));
            }
        private:
        struct mapNode {
            int r, c;
            double width;
            double cost;
        };

        void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
            *u = (std::int32_t)(x / mapResolution_);
            *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
        };

        struct Cell {
            int u, v;
            double cost;

            bool operator>(const Cell& other) const {
                return cost > other.cost;
            }
        };

        void generator() {
            double sx = 0.25;
            double sy = 0.25;
            double gx = 1.30;
            double gy = 0.70;

            std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> q;
            std::vector<std::vector<double>> distances(
                this->mapHeight_, std::vector<double>(this->mapWidth_, std::numeric_limits<double>::infinity())
            );
            std::vector<std::vector<std::pair<int, int>>> previous(
                this->mapHeight_, std::vector<std::pair<int, int>>(this->mapWidth_, {-1, -1})
            );
            

            int su, sv, gu, gv;
            xy2uv(sx, sy, &su, &sv);
            xy2uv(gx, gy, &gu, &gv);

            RCLCPP_INFO(this->get_logger(), "start %d %d", su, sv);

            distances[sv][su] = 0;
            q.push({su, sv, 0});

            const int du[4] = {-1, 1, 0, 0};
            const int dv[4] = {0, 0, -1, 1};

            while(!q.empty()) { 
                Cell cur = q.top(); q.pop();
                if (cur.u == gu && cur.v == gv) break;

                for (int dir = 0; dir < 4; dir++ ) {
                    int nu = cur.u + du[dir];
                    int nv = cur.v + dv[dir];

                    if (nu >= 0 && nu < this->mapWidth_ && nv >= 0 && nv < this->mapHeight_) {
                        double cost = cur.cost + distField_.at<double>(nv, nu);
                        if (cost < distances[nv][nu]) {
                            distances[nv][nu] = cost;
                            previous[nv][nu] = {cur.u, cur.v};
                            q.push({nu, nv, cost});
                        }
                    }
                }
            }

            // 経路再構築
            std::vector<std::pair<int, int>> path;
            for (int u = gu, v = gv; u != -1 && v != -1; ) {
                path.push_back({u, v});
                std::tie(u, v) = previous[v][u];
            }

            std::reverse(path.begin(), path.end());

            nav_msgs::msg::Path pathMsg;
            pathMsg.header.frame_id = "map";
            pathMsg.header.stamp    = this->now();

            for (auto [gr, gc] : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = pathMsg.header;

                // OccupancyGrid のセル → ワールド座標（セル中心にオフセット）
                pose.pose.position.x = (gr + 0.5) * mapResolution_;
                pose.pose.position.y = (static_cast<double>(mapHeight_ - gc - 1) + 0.5) * mapResolution_;
                pose.pose.position.z = 0.0;

                // 進行方向の yaw を持たせても良いが，ここでは単位クォータニオン
                pose.pose.orientation.w = 1.0;


                RCLCPP_INFO(this->get_logger(), "%d %d %.4f %.4f", gr, gc, pose.pose.position.x, pose.pose.position.y);
                
                pathMsg.poses.push_back(std::move(pose));
            }

            pubPath_->publish(pathMsg);
        }

        void readMap() {
            try {
                YAML::Node lconf = YAML::LoadFile(this->mapDir_ + "map.yaml");
                mapResolution_ = lconf["resolution"].as<std::double_t>();
                mapOrigin_ = lconf["origin"].as<std::vector<std::double_t>>();

                std::string imgFile = mapDir_ + "map.pgm";
                mapImg_ = cv::imread(imgFile, 0);
                mapWidth_ = mapImg_.cols;
                mapHeight_ = mapImg_.rows;

                cv::Mat mapImg = mapImg_.clone();
                for (int v = 0; v < mapHeight_; v++ ) {
                    for (int u = 0; u < mapWidth_; u++ ) {
                        uchar val = mapImg.at<uchar>(v, u);
                        if (val == 0) {
                            mapImg.at<uchar>(v, u) = 0;
                        } else {
                            mapImg.at<uchar>(v, u) = 1;
                        }
                    }
                }

                cv::Mat distFieldF(mapHeight_, mapWidth_, CV_32FC1);
                cv::Mat distFieldD(mapHeight_, mapWidth_, CV_64FC1);
                cv::distanceTransform(mapImg, distFieldF, cv::DIST_L2, 5);
                
                // 原点    : 左上
                // first  : 縦軸
                // second : 横軸
                
                for (int v = 0; v < mapHeight_; v++ ) {
                    for (int u = 0; u < mapWidth_; u++ ) {
                        std::float_t d = distFieldF.at<std::float_t>(v, u);
                        distFieldD.at<std::double_t>(v, u) = (std::double_t)d * mapResolution_;
                    }
                }

                double max_val;
                cv::minMaxLoc(distFieldD, nullptr, &max_val, nullptr, nullptr);
                cv::Mat diff = distFieldD - max_val;   // 要素ごとに引き算
                cv::Mat absDiff = cv::abs(diff);      // 要素ごとの絶対値


                // // 1) 距離場 distFieldD（CV_64F）を 0–255 に正規化して 8bit 化
                // cv::Mat normDist;
                // cv::normalize(distFieldD, normDist, 0.0, 255.0, cv::NORM_MINMAX);
                // cv::Mat dist8U;
                // normDist.convertTo(dist8U, CV_8U);

                // // 2) グレースケール→BGR に変換
                // cv::Mat colorImg;
                // cv::cvtColor(dist8U, colorImg, cv::COLOR_GRAY2BGR);

                // // 3) 特定ピクセルをマーク (row=50, col=11 を赤に)
                // //    .at は (y,x) = (row,col) の順番なので注意
                // colorImg.at<cv::Vec3b>(11, 50) = cv::Vec3b(0, 0, 255);

                // （任意）円マークを描く場合
                // cv::circle(colorImg, cv::Point(11, 50), /*半径*/ 3, cv::Scalar(0,255,0), /*塗りつぶし*/ -1);

                // 4) 画像を保存
                // cv::imwrite("distField_highlight.png", colorImg);

                distField_ = absDiff.clone();
            } catch (const YAML::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s\n", e.what());
            }
        }

        std::array<std::pair<int,int>, 8> directions8_ {{
            {-1,  0},   // 上        (north)
            { 1,  0},   // 下        (south)
            { 0, -1},   // 左        (west)
            { 0,  1},   // 右        (east)
            {-1, -1},   // 左上      (north-west)
            {-1,  1},   // 右上      (north-east)
            { 1, -1},   // 左下      (south-west)
            { 1,  1}    // 右下      (south-east)
        }};
        std::string mapDir_;
        std::double_t mapResolution_;
        std::int32_t mapWidth_, mapHeight_;
        std::vector<std::double_t> mapOrigin_;
        cv::Mat mapImg_;
        cv::Mat distField_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path::PathGenerator>());
    rclcpp::shutdown();
    return 0;
}