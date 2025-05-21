#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

using namespace std::chrono_literals; 

namespace mcl {
    class Particle {
        public:
            const std::double_t& getX() const& { return pose_.x; }
            const std::double_t& getY() const& { return pose_.y; }
            const std::double_t& getTheta() const& { return pose_.theta; }
            const geometry_msgs::msg::Pose2D& getPose() const& { return pose_; }
            void setPose(std::double_t x, std::double_t y, std::double_t theta) {
                pose_.set__x(x);
                pose_.set__y(y);
                pose_.set__theta(theta);
            }
            void setW(std::double_t w) {
                w_ = w;
            }
        private:
            geometry_msgs::msg::Pose2D pose_;
            std::double_t w_;
    };

    enum class MeasurementModel { LikelihoodFieldModel };

    class MCL: public rclcpp::Node {
        public:
            explicit MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("mcl_node", options) {
                this->declare_parameter<std::int32_t>("particleNum", 20);
                
                particleNum_ = this->get_parameter("particleNum").as_int();
                particles_.resize(particleNum_);
            
                // init robot pos
                geometry_msgs::msg::Pose2D pose;
                // TODO: get parameter from user
                pose.set__x(0.2);
                pose.set__y(0.3);
                pose.set__theta(0);
                // initalize mclPose
                setMCLPose(pose);
                velOdom_.set__x(0.2);
                velOdom_.set__y(0.3);
                velOdom_.set__theta(0);
                
                // initialize particle
                geometry_msgs::msg::Pose2D initialNoise;
                rclcpp::QoS qosCloud(rclcpp::KeepLast(10));
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", qosCloud);
                initialNoise.set__x(0.07); // var of x
                initialNoise.set__y(0.07); // var of y
                initialNoise.set__theta(M_PI/180.0); // var of theta
                resetParticlesDistribution(initialNoise);
                printParticlesMakerOnRviz2();
                
                // set mesurementModel
                measurementModel_ = MeasurementModel::LikelihoodFieldModel;
                
                last_timestamp_ = this->get_clock()->now();
                // setup subscriper
                rclcpp::QoS qos(rclcpp::KeepLast(10));
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel_feedback", qos, std::bind(&MCL::cmdVelCallback, this, std::placeholders::_1)
                );
                rclcpp::QoS tmp_qos(rclcpp::KeepLast(10));
                // s_odom_ = create_subscription<nav_msgs::msg::Odometry>(
                //     "/odom", tmp_qos, std::bind(&MCL::odomCallback, this, std::placeholders::_1)
                // );

                // setup publisher
                iter_=0;
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MCL::loop, this));

                // TODO: delete
                // pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            } 

        private:
            void setMCLPose(geometry_msgs::msg::Pose2D pose) { mclPose_=pose; }
            std::double_t randNormal(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }
            
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                auto &q = msg->pose.pose.orientation;
                // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                double yaw = std::atan2(siny_cosp, cosy_cosp);
                
                // if (!cmdVel_) {
                //     return;
                // }
                yaw_ = yaw;
                odom_twist_ = msg->twist.twist.angular.z;
                // RCLCPP_INFO(this->get_logger(), "%.3f", yaw_);
                // RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.3f %.3f %.3f", yaw, msg->twist.twist.angular.z, particles_[0].getTheta());
            }

            void loop() {
                // 並進速度・回転速度を取得
                if (!cmdVel_) {
                    return;
                }
                
                // ロボットから見た座標系
                std::double_t vx_ = cmdVel_->linear.x;
                std::double_t vy_ = cmdVel_->linear.y;
                std::double_t omega_ = cmdVel_->angular.z;
                
                geometry_msgs::msg::Twist delta_;
                delta_.linear.x = vx_*0.1;
                delta_.linear.y = vy_*0.1;
                delta_.angular.z = omega_*0.1;

                updateParticles(delta_);
                printParticlesMakerOnRviz2();
                // iter_ += 0.1;
            }

            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_=msg;
            }

            void getRobotInitialPos() {
                // TODO: split into robot_manager
            }

            void resetParticlesDistribution(geometry_msgs::msg::Pose2D noise) {
                std::double_t wo = 1.0 / (std::double_t)particles_.size();
                for (std::size_t i=0; i<particles_.size(); i++ ) {
                    // TODO: フィールドの中に入っていない場合はリサンプリングする
                    std::double_t x = mclPose_.x + randNormal(noise.x);
                    std::double_t y = mclPose_.y + randNormal(noise.y);
                    std::double_t theta = mclPose_.theta + randNormal(noise.theta);
                    particles_[i].setPose(x, y, theta);
                    particles_[i].setW(wo);
                }
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y + delta.linear.y;
                std::double_t dy2 = delta.angular.z * delta.angular.z;
                for (size_t i = 0; i < this->particles_.size(); i++ ) {
                    std::double_t dx = delta.linear.x + randNormal(
                        odomNoise1_*dd2 + odomNoise2_*dy2
                    );
                    std::double_t dy = delta.linear.y + randNormal(
                        odomNoise1_*dd2 + odomNoise2_*dy2
                    );
                    std::double_t dtheta = delta.angular.z;
                    
                    geometry_msgs::msg::Pose2D pose_ = this->particles_[i].getPose();
                    std::double_t theta_ = pose_.theta;
                    std::double_t x_ = pose_.x + std::cos(theta_)*dx - std::sin(theta_)*dy;
                    std::double_t y_ = pose_.y + std::sin(theta_)*dx + std::cos(theta_)*dy;
                    theta_ += dtheta;
                    particles_[i].setPose(x_, y_, theta_);

                    // if(i==0) RCLCPP_INFO(this->get_logger(), "%.4f", theta_);
                }
                // RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.3f %.3f %.3f %.3f", yaw_, odom_twist_, cmdVel_->angular.z, particles_[0].getTheta());
            }

            // void updateParticles(std::double_t deltaDist, std::double_t deltaTheta) {
            //     std::double_t dd2 = deltaDist * deltaDist;
            //     std::double_t dtheta2 = deltaTheta * deltaTheta;
            //     std::normal_distribution<std::double_t> distd_(0, odomNoise1_*dd2 + odomNoise2_*dtheta2);
            //     std::normal_distribution<std::double_t> distt_(0, odomNoise3_*dd2 + odomNoise4_*dtheta2);
            //     for (size_t particle_idx_=0; particle_idx_ < particles_.size(); particle_idx_++) {
            //         std::double_t dd = deltaDist + distd_(gen_);
            //         std::double_t dtheta = deltaTheta + distt_(gen_);
            //         std::double_t theta = particles_[particle_idx_].getTheta();
            //         std::double_t x = particles_[particle_idx_].getX() + dd*std::cos(theta);
            //         std::double_t y = particles_[particle_idx_].getY() + dd*std::sin(theta);
            //         theta += dtheta;
            //         particles_[particle_idx_].setPose(x, y, theta);
            //     }
            // }

            // void caculateMeasurementModel(sensor_msgs::msg::LaserScan scan) {
            //     totalLikelihood_ = 0.0;
            //     std::double_t maxLikelihood = 0.0;
            //     for (std::size_t i = 0; i < particles_.size(); i++ ) {
            //         std::double_t likelihood = 0.0;
            //         // 尤度場モデル
            //         if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
            //             likelihood = caculateLikelihoodFieldModel(particles_[i].getPose(), scan);
            //         }

            //         if (i == 0) {
            //             maxLikelihood = likelihood;
            //             maxLikelihoodParticleIdx_ = 0;
            //         } else if (maxLikelihood < likelihood) {
            //             maxLikelihood = likelihood;
            //             maxLikelihoodParticleIdx_ = i;
            //         }

            //         measurementLikelihoods_[i] = likelihood;
            //         totalLikelihood_ += likelihood;
            //     }
            //     averageLikelihood_ = totalLikelihood_ / (double)(particles_.size());
                
            //     // normalize likelihood and caculate valid sample num
            //     std::double_t sum = 0.0;
            //     for(std::size_t i = 0; i < particles_.size(); i++ ) {
            //         std::double_t w = measurementLikelihoods_[i] / totalLikelihood_;
            //         particles_[i].setW(w);
            //         sum += w*w;
            //     }
            //     effectiveSampleSize_ = 1.0 / sum;
            // }

            // std::double_t caculateLikelihoodFieldModel (geometry_msgs::msg::Pose2D pose, sensor_msgs::msg::LaserScan scan) {
            //     std::double_t var = lfmSigma_*lfmSigma_;
            //     std::double_t normConst = 1.0 / (sqrt(2.0*M_PI*var));
            //     std::double_t pMax = 1.0 / mapResolution_; // <- mapResolution_で割る必要なくない？
            //     std::double_t pRand = 1.0 / (scan.range_max / mapResolution_);
            //     std::double_t w = 0.0;
            //     for (std::size_t i = 0; i < scan.ranges.size(); i+=scanStep_) {
            //         std::double_t r = scan.ranges[i];
            //         // when r is max or min value
            //         if (r < scan.angle_min || scan.angle_max < r) {
            //             w += log(zMax_*pMax + zRand_*pRand);
            //             continue;
            //         }

            //         std::double_t a = scan.angle_min + ((std::double_t)(i))*scan.angle_increment + pose.theta;
            //         // TODO: ここの計算はだいぶ簡略化できそうなので，時間があればやる
            //         std::double_t x = pose.x + (cos(theta_lidar_)-sin(theta_lidar_))*x_lidar_ + r*(cos(theta_lidar_+a));
            //         std::double_t y = pose.y + (sin(theta_lidar_)+cos(theta_lidar_))*y_lidar_ + r*(sin(theta_lidar_+a));
            //         int u, v;
            //         xy2uv(x, y, &u, &v);
                    
            //         // シミュレーション動かしながら逐次実装していくよ
            //     }
            // }

            void printParticlesMakerOnRviz2() {
                sensor_msgs::msg::PointCloud2 cloud_;
                cloud_.header.stamp = this->get_clock()->now();
                cloud_.header.frame_id = "map";
                cloud_.height = 1;
                cloud_.width = particleNum_;
                cloud_.is_dense = false;
                cloud_.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(particleNum_);

                sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(cloud_, "x");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(cloud_, "y");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(cloud_, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_r(cloud_, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_g(cloud_, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_b(cloud_, "b");

                for (const Particle &p: particles_) {
                    *iter_x = p.getX();
                    *iter_y = p.getY();
                    *iter_z = 0;

                    *iter_r = 0;
                    *iter_g = 0;
                    *iter_b = 255;

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                particleMarker_->publish(cloud_);
            }

            // void vel2OdomCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
            //     rclcpp::Time now = this->get_clock()->now();

            //     double dt = (now - last_timestamp_).seconds();
            //     if (dt <= 0.0) {
            //         return;
            //     }
            //     last_timestamp_ = now;
                
            //     std::double_t theta = velOdom_.theta;
            //     std::double_t dx = (msg->linear.x) * dt;
            //     std::double_t dy = (msg->linear.y) * dt;
            //     velOdom_.x += std::cos(theta)*dx - std::sin(theta)*dy;
            //     velOdom_.y += std::sin(theta)*dx + std::cos(theta)*dy;
            //     velOdom_.theta += msg->angular.z*dt;

            //     RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.5f %.5f %.5f %.5f %.5f", yaw_, odom_twist_, msg->angular.z, velOdom_.theta, now.seconds());
            // }
            
            // TODO
            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {}

            // parameter for map
            std::double_t mapResolution_;
            std::int32_t mapWidth_, mapHeight_;

            // particle settings
            int particleNum_;
            // 絶対座標
            std::vector<Particle> particles_;
            geometry_msgs::msg::Pose2D mclPose_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;

            // likelihood
            int maxLikelihoodParticleIdx_;
            std::double_t totalLikelihood_;
            std::double_t averageLikelihood_;
            std::vector<std::double_t> measurementLikelihoods_;

            std::double_t effectiveSampleSize_;

            // model for mesurement
            mcl::MeasurementModel measurementModel_;
            std::int32_t scanStep_;

            // parameter for measurement model
            std::double_t zHit_, zShort_, zMax_, zRand_;
            std::double_t lfmSigma_;

            // parameter for lidar relitive position
            // lidarの絶対座標はtfが変換してpublishしてくれている可能性があるので，もしかしたらいらないかも
            std::double_t x_lidar_, y_lidar_, theta_lidar_;

            // noize when updateing particle
            std::double_t odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            std::mt19937 gen_;

            std::string base_footprint_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;

            // cmd_velのみから現在のodometryを計算する
            // last_time_に前回差分を取得したときの時刻
            // last_odom_に前回のodometryを保存
            rclcpp::Time last_timestamp_;
            geometry_msgs::msg::Pose2D velOdom_;
            geometry_msgs::msg::Pose2D last_odom_;

            // TODO: delete
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
            std::float_t iter_;
            std::double_t yaw_;
            std::double_t odom_twist_;

            // TODO: delete
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            void timer_callback() {
                RCLCPP_INFO(this->get_logger(), "In timer loop");
            }
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL>());
    rclcpp::shutdown();
    return 0;
}