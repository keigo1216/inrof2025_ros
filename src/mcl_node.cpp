#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

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

                // initalize mclPose
                geometry_msgs::msg::Pose2D pose;
                // TODO: get parameter from user
                pose.set__x(0.0);
                pose.set__y(0.0);
                pose.set__theta(0.0);
                setMCLPose(pose);
                
                geometry_msgs::msg::Pose2D initialNoise;
                initialNoise.set__x(0.07); // var of x
                initialNoise.set__y(0.07); // var of y
                initialNoise.set__theta(M_PI/180.0); // var of theta
                resetParticlesDistribution(initialNoise);

                measurementModel_ = MeasurementModel::LikelihoodFieldModel;
                
                // setup publisher
                particleMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("particles_marker", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MCL::timer_callback, this));
            } 

        private:
            void setMCLPose(geometry_msgs::msg::Pose2D pose) { mclPose_=pose; }
            std::double_t randNormal(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }

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

            void updateParticles(std::double_t deltaDist, std::double_t deltaTheta) {
                std::double_t dd2 = deltaDist * deltaDist;
                std::double_t dtheta2 = deltaTheta * deltaTheta;
                std::normal_distribution<std::double_t> distd_(0, odomNoise1_*dd2 + odomNoise2_*dtheta2);
                std::normal_distribution<std::double_t> distt_(0, odomNoise3_*dd2 + odomNoise4_*dtheta2);
                for (size_t particle_idx_=0; particle_idx_ < particles_.size(); particle_idx_++) {
                    std::double_t dd = deltaDist + distd_(gen_);
                    std::double_t dtheta = deltaTheta + distt_(gen_);
                    std::double_t theta = particles_[particle_idx_].getTheta();
                    std::double_t x = particles_[particle_idx_].getX() + dd*std::cos(theta);
                    std::double_t y = particles_[particle_idx_].getY() + dtheta*std::sin(theta);
                    theta += dtheta;
                    particles_[particle_idx_].setPose(x, y, theta);
                }
            }

            void caculateMeasurementModel(sensor_msgs::msg::LaserScan scan) {
                totalLikelihood_ = 0.0;
                std::double_t maxLikelihood = 0.0;
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t likelihood = 0.0;
                    // 尤度場モデル
                    if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
                        likelihood = caculateLikelihoodFieldModel(particles_[i].getPose(), scan);
                    }

                    if (i == 0) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = 0;
                    } else if (maxLikelihood < likelihood) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = i;
                    }

                    measurementLikelihoods_[i] = likelihood;
                    totalLikelihood_ += likelihood;
                }
                averageLikelihood_ = totalLikelihood_ / (double)(particles_.size());
                
                // normalize likelihood and caculate valid sample num
                std::double_t sum = 0.0;
                for(std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t w = measurementLikelihoods_[i] / totalLikelihood_;
                    particles_[i].setW(w);
                    sum += w*w;
                }
                effectiveSampleSize_ = 1.0 / sum;
            }

            std::double_t caculateLikelihoodFieldModel (geometry_msgs::msg::Pose2D pose, sensor_msgs::msg::LaserScan scan) {
                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 / (sqrt(2.0*M_PI*var));
                std::double_t pMax = 1.0 / mapResolution_; // <- mapResolution_で割る必要なくない？
                std::double_t pRand = 1.0 / (scan.range_max / mapResolution_);
                std::double_t w = 0.0;
                for (std::size_t i = 0; i < scan.ranges.size(); i+=scanStep_) {
                    std::double_t r = scan.ranges[i];
                    // when r is max or min value
                    if (r < scan.angle_min || scan.angle_max < r) {
                        w += log(zMax_*pMax + zRand_*pRand);
                        continue;
                    }

                    std::double_t a = scan.angle_min + ((std::double_t)(i))*scan.angle_increment + pose.theta;
                    // TODO: ここの計算はだいぶ簡略化できそうなので，時間があればやる
                    std::double_t x = pose.x + (cos(theta_lidar_)-sin(theta_lidar_))*x_lidar_ + r*(cos(theta_lidar_+a));
                    std::double_t y = pose.y + (sin(theta_lidar_)+cos(theta_lidar_))*y_lidar_ + r*(sin(theta_lidar_+a));
                    int u, v;
                    xy2uv(x, y, &u, &v);
                    
                    // シミュレーション動かしながら逐次実装していくよ
                }
            }

            void printParticlesMakerOnRviz2() {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "particles";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0;

                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;

                for (std::size_t i=0; i<particles_.size(); i++ ) {
                    geometry_msgs::msg::Point pt;
                    pt.x = particles_[i].getX();
                    pt.y = particles_[i].getY();
                    marker.points.push_back(pt);
                }

                particleMarker_->publish(marker);
            }
            
            // TODO
            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {}

            // parameter for map
            std::double_t mapResolution_;
            std::int32_t mapWidth_, mapHeight_;

            // particle settings
            int particleNum_;
            std::vector<Particle> particles_;
            geometry_msgs::msg::Pose2D mclPose_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr particleMarker_;

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

            // TODO: delete
            rclcpp::TimerBase::SharedPtr timer_;
            void timer_callback() {
                RCLCPP_INFO(this->get_logger(), "In timer loop");
                printParticlesMakerOnRviz2();
            }
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL>());
    rclcpp::shutdown();
    return 0;
}