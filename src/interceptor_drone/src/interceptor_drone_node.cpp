#include <chrono>
#include <functional>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <math.h>

using namespace std::chrono_literals;

class InterceptorPublisher : public rclcpp::Node {
    public:
        ~InterceptorPublisher() { if (csv_.is_open()) csv_.close(); }

        InterceptorPublisher() : Node("interceptor_publisher") {

            publisher_        = this->create_publisher<geometry_msgs::msg::PoseStamped>("/interceptor/pose", 10);
            publisher_kill_   = this->create_publisher<std_msgs::msg::Int32>("/interceptor/kill", 10);
            path_publisher_   = this->create_publisher<nav_msgs::msg::Path>("/interceptor/path", 10);
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/interceptor/marker", 10);

            lead_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/lead/pose", 1,
                std::bind(&InterceptorPublisher::poseCallback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(20ms, std::bind(&InterceptorPublisher::update, this));

            csv_.open("interceptor_log.csv");
            csv_ << "t,x,y,z,vx,vy,vz,yaw,pitch,omega_yaw,omega_pitch,"
                    "LOS_az,LOS_el,LOS_rate_az,LOS_rate_el,distance,closing_vel\n";
            RCLCPP_INFO(this->get_logger(), "Logging to interceptor_log.csv");
        }

    private:

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            if (!lead_received_) {
                lead_received_ = true;
            }
            prev_pose_    = current_pose_;
            current_pose_ = *msg;
        }

        void update() {
            if (lead_received_ && distance() < min_distance_) {
                detonate();
                return;
            }
            estimate_velocity();
            LOSfromPose();
            assign_angular_rates();

            yaw_   += omega_yaw_   * dt_;
            pitch_ += omega_pitch_ * dt_;

            vx = speed_ * std::cos(pitch_) * std::cos(yaw_);
            vy = speed_ * std::cos(pitch_) * std::sin(yaw_);
            vz = speed_ * std::sin(pitch_);

            x_ += vx * dt_;
            y_ += vy * dt_;
            z_ += vz * dt_;

            auto stamp = this->get_clock()->now();

            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp    = stamp;
            pose_msg.header.frame_id = "world";
            pose_msg.pose.position.x = x_;
            pose_msg.pose.position.y = y_;
            pose_msg.pose.position.z = z_;
            pose_msg.pose.orientation.z = std::sin(yaw_ / 2.0);
            pose_msg.pose.orientation.w = std::cos(yaw_ / 2.0);

            publisher_->publish(pose_msg);
            poses_.push_back(pose_msg);

            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp    = stamp;
            marker.header.frame_id = "world";
            marker.ns   = "interceptor";
            marker.id   = 0;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point tail, tip;
            tail.x = x_; tail.y = y_; tail.z = z_;
            const double arrow_len = 2.0;
            tip.x = x_ + std::cos(pitch_) * std::cos(yaw_) * arrow_len;
            tip.y = y_ + std::cos(pitch_) * std::sin(yaw_) * arrow_len;
            tip.z = z_ + std::sin(pitch_) * arrow_len;
            marker.points = {tail, tip};
            marker.scale.x = 0.15;
            marker.scale.y = 0.3;
            marker.scale.z = 0.4;
            marker.color.b = 1.0f;
            marker.color.g = 0.4f;
            marker.color.a = 1.0f;

            marker_publisher_->publish(marker);

            if (++count_ % 10 == 0) {
                auto path_msg = nav_msgs::msg::Path();
                path_msg.header.stamp    = stamp;
                path_msg.header.frame_id = "world";
                path_msg.poses = poses_;
                path_publisher_->publish(path_msg);
            }

            csv_ << std::fixed << std::setprecision(4)
                 << stamp.seconds() << ","
                 << x_ << "," << y_ << "," << z_ << ","
                 << vx << "," << vy << "," << vz << ","
                 << yaw_ << "," << pitch_ << ","
                 << omega_yaw_ << "," << omega_pitch_ << ","
                 << LOS_az_ << "," << LOS_el_ << ","
                 << LOS_rate_az_ << "," << LOS_rate_el_ << ","
                 << distance() << "," << closing_vel_ << "\n";
        }

        void estimate_velocity()
        {
            if (!lead_received_) return;

            double dx = current_pose_.pose.position.x - prev_pose_.pose.position.x;
            double dy = current_pose_.pose.position.y - prev_pose_.pose.position.y;
            double dz = current_pose_.pose.position.z - prev_pose_.pose.position.z;

            target_vel_[0] = dx/dt_;
            target_vel_[1] = dy/dt_;
            target_vel_[2] = dz/dt_;
        }

        void LOSfromPose()
        {
            if (!lead_received_) return;

            double dx = current_pose_.pose.position.x - x_;
            double dy = current_pose_.pose.position.y - y_;
            double dz = current_pose_.pose.position.z - z_;

            double range_xy = std::sqrt(dx*dx + dy*dy);
            double az = std::atan2(dy, dx);
            double el = std::atan2(dz, range_xy);

            LOS_rate_az_ = (az - prev_LOS_az_) / dt_;
            LOS_rate_el_ = (el - prev_LOS_el_) / dt_;

            prev_LOS_az_ = az;
            prev_LOS_el_ = el;

            LOS_az_ = az;
            LOS_el_ = el;
        }

        double distance() {
            double dx = current_pose_.pose.position.x - x_;
            double dy = current_pose_.pose.position.y - y_;
            double dz = current_pose_.pose.position.z - z_;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        void assign_angular_rates() {
            if (!lead_received_) {
                    RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for lead drone...");
                    return;
                }
            RCLCPP_INFO_ONCE(this->get_logger(), "Lead received, ProNav active.");

            double dx = current_pose_.pose.position.x - x_;
            double dy = current_pose_.pose.position.y - y_;
            double dz = current_pose_.pose.position.z - z_;
            double range = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (range < 1e-6) return;

            double rdot = (dx*(target_vel_[0] - vx) +
                           dy*(target_vel_[1] - vy) +
                           dz*(target_vel_[2] - vz)) / range;
            closing_vel_ = -rdot;

            if (closing_vel_ > 0.0) {
                const double N = 30.0;
                omega_yaw_   = N * closing_vel_ * LOS_rate_az_;
                omega_pitch_ = N * closing_vel_ * LOS_rate_el_;
            } else {
                // Missed — pure pursuit: steer heading directly toward current LOS
                double yaw_err = LOS_az_ - yaw_;
                if (yaw_err >  M_PI) yaw_err -= 2 * M_PI;
                if (yaw_err < -M_PI) yaw_err += 2 * M_PI;
                const double K = 5.0;
                omega_yaw_   = K * yaw_err;
                omega_pitch_ = K * (LOS_el_ - pitch_);
            }

            omega_yaw_   = std::clamp(omega_yaw_,   -max_LOS_az_rate, max_LOS_az_rate);
            omega_pitch_ = std::clamp(omega_pitch_, -max_LOS_el_rate, max_LOS_el_rate);
        }

        void detonate() {
            auto msg = std_msgs::msg::Int32();
            msg.data = 1;
            publisher_kill_->publish(msg);
            timer_->cancel();
            csv_.close();
        }

    double x_{0.0}, y_{0.0}, z_{0.0};
    double vx{0.0}, vy{0.0}, vz{0.0};
    double yaw_{0}, pitch_{M_PI/2};
    const double speed_{15.0};
    double omega_yaw_{0.0};
    double omega_pitch_{0.0};
    double closing_vel_{0.0};
    double dt_ = 0.02;

    double max_LOS_az_rate{55*M_PI/180};
    double max_LOS_el_rate{55*M_PI/180};

    double LOS_az_{0.0};
    double LOS_el_{0.0};
    double LOS_rate_az_{0.0};
    double LOS_rate_el_{0.0};
    double prev_LOS_az_{0.0};
    double prev_LOS_el_{0.0};

    bool lead_received_{false};
    double min_distance_{0.3};

    double target_vel_[3]{};

    geometry_msgs::msg::PoseStamped prev_pose_;
    geometry_msgs::msg::PoseStamped current_pose_;

    size_t count_{0};
    std::vector<geometry_msgs::msg::PoseStamped> poses_;

    std::ofstream csv_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lead_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_kill_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InterceptorPublisher>());
    rclcpp::shutdown();
    return 0;
}
