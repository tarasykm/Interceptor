#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <random>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class LeadDronePublisher : public rclcpp::Node {
    public:
    LeadDronePublisher() : Node("lead_publisher"), count_(0) {

        publisher_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lead/pose", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/lead/path", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/lead/marker", 10);

        kill_sub_ = this->create_subscription<std_msgs::msg::Int32>("/interceptor/kill", 1,
                std::bind(&LeadDronePublisher::kill, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&LeadDronePublisher::timer_callback, this));
    }
    private:

    void timer_callback()
    {
        update_random_rates();
        yaw_ += omega_yaw_ * dt_;
        pitch_ += omega_pitch_ * dt_;

        double vx = speed_ * std::cos(pitch_) * std::cos(yaw_);
        double vy = speed_ * std::cos(pitch_) * std::sin(yaw_);
        double vz = speed_ * std::sin(pitch_);

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

        // Arrow marker showing heading
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp    = stamp;
        marker.header.frame_id = "world";
        marker.ns   = "lead";
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
        marker.scale.x = 0.15;  // shaft diameter
        marker.scale.y = 0.3;   // head diameter
        marker.scale.z = 0.4;   // head length
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.a = 1.0f;

        marker_publisher_->publish(marker);

        if (++count_ % 10 == 0) {
            auto path_msg = nav_msgs::msg::Path();
            path_msg.header.stamp    = stamp;
            path_msg.header.frame_id = "world";
            path_msg.poses = poses_;
            path_publisher_->publish(path_msg);
        }
    }
    
    void update_random_rates() {
        rate_change_timer_ += dt_;
        if (rate_change_timer_ >= rate_change_interval_) {
            rate_change_timer_ = 0.0;
            target_omega_yaw_   = rate_dist_(rng_);
            target_omega_pitch_ = rate_dist_(rng_) * 0.5;  // less aggressive in pitch
        }

        const double alpha = 0.02;
        omega_yaw_   += alpha * (target_omega_yaw_   - omega_yaw_);
        omega_pitch_ += alpha * (target_omega_pitch_ - omega_pitch_);

        pitch_ = std::clamp(pitch_, -M_PI/3.0, M_PI/3.0);
    }

    void kill(const std_msgs::msg::Int32::SharedPtr msg) {
        (void)msg;
        timer_->cancel();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kill_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    size_t count_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;

    // State
    double yaw_{M_PI}, pitch_{0.1};

    // Params
    const double speed_ = 5.0;
    double omega_yaw_, omega_pitch_; // rad/s
    double dt_ = 0.02;
    double x_ = 10.0, y_ = 10.0, z_ = 10.0;

    double target_omega_yaw_{0.5};
    double target_omega_pitch_{0.3};
    double rate_change_timer_{0.0};
    const double rate_change_interval_{1.0};  // seconds between new random targets
    const double max_omega_{30*M_PI/180};             // rad/s max turn rate

    std::mt19937 rng_{std::random_device{}()};
    std::uniform_real_distribution<double> rate_dist_{-max_omega_, max_omega_};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadDronePublisher>());
    rclcpp::shutdown();
    return 0;
}
