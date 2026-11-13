// メカナムローバの車輪オドメトリ (/rover_odo, geometry_msgs/Twist) を積分して
// /robot_pose2d (geometry_msgs/Pose2D) と TF odom -> base_link を出版する。
//
// 参照元: ~/ros2_ws/src/mecanumrover3_ros2/mecanumrover3_bringup/src/pub_odom.cpp
// 元実装は timer_callback が未初期化の q, current_time を使う可能性があったため、
// ここでは「初回メッセージ受信前は出版しない」ガードで回避している。

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RoverOdomToPose2D : public rclcpp::Node
{
public:
  RoverOdomToPose2D() : Node("rover_odom_to_pose2d")
  {
    pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/robot_pose2d", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    kvx_ = this->declare_parameter<double>("kvx", 1.0);
    kvy_ = this->declare_parameter<double>("kvy", 1.0);
    kth_ = this->declare_parameter<double>("kth", 1.0);
    const double rate_hz = this->declare_parameter<double>("rate", 100.0);
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&RoverOdomToPose2D::timer_callback, this));

    sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/rover_odo", rclcpp::SensorDataQoS(),
      std::bind(&RoverOdomToPose2D::rover_callback, this, _1));

    last_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(),
      "rover_odom_to_pose2d 起動: /rover_odo を積分して /robot_pose2d + TF(odom->base_link) 出版 (rate=%.1f Hz)", rate_hz);
  }

private:
  void rover_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg)
  {
    if (!first_msg_received_) {
      first_msg_received_ = true;
      last_time_ = this->get_clock()->now();
      return;
    }
    const rclcpp::Time now = this->get_clock()->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) return;

    const double vx = kvx_ * msg->twist.linear.x;
    const double vy = kvy_ * msg->twist.linear.y;
    const double vth = kth_ * msg->twist.angular.z;

    // メカナム: 機体座標での速度を世界系へ
    const double dx = (vx * std::cos(th_) - vy * std::sin(th_)) * dt;
    const double dy = (vx * std::sin(th_) + vy * std::cos(th_)) * dt;
    const double dth = vth * dt;
    x_ += dx;
    y_ += dy;
    th_ += dth;
  }

  void timer_callback()
  {
    if (!first_msg_received_) return;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = x_;
    pose2d.y = y_;
    pose2d.theta = th_;
    pose2d_pub_->publish(pose2d);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  double x_ = 0.0;
  double y_ = 0.0;
  double th_ = 0.0;
  double kvx_ = 1.0;
  double kvy_ = 1.0;
  double kth_ = 1.0;
  bool first_msg_received_ = false;
  rclcpp::Time last_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverOdomToPose2D>());
  rclcpp::shutdown();
  return 0;
}
