// 山接近の経路計画ノードです。
//
// 責務: 外部が決めた目標姿勢(/pickup_pose)と、LiDARの裾点群(観測だけ)から、
//       車体矩形が観測した山に当たらない経路を計算し、nav_msgs/Pathとして出力します。
// 速度指令は出しません。経路追従と到着判定は pickup_approach_controller の責務です。
//
// 実機向けの方針:
//  - 計画は「新しい目標が来たとき」に一度だけ実行し、制御周期では再計算しません。
//  - 車体が静止し、融合自己位置が新鮮で、裾点群が目標より古くないことを確認してから計画します。
//  - 観測が足りない場合は推測せず、時間内に揃わなければ失敗(ADDITIONAL_SCAN_REQUIRED)を通知します。
//  - 出力経路のheader.stampには目標のstampをそのまま複写し、追従側が目標との対応を確認できるようにします。

#include "pickup_path_planner/planner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pickup_path_planner
{
namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool validQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(n) && n > 1.0e-8;
}

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

// 四元数qでベクトルvを回転します。
void rotateByQuaternion(
  const geometry_msgs::msg::Quaternion & q, const double vx, const double vy, const double vz,
  double * ox, double * oy, double * oz)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const double x = q.x / n;
  const double y = q.y / n;
  const double z = q.z / n;
  const double w = q.w / n;
  const double tx = 2.0 * (y * vz - z * vy);
  const double ty = 2.0 * (z * vx - x * vz);
  const double tz = 2.0 * (x * vy - y * vx);
  *ox = vx + w * tx + (y * tz - z * ty);
  *oy = vy + w * ty + (z * tx - x * tz);
  *oz = vz + w * tz + (x * ty - y * tx);
}

}  // namespace

class PickupPathPlannerNode : public rclcpp::Node
{
public:
  PickupPathPlannerNode()
  : Node("pickup_path_planner"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/odometry/filtered");
    goal_topic_ = declare_parameter<std::string>("topics.goal", "/pickup_pose");
    skirt_topic_ = declare_parameter<std::string>(
      "topics.skirt_points", "/scrap_pile_skirt_node/skirt_points");
    path_topic_ = declare_parameter<std::string>("topics.path", "/pickup_approach/path");
    status_topic_ = declare_parameter<std::string>(
      "topics.status", "/pickup_approach/planner_status");
    result_topic_ = declare_parameter<std::string>(
      "topics.result", "/pickup_approach/plan_result");

    maximum_localization_age_ = declare_parameter<double>("maximum_localization_age", 0.20);
    planning_timeout_ = declare_parameter<double>("planning_timeout", 5.0);
    stationary_speed_ = declare_parameter<double>("stationary_speed", 0.03);
    stationary_yaw_rate_ = declare_parameter<double>("stationary_yaw_rate", 0.03);
    maximum_position_variance_ = declare_parameter<double>("maximum_position_variance", 0.01);
    maximum_yaw_variance_ = declare_parameter<double>("maximum_yaw_variance", 0.007615435);
    require_cloud_not_older_than_goal_ =
      declare_parameter<bool>("require_cloud_not_older_than_goal", true);
    goal_cloud_tolerance_ = declare_parameter<double>("goal_cloud_tolerance", 0.5);
    skirt_max_height_ = declare_parameter<double>("skirt_max_height", -1.0);
    maximum_cloud_points_ = declare_parameter<int>("maximum_cloud_points", 20000);
    planning_rate_hz_ = declare_parameter<double>("planning_rate_hz", 10.0);

    geometry_.body_length = declare_parameter<double>("robot.body_length", geometry_.body_length);
    geometry_.body_width = declare_parameter<double>("robot.body_width", geometry_.body_width);

    config_.body_clearance = declare_parameter<double>("planner.body_clearance", config_.body_clearance);
    config_.tracking_error_allowance = declare_parameter<double>(
      "planner.tracking_error_allowance", config_.tracking_error_allowance);
    config_.transition_distance = declare_parameter<double>(
      "planner.transition_distance", config_.transition_distance);
    config_.orbit_extra_clearance = declare_parameter<double>(
      "planner.orbit_extra_clearance", config_.orbit_extra_clearance);
    config_.path_visible_sector_margin = declare_parameter<double>(
      "planner.path_visible_sector_margin", config_.path_visible_sector_margin);
    config_.search_step = declare_parameter<double>("planner.search_step", config_.search_step);
    config_.output_step = declare_parameter<double>("planner.output_step", config_.output_step);
    config_.validation_step = declare_parameter<double>(
      "planner.validation_step", config_.validation_step);
    config_.minimum_skirt_points = declare_parameter<int>(
      "pile_model.minimum_skirt_points", config_.minimum_skirt_points);
    config_.minimum_visible_arc = declare_parameter<double>(
      "pile_model.minimum_visible_arc", config_.minimum_visible_arc);
    config_.residual_coverage = declare_parameter<double>(
      "pile_model.residual_coverage", config_.residual_coverage);
    config_.radial_uncertainty = declare_parameter<double>(
      "pile_model.radial_uncertainty", config_.radial_uncertainty);
    config_.polygon_samples = declare_parameter<int>(
      "pile_model.polygon_samples", config_.polygon_samples);
    config_.circle_iterations = declare_parameter<int>(
      "pile_model.circle_iterations", config_.circle_iterations);
    config_.maximum_pile_radius = declare_parameter<double>(
      "pile_model.maximum_pile_radius", config_.maximum_pile_radius);
    config_.minimum_pile_radius = declare_parameter<double>(
      "pile_model.minimum_pile_radius", config_.minimum_pile_radius);
    config_.maximum_circle_rms_residual = declare_parameter<double>(
      "pile_model.maximum_circle_residual", config_.maximum_circle_rms_residual);
    config_.maximum_path_length_ratio = declare_parameter<double>(
      "planner.maximum_path_length_ratio", config_.maximum_path_length_ratio);
    validateParameters();

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);
    navigation_publisher_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("topics.navigation_result", "/navigation/result"), 10);
    publish_navigation_result_ = declare_parameter<bool>("publish_navigation_result", true);
    publish_navigation_planned_ = declare_parameter<bool>("publish_navigation_planned", false);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupPathPlannerNode::onOdometry, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&PickupPathPlannerNode::onGoal, this, std::placeholders::_1));
    skirt_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      skirt_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupPathPlannerNode::onSkirt, this, std::placeholders::_1));

    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      std::bind(
        &PickupPathPlannerNode::onCancel, this, std::placeholders::_1, std::placeholders::_2));
    replan_service_ = create_service<std_srvs::srv::Trigger>(
      "~/replan",
      std::bind(
        &PickupPathPlannerNode::onReplan, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / planning_rate_hz_)),
      std::bind(&PickupPathPlannerNode::onTimer, this));
    publishStatus("IDLE", "STARTUP");
  }

private:
  enum class State {kIdle, kWaiting, kDone, kFailed};

  void validateParameters() const
  {
    const bool ok =
      maximum_localization_age_ > 0.0 && planning_timeout_ > 0.0 &&
      planning_rate_hz_ > 0.0 && maximum_cloud_points_ > 100 &&
      geometry_.body_length > 0.0 && geometry_.body_width > 0.0 &&
      config_.body_clearance > 0.0 && config_.tracking_error_allowance >= 0.0 &&
      config_.search_step > 0.0 && config_.output_step > 0.0 && config_.validation_step > 0.0 &&
      config_.validation_step <= config_.output_step + 1e-12 &&
      config_.polygon_samples >= 16 && config_.minimum_skirt_points >= 3;
    if (!ok) {
      throw std::invalid_argument("pickup path planner parameters are invalid");
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    Odom sample;
    sample.pose.x = message->pose.pose.position.x;
    sample.pose.y = message->pose.pose.position.y;
    sample.valid = message->header.frame_id == map_frame_ &&
      validQuaternion(message->pose.pose.orientation) &&
      std::isfinite(sample.pose.x) && std::isfinite(sample.pose.y);
    if (sample.valid) {
      sample.pose.yaw = quaternionToYaw(message->pose.pose.orientation);
    }
    sample.speed = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
    sample.yaw_rate = std::abs(message->twist.twist.angular.z);
    const auto & c = message->pose.covariance;
    sample.position_variance = std::max(c[0], c[7]);
    sample.yaw_variance = c[35];
    sample.stamp = stampToSeconds(message->header.stamp);
    sample.valid = sample.valid && std::isfinite(sample.speed) && std::isfinite(sample.yaw_rate) &&
      std::isfinite(sample.position_variance) && std::isfinite(sample.yaw_variance);
    odometry_ = sample;
    odometry_receive_time_ = now();
    have_odometry_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_ || !std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y) || !validQuaternion(message->pose.orientation))
    {
      RCLCPP_ERROR(get_logger(), "Rejected goal: invalid pose or frame (expected %s)", map_frame_.c_str());
      publishResult("failed:INVALID_GOAL");
      publishStatus("FAILED", "INVALID_GOAL");
      state_ = State::kFailed;
      return;
    }
    goal_.x = message->pose.position.x;
    goal_.y = message->pose.position.y;
    goal_.yaw = quaternionToYaw(message->pose.orientation);
    goal_stamp_ = message->header.stamp;
    goal_stamp_seconds_ = stampToSeconds(message->header.stamp);
    have_goal_ = true;
    startWaiting("NEW_GOAL");
  }

  void onSkirt(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    // 山の検出が止まると空の点群が流れる場合があるため、空の点群は保持中の点群を上書きしません。
    if (static_cast<std::size_t>(message->width) * message->height == 0U) {
      return;
    }
    latest_cloud_ = message;
    cloud_receive_time_ = now();
  }

  void onCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    state_ = State::kIdle;
    publishResult("cancelled");
    publishStatus("IDLE", "CANCELLED");
    response->success = true;
    response->message = "planning cancelled";
  }

  void onReplan(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!have_goal_) {
      response->success = false;
      response->message = "no goal to plan for";
      return;
    }
    startWaiting("REPLAN_REQUESTED");
    response->success = true;
    response->message = "replanning started";
  }

  void startWaiting(const std::string & reason)
  {
    state_ = State::kWaiting;
    wait_deadline_ = now() + rclcpp::Duration::from_seconds(planning_timeout_);
    wait_reason_ = "WAITING_FOR_DATA";
    publishStatus("WAITING", reason);
  }

  void onTimer()
  {
    if (state_ != State::kWaiting) {
      return;
    }
    if (tryPlan()) {
      return;
    }
    if (now() > wait_deadline_) {
      state_ = State::kFailed;
      publishResult("failed:" + wait_reason_);
      publishNavigation("blocked");
      publishStatus("FAILED", wait_reason_);
      RCLCPP_ERROR(get_logger(), "Planning timed out: %s", wait_reason_.c_str());
    }
  }

  // 計画を試みます。結論(成功または確定した失敗)が出たらtrue、データ待ちならfalseを返します。
  bool tryPlan()
  {
    const rclcpp::Time current = now();
    if (!have_odometry_ || !odometry_.valid ||
      (current - odometry_receive_time_).seconds() > maximum_localization_age_)
    {
      wait_reason_ = "WAIT_LOCALIZATION";
      return false;
    }
    if (odometry_.position_variance > maximum_position_variance_ ||
      odometry_.yaw_variance > maximum_yaw_variance_)
    {
      wait_reason_ = "WAIT_LOCALIZATION_COVARIANCE";
      return false;
    }
    if (odometry_.speed > stationary_speed_ || odometry_.yaw_rate > stationary_yaw_rate_) {
      wait_reason_ = "WAIT_VEHICLE_STATIONARY";
      return false;
    }
    // DDS受信時刻だけでは古さを判定しません。点群ヘッダの時刻を目標時刻と比較し、
    // 接近中は別系統の too_close と monitor_enabled により最新の環境監視を保証します。
    if (!latest_cloud_) {
      wait_reason_ = "WAIT_SKIRT_POINTS";
      return false;
    }
    const double cloud_stamp = stampToSeconds(latest_cloud_->header.stamp);
    if (require_cloud_not_older_than_goal_ && goal_stamp_seconds_ > 0.0 && cloud_stamp > 0.0 &&
      cloud_stamp < goal_stamp_seconds_ - goal_cloud_tolerance_)
    {
      wait_reason_ = "WAIT_SKIRT_NEWER_THAN_GOAL";
      return false;
    }

    std::vector<Vec2> skirt;
    std::string error;
    if (!extractSkirtPoints(*latest_cloud_, &skirt, &error)) {
      wait_reason_ = error;
      return false;
    }
    ObservedPileModel model;
    if (!buildObservedPileModel(skirt, config_, &model, &error)) {
      wait_reason_ = "ADDITIONAL_SCAN_REQUIRED";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Pile model not usable yet: %s", error.c_str());
      return false;
    }

    const auto result = planPilePath(odometry_.pose, goal_, model, geometry_, config_);
    if (result.status != PlanStatus::kOk) {
      state_ = State::kFailed;
      const std::string code = toString(result.status);
      publishResult("failed:" + code);
      publishNavigation("blocked");
      publishStatus("FAILED", code + ": " + result.message);
      RCLCPP_ERROR(get_logger(), "Planning failed: %s (%s)", code.c_str(), result.message.c_str());
      return true;
    }
    publishPath(result.path);
    state_ = State::kDone;
    std::ostringstream stream;
    stream << "geometry=" << result.path.geometry << " direction=" << result.path.direction <<
      " length=" << result.path.length << " min_clearance=" << result.path.minimum_clearance <<
      " goal_clearance=" << result.path.goal_clearance <<
      " pile_radius=" << model.conservative_radius << " points=" << model.point_count <<
      " beam_clearance_verified=false";
    publishResult("planned");
    // masterのPlan→Drive遷移は、追従器が経路を受理してから行う。
    // 計画器からのplannedは診断用plan_resultだけを既定とし、二重通知を防ぐ。
    if (publish_navigation_planned_) {
      publishNavigation("planned");
    }
    publishStatus("PLANNED", stream.str());
    RCLCPP_INFO(get_logger(), "Planned: %s", stream.str().c_str());
    return true;
  }

  bool extractSkirtPoints(
    const sensor_msgs::msg::PointCloud2 & cloud, std::vector<Vec2> * points, std::string * error)
  {
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : cloud.fields) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      has_x = has_x || field.name == "x";
      has_y = has_y || field.name == "y";
      has_z = has_z || field.name == "z";
    }
    if (!has_x || !has_y || !has_z) {
      *error = "SKIRT_CLOUD_FORMAT_INVALID";
      return false;
    }
    const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (total == 0U) {
      *error = "WAIT_SKIRT_POINTS";
      return false;
    }

    bool transform_needed = cloud.header.frame_id != map_frame_;
    geometry_msgs::msg::TransformStamped transform;
    if (transform_needed) {
      try {
        transform = tf_buffer_->lookupTransform(
          map_frame_, cloud.header.frame_id, tf2::TimePointZero);
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "TF %s -> %s unavailable: %s",
          cloud.header.frame_id.c_str(), map_frame_.c_str(), exception.what());
        *error = "WAIT_TF";
        return false;
      }
    }

    // 点数が多い場合は等間隔に間引きます（円近似に十分で、計算量を一定にします）。
    const std::size_t stride = std::max<std::size_t>(
      1U, total / static_cast<std::size_t>(maximum_cloud_points_));
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    points->clear();
    points->reserve(std::min(total, static_cast<std::size_t>(maximum_cloud_points_) + 1U));
    for (std::size_t index = 0; index < total; ++index, ++iter_x, ++iter_y, ++iter_z) {
      if (index % stride != 0U) {
        continue;
      }
      double x = *iter_x;
      double y = *iter_y;
      double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (transform_needed) {
        double rx;
        double ry;
        double rz;
        rotateByQuaternion(transform.transform.rotation, x, y, z, &rx, &ry, &rz);
        x = rx + transform.transform.translation.x;
        y = ry + transform.transform.translation.y;
        z = rz + transform.transform.translation.z;
      }
      if (skirt_max_height_ > 0.0 && z > skirt_max_height_) {
        continue;
      }
      points->push_back({x, y});
    }
    return true;
  }

  void publishPath(const PlannedPath & planned)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = map_frame_;
    message.header.stamp = goal_stamp_;  // 目標のstampを複写し、追従側が対応を確認できるようにします。
    message.poses.reserve(planned.poses.size());
    for (const auto & pose : planned.poses) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = message.header;
      stamped.pose.position.x = pose.x;
      stamped.pose.position.y = pose.y;
      stamped.pose.position.z = 0.0;
      stamped.pose.orientation.x = 0.0;
      stamped.pose.orientation.y = 0.0;
      stamped.pose.orientation.z = std::sin(0.5 * pose.yaw);
      stamped.pose.orientation.w = std::cos(0.5 * pose.yaw);
      message.poses.push_back(stamped);
    }
    path_publisher_->publish(message);
  }

  void publishResult(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    result_publisher_->publish(message);
  }

  void publishNavigation(const std::string & text)
  {
    if (!publish_navigation_result_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = text;
    navigation_publisher_->publish(message);
  }

  void publishStatus(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = state + "|" + detail;
    status_publisher_->publish(message);
  }

  struct Odom
  {
    Pose2D pose{};
    double speed{0.0};
    double yaw_rate{0.0};
    double position_variance{0.0};
    double yaw_variance{0.0};
    double stamp{0.0};
    bool valid{false};
  };

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  PlannerConfig config_{};
  RobotGeometry geometry_{};
  std::string map_frame_;
  std::string odometry_topic_;
  std::string goal_topic_;
  std::string skirt_topic_;
  std::string path_topic_;
  std::string status_topic_;
  std::string result_topic_;
  double maximum_localization_age_{0.2};
  double planning_timeout_{5.0};
  double stationary_speed_{0.03};
  double stationary_yaw_rate_{0.03};
  double maximum_position_variance_{0.01};
  double maximum_yaw_variance_{0.007615435};
  bool require_cloud_not_older_than_goal_{true};
  double goal_cloud_tolerance_{0.5};
  double skirt_max_height_{-1.0};
  int maximum_cloud_points_{20000};
  double planning_rate_hz_{10.0};
  bool publish_navigation_result_{true};
  bool publish_navigation_planned_{false};

  State state_{State::kIdle};
  std::string wait_reason_{"WAITING_FOR_DATA"};
  rclcpp::Time wait_deadline_{0, 0, RCL_ROS_TIME};
  Pose2D goal_{};
  bool have_goal_{false};
  builtin_interfaces::msg::Time goal_stamp_{};
  double goal_stamp_seconds_{0.0};
  Odom odometry_{};
  bool have_odometry_{false};
  rclcpp::Time odometry_receive_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_{};
  rclcpp::Time cloud_receive_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr skirt_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pickup_path_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pickup_path_planner::PickupPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
