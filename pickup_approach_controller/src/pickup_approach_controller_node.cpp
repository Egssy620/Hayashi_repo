// 山接近の経路追従ノードです。
//
// 責務: 経路計画ノードが出した nav_msgs/Path を、融合自己位置(/odometry/filtered)を使って
//       空間的に追従し、車体座標の速度指令(vx, vy, omega)を出力します。到着も判定します。
// 経路は作りません。速度指令は /pickup_approach/cmd_vel へ出し、/cmd_vel へは直接出しません
// (巡回側との排他は上位の走行権限muxが行います)。

#include "pickup_approach_controller/controller_core.hpp"

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
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace pickup_approach_controller
{
namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-8;
}

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

geometry_msgs::msg::Twist makeTwist(const double vx_body, const double vy_body, const double omega)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = vx_body;
  message.linear.y = vy_body;
  message.angular.z = omega;
  return message;
}

bool positiveSemidefinitePlanarCovariance(const double covariance[3][3])
{
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      if (!std::isfinite(covariance[row][column])) {
        return false;
      }
    }
  }
  const double a = covariance[0][0];
  const double d = covariance[1][1];
  const double f = covariance[2][2];
  const double b = 0.5 * (covariance[0][1] + covariance[1][0]);
  const double c = 0.5 * (covariance[0][2] + covariance[2][0]);
  const double e = 0.5 * (covariance[1][2] + covariance[2][1]);
  constexpr double tolerance = 1.0e-12;
  const double determinant = a * d * f + 2.0 * b * c * e - a * e * e - d * c * c - f * b * b;
  return a >= -tolerance && d >= -tolerance && f >= -tolerance &&
         a * d - b * b >= -tolerance && a * f - c * c >= -tolerance &&
         d * f - e * e >= -tolerance && determinant >= -tolerance;
}

}  // namespace

class PickupApproachControllerNode : public rclcpp::Node
{
public:
  PickupApproachControllerNode()
  : Node("pickup_approach_controller"),
    config_(readControllerConfig()),
    controller_(config_)
  {
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/odometry/filtered");
    goal_topic_ = declare_parameter<std::string>("topics.goal", "/pickup_pose");
    path_topic_ = declare_parameter<std::string>("topics.path", "/pickup_approach/path");
    too_close_topic_ = declare_parameter<std::string>(
      "topics.too_close", "/scrap_pile_skirt_node/too_close");
    monitor_enabled_topic_ = declare_parameter<std::string>(
      "topics.monitor_enabled", "/scrap_pile_skirt_node/enabled");
    command_topic_ = declare_parameter<std::string>("topics.command", "/pickup_approach/cmd_vel");
    debug_command_topic_ = declare_parameter<std::string>(
      "topics.debug_command", "/pickup_approach/debug_cmd_vel");
    result_topic_ = declare_parameter<std::string>("topics.result", "/pickup_approach/result");
    status_topic_ = declare_parameter<std::string>("topics.status", "/pickup_approach/status");
    navigation_result_topic_ = declare_parameter<std::string>(
      "topics.navigation_result", "/navigation/result");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    maximum_localization_age_ = declare_parameter<double>("maximum_localization_age", 0.10);
    maximum_environment_age_ = declare_parameter<double>("maximum_environment_age", 0.25);
    maximum_position_variance_ = declare_parameter<double>("maximum_position_variance", 0.01);
    maximum_yaw_variance_ = declare_parameter<double>("maximum_yaw_variance", 0.007615435);
    pose_jump_threshold_ = declare_parameter<double>("pose_jump_threshold", 0.10);
    yaw_jump_threshold_ = declare_parameter<double>("yaw_jump_threshold", 0.10);
    localization_hold_seconds_ = declare_parameter<double>("localization_hold_seconds", 0.5);
    maximum_start_position_error_ = declare_parameter<double>("maximum_start_position_error", 0.10);
    maximum_start_yaw_error_ = declare_parameter<double>("maximum_start_yaw_error", 0.17);
    require_path_goal_stamp_match_ = declare_parameter<bool>("require_path_goal_stamp_match", true);
    blocked_hold_seconds_ = declare_parameter<double>("blocked_hold_seconds", 5.0);
    straight_path_spacing_ = declare_parameter<double>("straight_path_spacing", 0.02);
    require_environment_guard_ = declare_parameter<bool>("require_environment_guard", true);
    require_monitor_enabled_ = declare_parameter<bool>("require_monitor_enabled", true);
    enable_straight_line_fallback_ = declare_parameter<bool>("enable_straight_line_fallback", false);
    publish_motion_commands_ = declare_parameter<bool>("publish_motion_commands", false);
    publish_navigation_result_ = declare_parameter<bool>("publish_navigation_result", true);
    validateParameters();

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);
    debug_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(debug_command_topic_, 10);
    result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    navigation_result_publisher_ =
      create_publisher<std_msgs::msg::String>(navigation_result_topic_, 10);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupApproachControllerNode::onOdometry, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&PickupApproachControllerNode::onGoal, this, std::placeholders::_1));
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&PickupApproachControllerNode::onPath, this, std::placeholders::_1));
    too_close_subscription_ = create_subscription<std_msgs::msg::Bool>(
      too_close_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupApproachControllerNode::onTooClose, this, std::placeholders::_1));
    if (!monitor_enabled_topic_.empty()) {
      monitor_enabled_subscription_ = create_subscription<std_msgs::msg::Bool>(
        monitor_enabled_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PickupApproachControllerNode::onMonitorEnabled, this, std::placeholders::_1));
    }

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      std::bind(
        &PickupApproachControllerNode::onSetEnabled, this, std::placeholders::_1,
        std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      std::bind(
        &PickupApproachControllerNode::onCancel, this, std::placeholders::_1,
        std::placeholders::_2));

    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz_)),
      std::bind(&PickupApproachControllerNode::onControlTimer, this));

    publishZeroCommand();
    publishStatus("DISABLED", "STARTUP_SAFE_STATE", true);
    RCLCPP_WARN(
      get_logger(),
      "Started with motion authority OFF. publish_motion_commands=%s, straight fallback=%s",
      publish_motion_commands_ ? "true" : "false",
      enable_straight_line_fallback_ ? "true" : "false");
  }

private:
  void validateParameters() const
  {
    const bool timing_valid =
      control_rate_hz_ > 0.0 && maximum_localization_age_ > 0.0 && maximum_environment_age_ > 0.0 &&
      maximum_position_variance_ > 0.0 && maximum_yaw_variance_ > 0.0 &&
      pose_jump_threshold_ > 0.0 && yaw_jump_threshold_ > 0.0 &&
      localization_hold_seconds_ >= 0.0 && blocked_hold_seconds_ > 0.0;
    const bool geometry_valid =
      straight_path_spacing_ > 0.0 && maximum_start_position_error_ > 0.0 &&
      maximum_start_yaw_error_ > 0.0;
    const bool limits_valid =
      config_.maximum_speed > 0.0 && config_.cruise_speed > 0.0 &&
      config_.cruise_speed <= config_.maximum_speed + 1.0e-12 &&
      config_.maximum_yaw_rate > 0.0 && config_.maximum_acceleration > 0.0 &&
      config_.maximum_yaw_acceleration > 0.0 && config_.maximum_jerk > 0.0 &&
      config_.maximum_snap > 0.0 && config_.maximum_lateral_acceleration > 0.0 &&
      config_.path_deceleration > 0.0 && config_.maximum_progress_rate > 0.0 &&
      config_.path_resample_spacing > 0.0 && config_.curvature_window >= 1 &&
      config_.maximum_tracking_time > 0.0 && config_.terminal_minimum_speed >= 0.0 &&
      config_.terminal_minimum_speed <= config_.maximum_speed;
    const bool arrival_valid =
      config_.arrival_position_tolerance > 0.0 && config_.arrival_yaw_tolerance > 0.0 &&
      config_.arrival_window_seconds > 0.0 && config_.arrival_minimum_window_seconds > 0.0 &&
      config_.arrival_minimum_window_seconds <= config_.arrival_window_seconds &&
      config_.arrival_minimum_samples >= 2 && config_.confidence_sigma >= 0.0 &&
      config_.maximum_position_drift_rate > 0.0 && config_.maximum_yaw_drift_rate > 0.0 &&
      config_.path_goal_position_tolerance > 0.0 && config_.path_goal_yaw_tolerance > 0.0;
    if (!timing_valid || !geometry_valid || !limits_valid || !arrival_valid) {
      throw std::invalid_argument("pickup approach parameters must be positive and consistent");
    }
  }

  ControllerConfig readControllerConfig()
  {
    ControllerConfig c;
    const auto d = [this](const std::string & name, const double fallback) {
        return declare_parameter<double>(name, fallback);
      };
    c.maximum_speed = d("controller.maximum_speed", c.maximum_speed);
    c.cruise_speed = d("controller.cruise_speed", c.cruise_speed);
    c.maximum_acceleration = d("controller.maximum_acceleration", c.maximum_acceleration);
    c.maximum_jerk = d("controller.maximum_jerk", c.maximum_jerk);
    c.maximum_snap = d("controller.maximum_snap", c.maximum_snap);
    c.maximum_lateral_acceleration = d(
      "controller.maximum_lateral_acceleration", c.maximum_lateral_acceleration);
    c.path_deceleration = d("controller.path_deceleration", c.path_deceleration);
    c.maximum_yaw_rate = d("controller.maximum_yaw_rate", c.maximum_yaw_rate);
    c.maximum_yaw_acceleration = d(
      "controller.maximum_yaw_acceleration", c.maximum_yaw_acceleration);
    c.maximum_longitudinal_recovery_speed = d(
      "controller.maximum_longitudinal_recovery_speed", c.maximum_longitudinal_recovery_speed);
    c.position_gain = d("controller.position_gain", c.position_gain);
    c.maximum_correction_speed = d(
      "controller.maximum_correction_speed", c.maximum_correction_speed);
    c.yaw_kp = d("controller.yaw_kp", c.yaw_kp);
    c.yaw_ki = d("controller.yaw_ki", c.yaw_ki);
    c.yaw_kd = d("controller.yaw_kd", c.yaw_kd);
    c.yaw_integral_limit = d("controller.yaw_integral_limit", c.yaw_integral_limit);
    c.yaw_derivative_filter_time_constant = d(
      "controller.yaw_derivative_filter_time_constant", c.yaw_derivative_filter_time_constant);
    c.projection_search_back = d("controller.projection_search_back", c.projection_search_back);
    c.projection_search_ahead = d("controller.projection_search_ahead", c.projection_search_ahead);
    c.progress_correction_time_constant = d(
      "controller.progress_correction_time_constant", c.progress_correction_time_constant);
    c.maximum_progress_rate = d("controller.maximum_progress_rate", c.maximum_progress_rate);
    c.curvature_feedforward_gain = d(
      "controller.curvature_feedforward_gain", c.curvature_feedforward_gain);
    c.velocity_shaping_gain = d("controller.velocity_shaping_gain", c.velocity_shaping_gain);
    c.acceleration_shaping_gain = d(
      "controller.acceleration_shaping_gain", c.acceleration_shaping_gain);
    c.path_resample_spacing = d("controller.path_resample_spacing", c.path_resample_spacing);
    c.curvature_window = declare_parameter<int>("controller.curvature_window", c.curvature_window);
    c.maximum_heading_step = d("controller.maximum_heading_step", c.maximum_heading_step);
    c.path_goal_position_tolerance = d(
      "controller.path_goal_position_tolerance", c.path_goal_position_tolerance);
    c.path_goal_yaw_tolerance = d("controller.path_goal_yaw_tolerance", c.path_goal_yaw_tolerance);
    c.terminal_minimum_speed = d("controller.terminal_minimum_speed", c.terminal_minimum_speed);
    c.maximum_tracking_time = d("controller.maximum_tracking_time", c.maximum_tracking_time);
    c.arrival_position_tolerance = d("arrival.position_tolerance", c.arrival_position_tolerance);
    c.arrival_yaw_tolerance = d("arrival.yaw_tolerance", c.arrival_yaw_tolerance);
    c.arrival_command_speed_tolerance = d(
      "arrival.command_speed_tolerance", c.arrival_command_speed_tolerance);
    c.arrival_command_yaw_rate_tolerance = d(
      "arrival.command_yaw_rate_tolerance", c.arrival_command_yaw_rate_tolerance);
    c.arrival_window_seconds = d("arrival.window_seconds", c.arrival_window_seconds);
    c.arrival_minimum_window_seconds = d(
      "arrival.minimum_window_seconds", c.arrival_minimum_window_seconds);
    c.arrival_minimum_samples = declare_parameter<int>(
      "arrival.minimum_samples", c.arrival_minimum_samples);
    c.confidence_sigma = d("arrival.confidence_sigma", c.confidence_sigma);
    c.maximum_position_drift_rate = d(
      "arrival.maximum_position_drift_rate", c.maximum_position_drift_rate);
    c.maximum_yaw_drift_rate = d("arrival.maximum_yaw_drift_rate", c.maximum_yaw_drift_rate);
    return c;
  }

  // 融合自己位置を制御用に整形します。
  // 通番は時刻の変化から作ります。時刻の逆戻り、または大きな姿勢の飛びを検出したら、
  // 再位置合わせとして扱い、一定時間は自己位置を使わず(停止)、コアの進捗と到着窓を作り直させます。
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    LocalizationSample sample;
    sample.stamp = stampToSeconds(message->header.stamp);
    sample.pose.x = message->pose.pose.position.x;
    sample.pose.y = message->pose.pose.position.y;
    const bool quaternion_valid = finiteQuaternion(message->pose.pose.orientation);
    sample.pose.yaw = quaternion_valid ? quaternionToYaw(message->pose.pose.orientation) : 0.0;
    sample.vx_body = message->twist.twist.linear.x;
    sample.vy_body = message->twist.twist.linear.y;
    sample.omega = message->twist.twist.angular.z;

    const auto & covariance = message->pose.covariance;
    sample.covariance[0][0] = covariance[0];
    sample.covariance[0][1] = covariance[1];
    sample.covariance[0][2] = covariance[5];
    sample.covariance[1][0] = covariance[6];
    sample.covariance[1][1] = covariance[7];
    sample.covariance[1][2] = covariance[11];
    sample.covariance[2][0] = covariance[30];
    sample.covariance[2][1] = covariance[31];
    sample.covariance[2][2] = covariance[35];

    const bool frame_valid =
      message->header.frame_id == map_frame_ && message->child_frame_id == base_frame_;
    const bool pose_valid =
      std::isfinite(sample.pose.x) && std::isfinite(sample.pose.y) &&
      std::isfinite(sample.pose.yaw) && quaternion_valid;
    const bool covariance_valid =
      positiveSemidefinitePlanarCovariance(sample.covariance) &&
      sample.covariance[0][0] <= maximum_position_variance_ &&
      sample.covariance[1][1] <= maximum_position_variance_ &&
      sample.covariance[2][2] <= maximum_yaw_variance_;
    const bool stamp_valid = std::isfinite(sample.stamp) && sample.stamp > 0.0;

    bool reset_detected = false;
    if (frame_valid && pose_valid && stamp_valid) {
      if (have_previous_sample_) {
        const bool stamp_regressed = sample.stamp < previous_stamp_ - 1.0e-9;
        const double jump = std::hypot(
          sample.pose.x - previous_pose_.x, sample.pose.y - previous_pose_.y);
        const double yaw_jump = std::abs(wrapAngle(sample.pose.yaw - previous_pose_.yaw));
        reset_detected = stamp_regressed || jump > pose_jump_threshold_ ||
          yaw_jump > yaw_jump_threshold_;
      }
      if (!have_previous_sample_ || sample.stamp > previous_stamp_ + 1.0e-9 || reset_detected) {
        ++localization_sequence_;
      }
      previous_pose_ = sample.pose;
      previous_stamp_ = sample.stamp;
      have_previous_sample_ = true;
    }
    if (reset_detected) {
      ++reset_counter_;
      localization_hold_until_ = now() + rclcpp::Duration::from_seconds(localization_hold_seconds_);
      RCLCPP_WARN(get_logger(), "Localization reset/jump detected; holding for %.2f s",
        localization_hold_seconds_);
    }
    sample.sequence = localization_sequence_;
    sample.reset_counter = reset_counter_;
    sample.healthy = frame_valid && pose_valid && covariance_valid && stamp_valid &&
      std::isfinite(sample.vx_body) && std::isfinite(sample.vy_body) && !reset_detected;

    latest_localization_ = sample;
    have_localization_ = true;
    localization_receive_time_ = now();

    // 経路到着の瞬間だけ自己位置が更新周期の谷間だった場合、経路を失わず、
    // 次の正常な自己位置サンプルで受理処理を再試行する。
    if (pending_path_ && sample.healthy && localizationFresh()) {
      const auto pending = pending_path_;
      pending_path_.reset();
      RCLCPP_INFO(get_logger(), "Valid localization recovered; accepting deferred path");
      onPath(pending);
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "Ignored pickup goal because motion authority is OFF");
      publishStatus("DISABLED", "GOAL_WITHOUT_AUTHORITY", true);
      return;
    }
    if (message->header.frame_id != map_frame_ ||
      !std::isfinite(message->pose.position.x) || !std::isfinite(message->pose.position.y) ||
      !finiteQuaternion(message->pose.orientation))
    {
      RCLCPP_ERROR(get_logger(), "Rejected pickup goal: invalid map-frame pose");
      publishStatus("WAIT_GOAL", "GOAL_INVALID", true);
      return;
    }

    goal_.x = message->pose.position.x;
    goal_.y = message->pose.position.y;
    goal_.yaw = quaternionToYaw(message->pose.orientation);
    goal_stamp_ = message->header.stamp;
    have_goal_ = true;
    pending_path_.reset();
    resetEpisode();
    controller_.clearPath();
    publishZeroCommand();
    constexpr double kRadToDeg = 57.29577951308232;
    RCLCPP_INFO(
      get_logger(), "Pickup goal received: x=%.3f y=%.3f yaw=%.2f deg",
      goal_.x, goal_.y, goal_.yaw * kRadToDeg);

    if (enable_straight_line_fallback_ && have_localization_ && latest_localization_.healthy) {
      std::string error;
      const auto path = makeStraightPath(latest_localization_.pose, goal_, straight_path_spacing_);
      if (controller_.setPath(path, goal_, &error)) {
        publishResult("planned_straight_fallback");
        publishNavigation("planned");
        publishStatus("READY", "STRAIGHT_FALLBACK_PATH", true);
        return;
      }
      RCLCPP_ERROR(get_logger(), "Failed to create fallback path: %s", error.c_str());
    }
    publishStatus("WAIT_PATH", "GOAL_ACCEPTED", true);
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    const auto reject = [this](const std::string & reason) {
        RCLCPP_ERROR(get_logger(), "Rejected path: %s", reason.c_str());
        publishStatus("WAIT_PATH", reason, true);
      };
    if (!have_goal_) {
      RCLCPP_WARN(get_logger(), "Rejected path because no pickup goal is active");
      publishStatus("WAIT_GOAL", "PATH_WITHOUT_GOAL", true);
      return;
    }
    if (message->header.frame_id != map_frame_ || message->poses.size() < 2U) {
      reject("PATH_INVALID");
      return;
    }
    // 経路が現在の目標のために計画されたことを、目標のstampの一致で確認します。
    const double goal_stamp = stampToSeconds(goal_stamp_);
    const double path_stamp = stampToSeconds(message->header.stamp);
    if (require_path_goal_stamp_match_ && goal_stamp > 0.0 &&
      std::abs(path_stamp - goal_stamp) > 1.0e-6)
    {
      reject("PATH_GOAL_STAMP_MISMATCH");
      return;
    }

    std::vector<Pose2D> path;
    path.reserve(message->poses.size());
    for (const auto & stamped_pose : message->poses) {
      if ((!stamped_pose.header.frame_id.empty() && stamped_pose.header.frame_id != map_frame_) ||
        !std::isfinite(stamped_pose.pose.position.x) ||
        !std::isfinite(stamped_pose.pose.position.y) ||
        !finiteQuaternion(stamped_pose.pose.orientation))
      {
        reject("PATH_INVALID");
        return;
      }
      path.push_back(Pose2D{
        stamped_pose.pose.position.x, stamped_pose.pose.position.y,
        quaternionToYaw(stamped_pose.pose.orientation)});
    }

    // 経路の始点が現在位置から離れていれば、古い計画と見なして拒否します。
    if (!have_localization_ || !latest_localization_.healthy || !localizationFresh()) {
      pending_path_ = message;
      RCLCPP_WARN(get_logger(), "Deferring path until valid localization is received");
      publishStatus("WAIT_PATH", "PATH_WAITING_FOR_VALID_LOCALIZATION", true);
      return;
    }
    pending_path_.reset();
    const double start_error = std::hypot(
      path.front().x - latest_localization_.pose.x, path.front().y - latest_localization_.pose.y);
    const double start_yaw_error = std::abs(wrapAngle(path.front().yaw - latest_localization_.pose.yaw));
    if (start_error > maximum_start_position_error_ || start_yaw_error > maximum_start_yaw_error_) {
      reject("PATH_START_MISMATCH");
      return;
    }

    std::string error;
    if (!controller_.setPath(path, goal_, &error)) {
      RCLCPP_ERROR(get_logger(), "Path geometry rejected: %s", error.c_str());
      publishStatus("WAIT_PATH", "PATH_GEOMETRY_INVALID", true);
      return;
    }
    constexpr double kRadToDeg = 57.29577951308232;
    RCLCPP_INFO(
      get_logger(),
      "Pickup path accepted: points=%zu start_yaw=%.2f deg goal_yaw=%.2f deg odom_yaw=%.2f deg",
      path.size(), path.front().yaw * kRadToDeg, path.back().yaw * kRadToDeg,
      latest_localization_.pose.yaw * kRadToDeg);
    resetEpisode();
    publishResult("planned");
    // masterへは、経路のstamp・始点・形状を検査して受理した後でのみ通知する。
    publishNavigation("planned");
    publishStatus(enabled_ ? "READY" : "DISABLED", "PATH_ACCEPTED", true);
  }

  void onTooClose(const std_msgs::msg::Bool::SharedPtr message)
  {
    too_close_ = message->data;
    have_environment_guard_ = true;
    environment_receive_time_ = now();
  }

  void onMonitorEnabled(const std_msgs::msg::Bool::SharedPtr message)
  {
    monitor_enabled_ = message->data;
    have_monitor_enabled_ = true;
  }

  void onSetEnabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_ = request->data;
    controller_.clearPath();
    pending_path_.reset();
    have_goal_ = false;
    resetEpisode();
    publishZeroCommand();
    if (!enabled_) {
      last_arrival_reason_ = "AUTHORITY_RELEASED";
      publishResult("authority_released");
      publishStatus("DISABLED", "AUTHORITY_RELEASED", true);
      response->message = "Pickup approach motion authority released; active goal cleared";
    } else {
      last_arrival_reason_ = "WINDOW_NOT_READY";
      publishResult("authority_acquired");
      publishStatus("WAIT_GOAL", "AUTHORITY_ACQUIRED", true);
      response->message = publish_motion_commands_ ?
        "Pickup approach authority acquired; waiting for a new goal" :
        "Authority acquired in calculation-only mode; waiting for a new goal";
    }
    response->success = true;
  }

  void onCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    controller_.clearPath();
    pending_path_.reset();
    have_goal_ = false;
    resetEpisode();
    last_arrival_reason_ = "CANCELLED";
    publishZeroCommand();
    publishResult("cancelled");
    publishStatus(enabled_ ? "WAIT_GOAL" : "DISABLED", "CANCELLED", true);
    response->success = true;
    response->message = "Active pickup approach goal cancelled";
  }

  bool localizationFresh() const
  {
    if (!have_localization_) {
      return false;
    }
    const rclcpp::Time current_time = now();
    const double receive_age = (current_time - localization_receive_time_).seconds();
    const double stamp_age = current_time.seconds() - latest_localization_.stamp;
    return receive_age >= 0.0 && receive_age <= maximum_localization_age_ &&
           stamp_age >= 0.0 && stamp_age <= maximum_localization_age_ &&
           current_time >= localization_hold_until_;
  }

  void onControlTimer()
  {
    const rclcpp::Time current_time = now();
    const double environment_age = have_environment_guard_ ?
      (current_time - environment_receive_time_).seconds() :
      std::numeric_limits<double>::infinity();
    bool environment_fresh = !require_environment_guard_ ||
      (have_environment_guard_ && environment_age >= 0.0 &&
      environment_age <= maximum_environment_age_);
    // 山の監視ノードが無効化されている間は、近接監視がないものとして走行しません。
    const bool monitor_disabled =
      require_environment_guard_ && require_monitor_enabled_ && have_monitor_enabled_ &&
      !monitor_enabled_;
    if (monitor_disabled) {
      environment_fresh = false;
    }

    LocalizationSample localization = latest_localization_;
    localization.healthy = localization.healthy && localizationFresh();
    const bool effective_too_close = have_environment_guard_ ? too_close_ : require_environment_guard_;
    const ControllerOutput output = controller_.step(
      current_time.seconds(), localization, enabled_, environment_fresh, effective_too_close);

    constexpr double kRadToDeg = 57.29577951308232;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "pickup trace: state=%s progress=%.2f odom_yaw=%.2f deg ref_yaw=%.2f deg "
      "goal_yaw=%.2f deg yaw_error=%.2f deg omega=%+.3f rad/s pos_error=%.3f m",
      output.state.c_str(), output.path_progress,
      localization.pose.yaw * kRadToDeg, output.reference_yaw * kRadToDeg,
      goal_.yaw * kRadToDeg, output.yaw_error * kRadToDeg,
      output.omega, output.position_error);

    debug_command_publisher_->publish(makeTwist(output.vx_body, output.vy_body, output.omega));
    // 権限を持つ間はゼロ指令も含めて毎周期出力します（指令タイムアウトのある下位側に備える）。
    if (publish_motion_commands_ && enabled_) {
      command_publisher_->publish(makeTwist(output.vx_body, output.vy_body, output.omega));
    } else {
      publishZeroCommand();
    }

    if (output.arrival_pulse && !arrival_result_sent_) {
      publishResult("arrived");
      publishNavigation("arrived");
      arrival_result_sent_ = true;
    }
    if (output.timeout_pulse && !timeout_result_sent_) {
      publishResult("timeout");
      timeout_result_sent_ = true;
    }
    updateBlockedNotification(output, monitor_disabled, current_time);

    last_arrival_decision_ = output.arrival_decision;
    last_arrival_reason_ = output.arrival_reason;
    if (enabled_ && !have_goal_) {
      publishStatus("WAIT_GOAL", "NO_GOAL", false);
    } else if (monitor_disabled && output.state == "ENVIRONMENT_HOLD") {
      publishStatus(output.state, "PILE_MONITOR_DISABLED", false);
    } else {
      publishStatus(output.state, output.reason, false);
    }
  }

  // master_nodeへ"blocked"を、一つの走行につき一度だけ通知します。
  //  - 山への近接停止(SAFETY_STOP)とタイムアウトは直ちに通知します。
  //  - 自己位置・環境監視・周期異常による保持は、blocked_hold_seconds続いたときに通知します。
  void updateBlockedNotification(
    const ControllerOutput & output, const bool monitor_disabled, const rclcpp::Time & current_time)
  {
    const bool immediate = output.state == "SAFETY_STOP" || output.state == "TIMEOUT";
    const bool holding =
      output.state == "LOCALIZATION_HOLD" || output.state == "ENVIRONMENT_HOLD" ||
      output.state == "TIMING_HOLD";
    if (!holding) {
      hold_started_ = false;
    } else if (!hold_started_) {
      hold_started_ = true;
      hold_start_time_ = current_time;
    }
    const bool hold_long = holding && hold_started_ &&
      (current_time - hold_start_time_).seconds() >= blocked_hold_seconds_;
    if ((immediate || hold_long) && !blocked_sent_ && have_goal_) {
      std::string reason = output.reason;
      if (monitor_disabled && output.state == "ENVIRONMENT_HOLD") {
        reason = "PILE_MONITOR_DISABLED";
      }
      publishResult("blocked:" + reason);
      publishNavigation("blocked");
      blocked_sent_ = true;
    }
  }

  void resetEpisode()
  {
    arrival_result_sent_ = false;
    blocked_sent_ = false;
    timeout_result_sent_ = false;
    hold_started_ = false;
    last_arrival_decision_ = "COLLECTING";
    last_arrival_reason_ = "WINDOW_NOT_READY";
  }

  void publishZeroCommand() {command_publisher_->publish(makeTwist(0.0, 0.0, 0.0));}

  void publishResult(const std::string & result)
  {
    std_msgs::msg::String message;
    message.data = result;
    result_publisher_->publish(message);
  }

  void publishNavigation(const std::string & result)
  {
    if (!publish_navigation_result_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = result;
    navigation_result_publisher_->publish(message);
  }

  void publishStatus(const std::string & state, const std::string & reason, const bool force)
  {
    std::ostringstream stream;
    stream << state << "|reason=" << reason
           << "|enabled=" << (enabled_ ? "true" : "false")
           << "|motion_output=" << (publish_motion_commands_ ? "true" : "false")
           << "|goal=" << (have_goal_ ? "true" : "false")
           << "|path=" << (controller_.hasPath() ? "true" : "false")
           << "|arrival=" << last_arrival_decision_
           << "|arrival_reason=" << last_arrival_reason_;
    const std::string status = stream.str();
    const rclcpp::Time current_time = now();
    const bool heartbeat_due =
      last_status_publish_time_.nanoseconds() == 0 ||
      (current_time - last_status_publish_time_).seconds() >= 1.0;
    if (!force && status == last_status_ && !heartbeat_due) {
      return;
    }
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
    last_status_ = status;
    last_status_publish_time_ = current_time;
  }

  ControllerConfig config_;
  ControllerCore controller_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odometry_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string too_close_topic_;
  std::string monitor_enabled_topic_;
  std::string command_topic_;
  std::string debug_command_topic_;
  std::string result_topic_;
  std::string status_topic_;
  std::string navigation_result_topic_;
  double control_rate_hz_{50.0};
  double maximum_localization_age_{0.10};
  double maximum_environment_age_{0.25};
  double maximum_position_variance_{0.01};
  double maximum_yaw_variance_{0.007615435};
  double pose_jump_threshold_{0.10};
  double yaw_jump_threshold_{0.10};
  double localization_hold_seconds_{0.5};
  double maximum_start_position_error_{0.10};
  double maximum_start_yaw_error_{0.17};
  double blocked_hold_seconds_{5.0};
  double straight_path_spacing_{0.02};
  bool require_path_goal_stamp_match_{true};
  bool require_environment_guard_{true};
  bool require_monitor_enabled_{true};
  bool enable_straight_line_fallback_{false};
  bool publish_motion_commands_{false};
  bool publish_navigation_result_{true};

  bool enabled_{false};
  bool have_goal_{false};
  bool have_localization_{false};
  bool have_environment_guard_{false};
  bool too_close_{true};
  bool monitor_enabled_{false};
  bool have_monitor_enabled_{false};
  bool arrival_result_sent_{false};
  bool blocked_sent_{false};
  bool timeout_result_sent_{false};
  bool hold_started_{false};
  Pose2D goal_{};
  builtin_interfaces::msg::Time goal_stamp_{};
  nav_msgs::msg::Path::SharedPtr pending_path_{};
  LocalizationSample latest_localization_{};
  std::uint64_t localization_sequence_{0U};
  std::uint64_t reset_counter_{0U};
  Pose2D previous_pose_{};
  double previous_stamp_{0.0};
  bool have_previous_sample_{false};
  rclcpp::Time localization_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_hold_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time environment_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hold_start_time_{0, 0, RCL_ROS_TIME};
  std::string last_status_;
  std::string last_arrival_decision_{"COLLECTING"};
  std::string last_arrival_reason_{"WINDOW_NOT_READY"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr debug_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_result_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr too_close_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr monitor_enabled_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pickup_approach_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pickup_approach_controller::PickupApproachControllerNode>());
  rclcpp::shutdown();
  return 0;
}
