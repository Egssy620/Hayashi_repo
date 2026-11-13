#include "pickup_approach_controller/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>

namespace pickup_approach_controller
{
namespace
{

constexpr double kSmall = 1.0e-9;
constexpr double kMergeDistance = 1.0e-4;  // これより近い連続点は同一点として統合する [m]

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

void limitVector(double & x, double & y, const double limit)
{
  const double magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > kSmall) {
    const double scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

// 加速度が上限へ近づくとき、上限に達する前にジャークを絞ります（スナップ制限と整合）。
void limitJerkBeforeAccelerationBoundary(
  double & jerk_x, double & jerk_y, const double acceleration_x, const double acceleration_y,
  const double maximum_acceleration, const double maximum_snap, const double dt)
{
  const double acceleration_norm = std::hypot(acceleration_x, acceleration_y);
  if (acceleration_norm <= kSmall) {
    return;
  }
  const double direction_x = acceleration_x / acceleration_norm;
  const double direction_y = acceleration_y / acceleration_norm;
  const double outward_jerk = direction_x * jerk_x + direction_y * jerk_y;
  const double acceleration_margin = std::max(maximum_acceleration - acceleration_norm, 0.0);
  const double continuous_limit = std::sqrt(2.0 * maximum_snap * acceleration_margin);
  const double discrete_margin = maximum_snap * dt;
  const double maximum_outward_jerk = std::max(continuous_limit - discrete_margin, 0.0);
  if (outward_jerk > maximum_outward_jerk) {
    jerk_x -= (outward_jerk - maximum_outward_jerk) * direction_x;
    jerk_y -= (outward_jerk - maximum_outward_jerk) * direction_y;
  }
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if ((values.size() % 2U) != 0U) {
    return upper;
  }
  const auto lower_middle = std::max_element(values.begin(), middle);
  return 0.5 * (upper + *lower_middle);
}

double robustSigma(const std::vector<double> & values)
{
  if (values.size() < 2U) {
    return 0.0;
  }
  const double center = median(values);
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const double value : values) {
    deviations.push_back(std::abs(value - center));
  }
  return 1.4826 * median(deviations);
}

double circularMean(const std::vector<double> & values)
{
  double sine_sum = 0.0;
  double cosine_sum = 0.0;
  for (const double value : values) {
    sine_sum += std::sin(value);
    cosine_sum += std::cos(value);
  }
  return std::atan2(sine_sum, cosine_sum);
}

double largestPlanarVariance(const double covariance[3][3])
{
  const double a = covariance[0][0];
  const double d = covariance[1][1];
  const double b = 0.5 * (covariance[0][1] + covariance[1][0]);
  const double discriminant = std::sqrt(std::max((a - d) * (a - d) + 4.0 * b * b, 0.0));
  return std::max(0.5 * (a + d + discriminant), 0.0);
}

bool finitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

// 終端の基準速度の形（5次の滑らかな減速）。p=1で1、p=0で0、p=0付近の傾きは1です。
double terminalSpeedScale(const double progress_raw)
{
  const double p = clampValue(progress_raw, 0.0, 1.0);
  return p + 4.0 * p * p * p - 7.0 * p * p * p * p + 3.0 * p * p * p * p * p;
}

}  // namespace

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

ControllerCore::ControllerCore(const ControllerConfig & config)
: config_(config)
{
}

bool ControllerCore::setPath(
  const std::vector<Pose2D> & poses, const Pose2D & goal, std::string * error)
{
  const auto fail = [&](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  if (poses.size() < 2U) {
    return fail("経路には2点以上が必要です");
  }
  if (!finitePose(goal)) {
    return fail("目標姿勢が有限ではありません");
  }
  for (const auto & pose : poses) {
    if (!finitePose(pose)) {
      return fail("経路に有限でない姿勢が含まれています");
    }
  }

  // 1. 連続する重複点を統合します（終端の重複出力は終点を優先して許容します）。
  std::vector<Pose2D> points;
  points.reserve(poses.size());
  points.push_back(poses.front());
  for (std::size_t i = 1; i < poses.size(); ++i) {
    Pose2D next = poses[i];
    next.yaw = points.back().yaw + wrapAngle(next.yaw - points.back().yaw);
    const double distance = std::hypot(next.x - points.back().x, next.y - points.back().y);
    if (distance < kMergeDistance) {
      if (i + 1U == poses.size() && points.size() >= 2U) {
        points.back() = next;
      }
      continue;
    }
    points.push_back(next);
  }
  if (points.size() < 2U) {
    return fail("経路が1点に縮退しています");
  }

  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    cumulative[i] = cumulative[i - 1U] +
      std::hypot(points[i].x - points[i - 1U].x, points[i].y - points[i - 1U].y);
  }
  const double total = cumulative.back();
  if (!(total > 1.0e-3)) {
    return fail("経路が短すぎます(1 mm以下)");
  }

  // 2. 経路終点が目標と一致することを確認します。到着判定は目標姿勢で行います。
  if (std::hypot(points.back().x - goal.x, points.back().y - goal.y) >
    config_.path_goal_position_tolerance ||
    std::abs(wrapAngle(points.back().yaw - goal.yaw)) > config_.path_goal_yaw_tolerance)
  {
    return fail("経路終点が目標姿勢と一致しません");
  }

  // 3. 弧長で一様に再標本化します。
  const double spacing = std::max(config_.path_resample_spacing, 1.0e-3);
  const std::size_t segments =
    std::max<std::size_t>(1U, static_cast<std::size_t>(std::llround(total / spacing)));
  const double step = total / static_cast<double>(segments);
  std::vector<PathPoint> candidate(segments + 1U);
  std::size_t source = 1;
  for (std::size_t k = 0; k <= segments; ++k) {
    const double target = static_cast<double>(k) * step;
    while (source + 1U < points.size() && cumulative[source] < target) {
      ++source;
    }
    const double span = cumulative[source] - cumulative[source - 1U];
    const double ratio = span > kSmall ?
      clampValue((target - cumulative[source - 1U]) / span, 0.0, 1.0) : 0.0;
    candidate[k].s = target;
    candidate[k].pose.x = points[source - 1U].x + ratio * (points[source].x - points[source - 1U].x);
    candidate[k].pose.y = points[source - 1U].y + ratio * (points[source].y - points[source - 1U].y);
    candidate[k].pose.yaw =
      points[source - 1U].yaw + ratio * (points[source].yaw - points[source - 1U].yaw);
  }
  candidate.back().s = total;
  candidate.back().pose.x = goal.x;
  candidate.back().pose.y = goal.y;
  candidate.back().pose.yaw =
    candidate[candidate.size() - 2U].pose.yaw +
    wrapAngle(goal.yaw - candidate[candidate.size() - 2U].pose.yaw);

  // 4. 接線を求め、折返し・急な折れ点を拒否します。
  const std::size_t n = candidate.size();
  std::vector<double> heading(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t before = (i == 0U) ? 0U : i - 1U;
    const std::size_t after = (i + 1U >= n) ? n - 1U : i + 1U;
    const double dx = candidate[after].pose.x - candidate[before].pose.x;
    const double dy = candidate[after].pose.y - candidate[before].pose.y;
    const double norm = std::hypot(dx, dy);
    if (norm <= kSmall) {
      return fail("経路の接線方向を定義できません");
    }
    candidate[i].tangent_x = dx / norm;
    candidate[i].tangent_y = dy / norm;
    heading[i] = std::atan2(dy, dx);
    if (i > 0U) {
      const double change = wrapAngle(heading[i] - heading[i - 1U]);
      if (std::abs(change) > config_.maximum_heading_step) {
        std::ostringstream stream;
        stream << "経路に折返しまたは急な折れ点があります(s=" << candidate[i].s << " m)";
        return fail(stream.str());
      }
      heading[i] = heading[i - 1U] + change;
    }
  }

  // 5. 曲率・姿勢微分・局所速度上限を求めます。窓を持たせた中心差分でノイズを抑えます。
  const std::size_t window = static_cast<std::size_t>(std::max(config_.curvature_window, 1));
  const double cruise = std::min(config_.cruise_speed, config_.maximum_speed);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t before = (i >= window) ? i - window : 0U;
    const std::size_t after = std::min(n - 1U, i + window);
    const double ds = std::max(candidate[after].s - candidate[before].s, kSmall);
    candidate[i].curvature = (heading[after] - heading[before]) / ds;
    candidate[i].yaw_derivative =
      (candidate[after].pose.yaw - candidate[before].pose.yaw) / ds;
    double limit = cruise;
    limit = std::min(
      limit,
      std::sqrt(config_.maximum_lateral_acceleration / std::max(std::abs(candidate[i].curvature), 1.0e-6)));
    limit = std::min(
      limit,
      config_.maximum_yaw_rate / std::max(std::abs(candidate[i].yaw_derivative), 1.0e-6));
    candidate[i].speed_limit = limit;
  }

  // 6. 基準速度列: 終点で速度ゼロとなる5次の減速形に、制限速度の低下に対する減速度制限を加えます。
  double planning_speed = 0.0;
  for (const auto & point : candidate) {
    planning_speed = std::max(planning_speed, point.speed_limit);
  }
  const double acceleration_distance =
    0.976668 * planning_speed * planning_speed / std::max(config_.path_deceleration, kSmall);
  const double jerk_distance = std::sqrt(
    3.357422 * std::pow(planning_speed, 3.0) / std::max(config_.maximum_jerk, kSmall));
  const double snap_distance = std::cbrt(
    36.0 * std::pow(planning_speed, 4.0) / std::max(config_.maximum_snap, kSmall));
  const double terminal_distance = std::min(
    total, std::max({acceleration_distance, jerk_distance, snap_distance}));
  for (auto & point : candidate) {
    const double remaining = total - point.s;
    const double scale = terminal_distance > kSmall ?
      terminalSpeedScale(remaining / terminal_distance) : 0.0;
    point.reference_speed = std::min(point.speed_limit, planning_speed * scale);
  }
  candidate.back().reference_speed = 0.0;
  for (std::size_t i = n - 1U; i-- > 0U;) {
    const double ds = candidate[i + 1U].s - candidate[i].s;
    const double reachable = std::sqrt(
      candidate[i + 1U].reference_speed * candidate[i + 1U].reference_speed +
      2.0 * config_.path_deceleration * ds);
    candidate[i].reference_speed = std::min(candidate[i].reference_speed, reachable);
  }
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t before = (i == 0U) ? 0U : i - 1U;
    const std::size_t after = (i + 1U >= n) ? n - 1U : i + 1U;
    candidate[i].speed_derivative =
      (candidate[after].reference_speed - candidate[before].reference_speed) /
      std::max(candidate[after].s - candidate[before].s, kSmall);
  }

  trajectory_ = std::move(candidate);
  goal_ = goal;
  path_progress_ = 0.0;
  last_sequence_ = 0U;
  have_progress_ = false;
  have_step_time_ = false;
  arrived_ = false;
  timed_out_ = false;
  elapsed_ = 0.0;
  arrival_window_.clear();
  yaw_pid_initialized_ = false;
  yaw_error_integral_ = 0.0;
  filtered_yaw_error_derivative_ = 0.0;
  hardStop();
  return true;
}

void ControllerCore::clearPath()
{
  trajectory_.clear();
  arrival_window_.clear();
  path_progress_ = 0.0;
  elapsed_ = 0.0;
  arrived_ = false;
  timed_out_ = false;
  have_progress_ = false;
  have_step_time_ = false;
  yaw_pid_initialized_ = false;
  yaw_error_integral_ = 0.0;
  filtered_yaw_error_derivative_ = 0.0;
  hardStop();
}

bool ControllerCore::hasPath() const
{
  return trajectory_.size() >= 2U;
}

const std::vector<PathPoint> & ControllerCore::trajectory() const
{
  return trajectory_;
}

const Pose2D & ControllerCore::goal() const
{
  return goal_;
}

double ControllerCore::elapsedTrackingTime() const
{
  return elapsed_;
}

void ControllerCore::hardStop()
{
  world_velocity_x_ = 0.0;
  world_velocity_y_ = 0.0;
  world_acceleration_x_ = 0.0;
  world_acceleration_y_ = 0.0;
  world_jerk_x_ = 0.0;
  world_jerk_y_ = 0.0;
  omega_command_ = 0.0;
  angular_acceleration_ = 0.0;
}

void ControllerCore::resetTiming()
{
  have_step_time_ = false;
}

void ControllerCore::shapeTranslation(
  const double desired_vx, const double desired_vy, const double feedforward_ax,
  const double feedforward_ay, const double dt)
{
  double target_jerk_x = config_.velocity_shaping_gain * (desired_vx - world_velocity_x_) +
    config_.acceleration_shaping_gain * (feedforward_ax - world_acceleration_x_);
  double target_jerk_y = config_.velocity_shaping_gain * (desired_vy - world_velocity_y_) +
    config_.acceleration_shaping_gain * (feedforward_ay - world_acceleration_y_);
  limitVector(target_jerk_x, target_jerk_y, config_.maximum_jerk);
  limitJerkBeforeAccelerationBoundary(
    target_jerk_x, target_jerk_y, world_acceleration_x_, world_acceleration_y_,
    config_.maximum_acceleration, config_.maximum_snap, dt);
  double snap_x = (target_jerk_x - world_jerk_x_) / dt;
  double snap_y = (target_jerk_y - world_jerk_y_) / dt;
  limitVector(snap_x, snap_y, config_.maximum_snap);
  world_jerk_x_ += snap_x * dt;
  world_jerk_y_ += snap_y * dt;
  limitVector(world_jerk_x_, world_jerk_y_, config_.maximum_jerk);
  const double previous_acceleration_x = world_acceleration_x_;
  const double previous_acceleration_y = world_acceleration_y_;
  world_acceleration_x_ += world_jerk_x_ * dt;
  world_acceleration_y_ += world_jerk_y_ * dt;
  limitVector(world_acceleration_x_, world_acceleration_y_, config_.maximum_acceleration);
  world_jerk_x_ = (world_acceleration_x_ - previous_acceleration_x) / dt;
  world_jerk_y_ = (world_acceleration_y_ - previous_acceleration_y) / dt;
  world_velocity_x_ += world_acceleration_x_ * dt;
  world_velocity_y_ += world_acceleration_y_ * dt;
  limitVector(world_velocity_x_, world_velocity_y_, config_.maximum_speed);
}

void ControllerCore::shapeYaw(const double desired_omega, const double dt)
{
  const double desired = clampValue(desired_omega, -config_.maximum_yaw_rate, config_.maximum_yaw_rate);
  angular_acceleration_ = clampValue(
    (desired - omega_command_) / dt,
    -config_.maximum_yaw_acceleration, config_.maximum_yaw_acceleration);
  omega_command_ = clampValue(
    omega_command_ + angular_acceleration_ * dt, -config_.maximum_yaw_rate, config_.maximum_yaw_rate);
}

// 目標をゼロとして指令整形器を回し、ジャーク制限付きで減速させます。
// 座標変換には最後に有効だったyawを使います。
void ControllerCore::smoothStop(const double dt, ControllerOutput * output)
{
  if (!have_valid_yaw_) {
    hardStop();
    return;
  }
  shapeTranslation(0.0, 0.0, 0.0, 0.0, dt);
  shapeYaw(0.0, dt);
  if (std::hypot(world_velocity_x_, world_velocity_y_) < 1.0e-3 &&
    std::abs(omega_command_) < 1.0e-3)
  {
    hardStop();
  }
  const double cosine = std::cos(last_valid_yaw_);
  const double sine = std::sin(last_valid_yaw_);
  output->vx_body = cosine * world_velocity_x_ + sine * world_velocity_y_;
  output->vy_body = -sine * world_velocity_x_ + cosine * world_velocity_y_;
  output->omega = omega_command_;
}

double ControllerCore::projectProgress(const Pose2D & pose) const
{
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_progress = path_progress_;
  const double search_start = have_progress_ ?
    std::max(0.0, path_progress_ - config_.projection_search_back) : 0.0;
  const double search_end = have_progress_ ?
    std::min(trajectory_.back().s, path_progress_ + config_.projection_search_ahead) :
    trajectory_.back().s;
  for (std::size_t i = 0; i + 1U < trajectory_.size(); ++i) {
    if (trajectory_[i + 1U].s < search_start || trajectory_[i].s > search_end) {
      continue;
    }
    const double ax = trajectory_[i].pose.x;
    const double ay = trajectory_[i].pose.y;
    const double dx = trajectory_[i + 1U].pose.x - ax;
    const double dy = trajectory_[i + 1U].pose.y - ay;
    const double length_squared = std::max(dx * dx + dy * dy, kSmall);
    const double ratio = clampValue(
      ((pose.x - ax) * dx + (pose.y - ay) * dy) / length_squared, 0.0, 1.0);
    const double px = ax + ratio * dx;
    const double py = ay + ratio * dy;
    const double distance_squared = (pose.x - px) * (pose.x - px) + (pose.y - py) * (pose.y - py);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_progress = trajectory_[i].s + ratio * (trajectory_[i + 1U].s - trajectory_[i].s);
    }
  }
  return clampValue(best_progress, search_start, search_end);
}

PathPoint ControllerCore::interpolate(const double progress) const
{
  if (progress <= 0.0) {
    return trajectory_.front();
  }
  if (progress >= trajectory_.back().s) {
    return trajectory_.back();
  }
  const auto upper = std::upper_bound(
    trajectory_.begin(), trajectory_.end(), progress,
    [](const double value, const PathPoint & point) {return value < point.s;});
  const std::size_t after = static_cast<std::size_t>(std::distance(trajectory_.begin(), upper));
  const std::size_t before = after - 1U;
  const PathPoint & a = trajectory_[before];
  const PathPoint & b = trajectory_[after];
  const double ratio = (progress - a.s) / (b.s - a.s);
  const auto lerp = [ratio](double u, double v) {return u + ratio * (v - u);};

  PathPoint result;
  result.s = progress;
  result.pose.x = lerp(a.pose.x, b.pose.x);
  result.pose.y = lerp(a.pose.y, b.pose.y);
  result.pose.yaw = lerp(a.pose.yaw, b.pose.yaw);
  result.tangent_x = lerp(a.tangent_x, b.tangent_x);
  result.tangent_y = lerp(a.tangent_y, b.tangent_y);
  const double tangent_norm = std::hypot(result.tangent_x, result.tangent_y);
  if (tangent_norm > kSmall) {
    result.tangent_x /= tangent_norm;
    result.tangent_y /= tangent_norm;
  } else {
    result.tangent_x = a.tangent_x;
    result.tangent_y = a.tangent_y;
  }
  result.curvature = lerp(a.curvature, b.curvature);
  result.yaw_derivative = lerp(a.yaw_derivative, b.yaw_derivative);
  result.speed_limit = lerp(a.speed_limit, b.speed_limit);
  result.reference_speed = lerp(a.reference_speed, b.reference_speed);
  result.speed_derivative = lerp(a.speed_derivative, b.speed_derivative);
  return result;
}

ControllerOutput ControllerCore::step(
  const double now_seconds, const LocalizationSample & localization, const bool motion_authority,
  const bool environment_valid, const bool too_close)
{
  ControllerOutput output;
  output.path_progress = path_progress_;
  output.timed_out = timed_out_;

  if (!motion_authority) {
    hardStop();
    resetTiming();
    arrival_window_.clear();
    output.state = "DISABLED";
    output.reason = "NO_MOTION_AUTHORITY";
    return output;
  }
  if (!hasPath()) {
    hardStop();
    resetTiming();
    output.state = "WAIT_PATH";
    output.reason = "NO_PATH";
    return output;
  }
  if (arrived_) {
    hardStop();
    output.arrived = true;
    output.state = "ARRIVED";
    output.reason = "CONFIDENCE_REGION_INSIDE";
    output.arrival_decision = "ARRIVED";
    output.arrival_reason = "LATCHED";
    return output;
  }

  // 制御周期の検査。範囲外なら周期が信頼できないため即時にゼロ指令とします。
  double dt = config_.nominal_period;
  if (have_step_time_) {
    dt = now_seconds - last_step_time_;
    if (!(dt > 0.0) || dt > config_.maximum_period) {
      hardStop();
      arrival_window_.clear();
      have_step_time_ = false;
      output.state = "TIMING_HOLD";
      output.reason = "CONTROL_PERIOD_INVALID";
      return output;
    }
  }
  last_step_time_ = now_seconds;
  have_step_time_ = true;
  elapsed_ += dt;

  if (!environment_valid) {
    hardStop();
    arrival_window_.clear();
    output.state = "ENVIRONMENT_HOLD";
    output.reason = "ENVIRONMENT_STALE";
    return output;
  }
  if (too_close) {
    hardStop();
    arrival_window_.clear();
    output.state = "SAFETY_STOP";
    output.reason = "PILE_TOO_CLOSE";
    return output;
  }

  if (finitePose(localization.pose)) {
    output.position_error = std::hypot(goal_.x - localization.pose.x, goal_.y - localization.pose.y);
    output.yaw_error = wrapAngle(goal_.yaw - localization.pose.yaw);
  }

  // 制限時間を超えた場合は、ジャーク制限付きで停止します。
  if (!timed_out_ && elapsed_ > config_.maximum_tracking_time) {
    timed_out_ = true;
    output.timeout_pulse = true;
  }
  if (timed_out_) {
    arrival_window_.clear();
    smoothStop(dt, &output);
    output.timed_out = true;
    output.state = "TIMEOUT";
    output.reason = "TRACKING_TIME_EXCEEDED";
    return output;
  }

  if (!localization.healthy || !finitePose(localization.pose)) {
    arrival_window_.clear();
    smoothStop(dt, &output);
    output.state = "LOCALIZATION_HOLD";
    output.reason = "LOCALIZATION_INVALID";
    return output;
  }
  last_valid_yaw_ = localization.pose.yaw;
  have_valid_yaw_ = true;

  // 再位置合わせを検出したら、進捗を全経路への投影から作り直し、到着窓を破棄します。
  if (have_reset_counter_ && localization.reset_counter != last_reset_counter_) {
    have_progress_ = false;
    arrival_window_.clear();
    yaw_pid_initialized_ = false;
  }
  last_reset_counter_ = localization.reset_counter;
  have_reset_counter_ = true;

  // 進捗推定: 融合速度で予測し、新しい観測が来たときだけ投影位置で補正します。
  const bool first_progress = !have_progress_;
  const bool new_localization = first_progress || localization.sequence != last_sequence_;
  const double projected_progress = projectProgress(localization.pose);
  if (first_progress) {
    path_progress_ = projected_progress;
    have_progress_ = true;
  } else {
    const PathPoint previous_reference = interpolate(path_progress_);
    const double cosine = std::cos(localization.pose.yaw);
    const double sine = std::sin(localization.pose.yaw);
    double along_track_speed = 0.0;
    if (std::isfinite(localization.vx_body) && std::isfinite(localization.vy_body)) {
      const double measured_world_vx = cosine * localization.vx_body - sine * localization.vy_body;
      const double measured_world_vy = sine * localization.vx_body + cosine * localization.vy_body;
      along_track_speed = clampValue(
        previous_reference.tangent_x * measured_world_vx +
        previous_reference.tangent_y * measured_world_vy,
        -config_.maximum_progress_rate, config_.maximum_progress_rate);
    }
    const double predicted_progress = path_progress_ + along_track_speed * dt;
    double candidate_progress = predicted_progress;
    if (new_localization) {
      const double correction_weight = dt / (config_.progress_correction_time_constant + dt);
      candidate_progress += correction_weight * (projected_progress - predicted_progress);
    }
    const double maximum_change = config_.maximum_progress_rate * dt;
    path_progress_ += clampValue(candidate_progress - path_progress_, -maximum_change, maximum_change);
    path_progress_ = clampValue(path_progress_, 0.0, trajectory_.back().s);
  }
  if (new_localization) {
    last_sequence_ = localization.sequence;
  }

  const PathPoint reference = interpolate(path_progress_);
  output.reference_yaw = reference.pose.yaw;
  const double remaining = std::max(trajectory_.back().s - path_progress_, 0.0);
  const double goal_dx = goal_.x - localization.pose.x;
  const double goal_dy = goal_.y - localization.pose.y;

  // 経路上の速度: 基準速度列(終点で速度ゼロ)。目標を行き過ぎた場合だけ低速で戻ります。
  double path_speed = std::max(reference.reference_speed, 0.0);
  double tangential_feedforward = path_speed * reference.speed_derivative;
  if (config_.terminal_minimum_speed > 0.0 &&
    remaining > 0.5 * config_.arrival_position_tolerance)
  {
    if (path_speed < config_.terminal_minimum_speed) {
      path_speed = config_.terminal_minimum_speed;
      tangential_feedforward = 0.0;  // 速度を引き上げている間は、減速の先行項と矛盾させない
    }
  }
  const double recovery_gain = config_.path_deceleration / std::max(config_.cruise_speed, kSmall);
  const double goal_along_error = reference.tangent_x * goal_dx + reference.tangent_y * goal_dy;
  if (remaining <= config_.arrival_position_tolerance &&
    goal_along_error < -config_.arrival_position_tolerance)
  {
    path_speed = -std::min(
      config_.maximum_longitudinal_recovery_speed, recovery_gain * std::abs(goal_along_error));
    tangential_feedforward = 0.0;
  }

  // 位置補正は経路法線方向だけに適用します。
  const double error_x = reference.pose.x - localization.pose.x;
  const double error_y = reference.pose.y - localization.pose.y;
  const double along_error = error_x * reference.tangent_x + error_y * reference.tangent_y;
  double normal_error_x = (error_x - along_error * reference.tangent_x) * config_.position_gain;
  double normal_error_y = (error_y - along_error * reference.tangent_y) * config_.position_gain;
  limitVector(normal_error_x, normal_error_y, config_.maximum_correction_speed);

  double desired_vx = path_speed * reference.tangent_x + normal_error_x;
  double desired_vy = path_speed * reference.tangent_y + normal_error_y;
  limitVector(desired_vx, desired_vy, config_.maximum_speed);

  // 加速度の先行入力: 曲率による法線加速度と、基準速度の変化による接線加速度。
  const double normal_x = -reference.tangent_y;
  const double normal_y = reference.tangent_x;
  const double curvature_acceleration =
    config_.curvature_feedforward_gain * reference.curvature * path_speed * path_speed;
  double feedforward_ax = curvature_acceleration * normal_x + tangential_feedforward * reference.tangent_x;
  double feedforward_ay = curvature_acceleration * normal_y + tangential_feedforward * reference.tangent_y;
  limitVector(feedforward_ax, feedforward_ay, config_.maximum_acceleration);

  // 姿勢制御: P・I・低域フィルタ付きD。角速度の先行項は使いません（MATLAB第20段階）。
  const double yaw_error = wrapAngle(reference.pose.yaw - localization.pose.yaw);
  double yaw_error_derivative = 0.0;
  if (yaw_pid_initialized_) {
    const double raw_derivative = wrapAngle(yaw_error - previous_yaw_error_) / dt;
    const double filter_gain = dt / (config_.yaw_derivative_filter_time_constant + dt);
    yaw_error_derivative =
      filtered_yaw_error_derivative_ + filter_gain * (raw_derivative - filtered_yaw_error_derivative_);
  }
  const double previous_integral = yaw_error_integral_;
  double candidate_integral = clampValue(
    previous_integral + yaw_error * dt, -config_.yaw_integral_limit, config_.yaw_integral_limit);
  const double proportional = config_.yaw_kp * yaw_error;
  const double derivative = config_.yaw_kd * yaw_error_derivative;
  double unlimited_omega = proportional + config_.yaw_ki * candidate_integral + derivative;
  if (std::abs(unlimited_omega) > config_.maximum_yaw_rate && unlimited_omega * yaw_error > 0.0) {
    candidate_integral = previous_integral;  // 飽和中は積分を止める（アンチワインドアップ）
    unlimited_omega = proportional + config_.yaw_ki * candidate_integral + derivative;
  }
  const double desired_omega = clampValue(unlimited_omega, -config_.maximum_yaw_rate, config_.maximum_yaw_rate);

  shapeTranslation(desired_vx, desired_vy, feedforward_ax, feedforward_ay, dt);
  shapeYaw(desired_omega, dt);
  yaw_error_integral_ = candidate_integral;
  previous_yaw_error_ = yaw_error;
  filtered_yaw_error_derivative_ = yaw_error_derivative;
  yaw_pid_initialized_ = true;

  const double cosine = std::cos(localization.pose.yaw);
  const double sine = std::sin(localization.pose.yaw);
  output.vx_body = cosine * world_velocity_x_ + sine * world_velocity_y_;
  output.vy_body = -sine * world_velocity_x_ + cosine * world_velocity_y_;
  output.omega = omega_command_;
  output.path_progress = path_progress_;
  output.state = "TRACKING";
  output.reason = "OK";

  // 到着判定
  const double command_speed = std::hypot(output.vx_body, output.vy_body);
  if (new_localization) {
    updateArrivalWindow(localization, command_speed, std::abs(output.omega));
  }
  const bool command_stopped =
    command_speed <= config_.arrival_command_speed_tolerance &&
    std::abs(output.omega) <= config_.arrival_command_yaw_rate_tolerance;
  double arrival_position_error = output.position_error;
  double arrival_yaw_error = std::abs(output.yaw_error);
  bool arrival_now = false;
  if (command_stopped) {
    arrival_now = evaluateArrival(
      arrival_position_error, arrival_yaw_error, output.arrival_decision, output.arrival_reason);
  } else {
    output.arrival_decision = "COLLECTING";
    output.arrival_reason = "COMMAND_MOVING";
  }
  if (arrival_now) {
    arrived_ = true;
    output.arrival_pulse = true;
    output.arrived = true;
    hardStop();
    output.vx_body = 0.0;
    output.vy_body = 0.0;
    output.omega = 0.0;
    output.state = "ARRIVED";
    output.reason = "CONFIDENCE_REGION_INSIDE";
  } else if (output.arrival_decision == "INDETERMINATE") {
    output.state = "ARRIVAL_HOLD";
    output.reason = output.arrival_reason;
  }
  return output;
}

void ControllerCore::updateArrivalWindow(
  const LocalizationSample & localization, const double command_speed, const double command_yaw_rate)
{
  const bool command_stopped =
    command_speed <= config_.arrival_command_speed_tolerance &&
    command_yaw_rate <= config_.arrival_command_yaw_rate_tolerance;
  if (!command_stopped) {
    arrival_window_.clear();
    return;
  }
  ArrivalSample sample;
  sample.stamp = localization.stamp;
  sample.pose = localization.pose;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      sample.covariance[row][column] = localization.covariance[row][column];
    }
  }
  // 時刻が逆戻りした観測が混ざった場合は、窓を作り直します。
  if (!arrival_window_.empty() && sample.stamp < arrival_window_.back().stamp) {
    arrival_window_.clear();
  }
  arrival_window_.push_back(sample);
  while (!arrival_window_.empty() &&
    sample.stamp - arrival_window_.front().stamp > config_.arrival_window_seconds)
  {
    arrival_window_.pop_front();
  }
}

bool ControllerCore::evaluateArrival(
  double & position_error, double & yaw_error, std::string & decision, std::string & reason) const
{
  if (arrival_window_.size() < static_cast<std::size_t>(config_.arrival_minimum_samples)) {
    decision = "COLLECTING";
    reason = "WINDOW_NOT_READY";
    return false;
  }
  const double span = arrival_window_.back().stamp - arrival_window_.front().stamp;
  if (span < config_.arrival_minimum_window_seconds) {
    decision = "COLLECTING";
    reason = "WINDOW_NOT_READY";
    return false;
  }

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> yaws;
  xs.reserve(arrival_window_.size());
  ys.reserve(arrival_window_.size());
  yaws.reserve(arrival_window_.size());
  double reported_position_std = 0.0;
  double reported_yaw_std = 0.0;
  for (const auto & sample : arrival_window_) {
    xs.push_back(sample.pose.x);
    ys.push_back(sample.pose.y);
    yaws.push_back(sample.pose.yaw);
    reported_position_std = std::max(
      reported_position_std, std::sqrt(largestPlanarVariance(sample.covariance)));
    reported_yaw_std = std::max(
      reported_yaw_std, std::sqrt(std::max(sample.covariance[2][2], 0.0)));
  }

  const double center_x = median(xs);
  const double center_y = median(ys);
  const double center_yaw = circularMean(yaws);
  position_error = std::hypot(goal_.x - center_x, goal_.y - center_y);
  yaw_error = std::abs(wrapAngle(goal_.yaw - center_yaw));
  const double window_position_std = std::max(robustSigma(xs), robustSigma(ys));
  std::vector<double> yaw_deviations;
  yaw_deviations.reserve(yaws.size());
  for (const double yaw : yaws) {
    yaw_deviations.push_back(wrapAngle(yaw - center_yaw));
  }
  const double window_yaw_std = robustSigma(yaw_deviations);
  const double position_confidence =
    config_.confidence_sigma * std::max(reported_position_std, window_position_std);
  const double yaw_confidence =
    config_.confidence_sigma * std::max(reported_yaw_std, window_yaw_std);

  const std::size_t group_count = std::max<std::size_t>(arrival_window_.size() / 4U, 1U);
  std::vector<double> first_times;
  std::vector<double> last_times;
  std::vector<double> first_xs;
  std::vector<double> first_ys;
  std::vector<double> first_yaws;
  std::vector<double> last_xs;
  std::vector<double> last_ys;
  std::vector<double> last_yaws;
  for (std::size_t index = 0; index < group_count; ++index) {
    const auto & first = arrival_window_[index];
    const auto & last = arrival_window_[arrival_window_.size() - group_count + index];
    first_times.push_back(first.stamp);
    last_times.push_back(last.stamp);
    first_xs.push_back(first.pose.x);
    first_ys.push_back(first.pose.y);
    first_yaws.push_back(first.pose.yaw);
    last_xs.push_back(last.pose.x);
    last_ys.push_back(last.pose.y);
    last_yaws.push_back(last.pose.yaw);
  }
  const double drift_duration = std::max(median(last_times) - median(first_times), kSmall);
  const double position_drift_rate = std::hypot(
    median(last_xs) - median(first_xs), median(last_ys) - median(first_ys)) / drift_duration;
  const double yaw_drift_rate =
    std::abs(wrapAngle(circularMean(last_yaws) - circularMean(first_yaws))) / drift_duration;

  if (position_confidence > config_.arrival_position_tolerance ||
    yaw_confidence > config_.arrival_yaw_tolerance)
  {
    decision = "INDETERMINATE";
    reason = "UNCERTAINTY_TOO_LARGE";
    return false;
  }
  if (position_drift_rate > config_.maximum_position_drift_rate ||
    yaw_drift_rate > config_.maximum_yaw_drift_rate)
  {
    decision = "INDETERMINATE";
    reason = "LOCALIZATION_DRIFT";
    return false;
  }
  if (position_error + position_confidence <= config_.arrival_position_tolerance &&
    yaw_error + yaw_confidence <= config_.arrival_yaw_tolerance)
  {
    decision = "ARRIVED";
    reason = "CONFIDENCE_REGION_INSIDE";
    return true;
  }
  if (position_error - position_confidence > config_.arrival_position_tolerance ||
    yaw_error - yaw_confidence > config_.arrival_yaw_tolerance)
  {
    decision = "NOT_ARRIVED";
    reason = "CONFIDENCE_REGION_OUTSIDE";
    return false;
  }
  decision = "INDETERMINATE";
  reason = "CONFIDENCE_REGION_OVERLAP";
  return false;
}

std::vector<Pose2D> makeStraightPath(const Pose2D & start, const Pose2D & goal, const double spacing)
{
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const std::size_t segment_count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(distance / std::max(spacing, 0.001))));
  std::vector<Pose2D> path;
  path.reserve(segment_count + 1U);
  const double yaw_change = wrapAngle(goal.yaw - start.yaw);
  for (std::size_t i = 0; i <= segment_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(segment_count);
    const double smooth_ratio = ratio * ratio * ratio * (10.0 - 15.0 * ratio + 6.0 * ratio * ratio);
    Pose2D pose;
    pose.x = start.x + ratio * (goal.x - start.x);
    pose.y = start.y + ratio * (goal.y - start.y);
    pose.yaw = start.yaw + smooth_ratio * yaw_change;
    path.push_back(pose);
  }
  return path;
}

}  // namespace pickup_approach_controller
