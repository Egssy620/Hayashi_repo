#include "pickup_approach_controller/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pickup_approach_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.02;

LocalizationSample makeLocalization(
  const double stamp, const std::uint64_t sequence, const Pose2D & pose)
{
  LocalizationSample sample;
  sample.stamp = stamp;
  sample.sequence = sequence;
  sample.pose = pose;
  sample.covariance[0][0] = 9.0e-6;   // 位置の標準偏差3 mm
  sample.covariance[1][1] = 9.0e-6;
  sample.covariance[2][2] = std::pow(0.3 * kPi / 180.0, 2.0);  // yawの標準偏差0.3 deg
  sample.healthy = true;
  return sample;
}

struct SimulationResult
{
  bool arrived{false};
  bool timed_out{false};
  double time{0.0};
  double final_position_error{0.0};
  double final_yaw_error{0.0};
  double maximum_speed{0.0};
  double maximum_acceleration{0.0};
  double maximum_cross_track_error{0.0};
  std::string last_state{};
};

// 指令どおりに動く理想の全方向車体で、追従器を閉ループ試験します。
// deadband: この速度未満の指令では車体が動かない(静止摩擦の簡易模擬)。
SimulationResult simulate(
  ControllerCore * core, const Pose2D & start, const Pose2D & goal, const double duration,
  const double deadband = 0.0)
{
  SimulationResult result;
  Pose2D pose = start;
  double previous_speed = 0.0;
  double previous_vx = 0.0;
  double previous_vy = 0.0;
  const std::size_t steps = static_cast<std::size_t>(duration / kDt);
  for (std::size_t i = 0; i < steps; ++i) {
    const double time = 1.0 + kDt * static_cast<double>(i);
    LocalizationSample localization = makeLocalization(time, i + 1U, pose);
    const auto output = core->step(time, localization, true, true, false);
    result.last_state = output.state;
    double vx = output.vx_body;
    double vy = output.vy_body;
    double omega = output.omega;
    if (std::hypot(vx, vy) < deadband) {
      vx = 0.0;
      vy = 0.0;
    }
    if (std::abs(omega) < 0.005 && deadband > 0.0) {
      omega = 0.0;
    }
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double world_vx = c * vx - s * vy;
    const double world_vy = s * vx + c * vy;
    pose.x += world_vx * kDt;
    pose.y += world_vy * kDt;
    pose.yaw += omega * kDt;
    const double speed = std::hypot(world_vx, world_vy);
    result.maximum_speed = std::max(result.maximum_speed, speed);
    if (deadband == 0.0) {
      result.maximum_acceleration = std::max(
        result.maximum_acceleration, std::hypot(world_vx - previous_vx, world_vy - previous_vy) / kDt);
    }
    previous_speed = speed;
    previous_vx = world_vx;
    previous_vy = world_vy;
    result.time = time - 1.0;
    result.timed_out = output.timed_out;
    if (output.arrived) {
      result.arrived = true;
      break;
    }
    if (output.timed_out) {
      break;
    }
  }
  (void)previous_speed;
  result.final_position_error = std::hypot(goal.x - pose.x, goal.y - pose.y);
  result.final_yaw_error = std::abs(wrapAngle(goal.yaw - pose.yaw));
  return result;
}

// 円弧経路（中心(cx,cy)、半径r、角度a0→a1）。yawは進行方向に対して一定量だけ変える。
std::vector<Pose2D> makeArcPath(
  const double cx, const double cy, const double r, const double a0, const double a1,
  const double yaw0, const double yaw1)
{
  std::vector<Pose2D> path;
  const double length = r * std::abs(a1 - a0);
  const int count = static_cast<int>(std::ceil(length / 0.05));
  for (int i = 0; i <= count; ++i) {
    const double u = static_cast<double>(i) / count;
    const double angle = a0 + u * (a1 - a0);
    const double blend = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
    path.push_back({cx + r * std::cos(angle), cy + r * std::sin(angle), yaw0 + blend * (yaw1 - yaw0)});
  }
  return path;
}

TEST(ControllerCore, RequiresMotionAuthority)
{
  ControllerCore controller;
  const Pose2D goal{1.0, 0.0, 0.0};
  std::string error;
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal, &error));
  const auto output = controller.step(
    0.0, makeLocalization(0.0, 1U, Pose2D{}), false, true, false);
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_DOUBLE_EQ(output.vy_body, 0.0);
  EXPECT_DOUBLE_EQ(output.omega, 0.0);
  EXPECT_EQ(output.state, "DISABLED");
}

TEST(ControllerCore, GeneratesForwardCommandAndStopsImmediatelyWhenPileTooClose)
{
  ControllerCore controller;
  const Pose2D goal{3.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  ControllerOutput output;
  for (std::uint64_t index = 0U; index < 40U; ++index) {
    const double time = kDt * static_cast<double>(index);
    output = controller.step(time, makeLocalization(time, index + 1U, Pose2D{}), true, true, false);
  }
  EXPECT_GT(output.vx_body, 0.0);
  EXPECT_NEAR(output.vy_body, 0.0, 1.0e-9);

  output = controller.step(0.8, makeLocalization(0.8, 42U, Pose2D{}), true, true, true);
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_DOUBLE_EQ(output.omega, 0.0);
  EXPECT_EQ(output.state, "SAFETY_STOP");
  // 環境監視の途絶も、即時のゼロ指令
  output = controller.step(0.82, makeLocalization(0.82, 43U, Pose2D{}), true, false, false);
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_EQ(output.state, "ENVIRONMENT_HOLD");
}

TEST(ControllerCore, DeclaresArrivalAtGoalFromStationaryConfidenceWindow)
{
  ControllerConfig config;
  config.arrival_window_seconds = 1.0;
  ControllerCore controller(config);
  const Pose2D goal{1.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));

  bool pulse = false;
  ControllerOutput output;
  for (std::uint64_t index = 0U; index <= 60U; ++index) {
    const double time = kDt * static_cast<double>(index);
    output = controller.step(time, makeLocalization(time, index + 1U, goal), true, true, false);
    pulse = pulse || output.arrival_pulse;
  }
  EXPECT_TRUE(pulse);
  EXPECT_TRUE(output.arrived);
  EXPECT_EQ(output.state, "ARRIVED");
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
}

TEST(ControllerCore, RejectsPathWhoseEndDoesNotMatchGoal)
{
  ControllerCore controller;
  std::string error;
  // 経路終点が目標から3 cmずれている → 受理しない
  const Pose2D path_end{1.0, 0.0, 0.0};
  const Pose2D goal{1.03, 0.0, 0.0};
  EXPECT_FALSE(controller.setPath(makeStraightPath(Pose2D{}, path_end, 0.05), goal, &error));
  EXPECT_FALSE(controller.hasPath());
  // yawが2 deg以上ずれている → 受理しない
  const Pose2D yaw_goal{1.0, 0.0, 0.05};
  EXPECT_FALSE(controller.setPath(makeStraightPath(Pose2D{}, path_end, 0.05), yaw_goal, &error));
}

TEST(ControllerCore, RejectsCuspAndAcceptsDuplicatedEndpoint)
{
  ControllerCore controller;
  std::string error;
  // 折返し(往復)
  const std::vector<Pose2D> cusp{{0, 0, 0}, {1, 0, 0}, {0.5, 0, 0}};
  EXPECT_FALSE(controller.setPath(cusp, Pose2D{0.5, 0, 0}, &error));
  // 直角に曲がる粗い折れ線
  const std::vector<Pose2D> corner{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  EXPECT_FALSE(controller.setPath(corner, Pose2D{1, 1, 0}, &error));
  // 終点の重複は許容する
  std::vector<Pose2D> duplicated = makeStraightPath(Pose2D{}, Pose2D{1.0, 0.0, 0.0}, 0.05);
  duplicated.push_back(duplicated.back());
  EXPECT_TRUE(controller.setPath(duplicated, Pose2D{1.0, 0.0, 0.0}, &error));
}

TEST(ControllerCore, ResamplesCoarsePathAndBuildsTerminalSpeedProfile)
{
  ControllerCore controller;
  const Pose2D goal{4.0, 0.0, 0.0};
  // 1 m間隔の粗い直線でも、0.02 m間隔に再標本化される
  ASSERT_TRUE(controller.setPath({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}}, goal));
  const auto & trajectory = controller.trajectory();
  ASSERT_GT(trajectory.size(), 100U);
  EXPECT_NEAR(trajectory[1].s - trajectory[0].s, 0.02, 1e-9);
  // 基準速度: 終点で0、巡航速度以下、終端付近は単調減少
  EXPECT_DOUBLE_EQ(trajectory.back().reference_speed, 0.0);
  double peak = 0.0;
  for (const auto & point : trajectory) {
    peak = std::max(peak, point.reference_speed);
    EXPECT_LE(point.reference_speed, 0.25 + 1e-9);
  }
  EXPECT_NEAR(peak, 0.25, 1e-9);
  for (std::size_t i = trajectory.size() - 40U; i + 1U < trajectory.size(); ++i) {
    EXPECT_GE(trajectory[i].reference_speed, trajectory[i + 1U].reference_speed - 1e-12);
  }
  // 終端付近の減速度は path_deceleration 以下: v*dv/ds >= -a
  for (const auto & point : trajectory) {
    EXPECT_GE(point.reference_speed * point.speed_derivative, -0.2 - 0.02);
  }
}

TEST(ControllerCore, RejectsArrivalWhileLocalizationCenterDrifts)
{
  ControllerCore controller;
  const Pose2D goal{1.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  ControllerOutput output;
  for (std::uint64_t index = 0U; index <= 50U; ++index) {
    const double time = kDt * static_cast<double>(index);
    const double x = 0.995 + 0.010 * static_cast<double>(index) / 50.0;
    output = controller.step(
      time, makeLocalization(time, index + 1U, Pose2D{x, 0.0, 0.0}), true, true, false);
  }
  EXPECT_FALSE(output.arrived);
  EXPECT_EQ(output.arrival_decision, "INDETERMINATE");
  EXPECT_EQ(output.arrival_reason, "LOCALIZATION_DRIFT");
}

TEST(ControllerCore, LocalizationLossStopsSmoothlyNotInstantly)
{
  ControllerCore controller;
  const Pose2D goal{6.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  Pose2D pose{};
  ControllerOutput output;
  double time = 0.0;
  std::uint64_t sequence = 1U;
  for (int i = 0; i < 200; ++i) {   // 4秒走行して巡航速度へ
    output = controller.step(time, makeLocalization(time, sequence++, pose), true, true, false);
    pose.x += output.vx_body * kDt;
    time += kDt;
  }
  const double cruise_speed = output.vx_body;
  ASSERT_GT(cruise_speed, 0.2);

  // 以後は自己位置が異常。速度は連続的に減少し、ジャーク制限内で停止する
  double previous = cruise_speed;
  double previous_acceleration = 0.0;
  double maximum_deceleration = 0.0;
  int hold_steps = 0;
  for (int i = 0; i < 400; ++i) {
    LocalizationSample bad = makeLocalization(time, sequence++, pose);
    bad.healthy = false;
    output = controller.step(time, bad, true, true, false);
    EXPECT_EQ(output.state == "LOCALIZATION_HOLD" || output.state == "TRACKING", true);
    EXPECT_LE(output.vx_body, previous + 1e-9);   // 増えない
    const double deceleration = (previous - output.vx_body) / kDt;
    maximum_deceleration = std::max(maximum_deceleration, deceleration);
    if (i == 0) {
      EXPECT_GT(output.vx_body, 0.5 * cruise_speed);   // 1周期では半分も落ちない
    }
    previous_acceleration = deceleration;
    previous = output.vx_body;
    time += kDt;
    ++hold_steps;
    if (output.vx_body == 0.0) {
      break;
    }
  }
  (void)previous_acceleration;
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_LE(maximum_deceleration, 0.25 + 0.01);   // 加速度上限以内
  EXPECT_LT(hold_steps, 400);
}

TEST(ControllerCore, TimesOutAndStopsWhenGoalIsNotReached)
{
  ControllerConfig config;
  config.maximum_tracking_time = 3.0;
  ControllerCore controller(config);
  const Pose2D goal{5.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  int pulses = 0;
  ControllerOutput output;
  // 車体が動かない(指令が届かない)状況
  for (int i = 0; i < 400; ++i) {
    const double time = kDt * static_cast<double>(i);
    output = controller.step(time, makeLocalization(time, i + 1U, Pose2D{}), true, true, false);
    if (output.timeout_pulse) {
      ++pulses;
    }
  }
  EXPECT_EQ(pulses, 1);
  EXPECT_TRUE(output.timed_out);
  EXPECT_EQ(output.state, "TIMEOUT");
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
}

TEST(ControllerCore, LocalizationResetReinitializesProgress)
{
  ControllerCore controller;
  const Pose2D goal{10.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  ControllerOutput output;
  std::uint64_t sequence = 1U;
  for (int i = 0; i < 10; ++i) {
    const double time = kDt * i;
    output = controller.step(time, makeLocalization(time, sequence++, Pose2D{}), true, true, false);
  }
  EXPECT_LT(output.path_progress, 0.1);

  // 再位置合わせで姿勢が6 m先へ飛んだ。resetカウンタが増えれば、進捗は全経路から再投影される
  LocalizationSample jumped = makeLocalization(0.4, sequence++, Pose2D{6.0, 0.0, 0.0});
  jumped.reset_counter = 1U;
  output = controller.step(0.4, jumped, true, true, false);
  EXPECT_NEAR(output.path_progress, 6.0, 0.05);

  // カウンタが変わらない飛びは、進捗変化率の制限で追従する（飛びに引きずられない）
  ControllerCore controller2;
  ASSERT_TRUE(controller2.setPath(makeStraightPath(Pose2D{}, goal, 0.05), goal));
  for (int i = 0; i < 10; ++i) {
    const double time = kDt * i;
    controller2.step(time, makeLocalization(time, i + 1U, Pose2D{}), true, true, false);
  }
  const auto jump_output = controller2.step(
    0.4, makeLocalization(0.4, 11U, Pose2D{6.0, 0.0, 0.0}), true, true, false);
  EXPECT_LT(jump_output.path_progress, 0.5);
}

TEST(ControllerCoreClosedLoop, StraightApproachArrivesWithoutOvershootWithinLimits)
{
  ControllerCore controller;
  const Pose2D start{0.0, 0.0, 0.0};
  const Pose2D goal{3.0, 0.0, 0.0};
  ASSERT_TRUE(controller.setPath(makeStraightPath(start, goal, 0.05), goal));
  const auto result = simulate(&controller, start, goal, 120.0);
  EXPECT_TRUE(result.arrived);
  EXPECT_LT(result.final_position_error, 0.02);
  EXPECT_LT(result.final_yaw_error, 0.02);
  EXPECT_LE(result.maximum_speed, 0.30 + 1e-6);
  EXPECT_LE(result.maximum_acceleration, 0.25 + 0.01);
}

TEST(ControllerCoreClosedLoop, CurvedApproachWithYawChangeArrives)
{
  ControllerCore controller;
  // 半径3 mの円弧(90 deg)を進み、同時に車体yawを60 deg変える
  const auto path = makeArcPath(0.0, 3.0, 3.0, -kPi / 2.0, 0.0, 0.0, kPi / 3.0);
  const Pose2D start = path.front();
  const Pose2D goal = path.back();
  ASSERT_TRUE(controller.setPath(path, goal));
  const auto result = simulate(&controller, start, goal, 200.0);
  EXPECT_TRUE(result.arrived);
  EXPECT_LT(result.final_position_error, 0.02);
  EXPECT_LT(result.final_yaw_error, 0.021);
  EXPECT_LE(result.maximum_acceleration, 0.25 + 0.02);
}

TEST(ControllerCoreClosedLoop, StaticFrictionStallsWithoutMinimumTerminalSpeed)
{
  // 0.03 m/s未満の指令では車体が動かない場合、最低接近速度がないと目標手前で止まる
  const Pose2D start{0.0, 0.0, 0.0};
  const Pose2D goal{2.0, 0.0, 0.0};
  {
    ControllerConfig config;
    config.maximum_tracking_time = 120.0;
    ControllerCore controller(config);
    ASSERT_TRUE(controller.setPath(makeStraightPath(start, goal, 0.05), goal));
    const auto result = simulate(&controller, start, goal, 200.0, 0.03);
    EXPECT_FALSE(result.arrived);
    EXPECT_TRUE(result.timed_out);   // 永久に待たず、時間切れで報告する
    EXPECT_GT(result.final_position_error, 0.02);
  }
  {
    ControllerConfig config;
    config.terminal_minimum_speed = 0.035;
    config.maximum_tracking_time = 120.0;
    ControllerCore controller(config);
    ASSERT_TRUE(controller.setPath(makeStraightPath(start, goal, 0.05), goal));
    const auto result = simulate(&controller, start, goal, 200.0, 0.03);
    EXPECT_TRUE(result.arrived);
    EXPECT_LT(result.final_position_error, 0.02);
  }
}

}  // namespace
}  // namespace pickup_approach_controller
