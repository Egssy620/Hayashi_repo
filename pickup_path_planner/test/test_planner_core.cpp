#include "pickup_path_planner/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace pickup_path_planner
{
namespace
{

// 山の裾点を、半径radiusの円周上の指定角度範囲に疑似観測として生成します。
std::vector<Vec2> makeSkirt(
  const Vec2 & center, const double radius, const double from_deg, const double to_deg,
  const int count, const double noise_sigma, const unsigned seed = 1U)
{
  std::mt19937 generator(seed);
  std::normal_distribution<double> noise(0.0, noise_sigma);
  std::vector<Vec2> points;
  for (int i = 0; i < count; ++i) {
    const double angle = (from_deg + (to_deg - from_deg) * i / std::max(count - 1, 1)) * kPi / 180.0;
    const double r = radius + (noise_sigma > 0.0 ? noise(generator) : 0.0);
    points.push_back({center.x + r * std::cos(angle), center.y + r * std::sin(angle)});
  }
  return points;
}

ObservedPileModel makeModel(const Vec2 & center, const double radius)
{
  ObservedPileModel model;
  std::string error;
  const auto points = makeSkirt(center, radius, 0.0, 330.0, 120, 0.005);
  EXPECT_TRUE(buildObservedPileModel(points, PlannerConfig{}, &model, &error));
  return model;
}

TEST(PileModel, FitsCenterAndRadiusFromPartialArc)
{
  const Vec2 center{4.0, -2.0};
  const auto points = makeSkirt(center, 1.25, 200.0, 360.0, 80, 0.005);
  ObservedPileModel model;
  std::string error;
  ASSERT_TRUE(buildObservedPileModel(points, PlannerConfig{}, &model, &error));
  EXPECT_NEAR(model.center.x, center.x, 0.05);
  EXPECT_NEAR(model.center.y, center.y, 0.05);
  EXPECT_NEAR(model.fitted_radius, 1.25, 0.05);
  EXPECT_GE(model.conservative_radius, model.fitted_radius + 0.01 - 1e-9);
  EXPECT_NEAR(model.visible_arc * 180.0 / kPi, 160.0, 5.0);
}

TEST(PileModel, RejectsInsufficientObservation)
{
  ObservedPileModel model;
  std::string error;
  // 可視円弧が40 degしかない観測は、山領域の推定に使わない
  EXPECT_FALSE(buildObservedPileModel(
      makeSkirt({0, 0}, 1.25, 0.0, 40.0, 60, 0.002), PlannerConfig{}, &model, &error));
  // 点数不足
  EXPECT_FALSE(buildObservedPileModel(
      makeSkirt({0, 0}, 1.25, 0.0, 300.0, 5, 0.0), PlannerConfig{}, &model, &error));
  // 半径が異常（巨大な壁など）
  EXPECT_FALSE(buildObservedPileModel(
      makeSkirt({0, 0}, 6.0, 0.0, 300.0, 100, 0.002), PlannerConfig{}, &model, &error));
}

TEST(PileModel, ToleratesOutliers)
{
  auto points = makeSkirt({0.0, 0.0}, 1.25, 0.0, 330.0, 120, 0.004);
  points.push_back({3.0, 3.0});
  points.push_back({-2.5, 0.5});
  ObservedPileModel model;
  std::string error;
  ASSERT_TRUE(buildObservedPileModel(points, PlannerConfig{}, &model, &error));
  EXPECT_NEAR(model.fitted_radius, 1.25, 0.08);
}

TEST(Geometry, DistanceCollisionAndClearance)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const RobotGeometry geometry;
  // 前端が山の円周から0.30 m手前（円周は半径約1.26 m）
  const double front_x = -(model.conservative_radius + 0.30);
  const Pose2D approach{front_x - geometry.body_length / 2.0, 0.0, 0.0};
  EXPECT_NEAR(bodyPolygonDistance(approach, geometry, model, 10.0), 0.30, 0.01);
  // 重なる姿勢
  EXPECT_EQ(bodyPolygonDistance(Pose2D{0.0, 0.0, 0.0}, geometry, model, 10.0), 0.0);
  EXPECT_EQ(bodyPolygonDistance(Pose2D{-2.0, 0.0, 0.0}, geometry, model, 10.0), 0.0);
  // 遠方は大きな正の値
  EXPECT_GT(bodyPolygonDistance(Pose2D{-20.0, 0.0, 0.0}, geometry, model, 1.0), 1.0);
  // 車体の側面が円に向く姿勢
  const Pose2D side{0.0, -(model.conservative_radius + 0.20 + geometry.body_width / 2.0), 0.0};
  EXPECT_NEAR(bodyPolygonDistance(side, geometry, model, 10.0), 0.20, 0.01);
}

TEST(Planner, UsesDirectPathWhenClear)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const RobotGeometry geometry;
  const Pose2D start{-12.0, 8.0, 0.0};
  const Pose2D goal{-12.0, 12.0, 0.3};
  const auto result = planPilePath(start, goal, model, geometry, PlannerConfig{});
  ASSERT_TRUE(result.status == PlanStatus::kOk);
  EXPECT_EQ(result.path.geometry, "DIRECT");
  EXPECT_NEAR(result.path.length, 4.0, 1e-6);
}

TEST(Planner, DetoursAroundPileAndKeepsClearance)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const RobotGeometry geometry;
  const PlannerConfig config;
  const Pose2D start{-7.0, 0.0, 0.0};
  const Pose2D goal{7.0, 0.0, 0.0};
  const auto result = planPilePath(start, goal, model, geometry, config);
  ASSERT_TRUE(result.status == PlanStatus::kOk);
  const auto & path = result.path;
  EXPECT_NE(path.geometry, "DIRECT");
  EXPECT_GT(path.length, 14.0);
  EXPECT_LT(path.length, 30.0);
  // 端点は入力どおり
  EXPECT_DOUBLE_EQ(path.poses.front().x, start.x);
  EXPECT_DOUBLE_EQ(path.poses.back().x, goal.x);
  EXPECT_NEAR(path.poses.back().yaw, goal.yaw, 1e-9);
  // 標本間隔は出力間隔以下で、重複点がない
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double segment = std::hypot(
      path.poses[i].x - path.poses[i - 1].x, path.poses[i].y - path.poses[i - 1].y);
    EXPECT_GT(segment, 1e-4);
    EXPECT_LE(segment, 1.3 * config.output_step);  // 末尾の端数は直前の区間へ吸収する
  }
  // 独立に再計算しても、全姿勢で公称余裕より小さくならない（始終端付近の遷移を除く）
  double minimum_mid = 1e9;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double d = bodyPolygonDistance(path.poses[i], geometry, model, 5.0);
    EXPECT_GT(d, 0.0);
    if (path.distance[i] > 0.5 && path.length - path.distance[i] > 0.5) {
      minimum_mid = std::min(minimum_mid, d);
    }
  }
  EXPECT_GE(minimum_mid, config.body_clearance + config.tracking_error_allowance - 0.011);
}

TEST(Planner, RejectsGoalTooCloseToPile)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const RobotGeometry geometry;
  // 前端が山から5 cmしかない目標
  const double front_x = -(model.conservative_radius + 0.05);
  const Pose2D goal{front_x - geometry.body_length / 2.0, 0.0, 0.0};
  const auto result = planPilePath(Pose2D{-12.0, 4.0, 0.0}, goal, model, geometry, PlannerConfig{});
  EXPECT_TRUE(result.status == PlanStatus::kGoalInCollision);
}

TEST(Planner, RejectsStartInsidePileRegion)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const auto result = planPilePath(
    Pose2D{-2.0, 0.0, 0.0}, Pose2D{-12.0, 4.0, 0.0}, model, RobotGeometry{}, PlannerConfig{});
  EXPECT_TRUE(result.status == PlanStatus::kStartInCollision);
}

TEST(Planner, ApproachGoalNearPileFromStart)
{
  // 実運用に近い形: 目標は山の観測側で、前端が公称余裕0.15 mの位置
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const RobotGeometry geometry;
  const double front_x = -(model.conservative_radius + 0.15);
  const Pose2D goal{front_x - geometry.body_length / 2.0, 0.0, 0.0};
  const Pose2D start{-9.0, 6.0, 0.4};
  const auto result = planPilePath(start, goal, model, geometry, PlannerConfig{});
  ASSERT_TRUE(result.status == PlanStatus::kOk);
  EXPECT_NEAR(result.path.goal_clearance, 0.15, 0.011);
  EXPECT_GT(result.path.minimum_clearance, 0.0);
}

TEST(Planner, RejectsCoincidentStartAndGoal)
{
  const ObservedPileModel model = makeModel({0.0, 0.0}, 1.25);
  const auto result = planPilePath(
    Pose2D{-8.0, 0.0, 0.0}, Pose2D{-8.0, 0.001, 1.0}, model, RobotGeometry{}, PlannerConfig{});
  EXPECT_TRUE(result.status == PlanStatus::kInvalidInput);
}

}  // namespace
}  // namespace pickup_path_planner
