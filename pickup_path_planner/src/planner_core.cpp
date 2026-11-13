#include "pickup_path_planner/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <utility>

namespace pickup_path_planner
{
namespace
{

constexpr double kSmall = 1.0e-9;
constexpr double kClearanceTolerance = 5.0e-3;  // 目標側の離隔判定の許容 [m]
constexpr double kNearInterest = 1.0;           // この離隔を超える姿勢は厳密計算を省く [m]

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

// 5次の滑らかな0→1遷移です（始終端で速度・加速度がゼロ）。
double quinticBlend(const double u_raw)
{
  const double u = clampValue(u_raw, 0.0, 1.0);
  return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
}

std::vector<double> linspace(const double first, const double last, const int count)
{
  std::vector<double> values(static_cast<std::size_t>(std::max(count, 1)));
  if (count <= 1) {
    values[0] = first;
    return values;
  }
  for (int i = 0; i < count; ++i) {
    values[static_cast<std::size_t>(i)] =
      first + (last - first) * static_cast<double>(i) / static_cast<double>(count - 1);
  }
  return values;
}

// 3x3連立一次方程式をピボット選択付きのガウス消去法で解きます。
bool solve3x3(double a[3][3], double b[3], double x[3])
{
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    for (int row = column + 1; row < 3; ++row) {
      if (std::abs(a[row][column]) > std::abs(a[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot][column]) < 1.0e-12) {
      return false;
    }
    if (pivot != column) {
      for (int k = 0; k < 3; ++k) {
        std::swap(a[pivot][k], a[column][k]);
      }
      std::swap(b[pivot], b[column]);
    }
    for (int row = column + 1; row < 3; ++row) {
      const double factor = a[row][column] / a[column][column];
      for (int k = column; k < 3; ++k) {
        a[row][k] -= factor * a[column][k];
      }
      b[row] -= factor * b[column];
    }
  }
  for (int row = 2; row >= 0; --row) {
    double sum = b[row];
    for (int k = row + 1; k < 3; ++k) {
      sum -= a[row][k] * x[k];
    }
    x[row] = sum / a[row][row];
  }
  return true;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return (n % 2U == 1U) ? values[n / 2U] : 0.5 * (values[n / 2U - 1U] + values[n / 2U]);
}

// 重み付き最小二乗の円近似です。中心化して条件数を改善します。
bool weightedCircleFit(
  const std::vector<Vec2> & points, const Vec2 & mean, const std::vector<double> & weights,
  double * cx, double * cy, double * radius)
{
  double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  double rhs[3] = {0, 0, 0};
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double x = points[i].x - mean.x;
    const double y = points[i].y - mean.y;
    const double row[3] = {2.0 * x, 2.0 * y, 1.0};
    const double b = x * x + y * y;
    const double w2 = weights[i] * weights[i];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        m[r][c] += w2 * row[r] * row[c];
      }
      rhs[r] += w2 * row[r] * b;
    }
  }
  double solution[3] = {0, 0, 0};
  if (!solve3x3(m, rhs, solution)) {
    return false;
  }
  *cx = mean.x + solution[0];
  *cy = mean.y + solution[1];
  const double radius_squared = solution[2] + solution[0] * solution[0] + solution[1] * solution[1];
  if (!(radius_squared > 0.0)) {
    return false;
  }
  *radius = std::sqrt(radius_squared);
  return true;
}

struct ArcStats
{
  double visible_arc{0.0};
  double start{0.0};
  double span{0.0};
};

ArcStats visibleArcAround(const std::vector<Vec2> & points, const Vec2 & center)
{
  std::vector<double> angles;
  angles.reserve(points.size());
  for (const auto & p : points) {
    double a = std::atan2(p.y - center.y, p.x - center.x);
    if (a < 0.0) {
      a += 2.0 * kPi;
    }
    angles.push_back(a);
  }
  std::sort(angles.begin(), angles.end());
  double largest_gap = 0.0;
  std::size_t gap_index = 0;
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const double next = (i + 1U < angles.size()) ? angles[i + 1U] : angles[0] + 2.0 * kPi;
    const double gap = next - angles[i];
    if (gap > largest_gap) {
      largest_gap = gap;
      gap_index = i;
    }
  }
  ArcStats stats;
  stats.visible_arc = 2.0 * kPi - largest_gap;
  stats.start = std::fmod(angles[(gap_index + 1U) % angles.size()], 2.0 * kPi);
  stats.span = stats.visible_arc;
  return stats;
}

bool segmentsIntersect(
  const double ax, const double ay, const double bx, const double by,
  const double cx, const double cy, const double dx, const double dy)
{
  const auto cross = [](double ux, double uy, double vx, double vy) {return ux * vy - uy * vx;};
  const double d1 = cross(bx - ax, by - ay, cx - ax, cy - ay);
  const double d2 = cross(bx - ax, by - ay, dx - ax, dy - ay);
  const double d3 = cross(dx - cx, dy - cy, ax - cx, ay - cy);
  const double d4 = cross(dx - cx, dy - cy, bx - cx, by - cy);
  if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
    ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)))
  {
    return true;
  }
  const auto on_segment = [](double px, double py, double qx, double qy, double rx, double ry) {
      return std::min(px, qx) - kSmall <= rx && rx <= std::max(px, qx) + kSmall &&
             std::min(py, qy) - kSmall <= ry && ry <= std::max(py, qy) + kSmall;
    };
  if (std::abs(d1) <= kSmall && on_segment(ax, ay, bx, by, cx, cy)) {return true;}
  if (std::abs(d2) <= kSmall && on_segment(ax, ay, bx, by, dx, dy)) {return true;}
  if (std::abs(d3) <= kSmall && on_segment(cx, cy, dx, dy, ax, ay)) {return true;}
  if (std::abs(d4) <= kSmall && on_segment(cx, cy, dx, dy, bx, by)) {return true;}
  return false;
}

double pointSegmentDistance(
  const double px, const double py, const double ax, const double ay,
  const double bx, const double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  const double ratio = length_squared > kSmall ?
    clampValue(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0) : 0.0;
  return std::hypot(px - (ax + ratio * dx), py - (ay + ratio * dy));
}

// 経路候補の粗い点列です（始点から終点まで、重複点を含み得る）。
struct DenseCandidate
{
  std::vector<Vec2> xy{};
  std::string geometry{};
  std::string direction{};
  double parameter{0.0};
  bool valid{false};
};

// 点列を弧長で一様再標本化し、yawを5次遷移で割り当てます。
bool finalizePath(
  const std::vector<Vec2> & dense, const Pose2D & start, const Pose2D & goal,
  const double step, PlannedPath * path)
{
  if (dense.size() < 2U || !(step > 0.0)) {
    return false;
  }
  std::vector<double> cumulative(dense.size(), 0.0);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    cumulative[i] = cumulative[i - 1U] +
      std::hypot(dense[i].x - dense[i - 1U].x, dense[i].y - dense[i - 1U].y);
  }
  const double total = cumulative.back();
  if (!(total > 1.0e-6)) {
    return false;
  }
  std::vector<double> targets;
  const std::size_t count = static_cast<std::size_t>(std::floor(total / step));
  targets.reserve(count + 2U);
  for (std::size_t k = 0; k <= count; ++k) {
    targets.push_back(static_cast<double>(k) * step);
  }
  if (total - targets.back() < 0.25 * step) {
    targets.back() = total;
  } else {
    targets.push_back(total);
  }

  path->poses.clear();
  path->distance.clear();
  path->poses.reserve(targets.size());
  path->distance.reserve(targets.size());
  const double yaw_change = wrapAngle(goal.yaw - start.yaw);
  std::size_t segment = 1;
  for (const double target : targets) {
    while (segment + 1U < dense.size() && cumulative[segment] < target) {
      ++segment;
    }
    const double span = cumulative[segment] - cumulative[segment - 1U];
    const double ratio = span > kSmall ?
      clampValue((target - cumulative[segment - 1U]) / span, 0.0, 1.0) : 0.0;
    Pose2D pose;
    pose.x = dense[segment - 1U].x + ratio * (dense[segment].x - dense[segment - 1U].x);
    pose.y = dense[segment - 1U].y + ratio * (dense[segment].y - dense[segment - 1U].y);
    pose.yaw = start.yaw + yaw_change * quinticBlend(target / total);
    path->poses.push_back(pose);
    path->distance.push_back(target);
  }
  // 端点は入力どおりに固定します（丸め誤差で目標からずれないようにします）。
  path->poses.front().x = start.x;
  path->poses.front().y = start.y;
  path->poses.front().yaw = start.yaw;
  path->poses.back().x = goal.x;
  path->poses.back().y = goal.y;
  path->poses.back().yaw = start.yaw + yaw_change;
  path->length = total;
  return true;
}

DenseCandidate makeStraight(const Pose2D & start, const Pose2D & goal)
{
  DenseCandidate c;
  c.xy = {{start.x, start.y}, {goal.x, goal.y}};
  c.geometry = "DIRECT";
  c.valid = true;
  return c;
}

void appendLine(
  std::vector<Vec2> * xy, const Vec2 & a, const Vec2 & b, const double step, const bool skip_last)
{
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const int n = std::max(1, static_cast<int>(std::ceil(length / step)));
  const int last = skip_last ? n - 1 : n;
  for (int i = 0; i <= last; ++i) {
    const double r = static_cast<double>(i) / static_cast<double>(n);
    xy->push_back({a.x + r * (b.x - a.x), a.y + r * (b.y - a.y)});
  }
}

// 山中心の円へ接する直線→円弧→接する直線です。direction>0が反時計回り(CCW)です。
DenseCandidate makeTangentArc(
  const Pose2D & start, const Pose2D & goal, const Vec2 & center, const double radius,
  const int direction, const double step)
{
  DenseCandidate c;
  const double sx = start.x - center.x;
  const double sy = start.y - center.y;
  const double gx = goal.x - center.x;
  const double gy = goal.y - center.y;
  const double start_distance = std::hypot(sx, sy);
  const double goal_distance = std::hypot(gx, gy);
  if (radius <= 0.0 || start_distance <= radius || goal_distance <= radius) {
    return c;
  }
  const double start_angle = std::atan2(sy, sx);
  const double goal_angle = std::atan2(gy, gx);
  const double start_offset = std::acos(radius / start_distance);
  const double goal_offset = std::acos(radius / goal_distance);
  double tangent_start_angle;
  double tangent_goal_angle;
  double arc_angle;
  const auto positive_mod = [](double a, double m) {
      double r = std::fmod(a, m);
      if (r < 0.0) {r += m;}
      return r;
    };
  if (direction > 0) {
    tangent_start_angle = start_angle + start_offset;
    tangent_goal_angle = goal_angle - goal_offset;
    arc_angle = positive_mod(tangent_goal_angle - tangent_start_angle, 2.0 * kPi);
  } else {
    tangent_start_angle = start_angle - start_offset;
    tangent_goal_angle = goal_angle + goal_offset;
    arc_angle = -positive_mod(tangent_start_angle - tangent_goal_angle, 2.0 * kPi);
  }
  const Vec2 tangent_start{
    center.x + radius * std::cos(tangent_start_angle),
    center.y + radius * std::sin(tangent_start_angle)};
  const Vec2 tangent_goal{
    center.x + radius * std::cos(tangent_goal_angle),
    center.y + radius * std::sin(tangent_goal_angle)};
  const double arc_length = radius * std::abs(arc_angle);
  appendLine(&c.xy, {start.x, start.y}, tangent_start, step, true);
  const int arc_count = std::max(1, static_cast<int>(std::ceil(arc_length / step)));
  for (int i = 0; i < arc_count; ++i) {
    const double angle = tangent_start_angle +
      arc_angle * static_cast<double>(i) / static_cast<double>(arc_count);
    c.xy.push_back({center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)});
  }
  appendLine(&c.xy, tangent_goal, {goal.x, goal.y}, step, false);
  c.geometry = "TANGENT_ARC_TANGENT";
  c.direction = direction > 0 ? "CCW" : "CW";
  c.parameter = radius;
  c.valid = true;
  return c;
}

// 山中心まわりの極座標で、半径を線形補間＋正弦の膨らみで変える滑らかな曲線です。
DenseCandidate makePolarCurve(
  const Pose2D & start, const Pose2D & goal, const Vec2 & center, const double bulge,
  const double minimum_radius, const int direction, const double step)
{
  DenseCandidate c;
  const double sx = start.x - center.x;
  const double sy = start.y - center.y;
  const double gx = goal.x - center.x;
  const double gy = goal.y - center.y;
  const double start_radius = std::hypot(sx, sy);
  const double goal_radius = std::hypot(gx, gy);
  const double start_angle = std::atan2(sy, sx);
  const double goal_angle = std::atan2(gy, gx);
  const auto positive_mod = [](double a, double m) {
      double r = std::fmod(a, m);
      if (r < 0.0) {r += m;}
      return r;
    };
  const double angle_change = direction > 0 ?
    positive_mod(goal_angle - start_angle, 2.0 * kPi) :
    -positive_mod(start_angle - goal_angle, 2.0 * kPi);
  const double estimated = std::abs(angle_change) * std::max(start_radius, goal_radius) +
    std::abs(goal_radius - start_radius) + 2.0 * std::abs(bulge);
  const int count = std::max(30, static_cast<int>(std::ceil(estimated / (0.5 * step))));
  c.xy.reserve(static_cast<std::size_t>(count) + 1U);
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    const double radius = (1.0 - t) * start_radius + t * goal_radius + bulge * std::sin(kPi * t);
    if (radius < minimum_radius) {
      c.xy.clear();
      return c;
    }
    const double angle = start_angle + t * angle_change;
    c.xy.push_back({center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)});
  }
  c.xy.front() = {start.x, start.y};
  c.xy.back() = {goal.x, goal.y};
  c.geometry = "SMOOTH_POLAR_CURVE";
  c.direction = direction > 0 ? "CCW" : "CW";
  c.parameter = bulge;
  c.valid = true;
  return c;
}

bool withinObservedSector(
  const PlannedPath & path, const ObservedPileModel & model, const double margin)
{
  const double allowed_start = model.visible_arc_start - margin;
  const double allowed_span = std::min(2.0 * kPi, model.visible_arc_span + 2.0 * margin);
  for (const auto & pose : path.poses) {
    double angle = std::fmod(std::atan2(pose.y - model.center.y, pose.x - model.center.x), 2.0 * kPi);
    if (angle < 0.0) {angle += 2.0 * kPi;}
    double progress = std::fmod(angle - allowed_start, 2.0 * kPi);
    if (progress < 0.0) {progress += 2.0 * kPi;}
    if (progress > allowed_span + 1.0e-12) {
      return false;
    }
  }
  return true;
}

// 経路上の各姿勢での車体離隔を計算します。
std::vector<double> clearanceAlongPath(
  const PlannedPath & path, const RobotGeometry & geometry, const ObservedPileModel & model)
{
  std::vector<double> clearance(path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    clearance[i] = bodyPolygonDistance(path.poses[i], geometry, model, kNearInterest);
  }
  return clearance;
}

bool hasTrackingClearance(
  const PlannedPath & path, const std::vector<double> & clearance,
  const double body_clearance, const double allowance, const double transition)
{
  const double start_clearance = std::min(clearance.front(), body_clearance);
  const double safe_transition = std::max(transition, kSmall);
  for (std::size_t i = 0; i < clearance.size(); ++i) {
    const double s = path.distance[i];
    const double start_weight = quinticBlend(s / safe_transition);
    const double base = start_clearance + (body_clearance - start_clearance) * start_weight;
    const double endpoint_distance = std::min(s, path.length - s);
    const double allowance_weight = quinticBlend(endpoint_distance / safe_transition);
    const double required = base + allowance * allowance_weight;
    if (clearance[i] < required - kClearanceTolerance) {
      return false;
    }
  }
  return true;
}

bool pathIsCollisionFree(const std::vector<double> & clearance)
{
  for (const double value : clearance) {
    if (!(value > 0.0)) {
      return false;
    }
  }
  return true;
}

struct SafeCandidate
{
  double length{0.0};
  DenseCandidate dense{};
};

struct SearchContext
{
  const Pose2D & start;
  const Pose2D & goal;
  const ObservedPileModel & model;
  const RobotGeometry & geometry;
  const PlannerConfig & config;
  std::vector<SafeCandidate> safe{};
  int evaluated{0};
  double best_length{std::numeric_limits<double>::infinity()};
};

// 探索用の標本間隔で候補を検査します。安全なら記録し、最短更新ならtrueを返します。
bool evaluateCandidate(SearchContext * context, const DenseCandidate & candidate)
{
  if (!candidate.valid) {
    return false;
  }
  PlannedPath path;
  if (!finalizePath(candidate.xy, context->start, context->goal, context->config.search_step, &path)) {
    return false;
  }
  ++context->evaluated;
  const auto clearance = clearanceAlongPath(path, context->geometry, context->model);
  const bool body_clear = pathIsCollisionFree(clearance) && hasTrackingClearance(
    path, clearance, context->config.body_clearance, 0.0, context->config.transition_distance);
  if (!body_clear) {
    return false;
  }
  if (!withinObservedSector(path, context->model, context->config.path_visible_sector_margin)) {
    return false;
  }
  if (!hasTrackingClearance(
      path, clearance, context->config.body_clearance, context->config.tracking_error_allowance,
      context->config.transition_distance))
  {
    return false;
  }
  SafeCandidate record;
  record.length = path.length;
  record.dense = candidate;
  context->safe.push_back(record);
  if (path.length < context->best_length) {
    context->best_length = path.length;
    return true;
  }
  return false;
}

// 値列の中で、最良値の隣接2点の範囲を返します。
std::pair<double, double> neighbourBounds(const std::vector<double> & values, const std::size_t index)
{
  const std::size_t lower = index > 0U ? index - 1U : 0U;
  const std::size_t upper = std::min(values.size() - 1U, index + 1U);
  return {values[lower], values[upper]};
}

// 一方向（CCWまたはCW）の迂回候補を探索します。MATLABのfind_shortest_pile_detourと同じ手順です。
void searchDetour(
  SearchContext * context, const int direction, const double minimum_radius,
  const double maximum_radius)
{
  const auto & start = context->start;
  const auto & goal = context->goal;
  const auto & model = context->model;
  const double step = context->config.search_step;
  context->best_length = std::numeric_limits<double>::infinity();  // 方向ごとに最良値を初期化

  bool have_best = false;
  double best_parameter = 0.0;
  std::size_t best_index = 0;

  const auto search_values = [&](const std::vector<double> & values, const bool tangent) {
      for (std::size_t i = 0; i < values.size(); ++i) {
        const DenseCandidate candidate = tangent ?
          makeTangentArc(start, goal, model.center, values[i], direction, step) :
          makePolarCurve(start, goal, model.center, values[i], minimum_radius, direction, step);
        if (evaluateCandidate(context, candidate)) {
          have_best = true;
          best_parameter = values[i];
          best_index = i;
        }
      }
      if (have_best) {
        double best_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < values.size(); ++i) {
          const double d = std::abs(values[i] - best_parameter);
          if (d < best_distance) {
            best_distance = d;
            best_index = i;
          }
        }
      }
    };

  if (maximum_radius > minimum_radius) {
    auto radii = linspace(minimum_radius, maximum_radius, 28);
    search_values(radii, true);
    if (have_best) {
      for (int iteration = 0; iteration < 2; ++iteration) {
        const auto bounds = neighbourBounds(radii, best_index);
        radii = linspace(bounds.first, bounds.second, 15);
        search_values(radii, true);
      }
    }
  }

  const double start_radius = std::hypot(start.x - model.center.x, start.y - model.center.y);
  const double goal_radius = std::hypot(goal.x - model.center.x, goal.y - model.center.y);
  const double min_radius = std::min(start_radius, goal_radius);
  const double body_radius = std::hypot(
    context->geometry.body_length / 2.0, context->geometry.body_width / 2.0);
  const double safe_radius = minimum_radius + body_radius + context->config.orbit_extra_clearance;
  const double lower_bulge = -0.95 * std::max(0.0, min_radius - minimum_radius);
  const double upper_bulge = std::max(0.0, safe_radius - min_radius) + 0.75;
  auto bulges = linspace(lower_bulge, upper_bulge, 42);
  have_best = false;
  search_values(bulges, false);
  if (have_best) {
    for (int iteration = 0; iteration < 2; ++iteration) {
      const auto bounds = neighbourBounds(bulges, best_index);
      bulges = linspace(bounds.first, bounds.second, 17);
      search_values(bulges, false);
    }
  }
}

}  // namespace

std::string toString(const PlanStatus status)
{
  switch (status) {
    case PlanStatus::kOk: return "OK";
    case PlanStatus::kInvalidInput: return "INVALID_INPUT";
    case PlanStatus::kInsufficientObservation: return "ADDITIONAL_SCAN_REQUIRED";
    case PlanStatus::kStartInCollision: return "START_IN_COLLISION";
    case PlanStatus::kGoalInCollision: return "GOAL_CLEARANCE_VIOLATION";
    case PlanStatus::kNoCollisionFreePath: return "NO_COLLISION_FREE_PATH";
    case PlanStatus::kValidationFailed: return "FINAL_VALIDATION_FAILED";
    case PlanStatus::kPathTooLong: return "PATH_TOO_LONG";
  }
  return "UNKNOWN";
}

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool buildObservedPileModel(
  const std::vector<Vec2> & skirt_points, const PlannerConfig & config,
  ObservedPileModel * model, std::string * error)
{
  const auto fail = [&](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  std::vector<Vec2> points;
  points.reserve(skirt_points.size());
  for (const auto & p : skirt_points) {
    if (std::isfinite(p.x) && std::isfinite(p.y)) {
      points.push_back(p);
    }
  }
  if (static_cast<int>(points.size()) < std::max(config.minimum_skirt_points, 3)) {
    std::ostringstream stream;
    stream << "裾点が不足しています: " << points.size() << "点 (必要" <<
      config.minimum_skirt_points << "点)";
    return fail(stream.str());
  }

  Vec2 mean;
  for (const auto & p : points) {
    mean.x += p.x;
    mean.y += p.y;
  }
  mean.x /= static_cast<double>(points.size());
  mean.y /= static_cast<double>(points.size());

  std::vector<double> weights(points.size(), 1.0);
  double cx = 0.0;
  double cy = 0.0;
  double radius = 0.0;
  if (!weightedCircleFit(points, mean, weights, &cx, &cy, &radius)) {
    return fail("円近似が特異です（裾点が一直線に近い）");
  }
  for (int iteration = 0; iteration < std::max(config.circle_iterations, 1); ++iteration) {
    if (!weightedCircleFit(points, mean, weights, &cx, &cy, &radius)) {
      return fail("円近似が特異です（裾点が一直線に近い）");
    }
    std::vector<double> residual(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      residual[i] = std::hypot(points[i].x - cx, points[i].y - cy) - radius;
    }
    const double center_residual = median(residual);
    std::vector<double> deviation(residual.size());
    for (std::size_t i = 0; i < residual.size(); ++i) {
      deviation[i] = std::abs(residual[i] - center_residual);
    }
    const double scale = 1.4826 * median(deviation) + 1.0e-12;
    const double threshold = 1.5 * scale;
    for (std::size_t i = 0; i < residual.size(); ++i) {
      weights[i] = std::min(1.0, threshold / std::max(std::abs(residual[i]), 1.0e-12));
    }
  }

  const Vec2 center{cx, cy};
  double square_sum = 0.0;
  std::vector<double> positive_residuals;
  positive_residuals.reserve(points.size());
  for (const auto & p : points) {
    const double residual = std::hypot(p.x - cx, p.y - cy) - radius;
    square_sum += residual * residual;
    positive_residuals.push_back(std::max(0.0, residual));
  }
  const double rms = std::sqrt(square_sum / static_cast<double>(points.size()));
  // 妥当性の判定には、外れ値の影響を受けにくいMAD由来の散らばりを使います。
  std::vector<double> fit_residuals;
  fit_residuals.reserve(points.size());
  for (const auto & p : points) {
    fit_residuals.push_back(std::hypot(p.x - cx, p.y - cy) - radius);
  }
  const double residual_median = median(fit_residuals);
  for (auto & value : fit_residuals) {
    value = std::abs(value - residual_median);
  }
  const double robust_sigma = 1.4826 * median(fit_residuals);
  const ArcStats arc = visibleArcAround(points, center);

  if (radius > config.maximum_pile_radius || radius < config.minimum_pile_radius) {
    std::ostringstream stream;
    stream << "推定した山の半径が範囲外です: " << radius << " m";
    return fail(stream.str());
  }
  if (robust_sigma > config.maximum_circle_rms_residual) {
    std::ostringstream stream;
    stream << "円近似の残差が大きすぎます: " << robust_sigma << " m";
    return fail(stream.str());
  }
  if (arc.visible_arc < config.minimum_visible_arc) {
    std::ostringstream stream;
    stream << "可視の裾円弧が不足しています: " << arc.visible_arc * 180.0 / kPi << " deg";
    return fail(stream.str());
  }

  std::sort(positive_residuals.begin(), positive_residuals.end());
  const std::size_t coverage_index = std::min(
    positive_residuals.size() - 1U,
    static_cast<std::size_t>(std::max(
      1.0, std::ceil(config.residual_coverage * static_cast<double>(positive_residuals.size()))) -
    1.0));
  const double conservative_radius =
    radius + positive_residuals[coverage_index] + config.radial_uncertainty;

  ObservedPileModel result;
  result.center = center;
  result.fitted_radius = radius;
  result.conservative_radius = conservative_radius;
  result.rms_residual = rms;
  result.visible_arc = arc.visible_arc;
  result.visible_arc_start = arc.start;
  result.visible_arc_span = arc.span;
  result.point_count = points.size();
  const int samples = std::max(config.polygon_samples, 16);
  result.polygon.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double angle = -kPi + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
    result.polygon.push_back({
        center.x + conservative_radius * std::cos(angle),
        center.y + conservative_radius * std::sin(angle)});
  }
  *model = std::move(result);
  return true;
}

double bodyPolygonDistance(
  const Pose2D & pose, const RobotGeometry & geometry, const ObservedPileModel & model,
  const double interest)
{
  const double half_length = geometry.body_length / 2.0;
  const double half_width = geometry.body_width / 2.0;
  const double half_diagonal = std::hypot(half_length, half_width);
  const double center_distance = std::hypot(pose.x - model.center.x, pose.y - model.center.y);
  const double lower_bound = center_distance - model.conservative_radius - half_diagonal;
  if (lower_bound > interest) {
    return lower_bound;
  }

  const std::size_t n = model.polygon.size();
  static thread_local std::vector<Vec2> local;
  local.resize(n);
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = model.polygon[i].x - pose.x;
    const double dy = model.polygon[i].y - pose.y;
    local[i] = {c * dx + s * dy, -s * dx + c * dy};
  }

  // 衝突判定: 頂点が矩形内、辺が矩形辺と交差、矩形中心が多角形内
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(local[i].x) <= half_length && std::abs(local[i].y) <= half_width) {
      return 0.0;
    }
  }
  const double corners[4][2] = {
    {half_length, half_width}, {half_length, -half_width},
    {-half_length, -half_width}, {-half_length, half_width}};
  bool center_inside = true;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 & a = local[i];
    const Vec2 & b = local[(i + 1U) % n];
    for (int k = 0; k < 4; ++k) {
      const double * p = corners[k];
      const double * q = corners[(k + 1) % 4];
      if (segmentsIntersect(a.x, a.y, b.x, b.y, p[0], p[1], q[0], q[1])) {
        return 0.0;
      }
    }
    // 反時計回り多角形では、内部の点は全辺の左側にあります。
    const double cross = (b.x - a.x) * (0.0 - a.y) - (b.y - a.y) * (0.0 - a.x);
    if (cross < 0.0) {
      center_inside = false;
    }
  }
  if (center_inside) {
    return 0.0;
  }

  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = std::max(std::abs(local[i].x) - half_length, 0.0);
    const double dy = std::max(std::abs(local[i].y) - half_width, 0.0);
    minimum = std::min(minimum, std::hypot(dx, dy));
  }
  for (int k = 0; k < 4; ++k) {
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2 & a = local[i];
      const Vec2 & b = local[(i + 1U) % n];
      minimum = std::min(
        minimum, pointSegmentDistance(corners[k][0], corners[k][1], a.x, a.y, b.x, b.y));
    }
  }
  return minimum;
}

PlanResult planPilePath(
  const Pose2D & start, const Pose2D & goal, const ObservedPileModel & model,
  const RobotGeometry & geometry, const PlannerConfig & config)
{
  PlanResult result;
  const auto finish = [&](const PlanStatus status, const std::string & message) {
      result.status = status;
      result.message = message;
      return result;
    };
  const bool finite_input =
    std::isfinite(start.x) && std::isfinite(start.y) && std::isfinite(start.yaw) &&
    std::isfinite(goal.x) && std::isfinite(goal.y) && std::isfinite(goal.yaw) &&
    std::isfinite(model.center.x) && std::isfinite(model.center.y) && model.polygon.size() >= 8U;
  if (!finite_input) {
    return finish(PlanStatus::kInvalidInput, "入力に有限でない値、または山モデルの欠落があります");
  }
  const double straight = std::hypot(goal.x - start.x, goal.y - start.y);
  if (straight < 0.01) {
    return finish(PlanStatus::kInvalidInput, "始点と目標の位置が1 cm以内で一致しています");
  }

  const double start_clearance = bodyPolygonDistance(start, geometry, model, kNearInterest);
  if (!(start_clearance > 0.0)) {
    return finish(PlanStatus::kStartInCollision, "現在の車体が観測した山の領域と重なっています");
  }
  const double goal_clearance = bodyPolygonDistance(goal, geometry, model, kNearInterest);
  if (goal_clearance < config.body_clearance - kClearanceTolerance) {
    std::ostringstream stream;
    stream << "目標姿勢の山との離隔 " << goal_clearance << " m が公称余裕 " <<
      config.body_clearance << " m 未満です";
    return finish(PlanStatus::kGoalInCollision, stream.str());
  }

  SearchContext context{start, goal, model, geometry, config};
  evaluateCandidate(&context, makeStraight(start, goal));

  const bool direct_safe = !context.safe.empty();
  if (!direct_safe) {
    const double pile_radius = model.conservative_radius;
    const double start_radius = std::hypot(start.x - model.center.x, start.y - model.center.y);
    const double goal_radius = std::hypot(goal.x - model.center.x, goal.y - model.center.y);
    const double planning_clearance = config.body_clearance + config.tracking_error_allowance;
    const double minimum_radius = pile_radius + planning_clearance;
    const double maximum_radius = std::min(start_radius, goal_radius) - 1.0e-4;
    searchDetour(&context, 1, minimum_radius, maximum_radius);
    searchDetour(&context, -1, minimum_radius, maximum_radius);
  }
  if (context.safe.empty()) {
    return finish(
      PlanStatus::kNoCollisionFreePath, "直線と両回りの迂回候補がすべて山または余裕条件に抵触します");
  }

  std::sort(
    context.safe.begin(), context.safe.end(),
    [](const SafeCandidate & a, const SafeCandidate & b) {return a.length < b.length;});

  // 最短から順に、細かい標本間隔で再生成して再検証します。
  std::size_t attempts = 0;
  for (const auto & candidate : context.safe) {
    if (++attempts > 8U) {
      break;
    }
    DenseCandidate fine;
    if (candidate.dense.geometry == "DIRECT") {
      fine = makeStraight(start, goal);
    } else if (candidate.dense.geometry == "TANGENT_ARC_TANGENT") {
      fine = makeTangentArc(
        start, goal, model.center, candidate.dense.parameter,
        candidate.dense.direction == "CCW" ? 1 : -1, config.output_step);
    } else {
      const double pile_radius = model.conservative_radius;
      const double minimum_radius =
        pile_radius + config.body_clearance + config.tracking_error_allowance;
      fine = makePolarCurve(
        start, goal, model.center, candidate.dense.parameter, minimum_radius,
        candidate.dense.direction == "CCW" ? 1 : -1, config.output_step);
    }
    PlannedPath validation;
    if (!fine.valid || !finalizePath(fine.xy, start, goal, config.validation_step, &validation)) {
      continue;
    }
    const auto clearance = clearanceAlongPath(validation, geometry, model);
    const bool ok = pathIsCollisionFree(clearance) &&
      hasTrackingClearance(
      validation, clearance, config.body_clearance, 0.0, config.transition_distance) &&
      hasTrackingClearance(
      validation, clearance, config.body_clearance, config.tracking_error_allowance,
      config.transition_distance);
    if (!ok) {
      continue;
    }
    PlannedPath output;
    if (!finalizePath(fine.xy, start, goal, config.output_step, &output)) {
      continue;
    }
    if (straight > 0.05 && output.length > config.maximum_path_length_ratio * straight) {
      return finish(PlanStatus::kPathTooLong, "経路長が直線距離に対して長すぎます");
    }
    output.geometry = fine.geometry;
    output.direction = fine.direction;
    output.parameter = fine.parameter;
    output.start_clearance = clearance.front();
    output.goal_clearance = clearance.back();
    output.minimum_clearance = *std::min_element(clearance.begin(), clearance.end());
    output.candidates_evaluated = context.evaluated;
    result.path = std::move(output);
    return finish(PlanStatus::kOk, "OK");
  }
  return finish(PlanStatus::kValidationFailed, "細かい標本間隔での最終検証を通る候補がありません");
}

}  // namespace pickup_path_planner
