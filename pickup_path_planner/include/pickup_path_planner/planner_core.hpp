#ifndef PICKUP_PATH_PLANNER__PLANNER_CORE_HPP_
#define PICKUP_PATH_PLANNER__PLANNER_CORE_HPP_

#include <string>
#include <vector>

namespace pickup_path_planner
{

constexpr double kPi = 3.14159265358979323846;

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

// 地図座標系における車体中心の平面姿勢です。yawは+x軸から反時計回りで、単位はradです。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 車体矩形です（原点は車体中心、x軸が前方）。梁は計画では扱いません。
struct RobotGeometry
{
  double body_length{4.150};
  double body_width{1.800};
};

struct PlannerConfig
{
  // 経路の余裕
  double body_clearance{0.15};                  // 山と車体の公称余裕 [m]
  double tracking_error_allowance{0.08};        // 追従誤差見込み [m]
  double transition_distance{0.50};             // 始終端から見込みを増減させる距離 [m]
  double orbit_extra_clearance{0.25};           // 極座標曲線の探索上限へ足す余裕 [m]
  double path_visible_sector_margin{kPi};       // 既定のpiは観測扇形による制限を無効化
  // サンプリング
  double search_step{0.08};                     // 候補探索時の標本間隔 [m]
  double output_step{0.02};                     // 出力経路の標本間隔 [m]
  double validation_step{0.02};                 // 最終検証の標本間隔 [m]
  // 観測山モデル
  int minimum_skirt_points{12};
  double minimum_visible_arc{kPi / 2.0};        // 90 deg
  double residual_coverage{0.95};
  double radial_uncertainty{0.01};              // [m]
  int polygon_samples{180};
  int circle_iterations{6};
  // 実機向け追加の安全検査
  double maximum_pile_radius{2.5};              // 推定半径がこれを超えたら山と見なさない [m]
  double minimum_pile_radius{0.4};              // 推定半径がこれ未満なら山と見なさない [m]
  double maximum_circle_rms_residual{0.15};     // 円近似の残差(MAD由来)が大きい観測は棄却 [m]
  double maximum_path_length_ratio{4.0};        // 経路長/直線距離の上限（異常な迂回の棄却）
};

// LiDAR観測の裾点群だけから作る山の保守的な領域です。真の山形状は使いません。
struct ObservedPileModel
{
  Vec2 center{};
  double fitted_radius{0.0};
  double conservative_radius{0.0};
  double rms_residual{0.0};
  double visible_arc{0.0};
  double visible_arc_start{0.0};
  double visible_arc_span{0.0};
  std::size_t point_count{0};
  std::vector<Vec2> polygon{};  // 反時計回りの凸多角形
};

enum class PlanStatus
{
  kOk,
  kInvalidInput,
  kInsufficientObservation,
  kStartInCollision,
  kGoalInCollision,
  kNoCollisionFreePath,
  kValidationFailed,
  kPathTooLong
};

std::string toString(PlanStatus status);

struct PlannedPath
{
  std::vector<Pose2D> poses{};
  std::vector<double> distance{};
  double length{0.0};
  std::string geometry{};         // DIRECT / TANGENT_ARC_TANGENT / SMOOTH_POLAR_CURVE
  std::string direction{};        // CCW / CW / ""
  double parameter{0.0};          // 迂回半径または膨らみ量
  double minimum_clearance{0.0};  // 経路全体での車体と山の最小離隔 [m]
  double start_clearance{0.0};
  double goal_clearance{0.0};
  int candidates_evaluated{0};
};

struct PlanResult
{
  PlanStatus status{PlanStatus::kInvalidInput};
  std::string message{};
  PlannedPath path{};
};

double wrapAngle(double angle);

// 裾点（地図座標のxy）から観測山モデルを作ります。
// 点数・可視円弧・円近似の妥当性が足りない場合はfalseを返し、再スキャンを要求します。
bool buildObservedPileModel(
  const std::vector<Vec2> & skirt_points,
  const PlannerConfig & config,
  ObservedPileModel * model,
  std::string * error);

// 車体矩形と凸多角形の最小離隔です。交差・包含なら0を返します。
// 離隔が interest を十分に超えると分かった場合は、その下限値を返して計算を省きます。
double bodyPolygonDistance(
  const Pose2D & pose,
  const RobotGeometry & geometry,
  const ObservedPileModel & model,
  double interest);

// 直線、接線-円弧-接線、極座標曲線から最短の安全経路を選びます。
PlanResult planPilePath(
  const Pose2D & start,
  const Pose2D & goal,
  const ObservedPileModel & model,
  const RobotGeometry & geometry,
  const PlannerConfig & config);

}  // namespace pickup_path_planner

#endif  // PICKUP_PATH_PLANNER__PLANNER_CORE_HPP_
