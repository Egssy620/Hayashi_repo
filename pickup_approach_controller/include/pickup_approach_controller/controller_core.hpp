#ifndef PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_
#define PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pickup_approach_controller
{

// 地図座標系における車体中心の平面姿勢です。単位はm、radです。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 上流EKFが出す一つの融合自己位置メッセージを、制御コア用に整形した値です。
struct LocalizationSample
{
  double stamp{0.0};
  std::uint64_t sequence{0};
  std::uint64_t reset_counter{0};  // 再位置合わせ・姿勢の飛びを検出するたびに増える
  Pose2D pose{};
  double covariance[3][3]{};       // x, y, yaw の順
  double vx_body{0.0};
  double vy_body{0.0};
  double omega{0.0};
  bool healthy{false};
};

// 空間軌道表の一つの節点です。節点間隔は path_resample_spacing です。
struct PathPoint
{
  double s{0.0};
  Pose2D pose{};
  double tangent_x{1.0};
  double tangent_y{0.0};
  double curvature{0.0};
  double yaw_derivative{0.0};
  double speed_limit{0.0};       // 巡航・横加速度・角速度から決まる局所上限
  double reference_speed{0.0};   // 終点で速度ゼロとなる基準速度
  double speed_derivative{0.0};  // d(reference_speed)/ds
};

// 調整値です。単位はm、s、radです。
struct ControllerConfig
{
  // 速度・加速度の制限。初期値は実機の立上げ用に抑えた値です。
  double maximum_speed{0.30};
  double cruise_speed{0.25};
  double maximum_acceleration{0.25};
  double maximum_jerk{0.50};
  double maximum_snap{2.0};
  double maximum_lateral_acceleration{0.15};
  double path_deceleration{0.20};
  double maximum_yaw_rate{0.34906585};          // 20 deg/s
  double maximum_yaw_acceleration{0.34906585};  // 20 deg/s^2
  double maximum_longitudinal_recovery_speed{0.04};

  // 位置・姿勢の誤差フィードバック
  double position_gain{0.80};
  double maximum_correction_speed{0.16};
  double yaw_kp{1.20};
  double yaw_ki{0.02};
  double yaw_kd{0.10};
  double yaw_integral_limit{0.05235988};        // 3 deg*s
  double yaw_derivative_filter_time_constant{0.50};

  // 進捗推定
  double projection_search_back{0.60};
  double projection_search_ahead{0.80};
  double progress_correction_time_constant{0.10};
  double maximum_progress_rate{0.80};

  // 指令整形
  double curvature_feedforward_gain{1.0};
  double velocity_shaping_gain{25.0};
  double acceleration_shaping_gain{10.0};

  // 経路の受理条件・前処理
  double path_resample_spacing{0.02};
  int curvature_window{2};                      // 曲率・姿勢微分を求める前後の節点数
  double maximum_heading_step{0.35};            // 隣接節点間の接線方向変化の上限 [rad]
  double path_goal_position_tolerance{0.01};    // 経路終点と目標の位置差の上限 [m]
  double path_goal_yaw_tolerance{0.02};         // 経路終点と目標のyaw差の上限 [rad]

  // 終端・時間管理
  double terminal_minimum_speed{0.0};           // 静止摩擦対策の最低接近速度(0で無効) [m/s]
  double maximum_tracking_time{240.0};          // これを超えるとTIMEOUTで停止 [s]
  double nominal_period{0.02};
  double maximum_period{0.25};

  // 到着判定
  double arrival_position_tolerance{0.020};
  double arrival_yaw_tolerance{0.020943951};    // 1.2 deg
  double arrival_command_speed_tolerance{0.010};
  double arrival_command_yaw_rate_tolerance{0.010};
  double arrival_window_seconds{0.8};
  double arrival_minimum_window_seconds{0.5};
  int arrival_minimum_samples{10};
  double confidence_sigma{2.0};
  double maximum_position_drift_rate{0.005};
  double maximum_yaw_drift_rate{0.008726646};
};

// 一周期の制御計算結果です。速度は車体座標系です。
struct ControllerOutput
{
  double vx_body{0.0};
  double vy_body{0.0};
  double omega{0.0};
  double path_progress{0.0};
  double position_error{0.0};  // 目標位置までの距離
  double yaw_error{0.0};       // 目標yawとの差
  bool arrived{false};
  bool arrival_pulse{false};
  bool timed_out{false};
  bool timeout_pulse{false};
  std::string arrival_decision{"COLLECTING"};
  std::string arrival_reason{"WINDOW_NOT_READY"};
  std::string state{"IDLE"};
  std::string reason{"NO_PATH"};
};

// ROS 2に依存しない追従コアです。単体試験とログ再生から同じ処理を呼べます。
class ControllerCore
{
public:
  explicit ControllerCore(const ControllerConfig & config = ControllerConfig{});

  // 経路を受理し、距離基準の空間軌道表（基準速度列を含む）を生成します。
  // 経路終点が goal と一致しない、折返し・急な折れ点がある、などの場合は拒否します。
  bool setPath(const std::vector<Pose2D> & poses, const Pose2D & goal, std::string * error = nullptr);

  // 目標・経路・到着履歴・指令整形器をすべて安全な停止状態へ戻します。
  void clearPath();

  // 最新の融合自己位置から一制御周期分の速度指令を計算します。
  // environment_valid=false または too_close=true のときは即時にゼロ指令にします（非常停止）。
  // 自己位置異常・タイムアウトのときは、ジャーク制限付きで減速して停止します。
  ControllerOutput step(
    double now_seconds,
    const LocalizationSample & localization,
    bool motion_authority,
    bool environment_valid,
    bool too_close);

  bool hasPath() const;
  const std::vector<PathPoint> & trajectory() const;
  const Pose2D & goal() const;
  double elapsedTrackingTime() const;

private:
  struct ArrivalSample
  {
    double stamp{0.0};
    Pose2D pose{};
    double covariance[3][3]{};
  };

  PathPoint interpolate(double progress) const;
  double projectProgress(const Pose2D & pose) const;
  void hardStop();
  void resetTiming();
  void shapeTranslation(
    double desired_vx, double desired_vy, double feedforward_ax, double feedforward_ay, double dt);
  void shapeYaw(double desired_omega, double dt);
  void smoothStop(double dt, ControllerOutput * output);
  void updateArrivalWindow(
    const LocalizationSample & localization, double command_speed, double command_yaw_rate);
  bool evaluateArrival(
    double & position_error, double & yaw_error, std::string & decision,
    std::string & reason) const;

  ControllerConfig config_{};
  std::vector<PathPoint> trajectory_{};
  Pose2D goal_{};
  std::deque<ArrivalSample> arrival_window_{};
  double path_progress_{0.0};
  double last_step_time_{0.0};
  double elapsed_{0.0};
  std::uint64_t last_sequence_{0};
  std::uint64_t last_reset_counter_{0};
  bool have_step_time_{false};
  bool have_progress_{false};
  bool have_reset_counter_{false};
  bool have_valid_yaw_{false};
  bool arrived_{false};
  bool timed_out_{false};
  double last_valid_yaw_{0.0};

  double world_velocity_x_{0.0};
  double world_velocity_y_{0.0};
  double world_acceleration_x_{0.0};
  double world_acceleration_y_{0.0};
  double world_jerk_x_{0.0};
  double world_jerk_y_{0.0};
  double omega_command_{0.0};
  double angular_acceleration_{0.0};

  // 姿勢PIDの状態
  bool yaw_pid_initialized_{false};
  double yaw_error_integral_{0.0};
  double previous_yaw_error_{0.0};
  double filtered_yaw_error_derivative_{0.0};
};

// 試験・計算専用運転で使う直線経路を生成します。実運用では経路計画ノードの出力を使います。
std::vector<Pose2D> makeStraightPath(const Pose2D & start, const Pose2D & goal, double spacing);

double wrapAngle(double angle);

}  // namespace pickup_approach_controller

#endif  // PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_
