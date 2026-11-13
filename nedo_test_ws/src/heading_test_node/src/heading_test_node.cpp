// heading_test_node/src/heading_test_node.cpp
// 目標方向へのその場旋回用ノード (PID制御版)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <algorithm>

static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

class HeadingNodePID : public rclcpp::Node {
public:
  HeadingNodePID() : Node("heading_node")
  {
    // === Parameters ===
    kp_ = declare_parameter("k_p", 0.5);
    ki_ = declare_parameter("k_i", 0.0);
    kd_ = declare_parameter("k_d", 0.15);
    // ランドマーク自己位置は旋回中に更新が遅れることがあるため、過大な回り越しを防ぐ。
    u_max_    = declare_parameter("u_max", 0.3);
    deadband_ = declare_parameter("deadband", 0.01);
    i_limit_  = declare_parameter("i_limit", 0.5);
    alpha_max_radps2_ = declare_parameter("alpha_max_radps2", 0.3);
    alpha_d_  = declare_parameter("alpha_d", 0.4);
    heading_tol_deg_ = declare_parameter("heading_tolerance_deg", 1.0);
    heading_tol_rad_ = heading_tol_deg_ * M_PI / 180.0;
    hold_time_ = declare_parameter("hold_time", 0.03);
    dt_ctrl_ = declare_parameter("dt", 0.05);

    yaw_off_deg_  = declare_parameter("yaw_offset_deg", 0.0);
    yaw_off_rad_  = yaw_off_deg_ * M_PI / 180.0;
    axis_rot_deg_ = declare_parameter("axis_rot_deg", 0.0);
    axis_rot_rad_ = axis_rot_deg_ * M_PI / 180.0;

    rate_tol_radps_   = declare_parameter("rate_tol_radps", 0.03);  // 角速度閾値
    u_tol_radps_      = declare_parameter("u_tol_radps",    0.04);  // 出力閾値
    settle_time_s_    = declare_parameter("settle_time",    0.30);  // 連続保持
    alpha_rate_       = declare_parameter("alpha_rate",     0.2);    // 角速度LPF係数
    e_cap_deg_        = declare_parameter("e_cap_deg",      2.5);   // 安全弁
    e_cap_rad_        = e_cap_deg_ * M_PI / 180.0;
    // 自己位置がこれより古ければ止まる [s] (EKF が止まったとき、最後の向きのまま回り続けないように)
    pose_timeout_     = declare_parameter("pose_timeout",   0.5);

    // === Subscriptions ===
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      [&](const geometry_msgs::msg::Pose2D& p){
        const auto receive_time = now();
        const double yaw_map = wrapToPi(p.theta + yaw_off_rad_ + axis_rot_rad_);
        if (have_rate_sample_) {
          const double sample_dt = (receive_time - last_pose_time_).seconds();
          if (sample_dt > 1.0e-3) {
            const double rate_raw = wrapToPi(yaw_map - rate_prev_yaw_) / sample_dt;
            yaw_rate_hat_ =
              (1.0 - alpha_rate_) * yaw_rate_hat_ + alpha_rate_ * rate_raw;
          }
        } else {
          yaw_rate_hat_ = 0.0;
          have_rate_sample_ = true;
        }
        rate_prev_yaw_ = yaw_map;
        yaw_ = p.theta;
        have_yaw_ = true;
        last_pose_time_ = receive_time;
      });

    heading_target_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/heading_target", 10,
      [&](const std_msgs::msg::Float32& msg){
        target_yaw_ = wrapToPi(msg.data);
        got_target_ = true;
        finished_ = false;
        hold_acc_ = 0.0;
        settle_acc_ = 0.0;
        i_term_ = 0.0;
        has_prev_e_ = false;
        have_d_ = false;
        prev_u_ = 0.0;
      });

    mux_source_sub_ = create_subscription<std_msgs::msg::String>(
      "/cmd_source", 10,
      [&](const std_msgs::msg::String& msg){
        if (msg.data == "heading") {
          // 通常 heading モード
          is_check_heading_ = false;
          finished_ = false;
          hold_acc_ = 0.0;
          settle_acc_ = 0.0;
        }
        else if (msg.data == "check_heading") {
          // Obstacle-Check 用 heading
          is_check_heading_ = true;
          finished_ = false;
          hold_acc_ = 0.0;
          settle_acc_ = 0.0;
          RCLCPP_INFO(this->get_logger(), "[HeadingNode] Check heading mode");
        }
      });

    // === Publishers ===
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/heading/rover_twist", 10);
    rclcpp::QoS latched_qos(1); latched_qos.transient_local();
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/heading_done", latched_qos);

    // === Timer ===
    timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(dt_ctrl_ * 1000)),
      std::bind(&HeadingNodePID::onTimer, this));

    RCLCPP_INFO(get_logger(), "HeadingNode(PID) initialized");
  }

private:
  bool is_check_heading_ = false;

  void onTimer()
  {
    if(!have_yaw_ || !got_target_){
      publishStop();
      return;
    }

    if(finished_){
      publishStop();
      return;
    }

    if ((now() - last_pose_time_).seconds() > pose_timeout_){
      publishStop();
      settle_acc_ = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "自己位置が %.1f 秒届いていないので回転を止めています。届いたら続けます", pose_timeout_);
      return;
    }

    // --- yaw補正 ---
    const double yaw_map = wrapToPi(yaw_ + yaw_off_rad_ + axis_rot_rad_);

    // --- 誤差 ---
    const double e = wrapToPi(target_yaw_ - yaw_map);

    // --- PID計算 ---
    i_term_ += e * dt_ctrl_;
    if (ki_ > 0.0){
      double i_raw = ki_ * i_term_;
      if (i_raw >  i_limit_) i_term_ =  i_limit_ / ki_;
      if (i_raw < -i_limit_) i_term_ = -i_limit_ / ki_;
    }

    const double P = kp_ * e;
    const double I = ki_ * i_term_;

    // 目標姿勢は固定なので de/dt = -yaw_rate。実際の姿勢受信間隔で求めた
    // 角速度を使い、タイマ周期で同じ姿勢を微分して生じるスパイクを避ける。
    const double D = -kd_ * yaw_rate_hat_;
    prev_e_ = e; has_prev_e_ = true;

    double u_target = std::clamp(P + I + D, -u_max_, +u_max_);

    // --- 角加速度制限 ---
    double du_max = alpha_max_radps2_ * dt_ctrl_;
    double du = u_target - prev_u_;
    du = std::clamp(du, -du_max, du_max);
    double u = prev_u_ + du;
    prev_u_ = u;

    // --- デッドバンド ---
    if (std::abs(u) < deadband_) u = 0.0;

    // --- 完了判定 ---
    bool stable =
      (std::abs(yaw_rate_hat_) <= rate_tol_radps_) &&
      (std::abs(u)             <= u_tol_radps_)   &&
      (std::abs(e)             <= heading_tol_rad_);

    if (stable) settle_acc_ += dt_ctrl_;
    else        settle_acc_  = 0.0;

    // --- 出力 ---
    geometry_msgs::msg::Twist tw;
    tw.angular.z = u;
    cmd_pub_->publish(tw);

    if (settle_acc_ >= settle_time_s_) {
      finished_ = true;
      prev_u_ = 0.0;
      publishStop();
      std_msgs::msg::Bool done; done.data = true;
      done_pub_->publish(done);
      RCLCPP_INFO(get_logger(),
        "Heading done: |e|=%.2f deg, |rate|=%.3f rad/s, |u|=%.3f (held %.2fs)",
        std::abs(e)*180.0/M_PI, std::abs(yaw_rate_hat_), std::abs(u), settle_acc_);
    }
  }

  void publishStop(){
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  // === Members ===
  double kp_, ki_, kd_;
  double u_max_, deadband_, i_limit_, alpha_max_radps2_, alpha_d_;
  double heading_tol_deg_, heading_tol_rad_, hold_time_, dt_ctrl_;

  double yaw_{0.0}; bool have_yaw_{false};
  double target_yaw_{0.0}; bool got_target_{false};
  bool finished_{false};
  double i_term_{0.0}, prev_e_{0.0}, d_hat_{0.0}, prev_u_{0.0}, hold_acc_{0.0};
  bool has_prev_e_{false}, have_d_{false};

  // --- 座標系補正用パラメータ ---
  double yaw_off_deg_{0.0};
  double yaw_off_rad_{0.0};
  double axis_rot_deg_{90.0};
  double axis_rot_rad_{M_PI / 2.0};

  double rate_prev_yaw_{0.0};
  bool   have_rate_sample_{false};
  double yaw_rate_hat_{0.0};
  double u_prev_pub_{0.0};
  double settle_acc_{0.0};
  double rate_tol_radps_{0.02}, u_tol_radps_{0.03}, settle_time_s_{0.30}, alpha_rate_{0.2};
  double e_cap_deg_{15.0}, e_cap_rad_{15.0*M_PI/180.0};
  double pose_timeout_{0.5};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mux_source_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<HeadingNodePID>());
  rclcpp::shutdown();
  return 0;
}
