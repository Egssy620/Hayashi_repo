// drive_node/src/drive_node.cpp
// 走行制御ノード (Pure Pursuit)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#define VIS_DEBUG
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>

static inline double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}
static inline double clamp(double x, double lo, double hi){
  return std::max(lo, std::min(hi, x));
}

class DriveNode : public rclcpp::Node {
public:
  DriveNode() : Node("drive_node") {
    // ---- Parameters ----
    v_max_         = this->declare_parameter<double>("v_max",         0.30);  // m/s
    v_min_         = this->declare_parameter<double>("v_min",         0.075); // m/s
    slow_dist_     = this->declare_parameter<double>("slowdown_dist", 0.75);  // m
    stop_dist_     = this->declare_parameter<double>("stop_dist",     0.1); // m
    omega_max_     = this->declare_parameter<double>("omega_max",     0.40);  // rad/s
    ctrl_rate_hz_  = this->declare_parameter<double>("rate",          20.0); // Hz
    tau_ = this->declare_parameter("omega_filter_tau", 0.35);
    dt_  = this->declare_parameter("control_dt", 0.05);
    // 自己位置がこれより古ければ止まる [s] (EKF が止まったとき、最後の位置のまま走り続けないように)
    pose_timeout_ = this->declare_parameter("pose_timeout", 0.5);

    // Publishers
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/drive/rover_twist", 10);
    done_pub_  = this->create_publisher<std_msgs::msg::Bool>("/drive_done", 10);

    // Subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      std::bind(&DriveNode::onPose, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/current_goal", 10, std::bind(&DriveNode::onGoal, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/target_point", 10, std::bind(&DriveNode::onTarget, this, std::placeholders::_1));

    // Timer
    using namespace std::chrono_literals;
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / ctrl_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&DriveNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "DriveNode ready (Pure Pursuit, output=/rover_twist)");
  }

private:

  std::string makeLogDirectory()
  {
      // HOME ディレクトリ
      const char* home = std::getenv("HOME");
      if (!home) home = "/tmp"; // fallback

      // 今日の日付 YYYY-MM-DD
      std::time_t t = std::time(nullptr);
      std::tm tm{};
      localtime_r(&t, &tm);

      std::ostringstream oss;
      oss << home << "/ros2_logs/"
          << std::put_time(&tm, "%Y-%m-%d");

      std::string dir_path = oss.str();

      // ディレクトリを作成
      std::filesystem::create_directories(dir_path);

      return dir_path;
  }

  // Callbacks
  void onPose(const geometry_msgs::msg::Pose2D::SharedPtr msg){
      //const double dx_cam = -0.135;オフセットはaruco_pnp_node_wall.cppの方に移動しました。
      const double dx_cam = 0.0;
      const double dy_cam = 0.0;
      double yaw = msg->theta;
      xr_ = msg->x + dx_cam * std::cos(yaw) - dy_cam * std::sin(yaw);
      yr_ = msg->y + dx_cam * std::sin(yaw) + dy_cam * std::cos(yaw);
      thetar_ = yaw;
      have_pose_ = true;
      last_pose_time_ = this->now();
  }

  void onGoal(const geometry_msgs::msg::Pose2D::SharedPtr msg){
    bool changed = !have_goal_
                || std::hypot(xg_ - msg->x, yg_ - msg->y) > 1e-3;
    xg_ = msg->x; yg_ = msg->y; thetag_ = msg->theta; have_goal_ = true;
    if (changed) {
      done_sent_ = false;
      settle_count_ = 0;
    }
  }

  void onTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    xt_ = msg->point.x; yt_ = msg->point.y; have_target_ = true;
  }

  // Control loop
  void onTimer(){
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    if(!(have_pose_ && have_goal_ && have_target_)){
      cmd_pub_->publish(cmd);
      return;
    }

    if ((this->now() - last_pose_time_).seconds() > pose_timeout_){
      cmd_pub_->publish(cmd);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "自己位置が %.1f 秒届いていないので止まっています。届いたら走り出します", pose_timeout_);
      return;
    }

    // distances
    const double dx = xt_ - xr_;
    const double dy = yt_ - yr_;
    const double Ld = std::hypot(dx, dy);

    const double dg = std::hypot(xg_ - xr_, yg_ - yr_);

    // goal判定
    if (dg < stop_dist_){
      // ゼロ速度を送り続けてロボットを止める
      cmd_pub_->publish(cmd);
      if (!done_sent_){
        settle_count_++;
        if (settle_count_ >= SETTLE_TICKS){
          std_msgs::msg::Bool b; b.data = true;
          done_pub_->publish(b);
          done_sent_ = true;
          RCLCPP_INFO(get_logger(),
            "Drive goal reached after %d-tick settle.", settle_count_);
        }
      }
      return;
    } else {
      done_sent_ = false;
      settle_count_ = 0;
    }

    // map -> vehicle変換
    const double c = std::cos(thetar_), s = std::sin(thetar_);
    const double xL =  c*dx + s*dy;
    const double yL = -s*dx + c*dy;

    double kappa = 0.0;
    if (Ld > 1e-3){
      kappa = 2.0 * yL / (Ld * Ld);
    }

    double v = v_max_;
    if (dg < slow_dist_){
      double scale = clamp(dg / slow_dist_, 0.0, 1.0);
      v = v_min_ + (v_max_ - v_min_) * scale;
    }

    // angular speed
    double omega_cmd = v * kappa;
    omega_cmd = clamp(omega_cmd, -omega_max_, omega_max_);

    // 一次遅れフィルタ
    double alpha = dt_ / std::max(tau_, dt_);
    omega_filt_ += alpha * (omega_cmd - omega_filt_);

    // 出力
    cmd.linear.y  = -v;
    cmd.angular.z = omega_filt_;
    cmd_pub_->publish(cmd);

  }

  // Members
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // params
  double v_max_{0.30}, v_min_{0.08}, slow_dist_{1.5}, stop_dist_{0.15}, omega_max_{1.5};
  double ctrl_rate_hz_{20.0};
  double tau_;              // 時定数 [s]
  double dt_;               // サンプリング周期 [s]
  double omega_filt_ = 0.0; // フィルタ後角速度
  double pose_timeout_;     // 自己位置の鮮度 [s]
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  // states
  double xr_{0}, yr_{0}, thetar_{0};
  double xg_{0}, yg_{0}, thetag_{0};
  double xt_{0}, yt_{0};
  bool have_pose_{false}, have_goal_{false}, have_target_{false}, done_sent_{false};
  int settle_count_{0};
  static constexpr int SETTLE_TICKS{10};  // 20Hz x 10 = 0.5s
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveNode>());
  rclcpp::shutdown();
  return 0;
}
