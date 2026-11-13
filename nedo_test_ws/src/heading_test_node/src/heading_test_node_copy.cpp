#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <optional>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <std_msgs/msg/bool.hpp>

static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

static inline void exec_quiet(const std::string& cmd){
  int ret = std::system(cmd.c_str());
  (void)ret; // 音が出なくても制御は継続
}

static std::string fmt(double v, int p){
  std::ostringstream oss; oss.setf(std::ios::fixed);
  oss << std::setprecision(p) << v; return oss.str();
}

class HeadingTestNode : public rclcpp::Node {
public:
  HeadingTestNode() : Node("heading_test_node"){
    pose2d_topic_  = this->declare_parameter<std::string>("pose2d_topic", "/robot_pose2d");
    pose3d_topic_  = this->declare_parameter<std::string>("pose_stamped_topic", "/marker_pose");
    cmd_topic_     = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // 固定ゴール (マップ座標系で絶対)
    goal_x_ = this->declare_parameter<double>("goal_x", 0.0);
    goal_y_ = this->declare_parameter<double>("goal_y", -1.7);

    yaw_off_deg_   = this->declare_parameter<double>("yaw_offset_deg", 0.0);
    yaw_off_rad_   = yaw_off_deg_ * M_PI / 180.0;

    kp_ = this->declare_parameter<double>("k_p", 0.8);
    ki_ = this->declare_parameter<double>("k_i", 0.0);
    kd_ = this->declare_parameter<double>("k_d", 0.08);

    u_max_    = this->declare_parameter<double>("u_max", 0.5);
    deadband_ = this->declare_parameter<double>("deadband", 0.01);
    alpha_max_radps2_ = this->declare_parameter<double>("alpha_max_radps2", 0.2);

    i_limit_  = this->declare_parameter<double>("i_limit", 0.5);

    heading_tol_deg_  = this->declare_parameter<double>("heading_tolerance_deg", 2.9);
    heading_tol_rad_  = heading_tol_deg_ * M_PI / 180.0;

    // --- Sound params (defaults) ---
    sound_enabled_      = this->declare_parameter<bool>("sound_enabled", true);
    sound_backend_      = this->declare_parameter<std::string>("sound_backend", "canberra"); //"canberra","beep","sox","none"
    beep_interval_sec_  = this->declare_parameter<double>("beep_interval_sec", 1.0); // 調整中の間隔
    beep_freq_hz_       = this->declare_parameter<double>("beep_freq_hz", 880.0);   // beep/sox
    beep_len_ms_        = this->declare_parameter<int>("beep_len_ms", 500);         // 1回の長さ

    pose_timeout_sec_ = this->declare_parameter<double>("pose_timeout_sec", 1.5);

    scale_         = this->declare_parameter<double>("pixels_per_meter", 80.0);
    show_size_     = this->declare_parameter<int>("window_size", 700);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      pose2d_topic_, rclcpp::QoS(20),
      std::bind(&HeadingTestNode::onPose2D, this, std::placeholders::_1));
    pose3d_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose3d_topic_, rclcpp::QoS(5),
      std::bind(&HeadingTestNode::onPoseStamped, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/heading/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&HeadingTestNode::onTimer, this));
    
    // QoS設定（ラッチ相当）
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local();

    heading_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/heading_done", latched_qos);

    cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(),
      "HeadingTest started. Fixed Goal=(%.3f, %.3f), Kp=%.2f Ki=%.2f Kd=%.2f",
      goal_x_, goal_y_, kp_, ki_, kd_);
  }

private:
 void play_beep_once(){
   if(!sound_enabled_ || sound_backend_=="none") return;
   if(sound_backend_=="canberra"){
    exec_quiet("canberra-gtk-play -i bell -q");
   }else if(sound_backend_=="beep"){
     // 要: sudo apt install beep, 権限設定と pcspkr 有効化が必要な場合あり
     std::ostringstream oss; oss << "beep -f " << (int)beep_freq_hz_ << " -l " << beep_len_ms_;
     exec_quiet(oss.str());
   }else if(sound_backend_=="sox"){
     // 要: sudo apt install sox
     std::ostringstream oss; 
     // 例: 0.25秒のサイン波, 音量小さめ
     oss << "play -q -n synth " << (beep_len_ms_/1000.0) << " sine " << (int)beep_freq_hz_ << " vol 0.3";
     exec_quiet(oss.str());
   }
 }
 void play_beep_double(){ // 完了時の「ピピッ」
   play_beep_once();
   // 100msだけ待つ, systemで簡易に
   exec_quiet("/bin/sh -c 'sleep 0.10'");
   play_beep_once();
 }

  void onPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg){
    pose2d_ = *msg; have_pose2d_ = true; last_pose_time_ = this->now();
  }
  void onPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    last_z_ = msg->pose.position.z;
  }

  void publishStop(){
    geometry_msgs::msg::Twist tw; // 全0
    cmd_pub_->publish(tw);
  }

 void onTimer(){
   const int W = show_size_, H = show_size_;
   cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(15, 15, 15));
   const cv::Point C(W/2, H/2);

   const auto now_steady = std::chrono::steady_clock::now();
   double dt = 0.05;
   if(prev_steady_.time_since_epoch().count() != 0){
     dt = std::clamp(std::chrono::duration<double>(now_steady - prev_steady_).count(), 0.001, 0.5);
   }
   prev_steady_ = now_steady;

   bool pose_stale = true;
   if(have_pose2d_){
     const double age = (this->now() - last_pose_time_).seconds();
     pose_stale = (age > pose_timeout_sec_);
   }
   if(!have_pose2d_ || pose_stale){
     cv::putText(canvas, "Pose stale/none, stopping ...",
                {20,40}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,200,255), 2);
     cv::imshow(win_, canvas); cv::waitKey(1);
     publishStop();
     return;
   }

   // --- 既に姿勢合わせ完了なら以降は何もしない ---
   if(heading_done_){
     geometry_msgs::msg::Twist tw; // 全ゼロ
     cmd_pub_->publish(tw);
     if(sound_enabled_ && !done_beeped_){
        play_beep_double();
        done_beeped_ = true;
     }
     cv::putText(canvas, "Heading done.", {20,40},
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
     cv::imshow(win_, canvas); cv::waitKey(1);
     return;
    }
   // --- 姿勢制御 ---
   const double x = pose2d_.x, y = pose2d_.y;
   const double th = wrapToPi(pose2d_.theta + yaw_off_rad_ + M_PI/2.0);

   const double gx = goal_x_, gy = goal_y_;
   double dx = gx - x, dy = gy - y;

   const double target_yaw = std::atan2(dy, dx);
   double e = wrapToPi(target_yaw - th);   // yaw誤差

   // ===== PID =====
   i_term_ += e * dt;
   if(ki_ > 0.0){
     const double i_raw = ki_ * i_term_;
     if(i_raw >  i_limit_) i_term_ =  i_limit_ / ki_;
     if(i_raw < -i_limit_) i_term_ = -i_limit_ / ki_;
   }
   const double P = kp_ * e;
   const double I = ki_ * i_term_;
   // 微分（ローパス）
   double d_raw = 0.0;
   if(has_prev_e_) d_raw = (e - prev_e_) / dt;
   if(!have_d_) { d_hat_ = d_raw; have_d_ = true; }
   else         { d_hat_ = (1.0 - alpha_d_) * d_hat_ + alpha_d_ * d_raw; }

   double D = kd_ * d_hat_;

   prev_e_ = e; has_prev_e_ = true;

   double u_pid = P + I + D;
   double u_target = std::clamp(u_pid, -u_max_, +u_max_);

   const double du_per_s_max = (60.0 / M_PI) * alpha_max_radps2_;
   const double du_max = du_per_s_max * dt;
   double u = u_target;
   const double du_needed = u_target - prev_u_;
   if(du_needed >  du_max) u = prev_u_ + du_max;
   if(du_needed < -du_max) u = prev_u_ - du_max;

   // --- 完了判定 ---
   const bool at_heading = std::abs(e) <= heading_tol_rad_;
   if(at_heading){
     u = 0.0;
     i_term_ = 0.0;
     heading_done_ = true;    // 一度きりで完了
     geometry_msgs::msg::Twist tw0; // 即座に停止を出す
     cmd_pub_->publish(tw0);
     if(sound_enabled_ && !done_beeped_) {
       play_beep_double(); 
       done_beeped_ = true;
     }
     std_msgs::msg::Bool msg; msg.data = true;
     heading_done_pub_->publish(msg);
   }

   
   // --- 調整中の周期ビープ ---
   if(sound_enabled_ && !heading_done_){
     auto now = this->now();
     if(last_beep_time_.nanoseconds() == 0 || (now - last_beep_time_).seconds() >=  beep_interval_sec_){
       play_beep_once();
       last_beep_time_ = now;
     }
   }

   if(std::abs(u) < deadband_) u = 0.0;
   u = std::clamp(u, -1.0, 1.0);
   prev_u_ = u;

   geometry_msgs::msg::Twist tw;
   tw.angular.z = u;
   cmd_pub_->publish(tw);

   // 自分の位置を白丸で表示
   cv::circle(canvas, C, 6, cv::Scalar(255,255,255), -1, cv::LINE_AA);

   // 自分の向きを矢印で表示
   const double body_len = 0.8; // 矢印の長さ[m]
   cv::Point H_cur(C.x + body_len*scale_*std::cos(th),
                C.y - body_len*scale_*std::sin(th));
   cv::arrowedLine(canvas, C, H_cur, cv::Scalar(0,200,255), 3, cv::LINE_AA, 0, 0.15);

   // ゴール位置を緑点で表示
   cv::Point G(C.x + (gx-x)*scale_, C.y - (gy-y)*scale_);
   cv::circle(canvas, G, 6, cv::Scalar(0,255,0), -1, cv::LINE_AA);

   // 表示
   cv::imshow(win_, canvas);
   cv::waitKey(1);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose3d_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heading_done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose2D pose2d_{};
  bool have_pose2d_ = false;
  bool heading_done_ = false;

  std::optional<double> last_z_;
  rclcpp::Time last_pose_time_{};

  std::string pose2d_topic_;
  std::string pose3d_topic_;
  std::string cmd_topic_;
  
  double d_hat_ = 0.0;
  bool have_d_  = false;
  double alpha_d_ = 0.2;
  
  double goal_x_, goal_y_;
  double yaw_off_deg_, yaw_off_rad_;
  double kp_, ki_, kd_;
  double u_max_, deadband_;
  double alpha_max_radps2_;
  double i_limit_;
  double heading_tol_deg_, heading_tol_rad_;
  double pose_timeout_sec_;
  double scale_; int show_size_; const std::string win_ = "HeadingTest";

  double prev_e_ = 0.0; bool has_prev_e_ = false; double i_term_ = 0.0;
  double prev_u_ = 0.0;
  std::chrono::steady_clock::time_point prev_steady_{};
  
  // --- Sound params ---
  bool sound_enabled_;
  std::string sound_backend_;        // "canberra", "beep", "sox", "none"
  double beep_interval_sec_;
  double beep_freq_hz_;              // beep/sox用
  int    beep_len_ms_;               // 1回の「ピー」の長さ
  // --- Sound state ---
  rclcpp::Time last_beep_time_;
  bool done_beeped_ = false;

};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadingTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

