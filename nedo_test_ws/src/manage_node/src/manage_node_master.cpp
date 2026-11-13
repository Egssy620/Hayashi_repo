// manage_node/src/manage_node_master.cpp
// 自律移動の全体管理ノード (マスター接続版)
//   manage_node.cpp (シャトル運転) との違い:
//   - 目的地は goals の往復ではなく、マスターから /goal_pose (PoseStamped, map 系) で1つずつ受け取る
//   - 着いたら目的地の向き (/goal_pose の向き) を向いて止まる (次の /goal_pose を待つ)
//   - 目的地のそばが塞がっていたら、次へ飛ばさずに止まる (次の /goal_pose を待つ)
//   - 結果は /navigation/result (std_msgs/String) でマスターに返す。simple_goal_follower_node と同じ名前・同じ文字列:
//       "planned" = 目的地を受け取り経路を作った / "arrived" = 着いた / "blocked" = 塞がっていて止まった
//   - ゴール巡回を前提にした障害物チェック (フルスキャン)・task_node 連携は外した
//   - 障害物点群は PointCloud2 として受け、3D LiDAR/2D URG のどちらでも同じ回避処理を使う

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <deque>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <map>
#include <set>
#include <limits>

static inline double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

enum class Phase {
    IDLE,             // 目的地待ち (起動直後・到着後・塞がり停止後)
    HEADING,          // ゴール方向へ向く
    DRIVE,            // 前進
    HEADING_TO_GOAL,  // 目的地の向きに向く
};

namespace {
constexpr char kWindowName[] = "NEDO Path Planning Monitor";
constexpr int kMapWidth = 850;
constexpr int kPanelWidth = 430;
constexpr int kCanvasHeight = 950;

struct LabeledPoint {
  cv::Point2d position;
  std::string id;
};

struct TopicRow {
  std::string label;
  std::string topic;
  rclcpp::Time stamp;
  bool continuous;
  double freshness_limit;
};

struct TopicEndpointSnapshot {
  std::string type{"--"};
  std::vector<std::string> publishers;
  std::vector<std::string> subscribers;
};

}

class ManageNodeMaster : public rclcpp::Node {
public:
  ManageNodeMaster() : Node("manage_node_master")
  {
    // ログディレクトリ作成
    log_dir_ = makeLogDirectory();
    std::string pose_path = log_dir_ + "/pose_log.csv";
    log_pose_file_.open(pose_path, std::ios::out);
    log_pose_file_ << "time,x,y,yaw\n";
    RCLCPP_INFO(this->get_logger(), "Pose log saved to: %s", pose_path.c_str());

    // 走行・障害物回避パラメータ
    lookahead_dist_ = this->declare_parameter("lookahead_distance", 2.5);  // LookAhead距離
    lidar_history_sec_ = this->declare_parameter<double>("lidar_history_sec", 3.0);  // 過去点群の保持期間 [s]
    robot_radius_           = this->declare_parameter<double>("robot_radius", 1.8);            // ローバ半径 (最近点除外閾値)
    obstacle_avoid_radius_  = this->declare_parameter<double>("obstacle_avoid_radius", 0.5);   // 障害物回避円 r2
    path_band_half_width_   = this->declare_parameter<double>("path_band_half_width", 0.2);    // 経路バンド片側幅
    obstacle_cloud_topic_   = this->declare_parameter<std::string>("obstacle_cloud_topic", "/filtered_points_shuttle");
    planning_frame_         = this->declare_parameter<std::string>("planning_frame", "map");
    obstacle_z_min_         = this->declare_parameter<double>("obstacle_z_min", -0.2);
    obstacle_z_max_         = this->declare_parameter<double>("obstacle_z_max", 2.5);
    goal_block_radius_      = this->declare_parameter<double>("goal_block_radius", 0.5);
    goal_block_min_points_  = this->declare_parameter<int>("goal_block_min_points", 120);
    maximum_cloud_age_sec_  = this->declare_parameter<double>("maximum_obstacle_cloud_age", 0.5);
    obstacle_tf_timeout_ms_ = this->declare_parameter<int>("obstacle_tf_timeout_ms", 250);
    body_length_            = this->declare_parameter<double>("ui.body_length", 1.8);
    body_width_             = this->declare_parameter<double>("ui.body_width", 1.0);
    urg_mount_x_            = this->declare_parameter<double>("ui.urg_x", 0.0);
    urg_mount_y_            = this->declare_parameter<double>("ui.urg_y", 0.0);
    arm_base_x_             = this->declare_parameter<double>("ui.arm_base_x", 0.9);
    arm_base_y_             = this->declare_parameter<double>("ui.arm_base_y", 0.0);
    arm_yaw_                = this->declare_parameter<double>("ui.arm_yaw_deg", 90.0) * M_PI / 180.0;
    arm_length_             = this->declare_parameter<double>("ui.arm_length", 3.0);
    pickup_approach_limit_  = this->declare_parameter<double>("ui.pickup_approach_limit", 0.30);
    velocity_topic_         = this->declare_parameter<std::string>("ui.velocity_topic", "/body_twist");
    velocity_vector_horizon_= this->declare_parameter<double>("ui.velocity_vector_horizon", 2.0);
    angular_arc_radius_      = this->declare_parameter<double>("ui.angular_arc_radius", 1.0);
    voicevox_url_           = this->declare_parameter<std::string>("voicevox_url", "http://localhost:50021");  // 空文字 / none で無効
    if (voicevox_url_ == "none") voicevox_url_.clear();  // ros2 launch は空の値を渡せないので none で無効にする

    // 確認用の窓は画面があるときだけ開く (DISPLAY が無いと OpenCV が例外で落ちるため)
    const char *display = std::getenv("DISPLAY");
    show_window_ = display && *display;
    if (!show_window_) {
      RCLCPP_WARN(this->get_logger(), "画面が無いので、確認用の窓は開きません。経路計画はそのまま動きます");
    }

    if (planning_frame_.empty() || obstacle_cloud_topic_.empty() ||
        lidar_history_sec_ < 0.0 || obstacle_z_min_ > obstacle_z_max_ ||
        goal_block_radius_ <= 0.0 || goal_block_min_points_ <= 0 ||
        maximum_cloud_age_sec_ <= 0.0 || obstacle_tf_timeout_ms_ <= 0 ||
        velocity_vector_horizon_ <= 0.0 ||
        angular_arc_radius_ <= 0.0 || arm_length_ <= 0.0 || pickup_approach_limit_ <= 0.0) {
      throw std::runtime_error("invalid manage_node_master parameter");
    }

    //  仮自己位置
    car_pos_ = {0.0, 0.0};
    car_yaw_ = 0.0;
    received_pose_ = false;

    // Publisher
    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      heading_target_pub_ = this->create_publisher<std_msgs::msg::Float32>("/heading_target", latched);
    }
    target_pub_         = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    cmd_source_pub_     = this->create_publisher<std_msgs::msg::String>("/cmd_source", 10);
    result_pub_         = this->create_publisher<std_msgs::msg::String>("/navigation/result", 10);

    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/current_goal", latched);
    }

    // Subscriber
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&ManageNodeMaster::onGoalPose, this, std::placeholders::_1));

    pose2d_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      std::bind(&ManageNodeMaster::onPose2D, this, std::placeholders::_1));

    // RViz と同じランドマーク地図。frame_id に marker ID が入る既存契約を使う。
    landmark_map_sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
      "/landmark_map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ManageNodeMaster::onLandmarkMap, this, std::placeholders::_1));

    // 検出器が現在フレームで認識したランドマーク。地図側と色を分けて重ねる。
    matched_marker_sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
      "/matched_marker", rclcpp::SensorDataQoS(),
      std::bind(&ManageNodeMaster::onMatchedMarkers, this, std::placeholders::_1));

    // 実機から得られる実速度と、下位系へ送る指令速度を別々に監視する。
    body_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ManageNodeMaster::onBodyTwist, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ManageNodeMaster::onCmdVel, this, std::placeholders::_1));
    cmd_source_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cmd_source", 10,
      std::bind(&ManageNodeMaster::onCmdSource, this, std::placeholders::_1));
    pickup_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/pickup_approach/path", 10,
      std::bind(&ManageNodeMaster::onPickupPath, this, std::placeholders::_1));
    pickup_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pickup_approach/goal", 10,
      std::bind(&ManageNodeMaster::onPickupGoal, this, std::placeholders::_1));
    pickup_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/pickup_approach/cmd_vel", 10,
      std::bind(&ManageNodeMaster::onPickupCmd, this, std::placeholders::_1));
    pickup_planner_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pickup_approach/planner_status", 10,
      std::bind(&ManageNodeMaster::onPickupPlannerStatus, this, std::placeholders::_1));
    pickup_too_close_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/scrap_pile_skirt_node/too_close", rclcpp::SensorDataQoS(),
      std::bind(&ManageNodeMaster::onPickupTooClose, this, std::placeholders::_1));
    pickup_nearest_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/scrap_pile_skirt_node/nearest_point", rclcpp::SensorDataQoS(),
      std::bind(&ManageNodeMaster::onPickupNearest, this, std::placeholders::_1));
    navigation_result_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/navigation/result", 10,
      std::bind(&ManageNodeMaster::onNavigationResult, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_, 10,
      std::bind(&ManageNodeMaster::onLidar, this, std::placeholders::_1));

    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_markers", 10,
      std::bind(&ManageNodeMaster::onMarkerArray, this, std::placeholders::_1));

    marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/aruco_marker_ids", 10,
      std::bind(&ManageNodeMaster::onMarkerIDs, this, std::placeholders::_1));

    heading_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/heading_done", 10,
      std::bind(&ManageNodeMaster::onHeadingDone, this, std::placeholders::_1));

    drive_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/drive_done", 10,
      std::bind(&ManageNodeMaster::onDriveDone, this, std::placeholders::_1));

    // センサ座標から計画座標へ変換する TF buffer
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //  Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ManageNodeMaster::onTimer, this));

    // 初期フェーズ: 目的地が来るまで動かない
    phase_ = Phase::IDLE;

    std_msgs::msg::String src;
    src.data = "none";
    cmd_source_pub_->publish(src);

    RCLCPP_INFO(this->get_logger(),
                "準備完了: cloud=%s planning_frame=%s history=%.2fs goal_block=%dpts/%.2fm",
                obstacle_cloud_topic_.c_str(), planning_frame_.c_str(), lidar_history_sec_,
                goal_block_min_points_, goal_block_radius_);
  }

  ~ManageNodeMaster(){
    if (log_pose_file_.is_open())
      log_pose_file_.close();
  }

private:
  // ============================================================
  //                      VOICEVOX スピーカ
  // ============================================================
  std::string makeLogDirectory()
  {
    const char* home = std::getenv("HOME");
    if (!home) home = "/tmp"; // fallback

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << home << "/ros2_logs/" << std::put_time(&tm, "%Y-%m-%d");
    std::string dir_path = oss.str();

    std::filesystem::create_directories(dir_path);
    return dir_path;
  }

  void playVoice(const std::string &text)
  {
    if (voicevox_url_.empty()) {
      RCLCPP_INFO(this->get_logger(), "[VOICE disabled] %s", text.c_str());
      return;
    }

    auto url_encode = [&](const std::string &s) {
      std::ostringstream encoded;
      encoded << std::hex << std::uppercase;
      for (unsigned char c : s) {
        if (isalnum(c) || c=='-'||c=='_'||c=='.'||c=='~')
          encoded << c;
        else
          encoded << '%' << std::setw(2) << int(c);
      }
      return encoded.str();
    };

    std::string encoded_text = url_encode(text);
    std::string cmd =
      "curl -s -X POST '" + voicevox_url_ + "/audio_query?text=" + encoded_text +
      "&speaker=3' "
      "| curl -s -H 'Content-Type: application/json' -X POST -d @- "
      "'" + voicevox_url_ + "/synthesis?speaker=3' "
      "-o /tmp/voice.wav && aplay -q /tmp/voice.wav";

    RCLCPP_INFO(this->get_logger(), "[VOICE] %s", text.c_str());
    std::system(cmd.c_str());
  }

  // ============================================================
  //                  マスターへの結果
  // ============================================================
  void publishResult(const std::string &result)
  {
    std_msgs::msg::String msg;
    msg.data = result;
    result_pub_->publish(msg);
    last_result_ = result;
  }

  // ============================================================
  //                  目的地受信 (マスターから)
  // ============================================================
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_goal_receive_time_ = this->now();
    // 新しい走行を始めるまで、直前の停止理由は画面に保持する。
    last_stop_reason_.clear();
    if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) {
      RCLCPP_ERROR(this->get_logger(),
                   "目的地の座標系が不一致です (%s != %s)。目的地を受理しません",
                   msg->header.frame_id.c_str(), planning_frame_.c_str());
      // マスターとの既存I/O契約を保つため、未知の結果文字列は増やさず blocked を返す。
      publishResult("blocked");
      return;
    }

    const auto &q = msg->pose.orientation;
    goal_ = cv::Point2d(msg->pose.position.x, msg->pose.position.y);
    goal_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_goal_ = true;
    have_last_selected_ = false;

    RCLCPP_INFO(this->get_logger(),
                "目的地 (%.2f, %.2f, %.0f°) が届きました。向きを合わせてから進みます",
                goal_.x, goal_.y, goal_yaw_ * 180.0 / M_PI);

    // 経路生成と HEADING 開始 (自己位置がまだなら onPose2D で向きを送る)
    if (received_pose_) {
      updatePathToCurrentGoal();
      std_msgs::msg::Float32 msg_target;
      msg_target.data = std::atan2(goal_.y - car_pos_.y, goal_.x - car_pos_.x);
      heading_target_pub_->publish(msg_target);
      last_heading_target_ = msg_target.data;
      have_heading_target_ = true;
      heading_sent_ = true;
    }

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;
    publishResult("planned");
  }

  // ============================================================
  //                  ゴール近傍に障害物があるか
  // ============================================================
  bool isGoalBlocked()
  {
    last_goal_block_count_ = 0;
    if (!received_pose_) return false;
    if (lidar_points_.empty()) return false;

    cv::Point2d g = goal_;
    int cnt = 0;

    for (auto &p : lidar_points_) {
      double d_goal = cv::norm(p - g);
      if (d_goal < goal_block_radius_) cnt++;
    }
    last_goal_block_count_ = cnt;

    if (cnt >= goal_block_min_points_) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal blocked: %d points within %.2f m", cnt, goal_block_radius_);
      return true;
    }
    return false;
  }

  // ============================================================
  //                  ゴールが塞がっていたら止まる
  // ============================================================
  void stopAtBlockedGoal()
  {
    std_msgs::msg::String stop_src;
    stop_src.data = "none";
    cmd_source_pub_->publish(stop_src);

    phase_ = Phase::IDLE;
    std::ostringstream reason;
    reason << "GOAL BLOCKED: " << last_goal_block_count_ << " pts within "
           << std::fixed << std::setprecision(2) << goal_block_radius_ << " m";
    last_stop_reason_ = reason.str();
    last_stop_time_ = this->now();
    publishResult("blocked");

    RCLCPP_WARN(this->get_logger(),
                "目的地のそばに障害物があるので止まりました。次の目的地を待ちます");
    playVoice("目的地の付近に障害物があります。停止します。");
  }

  bool obstacleCloudFresh() const
  {
    if (!received_cloud_) return false;
    const double age = (this->now() - latest_cloud_receive_time_).seconds();
    return age >= 0.0 && age <= maximum_cloud_age_sec_;
  }

  void stopForObstacleCloudTimeout()
  {
    std_msgs::msg::String stop_src;
    stop_src.data = "none";
    cmd_source_pub_->publish(stop_src);
    phase_ = Phase::IDLE;
    std::ostringstream reason;
    reason << "SAFETY STOP: obstacle cloud age > " << std::fixed << std::setprecision(2)
           << maximum_cloud_age_sec_ << " s";
    last_stop_reason_ = reason.str();
    last_stop_time_ = this->now();
    publishResult("blocked");
    RCLCPP_ERROR(this->get_logger(),
                 "障害物点群が %.2f 秒以内に届いていないため安全停止しました",
                 maximum_cloud_age_sec_);
  }

  // ============================================================
  //                        Heading Done
  // ============================================================
  void onHeadingDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    last_heading_done_receive_time_ = this->now();
    last_heading_done_value_ = msg->data;
    if (!msg->data) return;

    if (phase_ == Phase::HEADING_TO_GOAL) {
      // 到着: 止まってマスターに知らせる
      std_msgs::msg::String src;
      src.data = "none";
      cmd_source_pub_->publish(src);

      phase_ = Phase::IDLE;
      last_stop_reason_ = "NORMAL STOP: goal arrived";
      last_stop_time_ = this->now();
      publishResult("arrived");

      RCLCPP_INFO(this->get_logger(), "目的地に着きました。次の目的地を待ちます");
      return;
    }

    if (phase_ == Phase::HEADING) {
      // 経路生成
      updatePathToCurrentGoal();
      heading_sent_ = false;

      std_msgs::msg::String src;
      src.data = "drive";
      cmd_source_pub_->publish(src);
      phase_ = Phase::DRIVE;
      return;
    }
  }

  // ============================================================
  //                        Drive Done
  // ============================================================
  void onDriveDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    last_drive_done_receive_time_ = this->now();
    last_drive_done_value_ = msg->data;
    if (!msg->data) return;
    if (phase_ != Phase::DRIVE) return;

    // 目的地の向きに合わせる
    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = goal_yaw_;
    heading_target_pub_->publish(msg_yaw);
    last_heading_target_ = msg_yaw.data;
    have_heading_target_ = true;

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_GOAL;
  }

  // ============================================================
  //                         Pose 受信
  // ============================================================
  void onPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    last_pose_receive_time_ = this->now();
    car_pos_ = cv::Point2d(msg->x, msg->y);
    car_yaw_ = wrapToPi(msg->theta);

    // 初回受信処理
    if (!received_pose_) {
        received_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "初回Pose受信");
        if (has_goal_) updatePathToCurrentGoal();
    }

    trajectory_.push_back(car_pos_);
    if (trajectory_.size() > 200)
      trajectory_.erase(trajectory_.begin());

    // 自己位置より先に目的地が届いていた場合の、最初の heading_target
    if (phase_ == Phase::HEADING && !heading_sent_) {
      double dx = goal_.x - car_pos_.x;
      double dy = goal_.y - car_pos_.y;
      double theta_target = std::atan2(dy, dx);

      std_msgs::msg::Float32 msg_target;
      msg_target.data = theta_target;
      heading_target_pub_->publish(msg_target);
      last_heading_target_ = msg_target.data;
      have_heading_target_ = true;

      heading_sent_ = true;
    }
  }

  // ============================================================
  //                       Marker / ID
  // ============================================================
  void onMarkerArray(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    marker_points_.clear();
    for (auto &p : msg->poses)
      marker_points_.emplace_back(p.position.x, p.position.y);
  }

  void onMarkerIDs(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    marker_ids_ = msg->data;
  }

  void onLandmarkMap(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
  {
    landmark_map_.clear();
    landmark_map_.reserve(msg->array.size());
    for (const auto & pose : msg->array) {
      landmark_map_.push_back({
        cv::Point2d(pose.pose.position.x, pose.pose.position.y),
        pose.header.frame_id});
    }
    last_landmark_map_receive_time_ = this->now();
  }

  void onMatchedMarkers(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
  {
    matched_markers_base_.clear();
    matched_markers_base_.reserve(msg->array.size());
    for (const auto & pose : msg->array) {
      matched_markers_base_.push_back({
        cv::Point2d(pose.pose.position.x, pose.pose.position.y),
        pose.header.frame_id});
    }
    last_matched_marker_receive_time_ = this->now();
  }

  void onBodyTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    measured_twist_ = msg->twist;
    last_body_twist_receive_time_ = this->now();
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    commanded_twist_ = *msg;
    last_cmd_vel_receive_time_ = this->now();
  }

  void onCmdSource(const std_msgs::msg::String::SharedPtr msg)
  {
    current_cmd_source_ = msg->data.empty() ? "(empty)" : msg->data;
    last_cmd_source_receive_time_ = this->now();
  }

  void onPickupPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) {
      RCLCPP_WARN(this->get_logger(), "Pickup path frame mismatch: %s != %s",
                  msg->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    pickup_path_.clear();
    pickup_path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      pickup_path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    last_pickup_path_receive_time_ = this->now();
  }

  void onPickupGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) return;
    pickup_goal_ = {msg->pose.position.x, msg->pose.position.y};
    have_pickup_goal_ = true;
    last_pickup_goal_receive_time_ = this->now();
  }

  void onPickupCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    pickup_commanded_twist_ = *msg;
    last_pickup_cmd_receive_time_ = this->now();
  }

  void onPickupPlannerStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    pickup_planner_status_ = msg->data.empty() ? "(empty)" : msg->data;
    last_pickup_planner_status_receive_time_ = this->now();

    // OpenCVの標準フォントは日本語を表示できないため、既知の失敗コードは
    // ASCIIだけの停止理由へ変換して上部バナーに保持する。
    if (msg->data.find("GOAL_CLEARANCE_VIOLATION") != std::string::npos) {
      last_stop_reason_ = "PICKUP PLAN BLOCKED: GOAL CLEARANCE VIOLATION";
      last_stop_time_ = this->now();
    } else if (msg->data.rfind("FAILED", 0) == 0) {
      last_stop_reason_ = "PICKUP PLAN FAILED: see planner status";
      last_stop_time_ = this->now();
    }
  }

  void onPickupTooClose(const std_msgs::msg::Bool::SharedPtr msg)
  {
    pickup_too_close_ = msg->data;
    last_pickup_too_close_receive_time_ = this->now();
    if (!msg->data) return;

    std::ostringstream reason;
    reason << "PICKUP SAFETY: TOO CLOSE";
    if (have_pickup_nearest_distance_) {
      reason << "  d=" << std::fixed << std::setprecision(2)
             << pickup_nearest_distance_ << " m";
    }
    reason << " < " << std::fixed << std::setprecision(2)
           << pickup_approach_limit_ << " m";
    last_stop_reason_ = reason.str();
    last_stop_time_ = this->now();
  }

  void onPickupNearest(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // nearest_pointはbase_footprint基準。REP-103車体矩形 (+X前, +Y左) との距離を表示する。
    const double half_x = 0.5 * body_length_;
    const double half_y = 0.5 * body_width_;
    const double dx = std::max(std::abs(msg->point.x) - half_x, 0.0);
    const double dy = std::max(std::abs(msg->point.y) - half_y, 0.0);
    pickup_nearest_distance_ = std::hypot(dx, dy);
    have_pickup_nearest_distance_ = true;
    last_pickup_nearest_receive_time_ = this->now();
  }

  void onNavigationResult(const std_msgs::msg::String::SharedPtr msg)
  {
    last_result_ = msg->data.empty() ? "(empty)" : msg->data;
    last_navigation_result_receive_time_ = this->now();
    if (msg->data == "blocked") {
      // blockedは直前フェーズのNORMAL STOPより新しく、かつ異常停止なので必ず優先する。
      if (pickup_planner_status_.find("GOAL_CLEARANCE_VIOLATION") != std::string::npos) {
        last_stop_reason_ = "PICKUP PLAN BLOCKED: GOAL CLEARANCE VIOLATION";
      } else if (pickup_too_close_) {
        std::ostringstream reason;
        reason << "PICKUP SAFETY: TOO CLOSE";
        if (have_pickup_nearest_distance_) {
          reason << "  d=" << std::fixed << std::setprecision(2)
                 << pickup_nearest_distance_ << " m";
        }
        reason << " < " << std::fixed << std::setprecision(2)
               << pickup_approach_limit_ << " m";
        last_stop_reason_ = reason.str();
      } else if (last_stop_reason_.empty() ||
                 last_stop_reason_.rfind("NORMAL STOP", 0) == 0) {
        last_stop_reason_ = "STOPPED: navigation blocked (detail unavailable)";
      }
      last_stop_time_ = this->now();
    }
  }

  // ============================================================
  //                         LiDAR（通常）
  // ============================================================
  void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_obstacle_frame_ = msg->header.frame_id.empty() ? "(empty)" : msg->header.frame_id;
    last_obstacle_raw_points_ = static_cast<size_t>(msg->width) * msg->height;
    if (!received_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "自己位置未受信のため LiDAR 点群を描画できません");
      return;
    }

    // 3D/2D LiDAR の点群を、計測時刻における計画座標系へ TF 変換
    sensor_msgs::msg::PointCloud2 cloud_planning;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        planning_frame_, msg->header.frame_id,
        rclcpp::Time(msg->header.stamp),
        std::chrono::milliseconds(obstacle_tf_timeout_ms_));
      tf2::doTransform(*msg, cloud_planning, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF 変換失敗 (%s → %s): %s",
                           msg->header.frame_id.c_str(), planning_frame_.c_str(), ex.what());
      return;
    }

    // 現在フレームの点群を計画座標系で抽出 (URG は z=0 のため同じ処理を通る)
    const auto now_stamp = this->now();
    LidarFrame frame;
    frame.stamp = now_stamp;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_planning, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_planning, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_planning, "z");
    last_obstacle_samples_.clear();
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) continue;
      if (*iter_z < obstacle_z_min_ || *iter_z > obstacle_z_max_) continue;
      frame.points.emplace_back(*iter_x, *iter_y);
      if (last_obstacle_samples_.size() < 3) {
        last_obstacle_samples_.emplace_back(*iter_x, *iter_y);
      }
    }

    latest_cloud_receive_time_ = now_stamp;
    received_cloud_ = true;

    // 履歴に追加し、lidar_history_sec_ より古いフレームを破棄
    lidar_history_.push_back(std::move(frame));
    while (!lidar_history_.empty() &&
           (now_stamp - lidar_history_.front().stamp).seconds() > lidar_history_sec_) {
      lidar_history_.pop_front();
    }

    // 全履歴フレームをフラット化して lidar_points_ を再構築
    lidar_points_.clear();
    for (const auto & f : lidar_history_) {
      lidar_points_.insert(lidar_points_.end(), f.points.begin(), f.points.end());
    }
  }

  // ============================================================
  //                        経路生成
  // ============================================================
  void updatePathToCurrentGoal()
  {
    path_.clear();
    cv::Point2d start = car_pos_;
    cv::Point2d goal  = goal_;
    const int N = 20;
    for (int i=0; i<=N; ++i) {
      double t = static_cast<double>(i) / N;
      double x = (1-t)*start.x + t*goal.x;
      double y = (1-t)*start.y + t*goal.y;
      path_.emplace_back(x, y);
    }
    heading_sent_ = false;
  }

  // Pure Pursuit 用の注視点
  cv::Point2d computeLookaheadPoint()
  {
    const double r = lookahead_dist_;
    const cv::Point2d C = car_pos_;

    cv::Point2d best_pt = goal_;
    bool found = false;

    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::Point2d P1 = path_[i];
      cv::Point2d P2 = path_[i+1];
      cv::Point2d d  = P2 - P1;

      double a = d.dot(d);
      double b = 2 * ((P1.x-C.x)*d.x + (P1.y-C.y)*d.y);
      double c = (P1.x-C.x)*(P1.x-C.x) + (P1.y-C.y)*(P1.y-C.y) - r*r;

      double D = b*b - 4*a*c;
      if (D < 0) continue;

      double sqrtD = std::sqrt(D);
      double t1 = (-b + sqrtD) / (2*a);
      double t2 = (-b - sqrtD) / (2*a);

      std::vector<double> ts;
      if (t1 >= 0.0 && t1 <= 1.0) ts.push_back(t1);
      if (t2 >= 0.0 && t2 <= 1.0) ts.push_back(t2);

      if (!ts.empty()) {
        double t = *std::max_element(ts.begin(), ts.end());
        best_pt = P1 + t*d;
        found = true;
      }
    }

    double dist_to_goal = cv::norm(goal_ - car_pos_);
    if (dist_to_goal < lookahead_dist_) {
      best_pt = goal_;
      found = true;
    }

    return found ? best_pt : goal_;
  }

  // ============================================================
  //                         可視化 Timer
  // ============================================================
  cv::Point toPix(const cv::Point2d& pt, double scale, const cv::Point2d& offset) {
    double x_vis = pt.x;
    double y_vis = -pt.y;
    return cv::Point(static_cast<int>(offset.x + x_vis*scale),
                     static_cast<int>(offset.y + y_vis*scale));
  }

  void showCanvas(const cv::Mat &canvas)
  {
    if (!show_window_) return;
    cv::imshow(kWindowName, canvas);
    cv::waitKey(1);
  }

  std::string phaseName() const
  {
    switch (phase_) {
      case Phase::IDLE: return "IDLE";
      case Phase::HEADING: return "HEADING";
      case Phase::DRIVE: return "DRIVE";
      case Phase::HEADING_TO_GOAL: return "FINAL HEADING";
    }
    return "UNKNOWN";
  }

  std::string ageText(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) return "--";
    const double age = std::max(0.0, (this->now() - stamp).seconds());
    std::ostringstream text;
    text << std::fixed << std::setprecision(age < 10.0 ? 1 : 0) << age << "s";
    return text.str();
  }

  bool isFresh(const rclcpp::Time & stamp, double limit) const
  {
    if (stamp.nanoseconds() == 0) return false;
    const double age = (this->now() - stamp).seconds();
    return age >= 0.0 && age <= limit;
  }

  std::vector<TopicRow> topicRows() const
  {
    const rclcpp::Time no_stamp(0, 0, RCL_ROS_TIME);
    return {
      {"robot pose", "/robot_pose2d", last_pose_receive_time_, true, 1.0},
      {"actual vel", velocity_topic_, last_body_twist_receive_time_, true, 0.5},
      {"obstacles", obstacle_cloud_topic_, latest_cloud_receive_time_, true, maximum_cloud_age_sec_},
      {"goal", "/goal_pose", last_goal_receive_time_, false, 0.0},
      {"heading target", "/heading_target", no_stamp, false, 0.0},
      {"target point", "/target_point", no_stamp, false, 0.0},
      {"cmd source", "/cmd_source", last_cmd_source_receive_time_, false, 0.0},
      {"heading done", "/heading_done", last_heading_done_receive_time_, false, 0.0},
      {"drive done", "/drive_done", last_drive_done_receive_time_, false, 0.0},
      {"cmd vel", "/cmd_vel", last_cmd_vel_receive_time_, true, 0.5},
      {"pickup path", "/pickup_approach/path", last_pickup_path_receive_time_, false, 0.0},
      {"pickup goal", "/pickup_approach/goal", last_pickup_goal_receive_time_, false, 0.0},
      {"pickup cmd", "/pickup_approach/cmd_vel", last_pickup_cmd_receive_time_, true, 0.5},
      {"pickup plan", "/pickup_approach/planner_status", last_pickup_planner_status_receive_time_, false, 0.0},
      {"pile safety", "/scrap_pile_skirt_node/too_close", last_pickup_too_close_receive_time_, true, 0.5},
      {"pile nearest", "/scrap_pile_skirt_node/nearest_point", last_pickup_nearest_receive_time_, true, 0.5},
      {"nav result", "/navigation/result", last_navigation_result_receive_time_, false, 0.0},
      {"landmark map", "/landmark_map", last_landmark_map_receive_time_, false, 0.0},
      {"matched LM", "/matched_marker", last_matched_marker_receive_time_, true, 1.0}};
  }

  static std::string endpointNodeName(const rclcpp::TopicEndpointInfo & endpoint)
  {
    std::string ns = endpoint.node_namespace();
    if (ns.empty() || ns == "/") return "/" + endpoint.node_name();
    if (ns.back() == '/') ns.pop_back();
    return ns + "/" + endpoint.node_name();
  }

  static void sortUnique(std::vector<std::string> & values)
  {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
  }

  void updateSelectedTopicDetail()
  {
    selected_topic_detail_ = TopicEndpointSnapshot{};
    if (selected_topic_.empty()) return;
    try {
      const auto publishers = this->get_publishers_info_by_topic(selected_topic_);
      for (const auto & endpoint : publishers) {
        selected_topic_detail_.publishers.push_back(endpointNodeName(endpoint));
        if (!endpoint.topic_type().empty()) selected_topic_detail_.type = endpoint.topic_type();
      }
      const auto subscribers = this->get_subscriptions_info_by_topic(selected_topic_);
      for (const auto & endpoint : subscribers) {
        selected_topic_detail_.subscribers.push_back(endpointNodeName(endpoint));
        if (selected_topic_detail_.type == "--" && !endpoint.topic_type().empty()) {
          selected_topic_detail_.type = endpoint.topic_type();
        }
      }
      sortUnique(selected_topic_detail_.publishers);
      sortUnique(selected_topic_detail_.subscribers);
    } catch (const std::exception & error) {
      selected_topic_detail_.type = std::string("graph error: ") + error.what();
    }
  }

  static std::string compact(const std::string & value, size_t maximum = 52)
  {
    return value.size() <= maximum ? value : value.substr(0, maximum - 3) + "...";
  }

  static std::string asciiDisplay(const std::string & value, size_t maximum = 52)
  {
    std::string out;
    for (const unsigned char c : value) {
      if (c >= 0x80) break;
      if (c >= 0x20 || c == '\t') out.push_back(static_cast<char>(c));
    }
    while (!out.empty() && (out.back() == ' ' || out.back() == ':' || out.back() == '|')) {
      out.pop_back();
    }
    if (out.empty()) out = "(non-ASCII detail: see terminal log)";
    return compact(out, maximum);
  }

  static std::string number(double value, int precision = 3)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
  }

  std::vector<std::string> twistValueLines(const geometry_msgs::msg::Twist & twist) const
  {
    return {
      "linear  x=" + number(twist.linear.x) + " y=" + number(twist.linear.y) +
        " z=" + number(twist.linear.z),
      "angular x=" + number(twist.angular.x) + " y=" + number(twist.angular.y) +
        " z=" + number(twist.angular.z)};
  }

  std::vector<std::string> topicValueLines(const std::string & topic) const
  {
    if (topic == "/robot_pose2d") {
      return {"x=" + number(car_pos_.x) + " y=" + number(car_pos_.y) +
        " theta=" + number(car_yaw_) + " rad"};
    }
    if (topic == velocity_topic_) return twistValueLines(measured_twist_);
    if (topic == "/cmd_vel") return twistValueLines(commanded_twist_);
    if (topic == "/pickup_approach/cmd_vel") return twistValueLines(pickup_commanded_twist_);
    if (topic == obstacle_cloud_topic_) {
      std::vector<std::string> lines = {
        "frame=" + last_obstacle_frame_ + " raw=" + std::to_string(last_obstacle_raw_points_) +
          " accumulated=" + std::to_string(lidar_points_.size())};
      for (size_t index = 0; index < last_obstacle_samples_.size(); ++index) {
        lines.push_back("p" + std::to_string(index) + " map=(" +
          number(last_obstacle_samples_[index].x, 2) + "," +
          number(last_obstacle_samples_[index].y, 2) + ")");
      }
      return lines;
    }
    if (topic == "/goal_pose") {
      return has_goal_ ? std::vector<std::string>{"map x=" + number(goal_.x) + " y=" +
        number(goal_.y) + " yaw=" + number(goal_yaw_) + " rad"} :
        std::vector<std::string>{"no goal received"};
    }
    if (topic == "/heading_target") {
      return {have_heading_target_ ? "data=" + number(last_heading_target_) + " rad" : "no value"};
    }
    if (topic == "/target_point") {
      return {have_target_point_ ? "map x=" + number(last_target_point_.x) + " y=" +
        number(last_target_point_.y) : "no value"};
    }
    if (topic == "/cmd_source") return {"data=\"" + current_cmd_source_ + "\""};
    if (topic == "/heading_done") return {"data=" + std::string(last_heading_done_value_ ? "true" : "false")};
    if (topic == "/drive_done") return {"data=" + std::string(last_drive_done_value_ ? "true" : "false")};
    if (topic == "/pickup_approach/path") {
      std::vector<std::string> lines = {"poses=" + std::to_string(pickup_path_.size()) + " frame=map"};
      if (!pickup_path_.empty()) {
        lines.push_back("first=(" + number(pickup_path_.front().x, 2) + "," +
          number(pickup_path_.front().y, 2) + ")");
        lines.push_back("last =(" + number(pickup_path_.back().x, 2) + "," +
          number(pickup_path_.back().y, 2) + ")");
      }
      return lines;
    }
    if (topic == "/pickup_approach/goal") {
      return {have_pickup_goal_ ? "map x=" + number(pickup_goal_.x) + " y=" +
        number(pickup_goal_.y) : "no goal received"};
    }
    if (topic == "/pickup_approach/planner_status") {
      return {"data=\"" + asciiDisplay(pickup_planner_status_, 46) + "\""};
    }
    if (topic == "/scrap_pile_skirt_node/too_close") {
      return {"data=" + std::string(pickup_too_close_ ? "true" : "false") +
        "  limit=" + number(pickup_approach_limit_, 2) + " m"};
    }
    if (topic == "/scrap_pile_skirt_node/nearest_point") {
      return {have_pickup_nearest_distance_ ?
        "body clearance=" + number(pickup_nearest_distance_, 2) + " m" : "no value"};
    }
    if (topic == "/navigation/result") return {"data=\"" + last_result_ + "\""};
    if (topic == "/landmark_map") {
      std::string ids = "ids=";
      for (size_t index = 0; index < std::min<size_t>(landmark_map_.size(), 5); ++index) {
        if (index) ids += ",";
        ids += landmark_map_[index].id;
      }
      return {"poses=" + std::to_string(landmark_map_.size()), compact(ids)};
    }
    if (topic == "/matched_marker") {
      std::string ids = "visible=";
      for (size_t index = 0; index < std::min<size_t>(matched_markers_base_.size(), 5); ++index) {
        if (index) ids += ",";
        ids += matched_markers_base_[index].id;
      }
      return {"poses=" + std::to_string(matched_markers_base_.size()), compact(ids)};
    }
    return {"value not captured"};
  }

  static void mouseCallback(int event, int x, int y, int, void * userdata)
  {
    if (event != cv::EVENT_LBUTTONDOWN || userdata == nullptr) return;
    static_cast<ManageNodeMaster *>(userdata)->onMouseClick(x, y);
  }

  void onMouseClick(int x, int y)
  {
    constexpr int first_row_top = 399;
    constexpr int row_height = 18;
    if (x < kMapWidth || y < first_row_top) return;
    const auto rows = topicRows();
    const int index = (y - first_row_top) / row_height;
    if (index < 0 || index >= static_cast<int>(rows.size())) return;
    const std::string clicked = rows[static_cast<size_t>(index)].topic;
    selected_topic_ = selected_topic_ == clicked ? "" : clicked;
    last_graph_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void updateGraphSnapshot()
  {
    const auto now = this->now();
    if (last_graph_update_.nanoseconds() != 0 &&
        (now - last_graph_update_).seconds() < 1.0) return;

    live_nodes_.clear();
    for (const auto & name : this->get_node_names()) live_nodes_.insert(name);

    topic_counts_.clear();
    for (const auto & row : topicRows()) {
      topic_counts_[row.topic] = {
        this->count_publishers(row.topic), this->count_subscribers(row.topic)};
    }
    updateSelectedTopicDetail();
    last_graph_update_ = now;
  }

  void drawGrid(cv::Mat & canvas, double scale, const cv::Point2d & offset,
                double xmin, double xmax, double ymin, double ymax)
  {
    const int x_first = static_cast<int>(std::floor(xmin));
    const int x_last = static_cast<int>(std::ceil(xmax));
    const int y_first = static_cast<int>(std::floor(ymin));
    const int y_last = static_cast<int>(std::ceil(ymax));
    for (int x = x_first; x <= x_last; ++x) {
      const int px = toPix({static_cast<double>(x), 0.0}, scale, offset).x;
      if (px < 0 || px >= kMapWidth) continue;
      const cv::Scalar color = x == 0 ? cv::Scalar(200, 200, 200) : cv::Scalar(225, 225, 225);
      cv::line(canvas, {px, 0}, {px, kCanvasHeight - 1}, color, 1);
    }
    for (int y = y_first; y <= y_last; ++y) {
      const int py = toPix({0.0, static_cast<double>(y)}, scale, offset).y;
      if (py < 0 || py >= kCanvasHeight) continue;
      const cv::Scalar color = y == 0 ? cv::Scalar(200, 200, 200) : cv::Scalar(225, 225, 225);
      cv::line(canvas, {0, py}, {kMapWidth - 1, py}, color, 1);
    }
  }

  void drawMapAxes(cv::Mat & canvas, double scale, const cv::Point2d & offset)
  {
    const cv::Point origin = toPix({0.0, 0.0}, scale, offset);
    const cv::Point x_end = toPix({1.5, 0.0}, scale, offset);
    const cv::Point y_end = toPix({0.0, 1.5}, scale, offset);
    cv::circle(canvas, origin, 3, cv::Scalar(105, 105, 105), -1, cv::LINE_AA);
    cv::arrowedLine(canvas, origin, x_end, cv::Scalar(95, 95, 175), 1,
                    cv::LINE_AA, 0, 0.10);
    cv::arrowedLine(canvas, origin, y_end, cv::Scalar(95, 155, 95), 1,
                    cv::LINE_AA, 0, 0.10);
    cv::putText(canvas, "map +X", x_end + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX,
                0.38, cv::Scalar(95, 95, 165), 1, cv::LINE_AA);
    cv::putText(canvas, "map +Y", y_end + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX,
                0.38, cv::Scalar(95, 145, 95), 1, cv::LINE_AA);
    cv::putText(canvas, "map (0,0)", origin + cv::Point(7, 18), cv::FONT_HERSHEY_SIMPLEX,
                0.39, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  }

  void drawRobotFootprint(cv::Mat & canvas, double scale, const cv::Point2d & offset)
  {
    // 実機の下位座標系に合わせ、長手 1.8 m をローカルY、短手 1.0 mをローカルXに置く。
    const double hx = body_width_ * 0.5;
    const double hy = body_length_ * 0.5;
    const std::vector<cv::Point2d> local = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
    std::vector<cv::Point> polygon;
    polygon.reserve(local.size());
    for (const auto & p : local) {
      polygon.push_back(toPix(nativeToMap(p), scale, offset));
    }
    cv::fillConvexPoly(canvas, polygon, cv::Scalar(245, 220, 190), cv::LINE_AA);
    cv::polylines(canvas, polygon, true, cv::Scalar(190, 90, 20), 2, cv::LINE_AA);

    // 下位座標系: +X=左(赤)、+Y=後方(緑)、したがって-Yが前進。
    const double axis_length = 1.2;
    const cv::Point origin = toPix(car_pos_, scale, offset);
    const cv::Point x_end = toPix(nativeToMap({axis_length, 0.0}), scale, offset);
    const cv::Point y_end = toPix(nativeToMap({0.0, axis_length}), scale, offset);
    const cv::Point forward_end = toPix(nativeToMap({0.0, -axis_length}), scale, offset);
    cv::arrowedLine(canvas, origin, x_end, cv::Scalar(90, 90, 180), 1,
                    cv::LINE_AA, 0, 0.10);
    cv::arrowedLine(canvas, origin, y_end, cv::Scalar(90, 155, 90), 1,
                    cv::LINE_AA, 0, 0.10);
    cv::putText(canvas, "+X left", x_end + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX,
                0.38, cv::Scalar(90, 90, 170), 1, cv::LINE_AA);
    cv::putText(canvas, "+Y back", y_end + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX,
                0.38, cv::Scalar(90, 145, 90), 1, cv::LINE_AA);
    // 前進方向の固定矢印は速度ベクトルと誤認されるため、文字だけで示す。
    cv::putText(canvas, "FRONT (-Y)", forward_end + cv::Point(4, 14),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(65, 65, 65), 1, cv::LINE_AA);
  }

  cv::Point2d localToMap(const cv::Point2d & local) const
  {
    const double c = std::cos(car_yaw_);
    const double s = std::sin(car_yaw_);
    return {car_pos_.x + c * local.x - s * local.y,
            car_pos_.y + s * local.x + c * local.y};
  }

  cv::Point2d nativeToMap(const cv::Point2d & native) const
  {
    // 下位 +X=左、下位 -Y=前進 → REP-103 (forward, left)=(-Y, +X)。
    const double forward = -native.y;
    const double left = native.x;
    return localToMap({forward, left});
  }

  void drawGrappleBeam(cv::Mat & canvas, double scale, const cv::Point2d & offset)
  {
    // ピックアップ計画と同じ REP-103 機体座標 (+X前、+Y左) で固定梁を描く。
    const cv::Point2d base_local{arm_base_x_, arm_base_y_};
    const cv::Point2d tip_local{
      arm_base_x_ + arm_length_ * std::cos(arm_yaw_),
      arm_base_y_ + arm_length_ * std::sin(arm_yaw_)};
    const cv::Point base = toPix(localToMap(base_local), scale, offset);
    const cv::Point tip = toPix(localToMap(tip_local), scale, offset);
    cv::arrowedLine(canvas, base, tip, cv::Scalar(20, 155, 245), 3,
                    cv::LINE_AA, 0, 0.05);
    cv::circle(canvas, base, 4, cv::Scalar(40, 130, 220), -1, cv::LINE_AA);
    cv::drawMarker(canvas, tip, cv::Scalar(15, 80, 235), cv::MARKER_DIAMOND,
                   12, 2, cv::LINE_AA);
    cv::putText(canvas, "GRAPPLE TIP", tip + cv::Point(6, -7),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(20, 80, 200), 1, cv::LINE_AA);
  }

  void drawLidarMounts(cv::Mat & canvas, double scale, const cv::Point2d & offset)
  {
    struct SensorStyle {
      std::string frame;
      std::string label;
      cv::Scalar color;
      cv::Point label_offset;
    };
    const std::vector<SensorStyle> lidars = {
      {"hesai_ot128", "OT128+IMU", cv::Scalar(45, 55, 235), {7, -12}},
      {"hesai_qt128", "QT128", cv::Scalar(235, 130, 45), {7, 17}}};

    for (const auto & lidar : lidars) {
      try {
        // map→LiDAR の最新TFなので、取付並進・姿勢と現在の車体姿勢を同時に反映する。
        const auto tf = tf_buffer_->lookupTransform(
          planning_frame_, lidar.frame, tf2::TimePointZero, std::chrono::milliseconds(5));
        const cv::Point p = toPix(
          {tf.transform.translation.x, tf.transform.translation.y}, scale, offset);
        cv::drawMarker(canvas, p, lidar.color, cv::MARKER_DIAMOND, 13, 2, cv::LINE_AA);
        cv::putText(canvas, lidar.label, p + lidar.label_offset,
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, lidar.color, 1, cv::LINE_AA);
      } catch (const tf2::TransformException &) {
        // TF未成立中は監視画面を止めず、成立したフレームだけを表示する。
      }
    }

    // URGには独立した取付TFがなく、scan_shadow_filterのx/yオフセットが取付値。
    const cv::Point urg = toPix(nativeToMap({urg_mount_x_, urg_mount_y_}), scale, offset);
    cv::drawMarker(canvas, urg, cv::Scalar(40, 175, 40), cv::MARKER_DIAMOND,
                   13, 2, cv::LINE_AA);
    cv::putText(canvas, "URG", urg + cv::Point(-38, 17), cv::FONT_HERSHEY_SIMPLEX,
                0.38, cv::Scalar(30, 150, 30), 1, cv::LINE_AA);
  }

  void drawTwistSymbol(cv::Mat & canvas, double scale, const cv::Point2d & offset,
                       const geometry_msgs::msg::Twist & twist,
                       const cv::Scalar & velocity_color, const cv::Scalar & yaw_color,
                       const std::string & label, bool lower_native)
  {
    // 最終 /cmd_vel は下位座標、Pickup controller出力はREP-103。
    // いずれも (forward,left) にそろえてからmapへ回す。角速度Zの符号は変えない。
    const double c = std::cos(car_yaw_);
    const double s = std::sin(car_yaw_);
    const double forward = lower_native ? -twist.linear.y : twist.linear.x;
    const double left = lower_native ? twist.linear.x : twist.linear.y;
    const double vx_map = c * forward - s * left;
    const double vy_map = s * forward + c * left;
    const double speed = std::hypot(twist.linear.x, twist.linear.y);
    const cv::Point origin = toPix(car_pos_, scale, offset);
    if (speed > 0.005) {
      const cv::Point2d end_world(
        car_pos_.x + velocity_vector_horizon_ * vx_map,
        car_pos_.y + velocity_vector_horizon_ * vy_map);
      const cv::Point end = toPix(end_world, scale, offset);
      cv::arrowedLine(canvas, origin, end, velocity_color, 2, cv::LINE_AA, 0, 0.14);
      std::ostringstream velocity_label;
      velocity_label << label << " v=" << std::fixed << std::setprecision(2)
                     << speed << "m/s";
      cv::putText(canvas, velocity_label.str(), end + cv::Point(5, -5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.36, velocity_color, 1, cv::LINE_AA);
    }

    // 正の角速度は map 上の反時計回り、負は時計回りとして円弧矢印で示す。
    const double omega = twist.angular.z;
    if (std::fabs(omega) > 0.005) {
      const double direction = omega > 0.0 ? 1.0 : -1.0;
      // 半径は固定し、表示時間内の回転量 |omega| * horizon で弧長を変える。
      const double arc_radius = angular_arc_radius_;
      const double span = std::min(
        std::fabs(omega) * velocity_vector_horizon_, 2.0 * M_PI);
      const double start = car_yaw_ - direction * 0.5 * span;
      std::vector<cv::Point> arc;
      constexpr int kSegments = 32;
      arc.reserve(kSegments + 1);
      for (int index = 0; index <= kSegments; ++index) {
        const double angle = start + direction * span * index / kSegments;
        arc.push_back(toPix(
          {car_pos_.x + arc_radius * std::cos(angle),
           car_pos_.y + arc_radius * std::sin(angle)}, scale, offset));
      }
      cv::polylines(canvas, arc, false, yaw_color, 2, cv::LINE_AA);
      const size_t arrow_start = arc.size() > 4 ? arc.size() - 4 : arc.size() - 2;
      cv::arrowedLine(canvas, arc[arrow_start], arc.back(), yaw_color,
                      2, cv::LINE_AA, 0, 0.24);
      std::ostringstream omega_label;
      omega_label << label << " w=" << std::fixed << std::setprecision(2)
                  << omega << "rad/s " << (omega > 0.0 ? "CCW" : "CW");
      cv::putText(canvas, omega_label.str(),
                  arc.back() + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX,
                  0.36, yaw_color, 1, cv::LINE_AA);
    }
  }

  void drawVelocityOverlays(cv::Mat & canvas, double scale, const cv::Point2d & offset)
  {
    if (isFresh(last_cmd_vel_receive_time_, 0.5)) {
      drawTwistSymbol(canvas, scale, offset, commanded_twist_,
                      cv::Scalar(190, 45, 190), cv::Scalar(20, 135, 220),
                      "cmd", true);
    }
    if (isFresh(last_pickup_cmd_receive_time_, 0.5)) {
      drawTwistSymbol(canvas, scale, offset, pickup_commanded_twist_,
                      cv::Scalar(165, 145, 25), cv::Scalar(55, 165, 75),
                      "pickup", false);
    }
  }

  void drawStatusDot(cv::Mat & canvas, int x, int y, bool ok) const
  {
    cv::circle(canvas, {x, y - 4}, 6,
               ok ? cv::Scalar(70, 190, 70) : cv::Scalar(55, 55, 220), -1, cv::LINE_AA);
  }

  void drawDashboard(cv::Mat & canvas)
  {
    const int x0 = kMapWidth;
    cv::rectangle(canvas, {x0, 0}, {x0 + kPanelWidth - 1, kCanvasHeight - 1},
                  cv::Scalar(28, 31, 36), -1);
    cv::line(canvas, {x0, 0}, {x0, kCanvasHeight}, cv::Scalar(90, 95, 105), 2);

    auto text = [&](const std::string & value, int x, int y, double size = 0.46,
                    cv::Scalar color = cv::Scalar(225, 230, 235), int thickness = 1) {
      cv::putText(canvas, value, {x, y}, cv::FONT_HERSHEY_SIMPLEX,
                  size, color, thickness, cv::LINE_AA);
    };
    text("PATH PLANNING MONITOR", x0 + 18, 30, 0.66, cv::Scalar(255, 255, 255), 2);
    text("phase", x0 + 18, 59, 0.42, cv::Scalar(150, 160, 170));
    text(phaseName(), x0 + 105, 59, 0.54, cv::Scalar(80, 220, 255), 2);

    std::ostringstream pose;
    pose << std::fixed << std::setprecision(2) << "pose  x " << car_pos_.x
         << "  y " << car_pos_.y << "  yaw " << car_yaw_ * 180.0 / M_PI << " deg";
    text(pose.str(), x0 + 18, 84);
    if (has_goal_) {
      std::ostringstream goal;
      goal << std::fixed << std::setprecision(2) << "goal  x " << goal_.x
           << "  y " << goal_.y << "  yaw " << goal_yaw_ * 180.0 / M_PI << " deg";
      text(goal.str(), x0 + 18, 106);
    } else {
      text("goal  waiting", x0 + 18, 106, 0.46, cv::Scalar(160, 165, 175));
    }
    text("cmd_source  " + current_cmd_source_, x0 + 18, 128, 0.47,
         cv::Scalar(100, 225, 255), 1);
    std::ostringstream command;
    command << std::fixed << std::setprecision(2)
            << "cmd lo (x,y)=(" << commanded_twist_.linear.x << "," << commanded_twist_.linear.y
            << ")  w=" << commanded_twist_.angular.z << "  "
            << (isFresh(last_cmd_vel_receive_time_, 0.5) ? "LIVE" : "STALE");
    text(command.str(), x0 + 18, 150, 0.40,
         isFresh(last_cmd_vel_receive_time_, 0.5) ? cv::Scalar(225, 105, 225)
                                                 : cv::Scalar(95, 95, 225));
    std::ostringstream pickup_command;
    pickup_command << std::fixed << std::setprecision(2)
                   << "pickup ROS(x,y)=(" << pickup_commanded_twist_.linear.x << ","
                   << pickup_commanded_twist_.linear.y << ") w="
                   << pickup_commanded_twist_.angular.z << " "
                   << (isFresh(last_pickup_cmd_receive_time_, 0.5) ? "LIVE" : "STALE");
    text(pickup_command.str(), x0 + 18, 172, 0.39,
         isFresh(last_pickup_cmd_receive_time_, 0.5) ? cv::Scalar(80, 205, 255)
                                                    : cv::Scalar(95, 95, 225));
    const std::string pickup_status = asciiDisplay(pickup_planner_status_, 42);
    text("pickup planner  " + pickup_status, x0 + 18, 194, 0.39,
         cv::Scalar(180, 205, 230));
    std::ostringstream cloud;
    cloud << "obstacles " << lidar_points_.size() << " pts  age " << ageText(latest_cloud_receive_time_)
          << "  result " << last_result_;
    text(cloud.str(), x0 + 18, 216, 0.40);

    if (!last_stop_reason_.empty()) {
      const bool normal_stop = last_stop_reason_.rfind("NORMAL STOP", 0) == 0;
      const cv::Scalar fill = normal_stop ? cv::Scalar(45, 95, 45) : cv::Scalar(45, 45, 150);
      const cv::Scalar edge = normal_stop ? cv::Scalar(90, 205, 90) : cv::Scalar(80, 90, 255);
      cv::rectangle(canvas, {x0 + 12, 226}, {x0 + kPanelWidth - 12, 258}, fill, -1);
      cv::rectangle(canvas, {x0 + 12, 226}, {x0 + kPanelWidth - 12, 258}, edge, 2);
      text(compact(last_stop_reason_, 55), x0 + 20, 247, 0.42,
           cv::Scalar(245, 245, 255), 2);
    } else {
      text("stop reason  --", x0 + 18, 247, 0.39, cv::Scalar(125, 130, 140));
    }

    text("CONTROL NODES", x0 + 18, 270, 0.48,
         cv::Scalar(255, 210, 120), 1);
    const std::vector<std::pair<std::string, std::string>> nodes = {
      {"manage", "/manage_node_master"}, {"heading", "/heading_node"},
      {"drive", "/drive_node"}, {"cmd mux", "/cmd_vel_mux_node"},
      {"URG", "/urg_node_driver"}, {"scan filter", "/scan_shadow_filter_node"},
      {"pickup planner", "/pickup_path_planner"},
      {"pickup ctrl", "/pickup_approach_controller"}};
    for (size_t index = 0; index < nodes.size(); ++index) {
      const auto & item = nodes[index];
      const bool live = live_nodes_.count(item.second) != 0;
      const int column = static_cast<int>(index % 2);
      const int row = static_cast<int>(index / 2);
      const int x = x0 + 18 + column * 205;
      const int y = 294 + row * 22;
      drawStatusDot(canvas, x + 6, y, live);
      text(item.first, x + 22, y, 0.39,
           live ? cv::Scalar(225, 230, 235) : cv::Scalar(100, 100, 225));
    }

    text("TOPICS  click to inspect     pub/sub   RX", x0 + 18, 388, 0.42,
         cv::Scalar(255, 210, 120));
    const auto rows = topicRows();
    int y = 413;
    for (const auto & row : rows) {
      const auto found = topic_counts_.find(row.topic);
      const size_t pubs = found == topic_counts_.end() ? 0 : found->second.first;
      const size_t subs = found == topic_counts_.end() ? 0 : found->second.second;
      const bool ok = row.continuous ? isFresh(row.stamp, row.freshness_limit) : (pubs > 0 && subs > 0);
      const bool selected = selected_topic_ == row.topic;
      if (selected) {
        cv::rectangle(canvas, {x0 + 5, y - 14}, {x0 + kPanelWidth - 6, y + 3},
                      cv::Scalar(62, 67, 77), -1);
      }
      text(selected ? "v" : ">", x0 + 12, y, 0.38,
           selected ? cv::Scalar(80, 220, 255) : cv::Scalar(155, 165, 175));
      drawStatusDot(canvas, x0 + 34, y, ok);
      text(row.label, x0 + 49, y, 0.38);
      text(std::to_string(pubs) + "/" + std::to_string(subs), x0 + 252, y, 0.38,
           cv::Scalar(190, 200, 210));
      text(row.stamp.nanoseconds() == 0 ? "--" : ageText(row.stamp), x0 + 333, y, 0.38,
           cv::Scalar(190, 200, 210));
      y += 18;
    }

    if (selected_topic_.empty()) {
      text("MAP LAYERS", x0 + 18, 730, 0.46, cv::Scalar(255, 210, 120));
      const std::vector<std::pair<cv::Scalar, std::string>> legend = {
        {cv::Scalar(205, 205, 205), "planned path (light gray)"},
        {cv::Scalar(185, 105, 65), "estimated trajectory (blue)"},
        {cv::Scalar(225, 210, 190), "lookahead circle (pale blue)"},
        {cv::Scalar(255, 135, 35), "pickup planned path"},
        {cv::Scalar(180, 70, 220), "pickup goal"},
        {cv::Scalar(0, 190, 255), "landmark visible now"},
        {cv::Scalar(145, 145, 145), "measured URG points"},
        {cv::Scalar(190, 45, 190), "final cmd: magenta v / amber w"},
        {cv::Scalar(165, 145, 25), "pickup: teal v / green w"}};
      y = 754;
      for (const auto & item : legend) {
        cv::circle(canvas, {x0 + 24, y - 4}, 5, item.first, -1, cv::LINE_AA);
        text(item.second, x0 + 40, y, 0.40);
        y += 18;
      }
      text("green = connected/fresh   red = missing/stale", x0 + 18, 932, 0.37,
           cv::Scalar(150, 160, 170));
    } else {
      cv::rectangle(canvas, {x0 + 8, 716}, {x0 + kPanelWidth - 9, 944},
                    cv::Scalar(40, 44, 51), -1);
      cv::rectangle(canvas, {x0 + 8, 716}, {x0 + kPanelWidth - 9, 944},
                    cv::Scalar(90, 100, 115), 1);
      text("TOPIC DETAIL  (click row again to close)", x0 + 18, 736, 0.40,
           cv::Scalar(80, 220, 255), 1);
      text(compact(selected_topic_, 52), x0 + 18, 756, 0.39,
           cv::Scalar(245, 245, 245));
      text("type  " + compact(selected_topic_detail_.type, 47), x0 + 18, 774, 0.34,
           cv::Scalar(175, 190, 205));

      int detail_y = 794;
      auto draw_endpoints = [&](const std::string & heading, const std::vector<std::string> & endpoints) {
        text(heading + " (" + std::to_string(endpoints.size()) + ")", x0 + 18, detail_y, 0.36,
             cv::Scalar(255, 210, 120));
        detail_y += 16;
        const size_t visible = std::min<size_t>(endpoints.size(), 2);
        for (size_t index = 0; index < visible; ++index) {
          text("  " + compact(endpoints[index], 49), x0 + 18, detail_y, 0.34);
          detail_y += 16;
        }
        if (endpoints.size() > visible) {
          text("  ... +" + std::to_string(endpoints.size() - visible), x0 + 18, detail_y, 0.33,
               cv::Scalar(155, 165, 175));
          detail_y += 16;
        }
      };
      draw_endpoints("Publishers", selected_topic_detail_.publishers);
      draw_endpoints("Subscribers", selected_topic_detail_.subscribers);
      if (detail_y < 907) {
        text("Latest value", x0 + 18, detail_y, 0.36, cv::Scalar(255, 210, 120));
        detail_y += 16;
        for (const auto & line : topicValueLines(selected_topic_)) {
          if (detail_y > 939) break;
          text("  " + compact(line, 50), x0 + 18, detail_y, 0.34,
               cv::Scalar(205, 225, 205));
          detail_y += 16;
        }
      }
    }
  }

  void presentCanvas(cv::Mat & canvas)
  {
    drawDashboard(canvas);
    showCanvas(canvas);
  }

  void onTimer()
  {
    updateGraphSnapshot();
    // === センサ途絶時は前進だけでなく旋回中も停止する ===
    if (phase_ != Phase::IDLE && !obstacleCloudFresh()) {
      stopForObstacleCloudTimeout();
      return;
    }

    // === ゴール付近の危険チェック (DRIVE中のみ) ===
    if (phase_ == Phase::DRIVE) {
      if (isGoalBlocked()) {
        stopAtBlockedGoal();
        return;
      }
    }

    static bool window_initialized = false;
    if (show_window_ && !window_initialized) {
      cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
      cv::resizeWindow(kWindowName, kMapWidth + kPanelWidth, kCanvasHeight);
      cv::moveWindow(kWindowName, 20, 20);
      cv::setMouseCallback(kWindowName, &ManageNodeMaster::mouseCallback, this);
      window_initialized = true;
    }

    // === 可視化のスケール・オフセットを動的計算 ===
    // バウンディングボックス (ローバ + ゴール + 経路) を全体表示
    const int W = kMapWidth, H = kCanvasHeight;
    double xmin = car_pos_.x, xmax = car_pos_.x;
    double ymin = car_pos_.y, ymax = car_pos_.y;
    auto expand_box = [&](const cv::Point2d & p) {
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
    };
    if (has_goal_) expand_box(goal_);
    for (const auto & p : path_)  expand_box(p);
    if (have_pickup_goal_) expand_box(pickup_goal_);
    for (const auto & p : pickup_path_) expand_box(p);
    for (const auto & landmark : landmark_map_) expand_box(landmark.position);
    // RViz風の地図全体表示。地図が未受信のときも周囲3mは見えるようにする。
    const double margin = landmark_map_.empty() ? 3.0 : 1.5;
    xmin -= margin; xmax += margin;
    ymin -= margin; ymax += margin;
    const double world_w = xmax - xmin;
    const double world_h = ymax - ymin;
    double scale = std::min(W / world_w, H / world_h);
    if (scale < 20.0)  scale = 20.0;    // 下限 (広域表示)
    if (scale > 200.0) scale = 200.0;   // 上限 (過拡大防止)
    // バウンディング中心が画面中心 (W/2, H/2) に来るよう offset 計算
    // toPix 内で y_vis = -pt.y しているので offset.y 側で調整
    const double cx = (xmin + xmax) / 2.0;
    const double cy = (ymin + ymax) / 2.0;
    const cv::Point2d offset(W / 2.0 - cx * scale, H / 2.0 + cy * scale);

    cv::Mat canvas(H, W + kPanelWidth, CV_8UC3, cv::Scalar(248,248,248));
    drawGrid(canvas, scale, offset, xmin, xmax, ymin, ymax);
    drawMapAxes(canvas, scale, offset);

    // Pickup Plannerが生成したmap座標の経路。走行権限がpickup中なら濃くする。
    const bool pickup_active = current_cmd_source_ == "pickup";
    for (size_t i = 0; i + 1 < pickup_path_.size(); ++i) {
      cv::line(canvas, toPix(pickup_path_[i], scale, offset),
                       toPix(pickup_path_[i + 1], scale, offset),
               cv::Scalar(255, 135, 35), pickup_active ? 5 : 2, cv::LINE_AA);
    }
    if (have_pickup_goal_) {
      const cv::Point goal = toPix(pickup_goal_, scale, offset);
      cv::drawMarker(canvas, goal, cv::Scalar(180, 70, 220),
                     cv::MARKER_DIAMOND, 18, 3, cv::LINE_AA);
      cv::putText(canvas, "PICKUP GOAL", goal + cv::Point(8, -8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(150, 45, 195), 1, cv::LINE_AA);
    }

    // 現行の計画経路は始点と終点を結ぶ直線。中間21点を個別描画すると
    // 画素丸めの継ぎ目が段状に見えるため、表示だけ一本のAA線にする。
    if (path_.size() >= 2) {
      cv::line(canvas, toPix(path_.front(), scale, offset),
                       toPix(path_.back(), scale, offset),
               cv::Scalar(205, 205, 205), 2, cv::LINE_AA);
    }

    // 地図登録済みランドマークを、現在見えているものと見えていないものに分ける。
    std::set<std::string> visible_landmark_ids;
    if (isFresh(last_matched_marker_receive_time_, 1.0)) {
      for (const auto & detected : matched_markers_base_) {
        visible_landmark_ids.insert(detected.id);
      }
    }
    for (const auto & landmark : landmark_map_) {
      const cv::Point p = toPix(landmark.position, scale, offset);
      const bool visible = visible_landmark_ids.count(landmark.id) != 0;
      if (visible) {
        cv::circle(canvas, p, 9, cv::Scalar(0, 190, 255), -1, cv::LINE_AA);
        cv::circle(canvas, p, 9, cv::Scalar(0, 115, 175), 2, cv::LINE_AA);
      } else {
        cv::circle(canvas, p, 8, cv::Scalar(145, 145, 145), 2, cv::LINE_AA);
      }
      std::string label = landmark.id;
      const std::string prefix = "marker_";
      if (label.rfind(prefix, 0) == 0) label = "ID:" + label.substr(prefix.size());
      cv::putText(canvas, label, p + cv::Point(9, -8), cv::FONT_HERSHEY_SIMPLEX,
                  0.46, visible ? cv::Scalar(0, 105, 165) : cv::Scalar(115, 115, 115),
                  1, cv::LINE_AA);
    }

    // 実際に観測された位置も十字で重ね、登録座標との差を見えるようにする。
    if (received_pose_ && isFresh(last_matched_marker_receive_time_, 1.0)) {
      for (const auto & detected : matched_markers_base_) {
        const cv::Point2d world = localToMap(detected.position);
        const cv::Point p = toPix(world, scale, offset);
        cv::drawMarker(canvas, p, cv::Scalar(255, 155, 0), cv::MARKER_TILTED_CROSS,
                       12, 2, cv::LINE_AA);
      }
    }

    // ゴール
    if (has_goal_) {
      cv::circle(canvas, toPix(goal_, scale, offset),
                 6, cv::Scalar(0,0,0), 2);
    }

    // マーカー描画
    for (size_t i=0; i<marker_points_.size(); ++i) {
      cv::Point2d m = marker_points_[i];
      cv::circle(canvas, toPix(m, scale, offset),
                 5, cv::Scalar(0,0,255), -1);
      if (i < marker_ids_.size()) {
        std::string label = "ID:" + std::to_string(marker_ids_[i]);
        cv::putText(canvas, label, toPix(m, scale, offset)+cv::Point(8,-8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
      }
    }

    if (!received_pose_) {
      presentCanvas(canvas);
      return;
    }

    // 軌跡
    for (size_t i=1; i<trajectory_.size(); ++i) {
      cv::line(canvas, toPix(trajectory_[i-1], scale, offset),
                        toPix(trajectory_[i],   scale, offset),
               cv::Scalar(185,105,65), 1, cv::LINE_AA);
    }

    // LiDAR
    for (auto &p : lidar_points_) {
      cv::circle(canvas, toPix(p, scale, offset),
                 1, cv::Scalar(145,145,145), -1);
    }

    // 現在位置と向き
    drawRobotFootprint(canvas, scale, offset);
    drawGrappleBeam(canvas, scale, offset);
    drawLidarMounts(canvas, scale, offset);
    drawVelocityOverlays(canvas, scale, offset);
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               6, cv::Scalar(160,70,30), -1, cv::LINE_AA);

    // 目的地がまだ無いときは、DriveNode に目標を出さない
    if (!has_goal_) {
      presentCanvas(canvas);
      return;
    }

    // 注視円は制御情報だが主経路を隠さないよう、淡い青を半透明で重ねる。
    cv::Mat lookahead_overlay = canvas.clone();
    cv::circle(lookahead_overlay, toPix(car_pos_, scale, offset),
               static_cast<int>(lookahead_dist_*scale),
               cv::Scalar(185, 125, 55), 1, cv::LINE_AA);
    cv::addWeighted(lookahead_overlay, 0.22, canvas, 0.78, 0.0, canvas);

    // ① Pure Pursuit 注視点
    cv::Point2d lookahead = computeLookaheadPoint();
    cv::circle(canvas, toPix(lookahead, scale, offset),
               6, cv::Scalar(0,200,255), -1);

    // ② 障害物回避
    cv::Point2d selected_pt = lookahead;

    if (phase_ == Phase::DRIVE && !lidar_points_.empty()) {
      // 経路バンド: car_pos → goal 直線を中心に幅 BAND_WIDTH の領域
      const double ROBOT_RADIUS   = robot_radius_;
      const double BAND_HALF      = path_band_half_width_;   // 片側 (全幅 = 2*BAND_HALF)
      const double BAND_GOAL_MARG = 0.3;   // ゴールを越えて先まで許容

      // 経路方向の単位ベクトル u と 法線 n
      double gx = goal_.x - car_pos_.x;
      double gy = goal_.y - car_pos_.y;
      double gl = std::hypot(gx, gy);

      double min_dist = 1e9;
      cv::Point2d nearest_pt;

      if (gl > 1e-3) {
        double ux = gx / gl;
        double uy = gy / gl;
        // 法線 (-uy, ux)
        const double s_max = gl + BAND_GOAL_MARG;

        for (auto &p : lidar_points_) {
          double dx = p.x - car_pos_.x;
          double dy = p.y - car_pos_.y;
          double d = std::hypot(dx, dy);
          if (d < ROBOT_RADIUS) continue;

          // 経路方向の射影 s と 法線方向距離 t
          double s = dx * ux + dy * uy;
          double t = std::fabs(-dx * uy + dy * ux);

          // バンド外・後方・ゴール先越しは無視
          if (t > BAND_HALF) continue;
          if (s < 0.0 || s > s_max) continue;

          if (d < min_dist) {
            min_dist = d;
            nearest_pt = p;
          }
        }
      }

      // 回避基点と回避円の可視化 + バンドの可視化
      if (gl > 1e-3) {
        double ux = gx / gl, uy = gy / gl;
        cv::Point2d n1(car_pos_.x + (-uy)*BAND_HALF, car_pos_.y + ux*BAND_HALF);
        cv::Point2d n2(car_pos_.x - (-uy)*BAND_HALF, car_pos_.y - ux*BAND_HALF);
        cv::Point2d g1(goal_.x + (-uy)*BAND_HALF, goal_.y + ux*BAND_HALF);
        cv::Point2d g2(goal_.x - (-uy)*BAND_HALF, goal_.y - ux*BAND_HALF);
        cv::line(canvas, toPix(n1, scale, offset), toPix(g1, scale, offset),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);
        cv::line(canvas, toPix(n2, scale, offset), toPix(g2, scale, offset),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);
      }

      if (min_dist < 1e9) {
        cv::circle(canvas, toPix(nearest_pt, scale, offset),
                   4, cv::Scalar(0,0,255), -1);
        cv::circle(canvas, toPix(nearest_pt, scale, offset),
                   static_cast<int>(obstacle_avoid_radius_ * scale),
                   cv::Scalar(0,0,255), 1, cv::LINE_AA);
        std::string r2_label = "r2=" + std::to_string(obstacle_avoid_radius_).substr(0, 3);
        cv::putText(canvas, r2_label,
                    toPix(nearest_pt, scale, offset) + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255), 1);
      }

      if (min_dist < 1e9) {
        double r1 = lookahead_dist_;
        double r2 = obstacle_avoid_radius_;

        cv::Point2d C1 = car_pos_;
        cv::Point2d C2 = nearest_pt;
        double dx = C2.x - C1.x;
        double dy = C2.y - C1.y;
        double d = std::hypot(dx, dy);

        std::vector<cv::Point2d> intersections;

        if (d < (r1 + r2) && d > std::fabs(r1 - r2)) {
          double a = (r1*r1 - r2*r2 + d*d) / (2*d);
          double h = std::sqrt(std::max(r1*r1 - a*a, 0.0));

          cv::Point2d P0 = C1 + a * (C2 - C1) / d;
          double rx = -dy * (h / d);
          double ry =  dx * (h / d);

          intersections.emplace_back(P0.x + rx, P0.y + ry);
          intersections.emplace_back(P0.x - rx, P0.y - ry);
        }

        // 前方 ±90° の交点のみ採用
        std::vector<cv::Point2d> front_inters;
        for (auto &p : intersections) {
          double dx2 = p.x - car_pos_.x;
          double dy2 = p.y - car_pos_.y;
          double angle = wrapToPi(std::atan2(dy2, dx2) - car_yaw_);
          if (std::fabs(angle) < M_PI/2.0) {
            front_inters.push_back(p);
          }
        }

        if (!front_inters.empty()) {
          if (have_last_selected_) {
            // 連続性優先: 前回 selected_pt に最も近い交点を選ぶ (スイッチング防止)
            double best_dist = 1e9;
            for (auto &p : front_inters) {
              double dist = cv::norm(p - last_selected_pt_);
              if (dist < best_dist) {
                best_dist = dist;
                selected_pt = p;
              }
            }
          } else {
            // 初回: ゴールに最も近い交点
            double best_dist = 1e9;
            for (auto &p : front_inters) {
              double dist = std::hypot(p.x - goal_.x, p.y - goal_.y);
              if (dist < best_dist) {
                best_dist = dist;
                selected_pt = p;
              }
            }
          }
          last_selected_pt_ = selected_pt;
          have_last_selected_ = true;

          // 描画
          for (auto &p : intersections) {
            cv::circle(canvas, toPix(p, scale, offset),
                       5, cv::Scalar(0,0,255), -1);
          }
          cv::circle(canvas, toPix(selected_pt, scale, offset),
                     7, cv::Scalar(0,0,0), 2);
        }
      } else {
        // 回避発動なし → 次回の回避は初回扱い (前回記憶をリセット)
        have_last_selected_ = false;
      }
    } else {
      // DRIVE 以外、または LiDAR 無し → リセット
      have_last_selected_ = false;
    }

    // ④ DriveNode 用 target_point を publish
    geometry_msgs::msg::PointStamped tgt_msg;
    tgt_msg.header.stamp = this->now();
    tgt_msg.header.frame_id = planning_frame_;
    tgt_msg.point.x = selected_pt.x;
    tgt_msg.point.y = selected_pt.y;
    tgt_msg.point.z = 0.0;
    target_pub_->publish(tgt_msg);
    last_target_point_ = selected_pt;
    have_target_point_ = true;

    // ゴールを publish
    geometry_msgs::msg::Pose2D goal_msg;
    goal_msg.x = goal_.x;
    goal_msg.y = goal_.y;
    goal_msg.theta = goal_yaw_;
    goal_pub_->publish(goal_msg);

    presentCanvas(canvas);
  }

  // ============================================================
  //                         メンバ変数
  // ============================================================
  // Sub / Pub / Timer
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_sub_;
  rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr landmark_map_sub_;
  rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr matched_marker_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr body_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_source_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pickup_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pickup_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pickup_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pickup_planner_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pickup_too_close_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pickup_nearest_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr marker_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_done_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            heading_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr        goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             cmd_source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             result_pub_;

  // TF (センサ座標 → planning_frame 変換用)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 状態管理
  Phase phase_;
  bool received_pose_ = false;
  bool heading_sent_  = false;

  cv::Point2d car_pos_;
  double car_yaw_;
  std::vector<cv::Point2d> trajectory_;

  bool        has_goal_ = false;
  cv::Point2d goal_;
  double      goal_yaw_ = 0.0;

  double lookahead_dist_;
  double lidar_history_sec_;    // 過去点群の保持期間 [s]
  double robot_radius_;         // ローバ半径 (最近点除外閾値)
  double obstacle_avoid_radius_;// 障害物回避円 r2
  double path_band_half_width_; // 経路バンド片側幅
  double obstacle_z_min_;       // 障害物として使用する高さ下限 [m]
  double obstacle_z_max_;       // 障害物として使用する高さ上限 [m]
  double goal_block_radius_;    // ゴール閉塞判定半径 [m]
  int goal_block_min_points_;   // ゴール閉塞とする最低点数
  double maximum_cloud_age_sec_;// 障害物点群の最大許容受信間隔 [s]
  int obstacle_tf_timeout_ms_;  // 点群計測時刻のTFを待つ最大時間 [ms]
  double body_length_;          // UI車体長手寸法 (ローカルY) [m]
  double body_width_;           // UI車体短手寸法 (ローカルX) [m]
  double urg_mount_x_;          // URG取付X (scan_shadow_filter設定と同じ) [m]
  double urg_mount_y_;          // URG取付Y (scan_shadow_filter設定と同じ) [m]
  double arm_base_x_;           // ピックアップ梁基部 +X前 [m]
  double arm_base_y_;           // ピックアップ梁基部 +Y左 [m]
  double arm_yaw_;              // 梁方位 (REP-103機体座標) [rad]
  double arm_length_;           // 梁長 [m]
  double pickup_approach_limit_;// 山接近の実行時停止閾値 [m]
  double velocity_vector_horizon_; // 速度矢印が示す将来時間 [s]
  double angular_arc_radius_;       // 角速度円弧の固定半径 [m]

  std::string voicevox_url_;    // Voicevox サーバ URL (空で無効)
  bool show_window_ = true;    // 確認用の窓を開くか (画面があるときだけ)
  std::string obstacle_cloud_topic_;
  std::string planning_frame_;
  std::string velocity_topic_;

  // 回避交点のスイッチング防止 (連続性維持)
  cv::Point2d last_selected_pt_{0.0, 0.0};
  bool have_last_selected_{false};
  std::vector<cv::Point2d> path_;

  std::vector<cv::Point2d> marker_points_;
  std::vector<int>         marker_ids_;
  std::vector<LabeledPoint> landmark_map_;
  std::vector<LabeledPoint> matched_markers_base_;
  geometry_msgs::msg::Twist measured_twist_;
  geometry_msgs::msg::Twist commanded_twist_;
  geometry_msgs::msg::Twist pickup_commanded_twist_;
  std::string current_cmd_source_{"--"};
  std::vector<cv::Point2d> pickup_path_;
  cv::Point2d pickup_goal_;
  bool have_pickup_goal_{false};
  std::string pickup_planner_status_{"--"};
  bool pickup_too_close_{false};
  double pickup_nearest_distance_{0.0};
  bool have_pickup_nearest_distance_{false};
  int last_goal_block_count_{0};
  std::string last_stop_reason_;
  double last_heading_target_{0.0};
  bool have_heading_target_{false};
  cv::Point2d last_target_point_{0.0, 0.0};
  bool have_target_point_{false};
  bool last_heading_done_value_{false};
  bool last_drive_done_value_{false};
  std::string last_obstacle_frame_{"--"};
  size_t last_obstacle_raw_points_{0};
  std::vector<cv::Point2d> last_obstacle_samples_;

  // 過去 LiDAR フレーム (planning_frame, z フィルタ済み) の時系列蓄積
  struct LidarFrame {
    rclcpp::Time stamp;
    std::vector<cv::Point2d> points;
  };
  std::deque<LidarFrame> lidar_history_;
  std::vector<cv::Point2d> lidar_points_;
  bool received_cloud_{false};
  rclcpp::Time latest_cloud_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heading_done_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_drive_done_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_landmark_map_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_matched_marker_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_body_twist_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_vel_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_source_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_path_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_goal_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_cmd_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_planner_status_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_too_close_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pickup_nearest_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigation_result_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stop_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_graph_update_{0, 0, RCL_ROS_TIME};
  std::set<std::string> live_nodes_;
  std::map<std::string, std::pair<size_t, size_t>> topic_counts_;
  std::string selected_topic_;
  TopicEndpointSnapshot selected_topic_detail_;
  std::string last_result_{"--"};

  std::ofstream log_pose_file_;
  std::string   log_dir_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManageNodeMaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
