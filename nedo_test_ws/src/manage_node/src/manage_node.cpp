// manage_node/src/manage_node.cpp
// 自律移動の全体管理ノード

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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

static inline double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

enum class Phase { 
    HEADING,          // ゴール方向へ向く
    DRIVE,            // 前進
    STOP,             // ゴール到着後 N 秒停止
    TASK,             // タスク実行 (シャトル運転では未使用)
    HEADING_TO_TASK,  // タスク用の姿勢に向く (シャトル運転では未使用)
    OBSTACLE_CHECK,   // フルスキャン
    DONE              // 終了
};

class ManageNode : public rclcpp::Node {
public:
  ManageNode() : Node("manage_node")
  {
    // ログディレクトリ作成
    log_dir_ = makeLogDirectory();
    std::string pose_path = log_dir_ + "/pose_log.csv";
    log_pose_file_.open(pose_path, std::ios::out);
    log_pose_file_ << "time,x,y,yaw\n";
    RCLCPP_INFO(this->get_logger(), "Pose log saved to: %s", pose_path.c_str());

    // パラメータ
    lookahead_dist_ = this->declare_parameter("lookahead_distance", 0.5);  // LookAhead距離
    stop_duration_sec_ = this->declare_parameter<double>("stop_duration", 2.0);  // ゴール到着後の停止時間
    lidar_history_sec_ = this->declare_parameter<double>("lidar_history_sec", 3.0);  // 過去点群の保持期間 [s]
    robot_radius_           = this->declare_parameter<double>("robot_radius", 0.2);            // ローバ半径 (最近点除外閾値)
    obstacle_avoid_radius_  = this->declare_parameter<double>("obstacle_avoid_radius", 0.5);   // 障害物回避円 r2
    path_band_half_width_   = this->declare_parameter<double>("path_band_half_width", 0.2);    // 経路バンド片側幅
    voicevox_url_           = this->declare_parameter<std::string>("voicevox_url", "http://localhost:50021");  // 空文字で無効

    // ゴール定義 (シャトル運転): 平坦配列 [x0,y0,x1,y1,...] を cv::Point2d 配列へ
    const auto goals_flat =
      this->declare_parameter<std::vector<double>>("goals", {0.0, 0.0, 2.0, 0.0});
    if (goals_flat.size() % 2 != 0) {
      RCLCPP_FATAL(this->get_logger(), "goals must have even length (x0,y0,x1,y1,...)");
      throw std::runtime_error("invalid goals parameter");
    }
    goals_.clear();
    for (size_t i = 0; i + 1 < goals_flat.size(); i += 2) {
      goals_.emplace_back(goals_flat[i], goals_flat[i + 1]);
    }
    if (goals_.size() < 2) {
      RCLCPP_FATAL(this->get_logger(), "goals must have at least 2 points");
      throw std::runtime_error("insufficient goals");
    }
    // 起動時は Goal 1 に向かって走り出す
    goal_index_ = 1;
    goal_ = goals_[goal_index_];

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
    task_start_pub_     = this->create_publisher<std_msgs::msg::Bool>("/task_start", 10);
    fullscan_pub_       = this->create_publisher<std_msgs::msg::Bool>("/request_full_scan", 10);
    cmd_source_pub_     = this->create_publisher<std_msgs::msg::String>("/cmd_source", 10);

    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/current_goal", latched);
    }

    // Subscriber
    pose2d_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      std::bind(&ManageNode::onPose2D, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points_shuttle", 10,
      std::bind(&ManageNode::onLidar, this, std::placeholders::_1));

    fullscan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/full_scan_points", 10,
      std::bind(&ManageNode::onFullScanReceived, this, std::placeholders::_1));

    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_markers", 10,
      std::bind(&ManageNode::onMarkerArray, this, std::placeholders::_1));

    marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/aruco_marker_ids", 10,
      std::bind(&ManageNode::onMarkerIDs, this, std::placeholders::_1));

    heading_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/heading_done", 10,
      std::bind(&ManageNode::onHeadingDone, this, std::placeholders::_1));

    drive_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/drive_done", 10,
      std::bind(&ManageNode::onDriveDone, this, std::placeholders::_1));

    task_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/task_done", 10,
      std::bind(&ManageNode::onTaskDone, this, std::placeholders::_1));

    // TF buffer (os_gravity -> odom 変換用)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //  Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ManageNode::onTimer, this));

    second_scan_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ManageNode::onSecondScanTimer, this));
    second_scan_timer_->cancel();

    fullscan_timeout_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ManageNode::onFullScanTimeout, this));
    fullscan_timeout_timer_->cancel();

    // シャトル運転: ゴール到着後 N 秒停止するためのタイマ
    const auto stop_ms = static_cast<int>(stop_duration_sec_ * 1000.0);
    stop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(stop_ms),
      std::bind(&ManageNode::onStopTimer, this));
    stop_timer_->cancel();

    wait_counter_sec_    = 0;
    waiting_second_scan_ = false;
    waiting_fullscan_    = false;

    // 初期フェーズ
    phase_ = Phase::HEADING;

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    RCLCPP_INFO(this->get_logger(), "ManageNode 起動");
  }

  ~ManageNode(){
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
  //                  ゴール近傍に障害物があるか
  // ============================================================
  bool isGoalBlocked()
  {
    if (!received_pose_) return false;
    if (lidar_points_.empty()) return false;

    const double R_GOAL   = 0.15; // ゴール周囲半径
    const int    MIN_PTS  = 120;   // 最低点数

    cv::Point2d g = goal_;
    int cnt = 0;

    for (auto &p : lidar_points_) {
      double d_goal = cv::norm(p - g);
      if (d_goal < R_GOAL) cnt++;
      RCLCPP_INFO(this->get_logger(), "Goal blocked check: %d pts within %.2f m", cnt, R_GOAL);
    }

    if (cnt >= MIN_PTS) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal #%d blocked: %d points within %.2f m",
                  goal_index_, cnt, R_GOAL);
      return true;
    }
    return false;
  }

  // ============================================================
  //                         Goal スキップ
  // ============================================================
  void skipCurrentGoal()
  {
    // 一旦停止
    std_msgs::msg::String stop_src;
    stop_src.data = "none";
    cmd_source_pub_->publish(stop_src);

    if (std::find(skipped_goals_.begin(), skipped_goals_.end(), goal_index_) 
        == skipped_goals_.end()) {
      skipped_goals_.push_back(goal_index_);
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "ゴール%d番の付近に障害物があります。スキップします。",
                    goal_index_);
      playVoice(buf);
    }

    // 次のゴールへ
    goal_index_ = (goal_index_ + 1) % goals_.size();
    goal_ = goals_[goal_index_];

    double dx = goal_.x - car_pos_.x;
    double dy = goal_.y - car_pos_.y;
    double yaw = std::atan2(dy, dx);

    std_msgs::msg::Float32 msg;
    msg.data = yaw;
    heading_target_pub_->publish(msg);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;

    RCLCPP_WARN(this->get_logger(),
                "Skip → Next goal %d に向けて HEADING (yaw=%.3f)",
                goal_index_, yaw);
  }

  // ============================================================
  //                        Heading Done
  // ============================================================
  void onHeadingDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    if (phase_ == Phase::OBSTACLE_CHECK) {
      RCLCPP_INFO(this->get_logger(),
                  "OBSTACLE_CHECK: heading done → full scan start");
      startObstacleScan();
      return;
    }

    if (phase_ == Phase::HEADING_TO_TASK) {
      RCLCPP_INFO(this->get_logger(),
                  "HEADING_TO_TASK done → TASK 開始");

      phase_ = Phase::TASK;

      std_msgs::msg::Bool start_msg;
      start_msg.data = true;
      task_start_pub_->publish(start_msg);

      std_msgs::msg::String src;
      src.data = "task";
      cmd_source_pub_->publish(src);
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
    if (!msg->data) return;
    if (phase_ != Phase::DRIVE) return;

    // Goal0に戻ってきた かつ スキップゴールあり → 障害物チェック
    if (goal_index_ == 0 && !skipped_goals_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Returned to Goal0 with skipped goals → OBSTACLE_CHECK");

      double yaw_origin = std::atan2(0.0 - car_pos_.y, 0.0 - car_pos_.x);

      std_msgs::msg::Float32 msg_yaw;
      msg_yaw.data = yaw_origin;
      heading_target_pub_->publish(msg_yaw);

      std_msgs::msg::String src;
      src.data = "heading";
      cmd_source_pub_->publish(src);

      phase_ = Phase::OBSTACLE_CHECK;
      return;
    }

    // シャトル運転: ゴール到着 → 停止 (cmd_source=none) → N 秒後に反対ゴールへ旋回
    std_msgs::msg::String src;
    src.data = "none";
    cmd_source_pub_->publish(src);

    phase_ = Phase::STOP;
    stop_timer_->reset();  // stop_duration_sec_ 経過後に onStopTimer 発火

    RCLCPP_INFO(this->get_logger(),
                "Goal #%d 到着 → %.1f 秒停止後に反対ゴールへ旋回",
                goal_index_, stop_duration_sec_);
  }

  // ============================================================
  //             STOP タイマ (ゴール到着後の停止時間経過)
  // ============================================================
  void onStopTimer()
  {
    stop_timer_->cancel();  // ワンショット化

    // 反対ゴールへ切り替え (0 ↔ 1)
    goal_index_ = (goal_index_ == 0) ? 1 : 0;
    goal_ = goals_[goal_index_];

    double dx = goal_.x - car_pos_.x;
    double dy = goal_.y - car_pos_.y;
    double yaw = std::atan2(dy, dx);

    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = yaw;
    heading_target_pub_->publish(msg_yaw);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;

    RCLCPP_INFO(this->get_logger(),
                "Stop done → Goal #%d に向けて HEADING (yaw=%.3f)",
                goal_index_, yaw);
  }

  // ============================================================
  //                        Task Done
  // ============================================================
  void onTaskDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;
    if (phase_ != Phase::TASK) return;

    RCLCPP_INFO(this->get_logger(),
                "Task done → heading toward NEXT goal");

    // 次のゴールに更新
    goal_index_ = (goal_index_ + 1) % goals_.size();
    goal_ = goals_[goal_index_];

    double dx = goal_.x - car_pos_.x;
    double dy = goal_.y - car_pos_.y;
    double next_dir = std::atan2(dy, dx);

    std_msgs::msg::Float32 msg_target;
    msg_target.data = next_dir;
    heading_target_pub_->publish(msg_target);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;
  }

  // ============================================================
  //                         Pose 受信
  // ============================================================
  void onPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    //const double dx_cam = -0.135;オフセットはaruco_pnp_node_wall.cppの方に移動しました。
    const double dx_cam = 0.0;
    const double dy_cam = 0.0;

    double yaw_cam = msg->theta;
    double x_robot = msg->x + dx_cam * std::cos(yaw_cam) - dy_cam * std::sin(yaw_cam);
    double y_robot = msg->y + dx_cam * std::sin(yaw_cam) + dy_cam * std::cos(yaw_cam);

    // 描画・運動モデルのための yaw 補正
    double yaw_robot = yaw_cam;

    car_pos_ = cv::Point2d(x_robot, y_robot);
    car_yaw_ = wrapToPi(yaw_robot);

    // 初回受信処理
    if (!received_pose_) {
        received_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "初回Pose受信: 経路を初期化します");
        updatePathToCurrentGoal();
    }

    trajectory_.push_back(car_pos_);
    if (trajectory_.size() > 200)
      trajectory_.erase(trajectory_.begin());

    // 起動直後の初回 HEADING のための heading_target
    if (phase_ == Phase::HEADING && !heading_sent_) {
      double dx = goal_.x - car_pos_.x;
      double dy = goal_.y - car_pos_.y;
      double theta_target = std::atan2(dy, dx);

      std_msgs::msg::Float32 msg_target;
      msg_target.data = theta_target;
      heading_target_pub_->publish(msg_target);

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

  // ============================================================
  //                         LiDAR（通常）
  // ============================================================
  void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!received_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "自己位置未受信のため LiDAR 点群を描画できません");
      return;
    }

    // /filtered_points (frame=os_gravity) を odom frame に TF 変換
    sensor_msgs::msg::PointCloud2 cloud_odom;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "odom", msg->header.frame_id,
        tf2::TimePointZero, std::chrono::milliseconds(50));
      tf2::doTransform(*msg, cloud_odom, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF 変換失敗 (%s → odom): %s",
                           msg->header.frame_id.c_str(), ex.what());
      return;
    }

    // 現在フレームの点群を odom frame で抽出
    const auto now_stamp = this->now();
    LidarFrame frame;
    frame.stamp = now_stamp;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_odom, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_odom, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_odom, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) continue;
      if (*iter_z < -0.2 || *iter_z > 0.5) continue;
      frame.points.emplace_back(*iter_x, *iter_y);
    }

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
  //                       フルスキャン要求
  // ============================================================
  void requestFullScan()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    fullscan_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Sent full scan request to LiDAR node");

    waiting_fullscan_ = true;
    fullscan_timeout_timer_->reset();
  }

  void startObstacleScan()
  {
    playVoice("ゴール付近に障害物を確認しました。スキャンします。");
    requestFullScan();

    std_msgs::msg::String src;
    src.data = "none";
    cmd_source_pub_->publish(src);
  }

  // skipped_goals_ 付近の障害物だけを抽出
  std::vector<cv::Point2d> extractSkippedGoalObstacles(
      const std::vector<cv::Point2d>& pts)
  {
    std::vector<cv::Point2d> result;
    const double R = 0.3;

    for (int gid : skipped_goals_) {
      cv::Point2d g = goals_[gid];
      for (auto &p : pts) {
        if (cv::norm(p - g) < R) {
          result.push_back(p);
        }
      }
    }
    return result;
  }

  void onFullScanTimeout()
  {
    if (!waiting_fullscan_)
      return;

    RCLCPP_ERROR(this->get_logger(),
        "[ERROR] full_scan timeout: LiDAR node not responding!");

    waiting_fullscan_    = false;
    waiting_second_scan_ = false;
    skipped_goals_.clear();

    playVoice("LiDARが応答しません。通常巡回に戻ります。");

    double back_yaw = M_PI / 2.0;

    std_msgs::msg::Float32 msg;
    msg.data = back_yaw;
    heading_target_pub_->publish(msg);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_TASK;
  }

  // フルスキャン受信
  void onFullScanReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    waiting_fullscan_ = false;
    fullscan_timeout_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Received full scan");

    std::vector<cv::Point2d> full_pts;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    double cx = car_pos_.x;
    double cy = car_pos_.y;
    double cth = car_yaw_;
    double c = std::cos(cth);
    double s = std::sin(cth);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) continue;
      if (*iter_z < -0.3 || *iter_z > 1.2) continue;

      double x_map = c * (*iter_x) - s * (*iter_y) + cx;
      double y_map = s * (*iter_x) + c * (*iter_y) + cy;
      full_pts.emplace_back(x_map, y_map);
    }

    if (!waiting_second_scan_) {
      // 1回目
      auto obs = extractSkippedGoalObstacles(full_pts);

      if (!obs.empty()) {
        const auto &p = obs.front();
        char buf[200];
        std::snprintf(buf, sizeof(buf),
             "ゴール付近の障害物を取り除いてください。座標は、%.2fメートル、%.2fメートル付近です。",
             p.x, p.y);
        playVoice(buf);
      } else {
        playVoice("スキップしたゴール付近に明確な障害物は検出されませんでした。");
      }

      waiting_second_scan_ = true;
      wait_counter_sec_ = 0;
      second_scan_timer_->reset();
      RCLCPP_INFO(this->get_logger(),
                  "[OBSTACLE_CHECK] First full scan done. Waiting 30 sec for second scan.");
      return;
    }

    // 2回目
    waiting_second_scan_ = false;
    second_scan_timer_->cancel();

    auto remaining = extractSkippedGoalObstacles(full_pts);

    if (remaining.empty()) {
      playVoice("障害物が除去されました。タスクを再開します。");
    } else {
      playVoice("まだ障害物がありますが、巡回を再開します。");
    }

    skipped_goals_.clear();

    double back_yaw = M_PI/2.0;
    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = back_yaw;
    heading_target_pub_->publish(msg_yaw);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_TASK;
  }

  // カウントダウン
  void onSecondScanTimer()
  {
    if (!waiting_second_scan_) {
      second_scan_timer_->cancel();
      return;
    }

    wait_counter_sec_++;
    RCLCPP_INFO(this->get_logger(),
        "[OBSTACLE_CHECK] wait %d / 30 sec", wait_counter_sec_);

    if (wait_counter_sec_ >= 30) {
      second_scan_timer_->cancel();
      playVoice("再スキャンします。");
      requestFullScan();
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

  double computePathLengthAtPoint(const cv::Point2d& pt)
  {
    double total = 0.0;
    for (size_t i=1; i<path_.size(); ++i) {
      double seg_len = cv::norm(path_[i] - path_[i-1]);
      if (cv::norm(pt - path_[i]) < seg_len) {
        total += cv::norm(pt - path_[i-1]);
        break;
      }
      total += seg_len;
    }
    return total;
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

  void onTimer()
  {
    // === ゴール付近の危険チェック (DRIVE中のみ) ===
    if (phase_ == Phase::DRIVE) {
      if (isGoalBlocked()) {
        skipCurrentGoal();
        return;
      }
    }

    static bool window_initialized = false;
    if (!window_initialized) {
      cv::namedWindow("Manage Visualization", cv::WINDOW_NORMAL);
      cv::moveWindow("Manage Visualization", 50, 50);
      window_initialized = true;
    }

    // === 可視化のスケール・オフセットを動的計算 ===
    // バウンディングボックス (ローバ + ゴール + 経路) を全体表示
    const int W = 800, H = 800;
    double xmin = car_pos_.x, xmax = car_pos_.x;
    double ymin = car_pos_.y, ymax = car_pos_.y;
    auto expand_box = [&](const cv::Point2d & p) {
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
    };
    for (const auto & g : goals_) expand_box(g);
    for (const auto & p : path_)  expand_box(p);
    // マージン 1.0m
    xmin -= 10.0; xmax += 10.0;
    ymin -= 10.0; ymax += 10.0;
    const double world_w = xmax - xmin;
    const double world_h = ymax - ymin;
    double scale = std::min(W / world_w, H / world_h);
    if (scale < 50.0)  scale = 50.0;    // 下限 (広域表示)
    if (scale > 200.0) scale = 200.0;   // 上限 (過拡大防止)
    // バウンディング中心が画面中心 (W/2, H/2) に来るよう offset 計算
    // toPix 内で y_vis = -pt.y しているので offset.y 側で調整
    const double cx = (xmin + xmax) / 2.0;
    const double cy = (ymin + ymax) / 2.0;
    const cv::Point2d offset(W / 2.0 - cx * scale, H / 2.0 + cy * scale);

    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(255,255,255));

    // Goal0 ↔ Goal1 の固定往復線 (薄いグレー)
    if (goals_.size() >= 2) {
      cv::line(canvas, toPix(goals_[0], scale, offset),
                        toPix(goals_[1], scale, offset),
               cv::Scalar(200,200,200), 1, cv::LINE_AA);
    }

    // 経路線 (car_pos_ → 現在の goal_)
    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::line(canvas, toPix(path_[i],   scale, offset),
                        toPix(path_[i+1], scale, offset),
               cv::Scalar(0,0,0), 2);
    }

    // ゴール群
    for (size_t i=0; i<goals_.size(); ++i) {
      cv::Scalar color = (i == static_cast<size_t>(goal_index_))
                        ? cv::Scalar(0,0,0)
                        : cv::Scalar(180,180,180);
      cv::circle(canvas, toPix(goals_[i], scale, offset),
                 6, color, 2);
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
      cv::imshow("Manage Visualization", canvas);
      cv::waitKey(1);
      return;
    }

    // 軌跡
    for (size_t i=1; i<trajectory_.size(); ++i) {
      cv::line(canvas, toPix(trajectory_[i-1], scale, offset),
                        toPix(trajectory_[i],   scale, offset),
               cv::Scalar(255,0,0), 1);
    }

    // LiDAR
    for (auto &p : lidar_points_) {
      cv::circle(canvas, toPix(p, scale, offset),
                 1, cv::Scalar(120,120,120), -1);
    }

    // 現在位置と向き
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               6, cv::Scalar(255,0,0), -1);

    cv::Point2d head = car_pos_ + 0.6*cv::Point2d(std::cos(car_yaw_), std::sin(car_yaw_));
    cv::arrowedLine(canvas, toPix(car_pos_, scale, offset),
                            toPix(head,    scale, offset),
                    cv::Scalar(255,0,0), 2);

    // 注視円
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               static_cast<int>(lookahead_dist_*scale),
               cv::Scalar(255,0,0), 1, cv::LINE_AA);

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
    tgt_msg.header.frame_id = "map";
    tgt_msg.point.x = selected_pt.x;
    tgt_msg.point.y = selected_pt.y;
    tgt_msg.point.z = 0.0;
    target_pub_->publish(tgt_msg);

    // ゴールを publish
    geometry_msgs::msg::Pose2D goal_msg;
    goal_msg.x = goal_.x;
    goal_msg.y = goal_.y;
    goal_msg.theta = 0.0;
    goal_pub_->publish(goal_msg);

    cv::imshow("Manage Visualization", canvas);
    cv::waitKey(1);
  }

  // ============================================================
  //                         メンバ変数
  // ============================================================
  // Sub / Pub / Timer
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr marker_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fullscan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_done_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            heading_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr        goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               task_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               fullscan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             cmd_source_pub_;

  // TF (os_gravity → odom 変換用)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr second_scan_timer_;
  rclcpp::TimerBase::SharedPtr fullscan_timeout_timer_;
  rclcpp::TimerBase::SharedPtr stop_timer_;       // シャトル運転: ゴール到着後の停止タイマ

  // full_scan 関連
  bool waiting_fullscan_    = false;
  bool waiting_second_scan_ = false;
  int  wait_counter_sec_    = 0;

  // 状態管理
  Phase phase_;
  bool received_pose_ = false;
  bool heading_sent_  = false;

  cv::Point2d car_pos_;
  double car_yaw_;
  std::vector<cv::Point2d> trajectory_;

  std::vector<cv::Point2d> pos_buffer_;
  std::vector<double>       yaw_buffer_;

  std::vector<cv::Point2d> goals_;
  int        goal_index_;
  cv::Point2d goal_;

  double lookahead_dist_;
  double stop_duration_sec_;    // シャトル運転: ゴール到着後の停止時間
  double lidar_history_sec_;    // 過去点群の保持期間 [s]
  double robot_radius_;         // ローバ半径 (最近点除外閾値)
  double obstacle_avoid_radius_;// 障害物回避円 r2
  double path_band_half_width_; // 経路バンド片側幅

  std::string voicevox_url_;    // Voicevox サーバ URL (空で無効)

  // 回避交点のスイッチング防止 (連続性維持)
  cv::Point2d last_selected_pt_{0.0, 0.0};
  bool have_last_selected_{false};
  std::vector<cv::Point2d> path_;

  std::vector<cv::Point2d> marker_points_;
  std::vector<int>         marker_ids_;

  // 過去 LiDAR フレーム (odom frame, z フィルタ済み) の時系列蓄積
  struct LidarFrame {
    rclcpp::Time stamp;
    std::vector<cv::Point2d> points;
  };
  std::deque<LidarFrame> lidar_history_;
  std::vector<cv::Point2d> lidar_points_;

  // ゴールスキップ関連
  std::vector<int> skipped_goals_;

  std::ofstream log_pose_file_;
  std::string   log_dir_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}