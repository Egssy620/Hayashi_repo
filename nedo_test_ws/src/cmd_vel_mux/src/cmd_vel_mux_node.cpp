// cmd_vel_mux_node/src/cmd_vel_mux_node.cpp
// cmd_velの統合ノード

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <string>
#include <optional>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>

class CmdVelMuxNode : public rclcpp::Node {
public:
  CmdVelMuxNode() : Node("cmd_vel_mux_node") {
    // ---- ログディレクトリ作成 ----
    log_dir_ = makeLogDirectory();
    std::string cmd_path = log_dir_ + "/cmd_log.csv";

    log_cmd_file_.open(cmd_path, std::ios::out);
    log_cmd_file_ << "time,source,v,omega\n";

    RCLCPP_INFO(this->get_logger(), "Cmd log saved to: %s", cmd_path.c_str());

    // --- 入力購読 ---
    sub_heading_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/heading/rover_twist", 10,
      std::bind(&CmdVelMuxNode::onHeading, this, std::placeholders::_1));

    sub_drive_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/drive/rover_twist", 10,
      std::bind(&CmdVelMuxNode::onDrive, this, std::placeholders::_1));

    sub_task_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/task/rover_twist", 10,
      std::bind(&CmdVelMuxNode::onTask, this, std::placeholders::_1));

    // ピックアップ位置への接近 (pickup_approach_controller)。全方向移動なので linear.y も使う
    sub_pickup_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/pickup_approach/cmd_vel", 10,
      std::bind(&CmdVelMuxNode::onPickup, this, std::placeholders::_1));

    // --- 出力 ---
    pub_out_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // --- source選択購読 ---
    sub_select_ = this->create_subscription<std_msgs::msg::String>(
      "/cmd_source", 10,
      [this](const std_msgs::msg::String &msg){
        std::lock_guard<std::mutex> lk(mtx_);
        source_ = msg.data;
        // 切り替えたら一度ゼロ速度を出す。"none" (停止) は以後何も流れないので、
        // これが無いと切替前の最後の指令 (走行中なら前進) が台車に残る
        pub_out_->publish(geometry_msgs::msg::Twist());
        RCLCPP_INFO(this->get_logger(), "mux: source -> %s", source_.c_str());
      });

    // --- デフォルトは heading ---
    {
      std::lock_guard<std::mutex> lk(mtx_);
      source_ = "heading";
    }
  }

  ~CmdVelMuxNode(){
    if (log_cmd_file_.is_open())
      log_cmd_file_.close();
  }

private:
  // ------------------------------
  // ログディレクトリ作成
  // ------------------------------
  std::string makeLogDirectory()
  {
      const char* home = std::getenv("HOME");
      if (!home) home = "/tmp";

      std::time_t t = std::time(nullptr);
      std::tm tm{};
      localtime_r(&t, &tm);

      std::ostringstream oss;
      oss << home << "/ros2_logs/"
          << std::put_time(&tm, "%Y-%m-%d");

      std::string dir = oss.str();
      std::filesystem::create_directories(dir);

      return dir;
  }

  // ------------------------------
  // ログ書き込み (publish 前)
  // ------------------------------
  void logCmd(const geometry_msgs::msg::Twist& msg, const std::string& src)
  {
      if(!log_cmd_file_.is_open()){
          RCLCPP_ERROR(this->get_logger(),
                      "logCmd called but file is not open!");
          return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "[LOG TEST] logCmd called: src=%s v=%.3f w=%.3f",
                  src.c_str(), msg.linear.x, msg.angular.z);

      double t = this->now().seconds();
      log_cmd_file_ << t << ","
                    << src << ","
                    << msg.linear.x  << ","
                    << msg.angular.z << "\n";

      log_cmd_file_.flush();
  }

  // ---- 各cmd_vel購読 ----
  void onHeading(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lk(mtx_);
    last_heading_ = *msg;
    if(source_=="heading"){
      pub_out_->publish(*msg);
    }
  }

  void onDrive(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lk(mtx_);
    last_drive_ = *msg;
    if(source_=="drive"){
      pub_out_->publish(*msg);
    }
  }

  void onTask(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lk(mtx_);
    last_task_ = *msg;
    if(source_=="task"){
      pub_out_->publish(*msg);
    }
  }

  void onPickup(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lk(mtx_);
    if(source_=="pickup"){
      // pickup_approach_controller は REP-103 body twist
      //   +X=前進, +Y=左
      // を出す。一方、実機下位制御の /cmd_vel は
      //   +X=左, +Y=後退 (-Y=前進)
      // なので、ハードウェア境界で並進軸だけを変換する。角速度符号は共通。
      geometry_msgs::msg::Twist lower = *msg;
      lower.linear.x = msg->linear.y;
      lower.linear.y = -msg->linear.x;
      pub_out_->publish(lower);
    }
  }

  // ---- メンバ変数 ----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_heading_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_drive_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_task_;  // ★追加★
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_pickup_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_out_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_select_;

  std::mutex mtx_;
  std::string source_; // "heading" / "drive" / "task" / "pickup" / "none"
  // ログ
  std::ofstream log_cmd_file_;
  std::string log_dir_;
  std::optional<geometry_msgs::msg::Twist> last_heading_, last_drive_, last_task_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMuxNode>());
  rclcpp::shutdown();
  return 0;
}
