  // /depth_clustering/segments (PointCloud2, os_lidar 傾斜座標系) と
  // /depth_clustering/gravity_quat (QuaternionStamped, センサ→重力回転) を入力し、
  // 重力水平座標系の 3D 点群 /filtered_points_shuttle (frame=os_gravity) を出版する。
  //
  // 傾き対応の手順:
  //   1. QuaternionStamped を四元数のまま受け取る
  //   2. 各点 p_lidar を四元数でベクトル回転 → p_gravity = R_sg * p_lidar
  //   3. 重力系 z で高さバンド足切り (z_min < z < z_max)
  //   4. 通過点は (x, y, z) のまま保持 (高さ情報を維持して publish)
  //      - 下流 (manage_node) が TF で odom 系に変換し、地面は z≈0、障害物は高さを持つ
  //   5. PointCloud2 を出版、frame_id = os_gravity

  #include <chrono>
  #include <memory>
  #include <vector>

  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/point_cloud2.hpp"
  #include "sensor_msgs/point_cloud2_iterator.hpp"
  #include "geometry_msgs/msg/quaternion_stamped.hpp"
  #include "tf2/LinearMath/Quaternion.h"
  #include "tf2/LinearMath/Matrix3x3.h"
  #include "tf2/convert.h"

  class PointsProjection : public rclcpp::Node
  {
  public:
    PointsProjection() : Node("points_projection")
    {
      input_topic_ = this->declare_parameter<std::string>("input_topic", "/depth_clustering/segments");
      gravity_quat_topic_ = this->declare_parameter<std::string>("gravity_quat_topic", "/depth_clustering/gravity_quat");
      output_topic_ = this->declare_parameter<std::string>("output_topic", "/filtered_points_shuttle");
      output_frame_ = this->declare_parameter<std::string>("output_frame", "os_gravity");
      sensor_height_ = this->declare_parameter<double>("sensor_height", 0);
      z_min_ = this->declare_parameter<double>("z_min", -3.0);
      z_max_ = this->declare_parameter<double>("z_max", 0.0);

      rclcpp::QoS best_effort = rclcpp::SensorDataQoS();
      points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, best_effort,
        std::bind(&PointsProjection::points_callback, this, std::placeholders::_1));
      quat_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        gravity_quat_topic_, rclcpp::QoS(10),
        std::bind(&PointsProjection::quat_callback, this, std::placeholders::_1));

      points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

      RCLCPP_INFO(this->get_logger(),
        "points_projection 起動: %s + %s -> %s (frame=%s, z=[%.2f, %.2f])",
        input_topic_.c_str(), gravity_quat_topic_.c_str(),
        output_topic_.c_str(), output_frame_.c_str(), z_min_, z_max_);
    }

  private:
    void quat_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
      tf2::Quaternion q(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
      tf2::Matrix3x3 m(q);
      // 行優先 3x3 (R_sg): p_gravity = R_sg * p_lidar
      r_[0] = m[0][0]; r_[1] = m[0][1]; r_[2] = m[0][2];
      r_[3] = m[1][0]; r_[4] = m[1][1]; r_[5] = m[1][2];
      r_[6] = m[2][0]; r_[7] = m[2][1]; r_[8] = m[2][2];
      have_quat_ = true;
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      if (!have_quat_) {
        // 重力回転未受信時はスキップ (誤射影を避ける)
        return;
      }

      sensor_msgs::PointCloud2Iterator<float> ix(*msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(*msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(*msg, "z");

      std::vector<float> out_xyz;  // (x, y, z) の並び
      out_xyz.reserve(msg->width * msg->height * 3);

      for (size_t i = 0; i < msg->width * msg->height; ++i, ++ix, ++iy, ++iz) {
        const float x_l = *ix;
        const float y_l = *iy;
        const float z_l = *iz;
        if (!std::isfinite(x_l) || !std::isfinite(y_l) || !std::isfinite(z_l)) continue;

        const float x_g = static_cast<float>(r_[0]*x_l + r_[1]*y_l + r_[2]*z_l);
        const float y_g = static_cast<float>(r_[3]*x_l + r_[4]*y_l + r_[5]*z_l);
        const float z_g = static_cast<float>(r_[6]*x_l + r_[7]*y_l + r_[8]*z_l);
        const float z_above_ground = z_g + static_cast<float>(sensor_height_);

        if (z_above_ground < z_min_ || z_above_ground > z_max_) continue;

        out_xyz.push_back(x_g);
        out_xyz.push_back(y_g);
        out_xyz.push_back(z_g);  // os_gravity系の高さを保持 (TF→odomで地面は z≈0, 障害物は高さを持つ)
      }

      // 出力 PointCloud2 (XYZRGB 互換のポイント数分、ここでは XYZ のみ)
      sensor_msgs::msg::PointCloud2 out;
      out.header.stamp = this->now();
      out.header.frame_id = output_frame_;
      out.height = 1;
      out.width = static_cast<uint32_t>(out_xyz.size() / 3);
      out.is_bigendian = false;
      out.is_dense = true;

      sensor_msgs::PointCloud2Modifier modifier(out);
      modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(out.width);

      sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
      sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
      sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
      for (size_t i = 0; i < out.width; ++i, ++ox, ++oy, ++oz) {
        *ox = out_xyz[i * 3 + 0];
        *oy = out_xyz[i * 3 + 1];
        *oz = out_xyz[i * 3 + 2];  // z_g をそのまま保存
      }

      points_pub_->publish(out);
    }

    std::string input_topic_;
    std::string gravity_quat_topic_;
    std::string output_topic_;
    std::string output_frame_;
    double z_min_{-3.0};
    double z_max_{0.0};
    double sensor_height_{0};

    bool have_quat_{false};
    // R_sg (row-major 3x3): p_gravity = R_sg * p_lidar
    double r_[9]{1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quat_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointsProjection>());
    rclcpp::shutdown();
    return 0;
  }