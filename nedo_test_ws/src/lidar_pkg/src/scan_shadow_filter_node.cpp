// URG の LaserScan を、経路計画用の平面障害物 PointCloud2 へ変換する。
// ros2_ws_林研/lidar_pkg の実験済み影除去処理から、今回必要な機能だけを移植した。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace scan_shadow_filter
{

class ScanShadowFilterNode : public rclcpp::Node
{
public:
  explicit ScanShadowFilterNode(const rclcpp::NodeOptions & options)
  : Node("scan_shadow_filter_node", options)
  {
    raw_mode_ = declare_parameter<bool>("raw_mode", false);
    invert_vertical_ = declare_parameter<bool>("invert_vertical", false);
    threshold_ = declare_parameter<double>("threshold", 0.30);
    proximity_radius_ = declare_parameter<double>("proximity_radius", 1.50);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");

    const double angle_min_deg = declare_parameter<double>("angle_min_deg", -90.0);
    const double angle_max_deg = declare_parameter<double>("angle_max_deg", 90.0);
    angle_min_rad_ = angle_min_deg * M_PI / 180.0;
    angle_max_rad_ = angle_max_deg * M_PI / 180.0;

    laser_offset_x_ = declare_parameter<double>("laser_offset_x", 0.0);
    laser_offset_y_ = declare_parameter<double>("laser_offset_y", 0.16);
    laser_offset_yaw_ = declare_parameter<double>("laser_offset_yaw", 0.0);
    min_range_ = declare_parameter<double>("min_range", 0.01);
    min_radius_robot_ = declare_parameter<double>("min_radius_robot", 0.30);
    epsilon_radius_ = declare_parameter<double>("epsilon_radius", 0.0);

    if (threshold_ < 0.0 || proximity_radius_ <= 0.0 ||
      angle_min_rad_ > angle_max_rad_ || min_range_ < 0.0 ||
      min_radius_robot_ < 0.0 || epsilon_radius_ < 0.0)
    {
      throw std::runtime_error("invalid scan_shadow_filter_node parameter");
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanShadowFilterNode::onScan, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);

    RCLCPP_INFO(
      get_logger(),
      "URG filter ready: /scan -> /filtered_points, frame=%s, range=(%.2f, %.2f] m",
      output_frame_id_.c_str(), min_radius_robot_, proximity_radius_);
  }

private:
  static std::vector<bool> makeShadowMask(
    const std::vector<float> & ranges, const double threshold)
  {
    std::vector<bool> mask(ranges.size(), true);
    if (threshold <= 0.0) {
      return mask;
    }

    for (std::size_t i = 0; i + 1 < ranges.size(); ++i) {
      if (std::isfinite(ranges[i]) && std::isfinite(ranges[i + 1]) &&
        std::fabs(ranges[i + 1] - ranges[i]) > threshold)
      {
        mask[i] = false;
        mask[i + 1] = false;
      }
    }
    return mask;
  }

  static sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<std::array<float, 2>> & points,
    const std_msgs::msg::Header & source_header,
    const std::string & output_frame_id)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = source_header;
    cloud.header.frame_id = output_frame_id;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point[0];
      *y = point[1];
      *z = 0.0F;
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const std::vector<bool> shadow_mask = makeShadowMask(scan->ranges, threshold_);
    std::vector<std::array<float, 2>> points;
    points.reserve(scan->ranges.size());

    const double yaw_cos = std::cos(laser_offset_yaw_);
    const double yaw_sin = std::sin(laser_offset_yaw_);
    const double valid_min_range = std::max<double>(scan->range_min, min_range_);
    const double max_radius_squared = proximity_radius_ * proximity_radius_;
    const double min_radius_squared = min_radius_robot_ * min_radius_robot_;

    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }

      const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      if (angle < angle_min_rad_ || angle > angle_max_rad_ || range <= valid_min_range) {
        continue;
      }
      if (!raw_mode_ && !shadow_mask[i]) {
        continue;
      }

      const double laser_x = range * std::cos(angle);
      const double laser_y = range * std::sin(angle);
      const double base_x = yaw_cos * laser_x - yaw_sin * laser_y + laser_offset_x_;
      double base_y = yaw_sin * laser_x + yaw_cos * laser_y + laser_offset_y_;
      if (invert_vertical_) {
        base_y = -base_y;
      }

      const double radius_squared = base_x * base_x + base_y * base_y;
      if (radius_squared < min_radius_squared || radius_squared > max_radius_squared) {
        continue;
      }
      if (epsilon_radius_ > 0.0 && radius_squared < epsilon_radius_ * epsilon_radius_) {
        continue;
      }
      points.push_back({static_cast<float>(base_x), static_cast<float>(base_y)});
    }

    cloud_pub_->publish(makeCloud(points, scan->header, output_frame_id_));
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  bool raw_mode_;
  bool invert_vertical_;
  double threshold_;
  double proximity_radius_;
  std::string output_frame_id_;
  double angle_min_rad_;
  double angle_max_rad_;
  double laser_offset_x_;
  double laser_offset_y_;
  double laser_offset_yaw_;
  double min_range_;
  double min_radius_robot_;
  double epsilon_radius_;
};

}  // namespace scan_shadow_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_shadow_filter::ScanShadowFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
