#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <deque>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <chrono>
#include <algorithm>

class DepthClusteringNode : public rclcpp::Node
{
public:
  DepthClusteringNode()
  : Node("depth_clustering_node")
  {
    // -----------------------------
    // Parameters (same as Python)
    // -----------------------------
    declare_parameter<int>("beam_num", 64);
    declare_parameter<int>("horizontal_resolution", 1024);
    declare_parameter<double>("vertical_fov_deg", 45.0);
    declare_parameter<double>("theta_deg", 8.0);
    declare_parameter<int>("min_cluster_size", 10);
    declare_parameter<bool>("remove_ground", true);
    declare_parameter<double>("sensor_height", 0.807);
    declare_parameter<double>("ground_height_tolerance", 0.05);
    declare_parameter<bool>("use_ransac_ground_removal", false);

    beam_num_ = get_parameter("beam_num").as_int();
    h_res_ = get_parameter("horizontal_resolution").as_int();
    v_fov_deg_ = get_parameter("vertical_fov_deg").as_double();
    theta_deg_ = get_parameter("theta_deg").as_double();
    min_cluster_size_ = get_parameter("min_cluster_size").as_int();
    remove_ground_ = get_parameter("remove_ground").as_bool();
    sensor_height_ = get_parameter("sensor_height").as_double();
    ground_tolerance_ = get_parameter("ground_height_tolerance").as_double();
    use_ransac_ = get_parameter("use_ransac_ground_removal").as_bool();

    theta_rad_ = deg2rad(theta_deg_);
    alpha_v_ = deg2rad(v_fov_deg_ / static_cast<double>(beam_num_));
    alpha_h_ = deg2rad(360.0 / static_cast<double>(h_res_));

    // vertical_angles_rad = linspace(0, -v_fov, beam_num) in radians (inclusive endpoints)
    vertical_angles_rad_.resize(beam_num_);
    if (beam_num_ == 1) {
      vertical_angles_rad_[0] = 0.0;
    } else {
      const double start = 0.0;
      const double end = -v_fov_deg_;
      for (int r = 0; r < beam_num_; ++r) {
        const double t = static_cast<double>(r) / static_cast<double>(beam_num_ - 1);
        vertical_angles_rad_[r] = deg2rad(start + (end - start) * t);
      }
    }

    // Logging
    RCLCPP_INFO(get_logger(), "============================================================");
    RCLCPP_INFO(get_logger(), "Depth Clustering Node (Bogoslavskyi Method) [C++ port]");
    RCLCPP_INFO(get_logger(), "============================================================");
    RCLCPP_INFO(get_logger(), "Sensor Configuration:");
    RCLCPP_INFO(get_logger(), "  Beams: %d", beam_num_);
    RCLCPP_INFO(get_logger(), "  H-resolution: %d", h_res_);
    RCLCPP_INFO(get_logger(), "  V-FOV: %.2f deg (0 to -%.2f deg)", v_fov_deg_, v_fov_deg_);
    RCLCPP_INFO(get_logger(), "Algorithm Parameters:");
    RCLCPP_INFO(get_logger(), "  Theta: %.2f deg", theta_deg_);
    RCLCPP_INFO(get_logger(), "  Alpha-V: %.3f deg", rad2deg(alpha_v_));
    RCLCPP_INFO(get_logger(), "  Alpha-H: %.3f deg", rad2deg(alpha_h_));
    RCLCPP_INFO(get_logger(), "Ground Removal:");
    RCLCPP_INFO(get_logger(), "  Enabled: %s", remove_ground_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Sensor height: %.3f m", sensor_height_);
    RCLCPP_INFO(get_logger(), "  Tolerance: %.3f m", ground_tolerance_);
    RCLCPP_INFO(get_logger(), "  Use RANSAC: %s", use_ransac_ ? "true" : "false");
    if (use_ransac_) {
      RCLCPP_WARN(get_logger(),
                  "use_ransac_ground_removal=true but advanced RANSAC is not implemented "
                  "(Python version also lacked remove_ground_advanced). Falling back to fast method.");
    }
    RCLCPP_INFO(get_logger(), "============================================================");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points",
      qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        this->pointCloudCallback(msg);
      }
    );

    auto pub_qos = rclcpp::SensorDataQoS().keep_last(5);

    seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/depth_clustering/segments", pub_qos);

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/depth_clustering/ground", pub_qos);

    RCLCPP_INFO(get_logger(), "Node started!");
    RCLCPP_INFO(get_logger(), "Publishing:");
    RCLCPP_INFO(get_logger(), "  /depth_clustering/segments (objects)");
    RCLCPP_INFO(get_logger(), "  /depth_clustering/ground (ground)");
    RCLCPP_INFO(get_logger(), "Waiting for point clouds...");
  }

private:
  struct PointXYZRGB
  {
    float x;
    float y;
    float z;
    float rgb; // packed rgb as float32 (rviz style)
  };

  static inline double deg2rad(double d) { return d * M_PI / 180.0; }
  static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

  static inline float packRGB_u32_to_f32(uint32_t rgb_u32)
  {
    union U { uint32_t u; float f; };
    U v;
    v.u = rgb_u32;
    return v.f;
  }

  static inline uint32_t rgb_u8(uint8_t r, uint8_t g, uint8_t b)
  {
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8)  |
           (static_cast<uint32_t>(b));
  }

  uint32_t labelToColorU32(int label) const
  {
    // Deterministic hash-like color (Python numpy seed equivalent isn't bit-identical,
    // but deterministic per label and functionally identical for visualization)
    std::mt19937 rng(static_cast<uint32_t>(label));
    std::uniform_int_distribution<int> dist(0, 255);
    uint8_t r = static_cast<uint8_t>(dist(rng));
    uint8_t g = static_cast<uint8_t>(dist(rng));
    uint8_t b = static_cast<uint8_t>(dist(rng));
    return rgb_u8(r, g, b);
  }

  // -----------------------------------------
  // 1) PointCloud2 -> Range Image (float32)
  // -----------------------------------------
  bool pointcloudToRangeImage(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::vector<float> & range_image_out
  )
  {
    range_image_out.assign(static_cast<size_t>(beam_num_) * static_cast<size_t>(h_res_), 0.0f);

    // iterate points
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    const double v_fov_rad = deg2rad(v_fov_deg_);

    size_t inserted = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      // skip NaN is handled by iterator? it does not skip NaN automatically; mimic skip_nans=True
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const float d = std::sqrt(x*x + y*y + z*z);
      if (!(d > 0.0f) || !std::isfinite(d)) {
        continue;
      }

      const double h_ang = std::atan2(static_cast<double>(y), static_cast<double>(x)); // [-pi, pi]
      const double v_ang = std::asin(std::clamp(static_cast<double>(z) / (static_cast<double>(d) + 1e-8), -1.0, 1.0));

      int c = static_cast<int>(((h_ang + M_PI) / (2.0 * M_PI)) * static_cast<double>(h_res_));
      c = std::clamp(c, 0, h_res_ - 1);

      int r = static_cast<int>((-v_ang / v_fov_rad) * static_cast<double>(beam_num_));
      r = std::clamp(r, 0, beam_num_ - 1);

      const size_t idx = static_cast<size_t>(r) * static_cast<size_t>(h_res_) + static_cast<size_t>(c);

      // keep nearest per pixel
      const float prev = range_image_out[idx];
      if (prev == 0.0f || d < prev) {
        range_image_out[idx] = d;
      }
      ++inserted;
    }

    // If no valid pixel, treat as empty
    size_t valid_pixels = 0;
    for (float v : range_image_out) {
      if (v > 0.0f) ++valid_pixels;
    }
    if (valid_pixels == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Range image has 0 valid pixels.");
      return false;
    }
    (void)inserted;
    return true;
  }

  // -----------------------------------------
  // 2) Ground removal (fast)
  //    Returns filtered range image + ground mask
  // -----------------------------------------
  void removeGroundFast(
    std::vector<float> & range_image_io,
    std::vector<uint8_t> & ground_mask_out
  )
  {
    ground_mask_out.assign(range_image_io.size(), 0);

    if (!remove_ground_) {
      return;
    }

    for (int r = 0; r < beam_num_; ++r) {
      const double v_angle = vertical_angles_rad_[r];
      const double sv = std::sin(v_angle);

      const size_t row_base = static_cast<size_t>(r) * static_cast<size_t>(h_res_);
      for (int c = 0; c < h_res_; ++c) {
        const size_t idx = row_base + static_cast<size_t>(c);
        const float d = range_image_io[idx];
        if (!(d > 0.0f)) continue;

        const double height = sensor_height_ + static_cast<double>(d) * sv;
        if (std::abs(height) <= ground_tolerance_) {
          ground_mask_out[idx] = 1;
          range_image_io[idx] = 0.0f; // remove ground from filtered image
        }
      }
    }
  }

  // -----------------------------------------
  // 3) Segmentation (Algorithm 1 style BFS)
  // -----------------------------------------
  inline bool shouldConnect(
    const std::vector<float> & range_image,
    int r1, int c1,
    int r2, int c2,
    double alpha
  ) const
  {
    const float d1 = range_image[static_cast<size_t>(r1) * h_res_ + static_cast<size_t>(c1)];
    const float d2 = range_image[static_cast<size_t>(r2) * h_res_ + static_cast<size_t>(c2)];

    if (!(d1 > 0.0f) || !(d2 > 0.0f)) return false;

    const double d_max = std::max<double>(d1, d2);
    const double d_min = std::min<double>(d1, d2);

    const double numerator = d_min * std::sin(alpha);
    const double denominator = d_max - d_min * std::cos(alpha);

    if (denominator <= 1e-6) return false;

    const double beta = std::atan(numerator / denominator);
    return beta > theta_rad_;
  }

  void labelComponentBFS(
    const std::vector<float> & range_image,
    std::vector<int> & labels,
    int start_r, int start_c,
    int label
  )
  {
    std::deque<int> q;
    q.push_back(start_r * h_res_ + start_c);

    while (!q.empty()) {
      const int idx = q.front();
      q.pop_front();

      if (labels[static_cast<size_t>(idx)] != 0) continue;
      labels[static_cast<size_t>(idx)] = label;

      const int r = idx / h_res_;
      const int c = idx % h_res_;

      // N4 neighbors: up/down/left/right (wrap in horizontal)
      // up
      if (r > 0) {
        const int nr = r - 1, nc = c;
        const int nidx = nr * h_res_ + nc;
        if (labels[static_cast<size_t>(nidx)] == 0) {
          const float dv = range_image[static_cast<size_t>(nidx)];
          if (dv > 0.0f && shouldConnect(range_image, r, c, nr, nc, alpha_v_)) q.push_back(nidx);
        }
      }
      // down
      if (r < beam_num_ - 1) {
        const int nr = r + 1, nc = c;
        const int nidx = nr * h_res_ + nc;
        if (labels[static_cast<size_t>(nidx)] == 0) {
          const float dv = range_image[static_cast<size_t>(nidx)];
          if (dv > 0.0f && shouldConnect(range_image, r, c, nr, nc, alpha_v_)) q.push_back(nidx);
        }
      }
      // left (wrap)
      {
        const int nr = r;
        const int nc = (c - 1 + h_res_) % h_res_;
        const int nidx = nr * h_res_ + nc;
        if (labels[static_cast<size_t>(nidx)] == 0) {
          const float dv = range_image[static_cast<size_t>(nidx)];
          if (dv > 0.0f && shouldConnect(range_image, r, c, nr, nc, alpha_h_)) q.push_back(nidx);
        }
      }
      // right (wrap)
      {
        const int nr = r;
        const int nc = (c + 1) % h_res_;
        const int nidx = nr * h_res_ + nc;
        if (labels[static_cast<size_t>(nidx)] == 0) {
          const float dv = range_image[static_cast<size_t>(nidx)];
          if (dv > 0.0f && shouldConnect(range_image, r, c, nr, nc, alpha_h_)) q.push_back(nidx);
        }
      }
    }
  }

  void segmentRangeImage(
    const std::vector<float> & range_image,
    std::vector<int> & labels_out
  )
  {
    labels_out.assign(range_image.size(), 0);
    int current_label = 1;

    for (int r = 0; r < beam_num_; ++r) {
      const size_t row_base = static_cast<size_t>(r) * static_cast<size_t>(h_res_);
      for (int c = 0; c < h_res_; ++c) {
        const size_t idx = row_base + static_cast<size_t>(c);
        if (labels_out[idx] == 0 && range_image[idx] > 0.0f) {
          labelComponentBFS(range_image, labels_out, r, c, current_label);
          ++current_label;
        }
      }
    }
  }

  // -----------------------------------------
  // 4) Filter small clusters
  // -----------------------------------------
  void filterSmallClusters(std::vector<int> & labels_io, int min_size)
  {
    std::unordered_map<int, int> counts;
    counts.reserve(1024);

    for (int lab : labels_io) {
      if (lab > 0) counts[lab] += 1;
    }
    for (int & lab : labels_io) {
      if (lab > 0) {
        auto it = counts.find(lab);
        if (it != counts.end() && it->second < min_size) lab = 0;
      }
    }
  }

  // -----------------------------------------
  // 5) Publish PointCloud2
  // -----------------------------------------
  void publishCloud(
    const std::vector<PointXYZRGB> & pts,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub
  )
  {
    if (pts.empty()) return;

    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
    out.height = 1;
    out.width = static_cast<uint32_t>(pts.size());
    out.is_bigendian = false;
    out.is_dense = true;

    out.fields.resize(4);
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[0].count = 1;

    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[1].count = 1;

    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[2].count = 1;

    out.fields[3].name = "rgb";
    out.fields[3].offset = 12;
    out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[3].count = 1;

    out.point_step = sizeof(PointXYZRGB);   // 16
    out.row_step = out.point_step * out.width;

    out.data.resize(static_cast<size_t>(out.row_step));
    std::memcpy(out.data.data(), pts.data(), out.data.size());

    pub->publish(out);
  }

  void publishSegments(const std::vector<float> & range_image, const std::vector<int> & labels, const sensor_msgs::msg::PointCloud2 & original_msg)
  {
    std::vector<PointXYZRGB> pts;
    pts.reserve(20000);

    for (int r = 0; r < beam_num_; ++r) {
      const double v_angle = vertical_angles_rad_[r];
      const double cv = std::cos(v_angle);
      const double sv = std::sin(v_angle);

      const size_t row_base = static_cast<size_t>(r) * static_cast<size_t>(h_res_);
      for (int c = 0; c < h_res_; ++c) {
        const size_t idx = row_base + static_cast<size_t>(c);
        const int lab = labels[idx];
        if (lab <= 0) continue;

        const float d = range_image[idx];
        if (!(d > 0.0f)) continue;

        const double h_angle = (static_cast<double>(c) / static_cast<double>(h_res_)) * (2.0 * M_PI) - M_PI;
        const double ch = std::cos(h_angle);
        const double sh = std::sin(h_angle);

        const float x = static_cast<float>(static_cast<double>(d) * cv * ch);
        const float y = static_cast<float>(static_cast<double>(d) * cv * sh);
        const float z = static_cast<float>(static_cast<double>(d) * sv);

        const uint32_t rgb_u32 = labelToColorU32(lab);
        const float rgb_f32 = packRGB_u32_to_f32(rgb_u32);

        pts.push_back(PointXYZRGB{x, y, z, rgb_f32});
      }
    }

    publishCloud(pts, original_msg.header, seg_pub_);
  }

  void publishGround(const std::vector<float> & range_image_original, const std::vector<uint8_t> & ground_mask, const sensor_msgs::msg::PointCloud2 & original_msg)
  {
    std::vector<PointXYZRGB> pts;
    pts.reserve(6000);

    const uint32_t green_u32 = rgb_u8(0, 255, 0);
    const float green_f32 = packRGB_u32_to_f32(green_u32);

    for (int r = 0; r < beam_num_; ++r) {
      const double v_angle = vertical_angles_rad_[r];
      const double cv = std::cos(v_angle);
      const double sv = std::sin(v_angle);

      const size_t row_base = static_cast<size_t>(r) * static_cast<size_t>(h_res_);
      for (int c = 0; c < h_res_; ++c) {
        const size_t idx = row_base + static_cast<size_t>(c);
        if (ground_mask[idx] == 0) continue;

        const float d = range_image_original[idx];
        if (!(d > 0.0f)) continue;

        const double h_angle = (static_cast<double>(c) / static_cast<double>(h_res_)) * (2.0 * M_PI) - M_PI;
        const double ch = std::cos(h_angle);
        const double sh = std::sin(h_angle);

        const float x = static_cast<float>(static_cast<double>(d) * cv * ch);
        const float y = static_cast<float>(static_cast<double>(d) * cv * sh);
        const float z = static_cast<float>(static_cast<double>(d) * sv);

        pts.push_back(PointXYZRGB{x, y, z, green_f32});
      }
    }

    publishCloud(pts, original_msg.header, ground_pub_);
  }

  // -----------------------------------------
  // Callback
  // -----------------------------------------
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto t_start = std::chrono::steady_clock::now();

    // 1) Convert
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<float> range_image;
    if (!pointcloudToRangeImage(*msg, range_image)) {
      return;
    }
    const auto t1 = std::chrono::steady_clock::now();

    // Keep original for ground visualization
    std::vector<float> range_image_original = range_image;
    int num_input_points = 0;
    for (float v : range_image_original) if (v > 0.0f) ++num_input_points;

    // 2) Ground removal
    const auto t2 = std::chrono::steady_clock::now();
    std::vector<uint8_t> ground_mask;
    removeGroundFast(range_image, ground_mask);
    const auto t3 = std::chrono::steady_clock::now();

    int num_after_ground = 0;
    int num_ground = 0;
    for (size_t i = 0; i < range_image.size(); ++i) {
      if (range_image[i] > 0.0f) ++num_after_ground;
      if (ground_mask[i] != 0) ++num_ground;
    }

    // 3) Segmentation
    const auto t4 = std::chrono::steady_clock::now();
    std::vector<int> labels;
    segmentRangeImage(range_image, labels);
    const auto t5 = std::chrono::steady_clock::now();

    // 4) Filter
    const auto t6 = std::chrono::steady_clock::now();
    filterSmallClusters(labels, min_cluster_size_);
    const auto t7 = std::chrono::steady_clock::now();

    // Count clusters
    std::unordered_map<int, int> cluster_seen;
    cluster_seen.reserve(1024);
    int num_object_points = 0;
    for (int lab : labels) {
      if (lab > 0) {
        ++num_object_points;
        cluster_seen.emplace(lab, 1);
      }
    }
    const int num_clusters = static_cast<int>(cluster_seen.size());

    // 5) Publish
    const auto t8 = std::chrono::steady_clock::now();
    publishSegments(range_image, labels, *msg);
    publishGround(range_image_original, ground_mask, *msg);
    const auto t9 = std::chrono::steady_clock::now();

    const auto ms = [](auto a, auto b) -> double {
      return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(b - a).count();
    };

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,  // 1秒に1回
      "Total: %.0fms | Convert: %.0fms | Ground: %.0fms | Segment: %.0fms | Filter: %.0fms | Publish: %.0fms | Points: %d->%d(%d ground) | Clusters: %d",
      ms(t_start, t9),
      ms(t0, t1),
      ms(t2, t3),
      ms(t4, t5),
      ms(t6, t7),
      ms(t8, t9),
      num_input_points,
      num_after_ground,
      num_ground,
      num_clusters
    );

  }

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr seg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;

  // Params
  int beam_num_{64};
  int h_res_{1024};
  double v_fov_deg_{45.0};
  double theta_deg_{8.0};
  double theta_rad_{0.0};
  int min_cluster_size_{10};
  bool remove_ground_{true};
  double sensor_height_{0.807};
  double ground_tolerance_{0.05};
  bool use_ransac_{false};

  // Angles
  double alpha_v_{0.0};
  double alpha_h_{0.0};
  std::vector<double> vertical_angles_rad_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthClusteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
