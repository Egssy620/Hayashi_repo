# Iterative Seed Convergence Wavefront Ground Removal

A novel ground removal algorithm for 3D LiDAR point clouds using wavefront propagation on Range Images, with Range Image-based segmentation.

## Overview

This package provides real-time ground removal and point cloud segmentation for 3D LiDAR sensors, specifically optimized for **Below Horizon** configurations (e.g., Ouster OS-1-32-BH).

### Key Features

- **Wavefront-based Ground Removal**: First application of wavefront propagation to 3D LiDAR ground segmentation
- **Iterative Seed Convergence**: Robust seed selection that automatically excludes obstacle bottoms
- **Real-time Performance**: ~35ms total processing at 28+ Hz (on standard CPU)
- **Below Horizon Optimization**: Exploits expected range geometry for improved accuracy

## Algorithm

### 1. Iterative Seed Convergence

Traditional ground removal methods often misclassify obstacle bottoms as ground. Our iterative seed convergence algorithm solves this by:

1. Collecting floor candidates from bottom LiDAR beams
2. Selecting initial representative points per beam
3. Iteratively replacing outliers (high Z-values = likely obstacles) until Z-value standard deviation converges
4. Expanding seeds from the converged reference Z-value

```
Initial:                    After Convergence:
Beam 0: z=-0.78 (floor)     Beam 0: z=-0.78 ✓
Beam 1: z=-0.80 (floor)     Beam 1: z=-0.80 ✓
Beam 2: z=-0.35 (obstacle!) Beam 2: z=-0.77 ✓ (replaced)
Beam 3: z=-0.79 (floor)     Beam 3: z=-0.79 ✓
Beam 4: z=-0.81 (floor)     Beam 4: z=-0.81 ✓

std_dev: 0.18 → 0.015 (converged!)
```

### 2. Wavefront Propagation

From converged seeds, the algorithm propagates across the Range Image using floor-specific transition rules:

- **Height difference**: Adjacent points must have similar Z-values
- **Slope constraint**: Local slope must be below threshold (e.g., 10°)
- **Range continuity**: No sudden distance jumps (indicates object boundary)
- **Normal check**: Local surface normal must be upward-facing

Propagation naturally stops at walls and objects.

### 3. Range Image Segmentation

Non-ground points are segmented using the Bogoslavskyi method [1], which uses angle-based connectivity on Range Images.

## Performance

| Metric | Value |
|--------|-------|
| Total processing time | ~35ms |
| Ground removal | ~0.6ms |
| Segmentation | ~2ms |
| Frame rate | 28+ Hz |
| Ground detection | ~18% of points |

Tested on: Intel Core i7, Ouster OS-1-32-BH13 (1024x32 @ 10Hz)

## Installation

### Prerequisites

- ROS2 Humble
- Ouster ROS2 driver
- nlohmann-json

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select depth_clustering_cpp --symlink-install
source install/setup.bash
```

## Usage

### Basic Usage

```bash
ros2 run depth_clustering_cpp depth_clustering_node
```

### With Parameters

```bash
ros2 run depth_clustering_cpp depth_clustering_node --ros-args \
  -p sensor_height:=0.385 \
  -p seed_convergence_threshold:=0.05 \
  -p min_normal_z:=0.85
```

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ouster/range_image` | `sensor_msgs/Image` | Range image (mono16) |
| `/ouster/metadata` | `std_msgs/String` | Sensor metadata (JSON) |
| `/ouster/imu` | `sensor_msgs/Imu` | IMU (used for tilt compensation when enabled) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/depth_clustering/segments` | `sensor_msgs/PointCloud2` | Segmented objects (colored) |
| `/depth_clustering/ground` | `sensor_msgs/PointCloud2` | Ground points (green) |

## Parameters

### Seed Selection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `seed_rows` | 5 | Number of bottom beams for seed candidates |
| `seed_convergence_threshold` | 0.05 | Z standard deviation threshold [m] |
| `seed_max_iterations` | 10 | Maximum convergence iterations |
| `seed_min_valid_rows` | 3 | Minimum valid beams required |
| `seed_z_expand_threshold` | 0.08 | Z tolerance for seed expansion [m] |

### Wavefront Propagation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_height_diff` | 0.08 | Max height difference between adjacent points [m] |
| `max_slope` | 0.18 | Max slope (tan of angle, 0.18 ≈ 10°) |
| `base_range_diff` | 0.1 | Base range difference tolerance [m] |
| `use_normal_check` | true | Enable local normal verification |
| `min_normal_z` | 0.85 | Minimum normal Z component (cos 30° ≈ 0.866) |

### Segmentation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `theta_deg` | 10.0 | Angle threshold for connectivity [deg] |
| `min_cluster_size` | 10 | Minimum points per cluster |
| `min_z_extent` | 0.05 | Minimum Z extent to keep cluster [m] |

### Sensor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensor_height` | 0.385 | Sensor height from ground [m] |
| `metadata_path` | - | Path to sensor metadata JSON |

### IMU Tilt Compensation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_imu_tilt_compensation` | true | Apply IMU-derived gravity alignment to range image XYZ before wavefront propagation |

When enabled, the node subscribes to `/ouster/imu` and computes a shortest-arc rotation matrix that maps the sensor-frame gravity direction (from `linear_acceleration`) to `(0, 0, 1)`. The rotation is applied to the wavefront's internal XYZ cache so that `sensor_height + z` corresponds to true height above the ground even when the sensor is pitched or rolled. The published point cloud remains in the sensor frame for geometrically correct visualization.

`use_expected_range_check` is forced off when tilt compensation is active, since the beam-angle-only expected-range formula assumes an upright sensor.

## Visualization

```bash
rviz2
# Add PointCloud2 displays for:
#   /depth_clustering/segments
#   /depth_clustering/ground
```

## Known Limitations

### Sensor Tilt

The wavefront propagation assumes `sensor_height + z` corresponds to absolute height above the ground. When the sensor is mounted tilted (the current robot has the OS-1-32 pitched **10° forward** at the center), this assumption breaks and ground detection accuracy degrades.

The package ships with **IMU tilt compensation** (`use_imu_tilt_compensation`, on by default) that reads the gravity direction from `/ouster/imu` and rotates each frame's XYZ cache into a gravity-aligned frame before wavefront propagation. The IMU stream is low-pass filtered (EMA, α=0.05, τ ≈ 0.2 s) so the orientation estimate stays stable when the platform is at rest and follows gradual pitch/roll changes during motion.

If tilt compensation needs to be disabled for diagnosis, set `use_imu_tilt_compensation:=false` and the algorithm falls back to the original sensor-frame behavior.

## Citation

If you use this work in your research, please cite:

```bibtex
@software{wavefront_ground_removal,
  author = {Egssy, Ryuzo},
  title = {Iterative Seed Convergence Wavefront Ground Removal},
  year = {2025},
  url = {https://github.com/YOUR_USERNAME/wavefront_ground_removal}
}
```

## References

[1] I. Bogoslavskyi and C. Stachniss, "Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2016. https://github.com/PRBonn/depth_clustering

## License

Copyright (c) 2025 Ryuzo Egssy. All rights reserved.

## Author

Ryuzo Egssy