# nedo_test_ws - シャトル運転システム

Ouster OS-1-32 LiDAR とメカナムローバを使ったシャトル運転 (Goal 0 ↔ Goal 1 往復) システム。

## システム概要

### ノード構成

```
[Ouster HW] → /ouster/range_image, /ouster/metadata ─┐
                                                       ├→ depth_clustering_node ─┬→ /depth_clustering/segments
                                                       │                          ├→ /depth_clustering/ground
                                                       │                          └→ /depth_clustering/gravity_quat + TF(os_lidar→os_gravity)
                                                       ↓
                                        points_projection_node (重力回転→z足切り→2D射影)
                                                       ↓
                                        /filtered_points (frame: os_gravity)
                                                       ↓
                                                   manage_node (Pure Pursuit + 障害物回避)
                                                       ↓
                                            /target_point, /current_goal, /cmd_source
                                                       ↓
              /robot_pose2d → heading_test_node → /heading/rover_twist ─┐
              /robot_pose2d → drive_node      → /drive/rover_twist    ─┤
                                                                       ↓
                                                    cmd_vel_mux_node → /rover_twist → [Mecanum HW]
```

### TF ツリー

```
odom → base_link → os_sensor → os_lidar → os_gravity
```

- `odom → base_link`: 自己位置推定ノードが出版 (rover_odom_to_pose2d または独自)
- `base_link → os_sensor`: LiDAR 取付位置 (静的、static_transform_publisher)
- `os_sensor → os_lidar`: Ouster 標準の 180° Z 回転 (ouster driver)
- `os_lidar → os_gravity`: LiDAR 傾斜の IMU リアルタイム推定 (depth_clustering_node)

## 必要環境

- Ubuntu 22.04
- ROS2 Humble
- 外部パッケージ: `ouster_ros` (別途インストール)
- システム依存: `libopencv-dev`, `nlohmann-json3-dev`

## セットアップ

### 1. ワークスペース取得

`git clone` または `scp` / `rsync` で `~/nedo_test_ws` を配置:

```bash
rsync -av --exclude='build' --exclude='install' --exclude='log' \
  <送信元PC>:~/nedo_test_ws/ ~/nedo_test_ws/
```

### 2. 依存インストール

```bash
cd ~/nedo_test_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

### 3. ouster_ros インストール (別途)

```bash
sudo apt update
sudo apt install ros-humble-ouster-ros
```

またはソースビルド:
```bash
mkdir -p ~/ouster_ws/src && cd ~/ouster_ws/src
git clone -b humble https://github.com/ouster-lidar/ouster-ros.git
cd ~/ouster_ws && colcon build --symlink-install
```

### 4. ビルド

```bash
cd ~/nedo_test_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. 必須ファイルの用意

- `ouster_params.yaml`: Ouster ドライバ設定 (sensor_hostname 等を記載)
- `192.168.1-metadata.json`: Ouster センサメタデータ (センサ起動時に自動生成される)

## 実行

### 1. 自己位置推定ノード起動

`shuttle_system.launch.py` は自己位置推定を含まない。別途起動して `/robot_pose2d` (Pose2D) と TF `odom → base_link` を供給する。

```bash
# 標準構成 (オドメトリ積分)
ros2 run rover_odom_to_pose2d rover_odom_to_pose2d_node &

# または独自推定ノード (EKF, SLAM 等)
# ros2 run <your_localization_pkg> <your_node> &
```

### 2. シャトルシステム起動

```bash
ros2 launch manage_node shuttle_system.launch.py \
  ouster_params:=<path-to-ouster_params.yaml> \
  metadata_path:=<path-to-192.168.1-metadata.json>

ros2 launch manage_node shuttle_system.launch.py \
  ouster_params:=/root/NEDO_ROS/src/ros_ws/nedo_test_ws/ouster_params.yaml\
  metadata_path:=/root/NEDO_ROS/src/ros_ws/nedo_test_ws/192.168.1-metadata.json

```


### 3. パラメータ上書き例

```bash
ros2 launch manage_node shuttle_system.launch.py \
  ouster_params:=~/ouster_params.yaml \
  metadata_path:=~/192.168.1-metadata.json \
  obstacle_avoid_radius:=0.7 \
  robot_radius:=0.3 \
  voicevox_url:=""
```

## 別PC 構成 (Ouster ドライバ分離)

Ouster ドライバを別PC で動かすことで、メインPC の負荷を分散できる。

### 別PC 側

```bash
# ouster_ros インストール後
source /opt/ros/humble/setup.bash
ros2 launch ouster_ros driver.launch.py params_file:=~/ouster_params.yaml viz:=false
```

### メインPC 側

1. `shuttle_system.launch.py` の以下をコメントアウト:
   - `ouster_launch = IncludeLaunchDescription(...)` ブロック
   - `return LaunchDescription([...])` 内の `ouster_launch`
   - `ouster_params_arg` (不要になれば)

2. 両PC で `ROS_DOMAIN_ID` を合わせる (`~/.bashrc` 等に記載):
   ```bash
   export ROS_DOMAIN_ID=30
   ```

3. メインPC 側は `ouster_params` 引数なしで起動:
   ```bash
   ros2 launch manage_node shuttle_system.launch.py \
     metadata_path:=~/192.168.1-metadata.json
   ```

### ネットワーク要件

- 有線 LAN 推奨 (Ouster 点群は 50-100 Mbps)
- 両 PC 同一サブネット
- ファイアウォール: `sudo ufw allow 7400:7500/udp` (DDS 通信用)

## パラメータ一覧

### manage_node

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `goals` | `vector<double>` | `[0,0,2,0]` | シャトルゴール平坦配列 `[x0,y0,x1,y1,...]` |
| `robot_radius` | `double` | `0.2` | 最近点除外閾値 [m] |
| `obstacle_avoid_radius` | `double` | `0.5` | 障害物回避円 r2 [m] |
| `path_band_half_width` | `double` | `0.2` | 経路バンド片側幅 [m] |
| `lookahead_distance` | `double` | `0.5` | Pure Pursuit 注視距離 [m] |
| `lidar_history_sec` | `double` | `3.0` | 過去点群保持期間 [s] |
| `stop_duration` | `double` | `2.0` | ゴール到着後停止時間 [s] |
| `voicevox_url` | `string` | `http://localhost:50021` | Voicevox URL (空文字で無効) |

### drive_node

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `v_max` | 0.20 | 最大速度 [m/s] |
| `v_min` | 0.05 | 最小速度 [m/s] |
| `slowdown_dist` | 2.5 | 減速開始距離 [m] |
| `stop_dist` | 0.05 | 到着判定距離 [m] |
| `omega_max` | 0.20 | 最大角速度 [rad/s] |
| `rate` | 20.0 | 制御レート [Hz] |
| `omega_filter_tau` | 0.45 | 角速度 LPF 時定数 |
| `control_dt` | 0.05 | 制御周期 [s] |

### depth_clustering_node

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `use_metadata_file` | false | メタデータファイルから読む |
| `metadata_path` | "" | メタデータ JSON パス |
| `min_range` | 0.2 | 最近点切り捨て [m] |
| `theta_deg` | 10.0 | 初期 LiDAR 傾斜推定 [deg] |
| `min_cluster_size` | 10 | 最小クラスタサイズ |
| `remove_ground` | true | 地面除去 |
| `sensor_height` | 1.8 (launch 経由) | センサ高さ [m] (cpp default 0.385 は上書き) |

### points_projection_node

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `input_topic` | `/depth_clustering/segments` | 入力点群 |
| `gravity_quat_topic` | `/depth_clustering/gravity_quat` | 重力クォータニオン |
| `output_topic` | `/filtered_points_shuttle` | 出力点群 (衝突回避のためリネーム) |
| `output_frame` | `os_gravity` | 出力フレーム |
| `sensor_height` | 1.85 | センサ高さ [m] (地面基準 z 計算用) |
| `z_min` | 0.05 | 足切り下限 [m] (地面から) |
| `z_max` | 1.85 | 足切り上限 [m] (地面から) |

### rover_odom_to_pose2d_node

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `kvx`, `kvy`, `kth` | 1.0 | オドメトリゲイン |
| `rate` | 100.0 | 出版レート [Hz] |

## トラブルシューティング

- **TF が繋がらない**: `static_transform_publisher (base_link → os_sensor)` 起動確認。`ros2 run tf2_tools view_frames` でツリー可視化。
- **音声が出ない**: `voicevox_url` が正しいか、Voicevox サーバ起動中か。未導入なら `voicevox_url:=""` で無効化。
- **LiDAR 点群が見えない**: ouster driver 起動確認、`metadata_path` が正しいか。`ros2 topic hz /ouster/points` で出版確認。
- **`/filtered_points` が出ない**: `points_projection_node` が `/depth_clustering/gravity_quat` を受信しているか (`depth_clustering_node` が IMU 初期化済みか)。
- **`ros2 launch` で Ctrl+C が効きにくい**: `sigterm_timeout` / `sigkill_timeout` を短く設定済み。最大 7 秒で強制終了。それでも遅い場合は `pkill -9 -f "nedo_test_ws/install"`。
- **別PC 構成でトピックが見えない**: `ROS_DOMAIN_ID` が両PCで同じか。`ping` で相互到達確認。マルチキャストが通らないネットワークでは FastDDS ユニキャスト設定が必要。

## ディレクトリ構成

```
nedo_test_ws/
├── README.md                      # このファイル
├── cmd.txt                        # ビルド・実行コマンド早見表
└── src/
    ├── cmd_vel_mux/               # Twist 切り替え
    ├── depth_clustering_cpp/      # 3D LiDAR セグメンテーション
    ├── drive_node/                # Pure Pursuit
    ├── heading_test_node/         # PID ヘディング制御
    ├── manage_node/               # ミッション調停 (launch 含む)
    ├── points_projection/         # 3D → 2D 射影
    └── rover_odom_to_pose2d/      # オドメトリ → Pose2D (移植時は差し替え)
```

## 変更履歴

### 2026-06-21 セグメンテーション破綻修正・トピック衝突解消

#### 1. depth_clustering の `sensor_height` を launch 引数に

セグメンテーション破綻の根本原因。センサ高さ 1.8 m に対し、depth_clustering が cpp デフォルト 0.385 m で動いていたため、地面除去の期待距離計算が 4.6 倍ずれ、床が巨大障害物クラスタとして segments に出て manage_node の回避が破綻していた。

- `shuttle_system.launch.py` に `sensor_height_arg` (default 1.8) を追加
- depth_clustering Node の parameters に `sensor_height` を渡す

#### 2. points_projection の z 基準を地面基準に統一

os_gravity 座標系の z (センサ位置基準) をそのまま使うと、`z_min/z_max` が「センサから上方向」の意味になり、下流の manage_node が期待する z クリップ (odom=base_link 基準 = 地面基準) とズレて全点が弾かれていた。

- `points_projection_node.cpp` に `sensor_height` パラメータ追加 (default 1.85)
- フィルタ条件を `z_above_ground = z_g + sensor_height` で判定 (地面基準)
- `z_min=0.05, z_max=1.85` は「地面から 5 cm 上〜 1.85 m 上」を意味する
- メンバ `z_max_` デフォルトの不整合 (0.5 vs 1.8) を解消
- launch の points_projection Node に `sensor_height` パラメータを渡す

#### 3. `/filtered_points` → `/filtered_points_shuttle` リネーム

ros_ws 側 (自己位置推定系) の `nedo_localization/hesai_region_filter_node` が同じ `/filtered_points` を publish し、シャトル系とトピック名衝突を起こしていた。

- `points_projection_node.cpp` の `output_topic` default を `/filtered_points_shuttle` に
- `manage_node.cpp` の subscribe topic を `/filtered_points_shuttle` に

シャトル系内部で完結 (launch 側は変更不要、パラメータ渡し無しで動作)。

### 検証用 (ws 外、コミットしない)

- `/tmp/odom_to_tf.py`: bag 再生時の TF ブリッジ
  - bag の `/odom` (LIO-SAM 等のローカル odom → base_link) を subscribe して TF `odom → base_link` を publish
  - stamp を `now()` で上書き (bag 過去時刻 vs Ouster 現在時刻の同期のため)
  - ※ bag の `/odometry/filtered` は EKF の `world_frame:=map` により `map → base_link` を表すため使えない
- `pose_bridge_ekf.py` (ros_ws/nedo_bringup/scripts): `/odometry/filtered` (Odometry) → `/robot_pose2d` (Pose2D) 変換

### 別途対応 (未解決)

- `/cmd_vel` の多重 publish 問題 (cmd_vel_mux 出力 vs nedo_navigation 系ノード)
- launch の `sensor_pitch` (-0.78 rad) とコメント「-23度」の不一致。IMU gravity_quat 実測では y 軸周り -32.5° (-0.567 rad) 付近。要実機確認
- `sensor_z=1.85` (static_tf) と `sensor_height=1.8` (depth_clustering, points_projection) の 5 mm ズレ (統一推奨)
- ouster IMU (os_imu frame) と os_lidar frame の座標系ズレが depth_clustering の gravity_quat 計算に影響している可能性
