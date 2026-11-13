"""シャトル運転システム起動 launch.

rover_odom_to_pose2d (自己位置推定) 以外の全ノードを起動する。
移植先では自己位置推定ノードを別途起動し、/robot_pose2d と TF odom->base_link を供給すればよい。

主な引数 (ros2 launch ... arg:=value で上書き):
  ouster_params           Ouster ドライバ params ファイル
  metadata_path           depth_clustering 用メタデータ JSON
  robot_radius            manage_node 最近点除外閾値
  obstacle_avoid_radius   manage_node 回避円 r2
  path_band_half_width    manage_node 経路バンド片側幅
  lookahead_distance      manage_node Pure Pursuit 注視距離
  sensor_{x,y,z,roll,pitch,yaw}  base_link -> os_sensor 静的 TF

注意: goals (ゴール座標リスト) は配列パラメータのため launch 引数からは上書きできず、
本ファイル内の MANAGE_GOALS を直接編集すること (YAML 外出しは別タスク)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ===== ロボット固有パラメータ (移植先で編集) =====
MANAGE_GOALS = [6.84, 2.66, 7.5, 4.5]  # [x0, y0, x1, y1, ...]


def generate_launch_description():
    # ===== DeclareLaunchArgument =====
    #ouster_params_arg = DeclareLaunchArgument(
    #    'ouster_params',
    #    description='Ouster driver params file path (required)',
    #)
    # metadata_path_arg = DeclareLaunchArgument(
    #    'metadata_path',
    #    description='Ouster metadata JSON for depth_clustering (required)',
    #)

    # manage_node パラメータ
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='1.8',
        description='Robot radius (m), nearest-point exclusion threshold',
    )
    obstacle_avoid_radius_arg = DeclareLaunchArgument(
        'obstacle_avoid_radius', default_value='2.5',
        description='Obstacle avoidance circle radius r2 (m)',
    )
    path_band_half_width_arg = DeclareLaunchArgument(
        'path_band_half_width', default_value='1.0',
        description='Path band half width (m)',
    )
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance', default_value='2.5',
        description='Pure Pursuit lookahead distance (m)',
    )
    voicevox_url_arg = DeclareLaunchArgument(
        'voicevox_url', default_value='http://localhost:50021',
        description='Voicevox server URL (empty string to disable voice)',
    )

    # LiDAR 取付位置 (base_link -> os_sensor)
    sensor_x_arg = DeclareLaunchArgument('sensor_x', default_value='1.0')
    sensor_y_arg = DeclareLaunchArgument('sensor_y', default_value='0.0')
    sensor_z_arg = DeclareLaunchArgument('sensor_z', default_value='1.68')
    sensor_roll_arg = DeclareLaunchArgument('sensor_roll', default_value='0.0')
    sensor_pitch_arg = DeclareLaunchArgument('sensor_pitch', default_value='0.0')
    # os_sensor -> os_lidar が yaw=180° で固定 (Ouster 仕様) のため、
    # base_link -> os_sensor でも 180° yaw を入れて相殺し、
    # os_lidar の +X がローバ前方を向くようにする
    sensor_yaw_arg = DeclareLaunchArgument('sensor_yaw', default_value='0')

    # ===== Ouster ドライバ (外部パッケージ) =====
    #ouster_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(
    #            get_package_share_directory('ouster_ros'),
    #            'launch', 'driver.launch.py')),
    #    launch_arguments={
    #        'params_file': LaunchConfiguration('ouster_params'),
    #        'viz': 'false',
    #    }.items(),
    #)

    # ===== depth_clustering =====
    #depth_clustering = Node(
    #    package='depth_clustering_cpp',
    #    executable='depth_clustering_node',
    #    parameters=[{
    #        'use_metadata_file': True,
    #        'metadata_path': LaunchConfiguration('metadata_path'),
    #    }],
    #    output='screen',
    #    sigterm_timeout='2',
    #    sigkill_timeout='5',
    #)

    # ===== points_projection =====
    points_projection = Node(
        package='points_projection',
        executable='points_projection_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    # ===== heading_test =====
    heading_test = Node(
        package='heading_test_node',
        executable='heading_test_node',
        output='screen',
        parameters=[{'u_max': 0.3}],
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    # ===== drive_node =====
    drive = Node(
        package='drive_node',
        executable='drive_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    # ===== cmd_vel_mux =====
    cmd_vel_mux = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    # ===== manage_node =====
    manage = Node(
        package='manage_node',
        executable='manage_node',
        parameters=[{
            'goals': MANAGE_GOALS,
            'robot_radius': LaunchConfiguration('robot_radius'),
            'obstacle_avoid_radius': LaunchConfiguration('obstacle_avoid_radius'),
            'path_band_half_width': LaunchConfiguration('path_band_half_width'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'voicevox_url': LaunchConfiguration('voicevox_url'),
        }],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    # ===== 静的 TF: base_link -> os_sensor =====
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', LaunchConfiguration('sensor_x'),
            '--y', LaunchConfiguration('sensor_y'),
            '--z', LaunchConfiguration('sensor_z'),
            '--roll', LaunchConfiguration('sensor_roll'),
            '--pitch', LaunchConfiguration('sensor_pitch'),
            '--yaw', LaunchConfiguration('sensor_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor',
        ],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='5',
    )

    return LaunchDescription([
        # ouster_params_arg,
        # metadata_path_arg,
        robot_radius_arg,
        obstacle_avoid_radius_arg,
        path_band_half_width_arg,
        lookahead_distance_arg,
        voicevox_url_arg,
        sensor_x_arg,
        sensor_y_arg,
        sensor_z_arg,
        sensor_roll_arg,
        sensor_pitch_arg,
        sensor_yaw_arg,
        # ouster_launch,
        # depth_clustering,
        points_projection,
        heading_test,
        drive,
        cmd_vel_mux,
        manage,
        static_tf,
    ])
