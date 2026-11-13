# pickup_approach_controller（経路追従ノード）

経路計画ノード（`pickup_path_planner`）が出した `nav_msgs/Path` を、融合自己位置で空間的に追従し、
車体座標の速度指令 (vx, vy, ω) を出します。経路は作りません。到着判定も行います。
全方向移動（x, y, yaw を独立に制御）の車体を前提とします。

## 入出力

| 方向 | トピック | 型 | 内容 |
|---|---|---|---|
| Sub | `/odometry/filtered` | `nav_msgs/Odometry` | 自己位置（共分散・速度を使用） |
| Sub | `/pickup_approach/goal` | `geometry_msgs/PoseStamped` | マスターが権限取得後に中継する目標（到着判定の基準） |
| Sub | `/pickup_approach/path` | `nav_msgs/Path` | 計画経路（stampは目標と一致が必要） |
| Sub | `/scrap_pile_skirt_node/too_close` | `std_msgs/Bool` | 山への近接（立つと即停止） |
| Sub | `/scrap_pile_skirt_node/enabled` | `std_msgs/Bool` | 山監視の有効状態（無効の間は走行しない） |
| Pub | `/pickup_approach/cmd_vel` | `geometry_msgs/Twist` | 速度指令（上位の走行権限muxが選択） |
| Pub | `/navigation/result` | `std_msgs/String` | 経路受理後の`planned`、および`arrived` / `blocked`（各1回） |
| Pub | `/pickup_approach/result`, `/status` | `std_msgs/String` | 詳細結果・状態 |

サービス: `~/set_enabled`（走行権限）, `~/cancel`。起動時の走行権限はOFFです。ノード単体の既定値は `publish_motion_commands: false`、統合用 `controller.yaml` は `cmd_vel_mux` による排他を前提に `true` です。

## 安全動作

- 近接(`too_close`)・監視データ途絶・制御周期異常: 即時停止。
- 自己位置無効・共分散超過・時刻逆戻り・姿勢の飛び（再位置合わせ）: 滑らかに停止し、復帰後は経路へ再投影して再開（進捗を0に戻さない）。到着ラッチは維持します。
- 追従時間超過: `TIMEOUT` で停止。
- 保持状態が `blocked_hold_seconds` 続くか、安全停止・タイムアウトで `blocked` を一度だけ通知します。

## 重要な前提と注意

- 立上げ用の速度・加速度は控えめです（最大0.30 m/s など）。実機で段階的に上げてください。
- `base_link` は車体矩形の中心である前提です（計画ノード側と共通。要確認）。
- ROS部分は未ビルドです（コア `controller_core` のみ単体試験済み）。`colcon build` で確認してください。
- 山の近接停止は `approach_limit` を目標離隔(0.15 m)より小さく（0.10 m 目安）設定したときだけ、到着前に誤作動しません。
- マスターはピックアップ接近完了まで山検出を有効に保ちます。接近中に山監視が止まると、本ノードは保持→`blocked` になります。
