# pickup_path_planner（経路計画ノード）

山の裾点（`skirt_points`）から山を円で近似し、現在位置から `/pickup_approach/goal` までの経路を計画して
`nav_msgs/Path` として出力します。追従は `pickup_approach_controller` が担当します。

## 入出力

| 方向 | トピック | 内容 |
|---|---|---|
| Sub | `/odometry/filtered` | 自己位置（静止・共分散を確認してから計画） |
| Sub | `/pickup_approach/goal` | マスターが権限取得後に中継する目標 |
| Sub | `/scrap_pile_skirt_node/skirt_points` | 山の裾点。目標時刻付近の最後の非空点群を保持して使用 |
| Pub | `/pickup_approach/path` | 経路（header.stampは目標のstampをコピー） |
| Pub | `/navigation/result` | 既定では`blocked`（計画失敗時）のみ。`planned`は追従器が経路を受理した後に通知 |
| Pub | `/pickup_approach/planner_status`, `/plan_result` | 状態・詳細 |

サービス: `~/cancel`, `~/replan`。

`publish_navigation_planned`の既定値は`false`です。計画成功は`/pickup_approach/plan_result=planned`へ
出しますが、masterをPlanからDriveへ進める`/navigation/result=planned`は
`pickup_approach_controller`が経路を検査・受理した後にだけ出します。計画失敗時の`blocked`は、
`publish_navigation_result=true`によりこのノードからmasterへ通知します。

## 計画

直進 → 接線‐円弧‐接線（左右）→ 滑らかな極座標曲線の順に候補を作り、最短で、安全余裕
(`body_clearance` 0.15 m ＋追従誤差 0.08 m) を満たすものを選びます。山は裾点への頑健円当てはめ半径に
残差の95%値を足した保守的な円です。姿勢は経路全体で5次式により目標yawへ移します。

## 注意

- 検証するのは車体矩形のみです。**ビーム（3.0 m×0.15 m）との干渉は未検証**で、ビーム高さが不明なため、山(高さ1.5 m)との干渉確認が必要です。
- `base_link` が車体矩形の中心である前提です（要確認）。
- ROS部分は未ビルドです。コア `planner_core` は単体試験10件が通っています。
- 点群は目標時刻の0.5秒前より古いものは使いません。接近中も山検出を動かし、裾点群と近接判定を更新します。
