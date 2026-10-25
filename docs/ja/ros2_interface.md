# ROS 2 インターフェース

[English](../ros2_interface.md) · [README](../../README.ja.md)

実機と同じく名前空間は `/kachaka` です。

| Topic | 型 | 周期 |
|---|---|---|
| `manual_control/cmd_vel` | `geometry_msgs/Twist`（購読）、最高 0.3 m/s | |
| `goal_pose` | `geometry_msgs/PoseStamped`（購読）: MOVE_TO_POSE | |
| `odometry/odometry` | `nav_msgs/Odometry` | 20 Hz |
| `wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | 20 Hz |
| `lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `imu/imu` | `sensor_msgs/Imu` | 100 Hz |
| `front_camera/image_raw`、`back_camera/image_raw` | `sensor_msgs/Image`（rgb8、640×480）+ `camera_info` | 10 Hz |
| `tof_camera/image_raw` | `sensor_msgs/Image`（16UC1、mm）+ `camera_info` | 10 Hz |
| `object_detection/result` | `kachaka_interfaces/ObjectDetectionListStamped`、[物体検出](robot.md#物体検出)を参照（`object_detection_relay`） | 10 Hz |
| `joint_states`、`robot_description` | | |
| `robot_info/battery_state` | `sensor_msgs/BatteryState`（残量、充電中または放電中、電圧、電流。[バッテリー](robot.md#バッテリー)を参照） | 1 Hz |
| `robot_info/version` | スタブ | |
| `sim/ground_truth` | `nav_msgs/Odometry`（シミュレータのみ） | 20 Hz |
| `sim/charger_pose` | `geometry_msgs/PoseStamped`（シミュレータのみ、latched） | |
| `sim/charger_detection` | `geometry_msgs/PoseStamped`、1.5 m 以内のときロボットから見た充電器の位置（シミュレータのみ） | 20 Hz |
| `sim/layout` | `std_msgs/String`、シェルフと追加の場所の JSON（シミュレータのみ、latched） | |
| `sim/shelf_poses` | `geometry_msgs/PoseArray`、map 座標のシェルフ位置（シミュレータのみ） | 2 Hz |
| `sim/shelf_detection` | `geometry_msgs/PoseArray`、ロボットから見たシェルフ位置（シミュレータのみ） | 20 Hz |
| `sim/shelf_docked` | `std_msgs/String`、牽引中のシェルフ ID か空文字（シミュレータのみ、latched） | |
| `sim/shelf_dock` | `std_srvs/SetBool` サービス、ピンの上げ下げ（シミュレータのみ） | |
| `sim/autonomous/cmd_vel` | `geometry_msgs/Twist`（購読）、`nav_bridge` と `command_server` からの自律走行の指令、最高 0.8 m/s（シミュレータのみ、[速度上限](robot.md#速度上限)） | |
| `sim/speech` | `std_msgs/String`、SPEAK と `tts_on_success` のテキスト（シミュレータのみ） | |
| `sim/front_stop/state` | `std_msgs/String`、前方障害物停止の状態 JSON `{"enabled", "muted", "blocked"}`（シミュレータのみ、latched、変化時） | |
| `sim/cliff` | `std_msgs/String`、段差センサーの状態 JSON `{"enabled", "paused", "over"}`（シミュレータのみ、latched、変化時） | |
| `sim/collision` | `std_msgs/String`、本体の衝突ごとの JSON `{"stamp", "count", "with", "pose"}`（シミュレータのみ） | |
| `sim/errors` | `std_msgs/String`、`GetError` が返すエラーコードの JSON リスト（シミュレータのみ、latched、変化時） | |
| `sim/power_button` | `std_srvs/Trigger` サービス: 段差や衝突による一時停止から復帰（シミュレータのみ） | |
| `sim/object_detection` | `std_msgs/String`、前方カメラの画像ごとの検出結果の JSON（シミュレータのみ） | 10 Hz |
| `sim/torch` | `std_msgs/String`、トーチの明るさ 0〜255 の JSON `{"front", "back"}`（シミュレータのみ、latched） | |
| `sim/reset` | `std_srvs/SetBool` サービス、開始時の状態に戻す。`data: true` で SLAM の地図も消す（シミュレータのみ、[シミュレータのリセット](robot.md#シミュレータのリセット)を参照） | |
| `sim/reset/event` | `std_msgs/String`、リセットのたびに JSON `{"id", "clear_map"}`（シミュレータのみ） | |
| `sim/movers` | `std_msgs/String`、JSON `{"stamp", "frame_id", "movers": [...], "closest"}`: 動く人と箱の正解位置（シミュレータのみ、20 Hz、[動く人と箱](worlds.md#動く人と箱)を参照） | |
| `sim/doors` | `std_msgs/String`、JSON `{"doors": [...]}`: 各ドアの状態（シミュレータのみ、latched、変化時とパネルが動いている間は 0.1 秒ごと、[ドア](worlds.md#ドア)を参照） | |
| `sim/doors/<id>/set_open` | `std_srvs/SetBool` サービス: ドアを開ける（`true`）/閉める（シミュレータのみ） | |
| `sim/front_stop/set_enabled`、`sim/front_stop/mute` | `std_srvs/SetBool` サービス。前方障害物停止の ON/OFF（アプリの設定に相当）と、コマンド実行中の一時無効化（`command_server` が使う）（シミュレータのみ） | |
| `sim/command/start`、`sim/command/cancel` | `std_msgs/String`、JSON。kachaka-api サーバーから `command_server` へのコマンド（シミュレータのみ） | |
| `sim/command/state` | `std_msgs/String`、JSON。実行中のコマンドと最後の結果（シミュレータのみ、latched） | |

実機が自分で提供している機能は `kachaka_sim_bringup` の `command_server` が提供します。
`kachaka_command/execute` action、`goal_pose`、
`layout/locations/list` と `layout/shelves/list`、`auto_homing/set_enabled` サービスです。実機と一緒には起動しないでください
（`command_server:=false`）。

`/clock` と `/tf` はトップレベルに出ます。ROS 2 側では `use_sim_time:=true` を使ってください。
詳細と設計判断は [docs/design.md](../design.md) にあります（英語）。

## ROS 2 パッケージを実機で使う

`ros2_ws/src/kachaka_sim_bringup` は `/kachaka` の topic にしか依存しないので、実機
（`use_sim_time:=false`、`command_server:=false`）でも動くはずです。ただし未確認です。
このとき `nav_bridge` は Nav2 の指令を `manual_control/cmd_vel` に送り、実機のブリッジは 0.3 m/s で頭打ちにします。
`nav2_params.yaml` の MPPI の `vx_max` と velocity smoother の `max_velocity`（シミュレータ向けに 0.8 m/s）を合わせて下げてください。
