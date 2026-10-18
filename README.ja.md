# kachaka_isaac_sim

[English](README.md)

家庭用ロボット [カチャカ](https://kachaka.life/) のシミュレータです。
NVIDIA Isaac Sim 6.1 上で動き、公式 [kachaka-api](https://github.com/pf-robotics/kachaka-api)
の ROS 2 ブリッジと同じインターフェースを持ちます。実機向けに書いたコード（`/kachaka` 以下の topic）を
そのままシミュレータに向けて動かせます。

![slam_toolbox で地図を作りながら Nav2 でシミュレータ上のカチャカを走らせている様子](docs/demo.gif)

*左: Isaac Sim、右: slam_toolbox と Nav2 を表示した RViz（2 倍速）*

## 特徴

- 公式 URDF とメッシュから変換したロボットモデル
- 実機と同じ速度上限（0.3 m/s、1.57 rad/s）の差動二輪。車輪トルク制限とキャスター付き
- センサー: 2D LiDAR、前後の RGB カメラ、ToF 深度カメラ、IMU
- オドメトリ（統合・車輪）、TF、`joint_states`、`robot_description`、バージョンのスタブ
- 充電器とバッテリー: 走行中は減り、充電器の上で充電されます
- topic 名・メッセージ型・QoS は実機ブリッジと同じ
- ワールド: カチャカシェルフを置いた壁付きの部屋、または Isaac Sim のシーン（office、hospital、warehouse）
- カチャカ向けに設定済みの slam_toolbox・Nav2・RViz・teleop を入れた Docker イメージ。
  カチャカには後方の距離センサーがないので、Nav2 は後退しない設定です
- Nav2 のドッキングサーバーを使った `kachaka_command/execute` の RETURN_HOME、自動充電、
  カチャカシェルフのコマンド（DOCK_SHELF、UNDOCK_SHELF、MOVE_SHELF、RETURN_SHELF）

ほかのコマンドは今後対応予定です（[#2](https://github.com/atinfinity/kachaka_isaac_sim/issues/2)）。

### 充電器に戻る

![RETURN_HOME: Nav2 で充電器の前まで走り、後退して充電器に乗る様子](docs/dock.gif)

*RETURN_HOME（2 倍速）。Nav2 で充電器の前まで走り、実機と同じく後ろ向きに充電器へ乗ります。*

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 8}}"   # RETURN_HOME_COMMAND
```

Nav2 を含む ROS 2 スタック（`NAV=true ... up slam` または `up navigation`）が必要です。
`command_server` もその中で起動します。充電器から離れるには Nav2 のゴールを送るだけです。
バッテリーの増減速度と充電器の寸法は推定値です（`config/robot.yaml`）。

### シェルフの運搬

![シェルフの下に入ってドッキングし、途中で旋回しながら別の場所まで牽引し、置いて前進で抜ける様子](docs/shelf.gif)

*DOCK_SHELF のあと MOVE_SHELF で別の場所へ運び、UNDOCK_SHELF で離れる（3 倍速）。*

部屋にはカチャカシェルフが 1 台（`S01`、ホーム `S01_home`）と、ほかの場所が 1 つ（`L01`）あります。
ロボットは前進でシェルフの下に入り、ピンを上げてキャスター付きのシェルフを牽引します
（最高 0.5 m/s、`drive.max_linear_vel_towing`。Nav2 の上限 0.3 m/s のほうが低い）。
牽引中は Nav2 のフットプリントがシェルフの大きさになります。
アンドックではピンを下げて前進でシェルフから抜けます。前がふさがっているときは失敗します。

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 1, move_shelf_command_target_shelf_id: S01,
      move_shelf_command_destination_location_id: L01, move_shelf_command_undock_on_destination: true}}"   # MOVE_SHELF_COMMAND
```

| コマンド | 動作 |
|---|---|
| DOCK_SHELF | ロボットの前（1.5 m 以内）にあるシェルフとドッキング |
| UNDOCK_SHELF | 牽引中のシェルフを置いて前進で抜ける |
| MOVE_SHELF | 必要ならドッキングし、指定の場所まで運ぶ。アンドックは任意 |
| RETURN_SHELF | シェルフのホームまで運んでアンドック |
| RETURN_HOME | 牽引中ならシェルフをホームに戻してから充電器へ |

シェルフは `layout/shelves/list`、ホームは `layout/locations/list` に載ります。
`--shelf X Y YAW` でシェルフを置けます。シェルフは SLAM 上の場所に置かれるので、
ワールド上では数 cm ずれます。

## 動作環境

- Ubuntu 24.04、x86_64
- Isaac Sim 6.1 が対応する NVIDIA RTX GPU とドライバ（RTX 2080 Ti、ドライバ 595 で確認）
- [uv](https://docs.astral.sh/uv/)、Docker（Compose 付き）、git、約 40 GB の空きディスク

## クイックスタート

```bash
# 1. Isaac Sim 6.1 を .venv にインストール（ダウンロードが大きい）
uv sync

# 2. 公式のカチャカモデルを取得してロボットの USD を作る
uv run python scripts/fetch_assets.py
uv run python scripts/build_usd.py

# 3. シミュレータを起動（GUI。ウィンドウなしなら --headless）
scripts/run_sim.sh
```

`scripts/run_sim.sh` は `OMNI_KIT_ACCEPT_EULA=YES` を設定します。実行すると
[NVIDIA Isaac Sim のライセンス](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html)
に同意したことになります。初回はシェーダのコンパイルで数分かかります。

別のターミナルで、ROS 2 のツールを Docker で動かします。

```bash
docker compose -f docker/compose.yaml build
xhost +local:            # コンテナから RViz のウィンドウを開けるようにする

# キーボードで操縦
docker compose -f docker/compose.yaml run --rm teleop

# slam_toolbox で地図を作る（NAV=true で RViz から Nav2 のゴールも送れる）
NAV=true docker compose -f docker/compose.yaml up slam

# 地図を保存し、その地図でナビゲーション
docker compose -f docker/compose.yaml run --rm shell \
    ros2 run nav2_map_server map_saver_cli -f /maps/map
docker compose -f docker/compose.yaml up navigation
```

コンテナはホストネットワークを使うので、同じマシンのシミュレータを DDS のディスカバリで見つけます。

## シミュレータのオプション

```bash
scripts/run_sim.sh --world office        # Isaac Sim のシーン: office, hospital, warehouse, simple_room
scripts/run_sim.sh --world /path/to.usd  # 任意の USD シーン
scripts/run_sim.sh --x 1.0 --y 0.5 --yaw 1.57
scripts/run_sim.sh --no-cameras          # LiDAR とオドメトリだけでよいとき（速い）
scripts/run_sim.sh --render-hz 20        # 描画・カメラのループ周期（「性能」を参照）
scripts/run_sim.sh --charger X Y YAW     # 充電器の位置（既定: 部屋の西の壁際）
scripts/run_sim.sh --no-charger
scripts/run_sim.sh --shelf X Y YAW       # シェルフを置く（複数可。デフォルトは部屋に 1 台）
scripts/run_sim.sh --no-shelves
scripts/run_sim.sh --help
```

ロボットのパラメータ（速度上限、センサーの解像度と周期、ノイズ）は
[`config/robot.yaml`](config/robot.yaml) にあります。Preferred Robotics が公開していない値には
*(est.)* と書いてあります。

Isaac Sim のシーンは NVIDIA のアセットサーバーから読み込みます。office シーンの初回読み込みには数分かかります。

## ROS 2 インターフェース

実機と同じく名前空間は `/kachaka` です。

| Topic | 型 | 周期 |
|---|---|---|
| `manual_control/cmd_vel` | `geometry_msgs/Twist`（購読） | |
| `odometry/odometry` | `nav_msgs/Odometry` | 20 Hz |
| `wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | 20 Hz |
| `lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `imu/imu` | `sensor_msgs/Imu` | 100 Hz |
| `front_camera/image_raw`、`back_camera/image_raw` | `sensor_msgs/Image`（rgb8、640×480）+ `camera_info` | 10 Hz |
| `tof_camera/image_raw` | `sensor_msgs/Image`（16UC1、mm）+ `camera_info` | 10 Hz |
| `joint_states`、`robot_description` | | |
| `robot_info/battery_state` | `sensor_msgs/BatteryState`（充電中または放電中） | 1 Hz |
| `robot_info/version` | スタブ | |
| `sim/ground_truth` | `nav_msgs/Odometry`（シミュレータのみ） | 20 Hz |
| `sim/charger_pose` | `geometry_msgs/PoseStamped`（シミュレータのみ、latched） | |
| `sim/charger_detection` | `geometry_msgs/PoseStamped`、1.5 m 以内のときロボットから見た充電器の位置（シミュレータのみ） | 20 Hz |
| `sim/layout` | `std_msgs/String`、シェルフと追加の場所の JSON（シミュレータのみ、latched） | |
| `sim/shelf_poses` | `geometry_msgs/PoseArray`、map 座標のシェルフ位置（シミュレータのみ） | 2 Hz |
| `sim/shelf_detection` | `geometry_msgs/PoseArray`、ロボットから見たシェルフ位置（シミュレータのみ） | 20 Hz |
| `sim/shelf_docked` | `std_msgs/String`、牽引中のシェルフ ID か空文字（シミュレータのみ、latched） | |
| `sim/shelf_dock` | `std_srvs/SetBool` サービス、ピンの上げ下げ（シミュレータのみ） | |

実機が自分で提供している機能は `kachaka_sim_bringup` の `command_server` が提供します。
`kachaka_command/execute` action（RETURN_HOME とシェルフのコマンド。ほかは abort）、
`layout/locations/list` と `layout/shelves/list`、`auto_homing/set_enabled` サービスです。実機と一緒には起動しないでください
（`command_server:=false`）。

`/clock` と `/tf` はトップレベルに出ます。ROS 2 側では `use_sim_time:=true` を使ってください。
詳細と設計判断は [docs/design.md](docs/design.md) にあります（英語）。

### ROS 2 パッケージを実機で使う

`ros2_ws/src/kachaka_sim_bringup` は `/kachaka` の topic にしか依存しないので、実機
（`use_sim_time:=false`、`command_server:=false`）でも動くはずです。ただし未確認です。

## 性能

物理は 120 Hz で動きます。カメラに必要な描画は `--render-hz`（既定 10 Hz）で動きます。
実時間比の実測値（headless、RTX 2080 Ti、カメラ 640×480）:

| `--render-hz` | 10 | 20 | 30 |
|---|---|---|---|
| 実時間比 | 1.45 | 0.96 | 0.71 |

シミュレータは実時間に合わせて進む（`--realtime 1.0`）ので、1 を超えていれば遅れません。
画像が不要なら `--no-cameras` を使ってください。

### ROS 2 側でカメラ画像を受け取る

Fast DDS の既定のソケットバッファでは大きな画像が落ちます。購読側でバッファを大きくしてください。

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304
```

`docker/compose.yaml` では設定済みです。

## テスト

シミュレータを起動した状態で、ROS 2 コンテナ
（`docker compose -f docker/compose.yaml run --rm -v $PWD/tests:/tests shell`）から:

```bash
python3 /tests/ros_teleop_check.py   # ロボットを走らせてオドメトリを表示
python3 /tests/ros_sensor_check.py   # topic の周期、部屋の壁に対する LiDAR の距離、IMU
python3 /tests/ros_camera_check.py /tmp/images
python3 /tests/ros_nav_check.py      # Nav2 のゴール（slam.launch.py nav:=true が必要）
python3 /tests/ros_dock_check.py     # 充電器、バッテリー、RETURN_HOME（同上）
python3 /tests/ros_shelf_check.py    # シェルフのドッキング、牽引、設置（同上）
```

上の GIF は `scripts/record_demo.sh` で録画しています（充電器は `scripts/record_demo.sh dock`、シェルフは `shelf`）。

## ライセンス

Apache License 2.0。[LICENSE](LICENSE) と [NOTICE](NOTICE) を参照してください。
カチャカの開発元である Preferred Robotics, Inc. とは関係のない非公式プロジェクトです。

ロボットモデルは [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)（Apache-2.0）、
シェルフのモデルは [GAI-313/kachaka_shelf_description](https://github.com/GAI-313/kachaka_shelf_description)（MIT）
のものです。どちらも `scripts/fetch_assets.py` でダウンロードし、このリポジトリには含めていません。
