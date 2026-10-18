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
- オドメトリ（統合・車輪）、TF、`joint_states`、`robot_description`、バッテリー・バージョンのスタブ
- topic 名・メッセージ型・QoS は実機ブリッジと同じ
- ワールド: カチャカシェルフを置いた壁付きの部屋、または Isaac Sim のシーン（office、hospital、warehouse）
- カチャカ向けに設定済みの slam_toolbox・Nav2・RViz・teleop を入れた Docker イメージ。
  カチャカには後方の距離センサーがないので、Nav2 は後退しない設定です

シェルフのドッキングは今後対応予定です（フェーズ 2）。

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
| `robot_info/battery_state`、`robot_info/version` | スタブ | |
| `sim/ground_truth` | `nav_msgs/Odometry`（シミュレータのみ） | 20 Hz |

`/clock` と `/tf` はトップレベルに出ます。ROS 2 側では `use_sim_time:=true` を使ってください。
詳細と設計判断は [docs/design.md](docs/design.md) にあります（英語）。

### ROS 2 パッケージを実機で使う

`ros2_ws/src/kachaka_sim_bringup` は `/kachaka` の topic にしか依存しないので、実機
（`use_sim_time:=false`）でも動くはずです。ただし未確認です。

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
```

上の GIF は `scripts/record_demo.sh` で録画しています。

## ライセンス

Apache License 2.0。[LICENSE](LICENSE) と [NOTICE](NOTICE) を参照してください。
カチャカの開発元である Preferred Robotics, Inc. とは関係のない非公式プロジェクトです。

ロボットモデルは [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)（Apache-2.0）、
シェルフのモデルは [GAI-313/kachaka_shelf_description](https://github.com/GAI-313/kachaka_shelf_description)（MIT）
のものです。どちらも `scripts/fetch_assets.py` でダウンロードし、このリポジトリには含めていません。
