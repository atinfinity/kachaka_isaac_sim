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
- 実機ブリッジの全コマンドに対応した `kachaka_command/execute`（Nav2 ベース）: MOVE_TO_POSE、
  MOVE_TO_LOCATION、RETURN_HOME、SPEAK（テキストのみ）、カチャカシェルフのコマンド
  （DOCK_SHELF、UNDOCK_SHELF、MOVE_SHELF、RETURN_SHELF）、移動コマンド（MOVE_FORWARD、
  ROTATE_IN_PLACE など）。`goal_pose` と自動充電にも対応
- ポート 26400 で [kachaka-api](https://github.com/pf-robotics/kachaka-api) の gRPC API を全 RPC 提供。
  公式の Python クライアント（`KachakaApiClient`）が実機なしで使えます

### コマンド

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 13, move_to_pose_command_x: 1.0, move_to_pose_command_y: 0.5,
      move_to_pose_command_yaw: 1.57}, cancel_all: true}"   # MOVE_TO_POSE_COMMAND
ros2 topic pub --once /kachaka/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
```

| コマンド | 動作 |
|---|---|
| MOVE_TO_POSE | map 座標で Nav2 のゴールを送る |
| MOVE_TO_LOCATION | `layout/locations/list` の場所へ Nav2 で移動。充電器（`home`）なら RETURN_HOME |
| SPEAK | 音声なし。テキストをログと `sim/speech` に出す |
| MOVE_FORWARD、ROTATE_IN_PLACE | オドメトリで距離を進む・角度を回る。MOVE_FORWARD は前方に障害物があると止まる（−9） |
| MOVE_BY_VELOCITY_WITH_MUTED_SENSORS | 指定の速度で指定の時間走る |
| LOCK | 指定の時間その場で止まる |
| DOCK_ANY_SHELF_WITH_REGISTRATION | 場所に最も近い（0.5 m 以内の）シェルフにドッキング |
| LOCALIZE | すぐ成功する（シミュレータは自己位置を見失わない） |
| DEPART_FROM_CHARGER | 充電器から 0.3 m 離れる。充電器の上にいなければ失敗（−13） |

実機と同じく、`cancel_all: true` なら実行中のコマンドを止めます。付けなければ前のコマンドが
終わるまで待ちます。`tts_on_success` はコマンド成功後に `sim/speech` に出ます。`goal_pose` は
実機ブリッジと同じく `cancel_all` なしで MOVE_TO_POSE を送ります。

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
`robot_description` は実機と同じくロボット単体です。RViz では `shelf_markers` ノードが配信する
`/shelf_markers`（MarkerArray）でシェルフを表示します。

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
scripts/run_sim.sh --api-port 26400      # kachaka-api の gRPC ポート（既定 26400。0 で無効）
scripts/run_sim.sh --help
```

ロボットのパラメータ（速度上限、センサーの解像度と周期、ノイズ）は
[`config/robot.yaml`](config/robot.yaml) にあります。Preferred Robotics が公開していない値には
*(est.)* と書いてあります。

### Isaac Sim のシーン

Isaac Sim のシーンは NVIDIA のアセットサーバーから読み込み、初回以降はローカルのキャッシュを使います。
シーンごとに床の空いた場所に既定のスポーン位置があり、`--x`、`--y`、`--yaw` を省略するとそこに置きます
（`room`、`empty`、USD のパスでは 0）。SLAM の map 座標系の原点はスポーン位置です。

| `--world` | スポーン (x, y, yaw) | 初回読み込み | 起動（キャッシュ後） | 実時間比（カメラあり / `--no-cameras`） |
|---|---|---|---|---|
| `office` | 0, 0, 1.5708（ロビー、北向き） | 数分 | 22 秒 | 1.2 / 1.9 |
| `hospital` | 0, 0, 0（病棟の南のロビー） | 7 分 | 21 秒 | 1.3 / 2.0 |
| `warehouse` | −13, 11, 1.5708（ラックの間の通路） | — | 40 秒 | 1.2 / 1.9 |
| `simple_room` | −2.5, 0, 0（テーブルの横） | 数秒 | 15 秒 | 1.4 / 2.0 |

初回読み込みでは Isaac Sim のアセットサーバーからシーンを取得します。起動時間は、アセットがキャッシュされた状態で起動してから `[kachaka] running` が出るまでです。実時間比はシミュレーション時間 / 実時間で、`--realtime 0`、ロボット静止、RTX 2080 Ti での値です。デフォルトの `--realtime 1` では 1 で頭打ちになります。

`tests/ros_nav_check.py --world <名前>` で、各シーンのルートを SLAM と Nav2 で走らせて確認できます。シーンの注意点:

- **低い家具・細い家具。** LiDAR は床から 10.5 cm の 1 平面だけを測り、Nav2 はそこに見えるものしか知りません。
  simple_room のテーブルの脚（高さ 5 cm の板）、hospital の椅子の脚、office の机の脚は小さな点になるか、まったく映りません。
  こうした家具は避けるルートにしてください。
- **LiDAR の到達距離より長い空間。** 10 m 以内で反射のない光線は地図に残らないため、長い廊下やホールの中央は未知領域になります。
  NavFn は 8 m ほどの未知領域を越える経路を作れません。遠いゴールはテストのルートのように経由点を挟んでください。
- **ドアの段差。** office の y = 12 と y = 24 の出入口には 4〜5 mm の段差があります。ロボットは 6 mm 程度までの段差を越えられます。
  車体は剛体なので、前側が乗り上げるときの傾きの上限を後輪キャスターと床の 6 mm のすき間が決めます（車体の箱と床のすき間は 8 mm）。
  Preferred Robotics は段差の上限を公表しておらず、段差のない床を推奨しています。
- office の**ガラス扉**は開いた状態で、枠には当たり判定があります。
- **warehouse のラック**は LiDAR の高さでは脚しか映らないため、コストマップ上は壁ではなく柱の列になり、
  Nav2 が棚の下の脚の間を通ろうとすることがあります。テストは 1 本の通路（x = −13）の中だけを走ります。
  ラックの北のホール（y > 25）は通路から LiDAR が届かず、近づくまで地図に載りません。

## ROS 2 インターフェース

実機と同じく名前空間は `/kachaka` です。

| Topic | 型 | 周期 |
|---|---|---|
| `manual_control/cmd_vel` | `geometry_msgs/Twist`（購読） | |
| `goal_pose` | `geometry_msgs/PoseStamped`（購読）: MOVE_TO_POSE | |
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
| `sim/speech` | `std_msgs/String`、SPEAK と `tts_on_success` のテキスト（シミュレータのみ） | |
| `sim/command/start`、`sim/command/cancel` | `std_msgs/String`、JSON。kachaka-api サーバーから `command_server` へのコマンド（シミュレータのみ） | |
| `sim/command/state` | `std_msgs/String`、JSON。実行中のコマンドと最後の結果（シミュレータのみ、latched） | |

実機が自分で提供している機能は `kachaka_sim_bringup` の `command_server` が提供します。
`kachaka_command/execute` action、`goal_pose`、
`layout/locations/list` と `layout/shelves/list`、`auto_homing/set_enabled` サービスです。実機と一緒には起動しないでください
（`command_server:=false`）。

`/clock` と `/tf` はトップレベルに出ます。ROS 2 側では `use_sim_time:=true` を使ってください。
詳細と設計判断は [docs/design.md](docs/design.md) にあります（英語）。

### ROS 2 パッケージを実機で使う

`ros2_ws/src/kachaka_sim_bringup` は `/kachaka` の topic にしか依存しないので、実機
（`use_sim_time:=false`、`command_server:=false`）でも動くはずです。ただし未確認です。

## kachaka-api（gRPC）

シミュレータは実機と同じくポート 26400 で [kachaka-api](https://github.com/pf-robotics/kachaka-api)
の gRPC API を提供します。ホストに公式クライアントを入れて（ROS は不要）シミュレータに接続します。

```bash
pip install kachaka-api
```

```python
from kachaka_api import KachakaApiClient

client = KachakaApiClient("127.0.0.1:26400")
print(client.get_robot_pose())
client.update_resolver()
client.speak("hello")
client.move_to_location("L01")
client.move_shelf("S01", "L01")
client.return_shelf("S01")
client.return_home()
```

コマンドと場所の一覧は `command_server` が扱うので、ROS 2 側（`slam.launch.py nav:=true` または
`navigation.launch.py`）が必要です。ロボットの位置は slam_toolbox または AMCL の map 座標です。

`KachakaApi` サービス（kachaka-api 3.17.5.0）の RPC はすべて実装しています。
クライアントのメソッド名は RPC 名を snake_case にしたものです（`GetRobotPose` → `get_robot_pose`）。
実機との違いは次のとおりです。

| 分類 | RPC | シミュレータでの動作 |
|---|---|---|
| ロボット情報 | `GetRobotSerialNumber`、`GetRobotVersion`、`IsReady`、`GetError`、`GetRobotErrorCodeJson`、`GetRobotPose`、`GetBatteryInfo` | |
| センサー | `GetRosImu`、`GetRosOdometry`、`GetRosWheelOdometry`、`GetRosLaserScan`、`Get{Front,Back,Tof}CameraRos{CameraInfo,Image,CompressedImage}`、`GetStaticTransform`、`GetDynamicTransform` | |
| 地図 | `GetPngMap`、`GetMapList`、`GetCurrentMapId`、`LoadMapPreview`、`ExportMap`、`ImportMap`、`ImportImageAsMap` | [地図](#地図)を参照 |
| 地図（保存済み地図） | `SwitchMap`、`SetRobotPose` | `navigation.launch.py`（map_server と AMCL）が必要。SLAM 中は −16 で失敗。`inherit_docking_state_and_docked_shelf` はカチャカ Pro 専用（−17） |
| コマンド | `StartCommand`、`CancelCommand`、`GetCommandState`、`GetLastCommandResult`、`Proceed`、`GetHistoryList` | 履歴は直近 100 件。シミュレータを止めると消える |
| ショートカット | `GetShortcuts`、`StartShortcutCommand` | `config/robot.yaml`（`api.shortcuts`）で定義 |
| レイアウト | `GetLocations`、`GetLocationsDigest`、`GetShelves`、`GetShelvesDigest`、`GetMovingShelfId`、`ResetShelfPose` | `ResetShelfPose` はシェルフをホームに戻す。ロボットが運搬中なら失敗 |
| 操作 | `SetManualControlEnabled`、`GetManualControlEnabled`、`SetRobotVelocity`、`SetAutoHomingEnabled`、`GetAutoHomingEnabled`、`SetSpeakerVolume`、`GetSpeakerVolume` | |
| 安全 | `SetEmergencyStop`、`RestartRobot` | 非常停止で車輪を止め、全コマンドをキャンセルし、`RestartRobot` で解除するまで新しいコマンドを拒否（−14）。シミュレータ自体は再起動しない |
| サウンド | `GetSoundList`、`PlaySound`、`StopSound`、`AddSound`、`DeleteSound` | 音声なし。サウンド名を `sim/sound` に出す。追加したサウンドはシミュレータを止めると消える |
| 何もしない | `SetFrontTorchIntensity`、`SetBackTorchIntensity`、`ActivateLaserScan`、`GetObjectDetection`、`GetObjectDetectionFeatures` | 物体検出は常に空 |

kachaka-api 3.17.5.0 の `StartCommand` のコマンドはすべて実装しています（[コマンド](#コマンド)を参照）。
DEPART_FROM_CHARGER はこのリリースより新しいので、kachaka-api のリリースに入るまでは
ROS 2 のアクション（`command_type: 23`）からのみ送れます。

シミュレータのエラーコード（実機のものと同じく `get_robot_error_code` で説明が取れます）:

| コード | 意味 |
|---|---|
| −1 … −11 | `command_server` のコマンドエラー（未対応、充電器なし、ドッキング失敗、キャンセル、ナビゲーション失敗、障害物など） |
| −12 | 手動操作が無効 |
| −13 | 充電器の上にいない |
| −14 | 非常停止中 |
| −15 | 地図のエラー（存在しない ID、壊れたファイル） |
| −16 | 保存済み地図が必要（`navigation.launch.py`） |
| −17 | カチャカ Pro 専用 |

### 地図

地図は `maps/`（コンテナでは `/maps`）に置く Nav2 形式の 2 ファイル、`<id>.yaml` と `<id>.pgm` です
（`map_saver_cli` が書く形式）。ファイル名が ID になります。yaml には `name`（`get_map_list` で表示）と
`charger_pose: [x, y, yaw]`（`switch_map` 後の既定の位置）も書けます。

- SLAM 中の現在の地図は `sim`（slam_toolbox のその場の `/map`）です。残すにはエクスポートします:
  `export_map("sim", "slam.zip")` のあと `import_map("slam.zip")` で `maps/slam.yaml` ができます。
  `sim` という ID は保存する地図には付きません。
- `navigation.launch.py` では起動時の地図が現在の地図です。`get_map_list` は `maps/` のファイルを返し、
  `switch_map` は別の地図を map_server に読み込んで初期位置を設定します。
- `export_map` は 2 ファイルの zip を書き、`import_map` はそれを読み込みます。`import_image_as_map` は
  PNG（カチャカの配色、または任意の画像: 暗いと障害物、明るいと空き、中間の灰色は未知）を受け取り、
  画像の左下を原点にします。

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
python3 /tests/ros_nav_check.py --world office   # Isaac のシーンのルート（シミュレータも同じ --world で起動）
python3 /tests/ros_dock_check.py     # 充電器、バッテリー、RETURN_HOME（同上）
python3 /tests/ros_shelf_check.py    # シェルフのドッキング、牽引、設置（同上）
python3 /tests/ros_command_check.py  # MOVE_TO_POSE、MOVE_TO_LOCATION、SPEAK、goal_pose、cancel_all（同上）
```

kachaka-api のテストはホストで公式クライアントを使って実行します（準備は同上）:

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_check.py
```

`tests/api_map_check.py` は保存済み地図での `switch_map` と `set_robot_pose` を確かめます。手順
（SLAM の地図を保存し、ROS 2 側を `navigation.launch.py` で起動し直す）は docstring にあります。

上の GIF は `scripts/record_demo.sh` で録画しています（充電器は `scripts/record_demo.sh dock`、シェルフは `shelf`）。

## ライセンス

Apache License 2.0。[LICENSE](LICENSE) と [NOTICE](NOTICE) を参照してください。
カチャカの開発元である Preferred Robotics, Inc. とは関係のない非公式プロジェクトです。

ロボットモデルは [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)（Apache-2.0）、
シェルフのモデルは [GAI-313/kachaka_shelf_description](https://github.com/GAI-313/kachaka_shelf_description)（MIT）
のものです。どちらも `scripts/fetch_assets.py` でダウンロードし、このリポジトリには含めていません。
