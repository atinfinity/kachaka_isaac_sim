# テスト

[English](../testing.md) · [README](../../README.ja.md)

## シミュレータなし

以下は push と pull request のたびに CI（`.github/workflows/ci.yml`）で実行されます:

```bash
uvx ruff@0.17.1 check                                   # lint
uv run --no-project --with pytest --with numpy --with pyyaml --with pillow python -m pytest
                                                        # tests/unit: config、drive、power、IMU、URDF、STL、地図
docker run --rm kachaka-isaac-sim-ros:jazzy bash -c \
    'colcon test --packages-select kachaka_sim_bringup && colcon test-result --verbose'
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/nav2_config_check.py
python3 scripts/build_usd.py --urdf-only                # 次のテスト用の build/kachaka.urdf
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/api_fake_check.py
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/speed_limit_check.py
uv run --no-project --with pyyaml python scripts/run_scenarios.py scenarios/fake_room.yaml --backend fake
```

URDF の単体テストには先に `scripts/fetch_assets.py` が必要です。
`tests/nav2_config_check.py` はシミュレータの代わりに `fake_robot`（何もない部屋の中の点ロボット、
`ros2 run kachaka_sim_bringup fake_robot`）を使い、slam.launch.py（Nav2 付き）、続いて保存した地図で
navigation.launch.py を起動します。すべての lifecycle manager が active になり、Nav2 のゴールが成功することを確かめます。

`tests/speed_limit_check.py` は `scripts/fake_sim.py` で、シェルフあり・なしの手動と自律の最高速度を測ります
（`--external` を付けると、`--shelf 0 0 0 --no-people` で起動したシミュレータを確かめます）。

`tests/api_fake_check.py` は `scripts/fake_sim.py` に対して、シミュレータと同じく SLAM・Nav2・コマンドサーバを
起動し、`tests/api_check.py --no-cameras`（kachaka-api の回帰テスト: move_to_pose、rotate_in_place、return_home、
シェルフ、エラーコード、履歴など）を実行します。`fake_sim.py` はシミュレータの ROS ノードと gRPC サーバを
そのまま使い、Isaac Sim の代わりに既定の部屋を 2D にした世界で運動学モデルのロボットを動かします。
LiDAR は壁・箱・人・シェルフの脚を見て、充電器とシェルフも動き、ロボットはぶつかる手前で止まります。
`-e FAKE_SIM_ARGS=--trace` を付けると、ロボットの姿勢と指令を 1 秒ごとに記録します（`CHECK_LOG_DIR` の `fake_sim.log`）。
物理とカメラはないため、次はシミュレータでの確認が必要です: カメラ画像と物体検出、段差と衝突の検知、
物理での動き（車輪の滑り、衝突、シェルフを押すこと）、Isaac のシーン、保存済み地図への切り替え
（`tests/api_map_check.py`、`navigation.launch.py`）。

CI には GPU がないため、シミュレータ自体はローカルで確認します。`scripts/smoke_test.sh --nav` は
シミュレータをヘッドレスで起動して `tests/ros_smoke_check.py` を実行し、続いて SLAM + Nav2 で
ナビゲーション・充電・シェルフ・コマンド・物体検出のチェックを同じシミュレータで順に実行します（各チェックは `sim/reset` から始まります）。次に壁とドアのあるシミュレータを起動し直して `tests/ros_door_check.py`（ドアの通過、閉まったドアの迂回、開いたドアの再通過）を実行します。次に `layouts/room_auto_door.yaml` で起動し直して `tests/ros_auto_door_check.py`（コマンドを送らない自動ドアを Nav2 で通過。ロボットの後ろで閉まり、戻ってくると開く）を実行します。続いて `layouts/room_movers.yaml` で起動し直して `tests/ros_nav_check.py --no-collisions`（動く人と台車の間を、触れずに Nav2 のゴールへ）を実行し、最後に段差を開けたシミュレータを起動し直して `tests/ros_hazard_check.py` を実行します。Nav2 のチェック中の衝突は `build/smoke_collisions.log` に記録します。

## シミュレータあり

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
python3 /tests/ros_detection_check.py   # その場で旋回しながら object_detection/result を確認（同上）
python3 /tests/ros_hazard_check.py   # 段差と衝突（Nav2 不要。シミュレータは --drop 1.0 0.0 0.4 0.8 で起動）
python3 /tests/ros_degradation_check.py   # センサーの劣化（シミュレータは --config config/degraded.yaml --person 0.0 1.0 0.0 で起動）
python3 /tests/ros_door_check.py     # Nav2 とドア（シミュレータは --layout layouts/room_door.yaml で起動）
python3 /tests/ros_auto_door_check.py   # Nav2 と自動ドア（シミュレータは --layout layouts/room_auto_door.yaml で起動）
python3 /tests/ros_nav_check.py --no-collisions   # 動く人の間で Nav2 のゴール（シミュレータは --layout layouts/room_movers.yaml で起動）
```

kachaka-api のテストはホストで公式クライアントを使って実行します（準備は同上）:

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_check.py
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_aio_check.py  # asyncio 版クライアントとストリーム
```

`tests/api_map_check.py` は保存済み地図での `switch_map` と `set_robot_pose` を確かめます。手順
（SLAM の地図を保存し、ROS 2 側を `navigation.launch.py` で起動し直す）は docstring にあります。

`docs/images/` の GIF は `scripts/record_demo.sh` で録画しています（充電器は `scripts/record_demo.sh dock`、シェルフは `shelf`、前方障害物での停止は `front_stop`、JupyterLab でのデモノートブックは `notebooks sensors|control|map`。ホストに Chromium が必要）。トーチライトは `scripts/record_torch_demo.sh docs/images/torch.gif`、Web ダッシュボードは `scripts/record_dashboard_demo.py docs/images/dashboard.gif`（シミュレータ・Nav2・ダッシュボードを起動し、ホストに Chromium が必要）で、`--scenario reset` を付けると `docs/images/sim_reset.gif` を録画します。物体検出は `scripts/detection_demo.py --out docs/images/object_detection.gif`（kachaka-api のみ使用）で録画します。
