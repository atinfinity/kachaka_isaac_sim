# 複数ロボット

[English](../multi_robot.md) · [README](../../README.ja.md)

1 つのシミュレータで、同じワールドに複数台のカチャカを動かせます。ロボットごとに ROS 2 の名前空間、TF フレーム、kachaka-api のポートが分かれ、
map 座標系は全台で共通です。`fleet.launch.py` は保存した地図の上で全台の Nav2 を起動します。各ロボットは LiDAR でほかのロボットを見て、
コストマップにもほかのロボットのフットプリントを持ちます。

![2 台のロボットが直角に交差する。RViz では、ロボット 0 のローカルコストマップにロボット 1 の輪郭（黄: 現在、青: 0.5 秒先）が出る](../images/fleet.gif)

*1 つの地図の上で 2 台の経路が交差する（`scripts/record_demo.sh fleet`）。*

## クイックスタート

既定の部屋に 2 台を置き、共通の地図でナビゲーションします。`python3` と `ros2` のコマンドは ROS 2 のコンテナ
（`docker compose -f docker/compose.yaml run --rm shell`）で実行します。

```bash
# 1. 部屋の地図。map 座標系の原点はロボット 0 のスポーン位置（maps/room.yaml）
python3 scripts/make_maps.py --spawn -1.7 0.2 0

# 2. 2 台のロボットでシミュレータを起動（ホストで）
scripts/run_sim.sh --no-cameras --no-shelves --robot -1.7 0.2 0 --robot 0.3 -1.8 1.5708

# 3. 2 台分の Nav2。ロボット 1 の初期位置を map 座標系で渡す
ros2 launch kachaka_sim_bringup fleet.launch.py map:=/maps/room.yaml robots:=2 initial_poses:=2.0,-2.0,1.5708

# 4. ロボット 1 にゴールを送る
ros2 action send_goal /kachaka_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5}, orientation: {z: 0.7071, w: 0.7071}}}}'
```

シミュレータは準備ができる前に、追加したロボットごとに 1 行出力します:

```
[kachaka] robot 1: ROS namespace '/kachaka_1', frames 'kachaka_1/...', start (2.000, -2.000, 1.571) in the map frame, kachaka-api on port 26401
[kachaka] running: ROS namespace '/kachaka', world 'room', charger at (-2.27, 0.0, 0.0), kachaka-api on port 26400
```

ここに出る `start` が `initial_poses` に渡す値です。

## ロボットの配置

`--robot X Y YAW` はロボットを 1 台、ワールド座標で追加します。何回でも指定でき、`--x`、`--y`、`--yaw` の代わりになります。
最初の `--robot` がロボット 0、次がロボット 1 です。エレベーターのあるレイアウトでは 1 台しか使えません。

## 名前

ロボット 0 は 1 台のときと同じ名前を使うので、1 台用に書いたコードはそのまま動きます。ロボット i（1、2、…）には `_i` が付きます:

| | ロボット 0 | ロボット i |
|---|---|---|
| プリム | `/World/kachaka` | `/World/kachaka_i` |
| トピック | `/kachaka/...` | `/kachaka_i/...` |
| TF フレーム | `odom`、`base_footprint`、… | `kachaka_i/odom`、`kachaka_i/base_footprint`、… |
| Nav2（`fleet.launch.py` 使用時） | `/navigate_to_pose`、`/initialpose`、… | `/kachaka_i/navigate_to_pose`、`/kachaka_i/initialpose`、… |
| kachaka-api | `--api-port`（26400） | `--api-port` + i（26401、…） |
| ノイズのシード | `--seed` | `--seed` + 1000 i |

`/clock`、`/tf`、map 座標系は全台で 1 つです。map 座標系の原点はロボット 0 のスポーン位置です。

## ナビゲーション

`fleet.launch.py map:=<yaml> robots:=N` は、ロボット 0 に `navigation.launch.py`（RViz と、ロボット 0 のコマンド・充電器・シェルフのサーバーを含む）を、
ほかのロボットには `/kachaka_i` 以下に `robot_nav.launch.py` を起動します。ロボットごとに AMCL・プランナー・コントローラーがあり、
`nav2_params.yaml` はそのロボットのフレームとトピックに書き換えて使います。

| 引数 | 既定値 | |
|---|---|---|
| `map` | | 全台で共通の地図の YAML |
| `robots` | 2 | シミュレータが動かすロボットの台数 |
| `initial_poses` | | ロボット 1、2、… の `x,y,yaw`（map 座標系）を続けて並べる |
| `rviz` | true | RViz。ロボットごとにモデルを表示する |

- **初期位置。** ロボット 0 の AMCL は地図の原点、つまりスポーン位置から始まります。`initial_poses` でほかのロボットの初期位置を渡すと、
  全台が最初から自己位置を持ち、RViz にも表示されます。渡さなかったロボットは、自分の `/kachaka_i/initialpose` に位置が来るまで待ちます。
- **地図。** フリートを起動する前に作っておきます。部屋やレイアウトなら `make_maps.py`、またはロボット 0 だけで slam_toolbox を使います。
  map 座標系の原点はロボット 0 のスポーン位置にしてください。複数台で SLAM をする launch ファイルはありません。
- **ゴール。** RViz の **2D Goal Pose** はロボット 0 に送られます。ほかのロボットには、クイックスタートのようにそれぞれの `navigate_to_pose` アクションで、
  または自作のノードから送ります。
- **ロボット同士。** LiDAR に映るのは、ほかのロボットの幅 24 cm のうち 9 cm だけです（車体から突き出たレーザーのハウジング。それ以外は走査面より低い）。
  そこで `fleet_outlines` が、各ロボットの Nav2 のフットプリントを AMCL の位置に置いて、ほかのロボットのコストマップとコリジョンモニターに
  `{namespace}/other_robots` として渡します（RViz では黄色の輪郭、0.5 秒先は青）。
  LiDAR の高さにある車体の部分（ハウジングと後部の出っ張り。公式の車体メッシュの断面から作る）は走査と 2D で交差させるだけで、当たり判定はありません。
  そのため、シェルフの底板（0.107 m）は今までどおり車体の上を通ります。
- **交通整理はしません。** 各ロボットの Nav2 は互いに調整しません。狭い所で正面から出会った 2 台は、ゴールが中断されるまで互いをふさぐことがあります。
  譲り合いはシミュレータの外のフリートマネージャーの仕事です。

## ロボットごとの kachaka-api

各ロボットはそれぞれのポートで kachaka-api を提供します。ロボット 0 の API はすべて使えます（[kachaka-api](kachaka_api.md)を参照）。
コマンド・シェルフ・場所を扱うサーバーはロボット 0 のものなので、ロボット i の API でできることは限られます:

| ロボット i のポートで | 使えるか |
|---|---|
| 位置とバッテリー（`get_robot_pose`、`get_battery_info`） | 使える。位置は `fleet.launch.py` の起動後は共通の map 座標系、起動前はロボット i の odom 座標系 |
| 手動操作（`set_manual_control_enabled`、`set_robot_velocity`） | 使える |
| コマンド（`start_command`、`move_to_pose`、`rotate_in_place` など） | 使えない: エラー −6（サーバーが使えない） |
| 場所とシェルフ（`get_locations`、`get_shelves`） | 使えない: 呼び出しが戻らないので呼ばないこと |

```python
from kachaka_api import KachakaApiClient

robot1 = KachakaApiClient("127.0.0.1:26401")
print(robot1.get_robot_pose())
```

## ロボット 1 以降にないもの

- カメラ（前、後ろ、ToF）と物体検出。
- シェルフ: シェルフとそのドッキングはロボット 0 のものです。
- 専用の充電器: ロボット 0 の充電器を共有します。
- ロボット i の `sim/reset` はロボット i だけをスポーン位置に戻します。ロボット 0 のリセットはワールドもリセットします。

ドア、自動ドア、動く人はどのロボットにも反応します。

## テスト

| | |
|---|---|
| `tests/ros_multi_check.py` | 向かい合った 2 台: トピック・TF・オドメトリが分かれ、それぞれ自分の命令で動き、互いが見える |
| `tests/ros_multi_nav_check.py` | 1 つの地図の上の 2 台が直角に交差するように走り、触れずに着く |

`scripts/smoke_test.sh` は前者を、`smoke_test.sh --nav` は両方を実行します。期待するシミュレータの起動コマンドは各ファイルの docstring にあります。
