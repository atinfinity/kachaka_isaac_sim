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
- 車輪トルク制限とキャスター付きの差動二輪。手動の cmd_vel は実機ブリッジと同じ 0.3 m/s、自律走行はそれより速い（[速度上限](#速度上限)）
- センサー: 2D LiDAR、前後の RGB カメラ、ToF 深度カメラ、IMU
- オドメトリ（統合・車輪）、TF、`joint_states`、`robot_description`、バージョンのスタブ
- 充電器とバッテリー: 走行中は減り、充電器の上で充電されます
- 実機と同じ前方障害物での自動停止。teleop・Nav2・kachaka-api のどこからの指令でも、障害物の手前で
  前進を止めます。スマホアプリと同じく OFF にできます
- 前後のトーチライト。実機と同じく kachaka-api で点灯します
- 前方カメラでの物体検出（人・シェルフ・充電器）。シーンの正解データから作ります
- 段差センサー: 段差で停止して一時停止します（実機と同じエラー 21051/21052）。
  本体の衝突は回数を数えて publish します。`--drop` で床に穴を開けられます
- topic 名・メッセージ型・QoS は実機ブリッジと同じ
- ワールド: カチャカシェルフを置いた壁付きの部屋、または Isaac Sim のシーン（office、hospital、warehouse）
  レイアウトファイルで充電器・シェルフ・ロケーション・人・箱・壁・ドアを配置（ランダム配置も可）。
  人と箱は経由点を巡回・範囲内を歩き回り・現れたり消えたりもできます
- シナリオランナー: Nav2 と kachaka-api のシナリオをヘッドレスでまとめて実行し、成功・時間・走行距離・衝突・
  障害物との最小距離・リカバリー回数を CSV/JSON で出力（[シナリオランナー](#シナリオランナー)）
- カチャカ向けに設定済みの slam_toolbox・Nav2・RViz・teleop を入れた Docker イメージ。
  カチャカには後方の距離センサーがないので、Nav2 は後退しない設定です
- 実機ブリッジの全コマンドに対応した `kachaka_command/execute`（Nav2 ベース）: MOVE_TO_POSE、
  MOVE_TO_LOCATION、RETURN_HOME、SPEAK（テキストのみ）、カチャカシェルフのコマンド
  （DOCK_SHELF、UNDOCK_SHELF、MOVE_SHELF、RETURN_SHELF）、移動コマンド（MOVE_FORWARD、
  ROTATE_IN_PLACE など）。`goal_pose` と自動充電にも対応
- ポート 26400 で [kachaka-api](https://github.com/pf-robotics/kachaka-api) の gRPC API を全 RPC 提供。
  公式の Python クライアント（`KachakaApiClient`）が実機なしで使えます
- Web ダッシュボード（既定では起動しない）: 地図上のロボット・シェルフ・充電器、ロボットの状態、
  コマンド送信ボタン。kachaka-api だけを使うので実機のカチャカにもつながります

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
| MOVE_BY_VELOCITY_WITH_MUTED_SENSORS | 指定の速度で指定の時間走る。その間は前方障害物での停止を無効にする |
| LOCK | 指定の時間その場で止まる |
| DOCK_ANY_SHELF_WITH_REGISTRATION | 場所に最も近い（0.5 m 以内の）シェルフにドッキング |
| LOCALIZE | すぐ成功する（シミュレータは自己位置を見失わない） |
| DEPART_FROM_CHARGER | 充電器から 0.3 m 離れる。充電器の上にいなければ失敗（−13） |

実機と同じく、`cancel_all: true` なら実行中のコマンドを止めます。付けなければ前のコマンドが
終わるまで待ちます。`tts_on_success` はコマンド成功後に `sim/speech` に出ます。`goal_pose` は
実機ブリッジと同じく `cancel_all` なしで MOVE_TO_POSE を送ります。

### 前方障害物での停止

![ON では壁の 5 cm 手前で止まり、後退と旋回はでき、OFF では壁まで進み、ON に戻すとまた止まる様子](docs/front_stop.gif)

実機のカチャカは、すぐ前に障害物があると自動で止まります（スマホアプリで OFF にできます）。
シミュレータも、teleop・Nav2・`set_robot_velocity`・コマンドのどこから来た速度指令にも同じように効きます。

- 車体の前、幅 0.30 m の範囲の LiDAR の空き距離が `margin + v × スキャン周期 + v² / (2 × 減速度)`
  まで縮むと前進にブレーキをかけます。止まっているときは 3 cm（`front_stop.margin`）空けます。
  旋回と後退はでき、前が空けばまた進みます
- 範囲はシェルフの脚の間隔より狭いので、カチャカシェルフの下には入れます。牽引中のシェルフは見ません
- MOVE_BY_VELOCITY_WITH_MUTED_SENSORS と `mute_sensors` 付きの MOVE_FORWARD は、実行中この停止を
  無効にします。MOVE_FORWARD はこの停止で 1 秒止められると −9 で失敗します
- 初期状態と範囲は `config/robot.yaml`（`front_stop`）で設定します。アプリの設定は kachaka-api に
  ないので、実行中の切り替えはシミュレータ専用のサービス `sim/front_stop/set_enabled` で行います

```bash
ros2 service call /kachaka/sim/front_stop/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /kachaka/sim/front_stop/state   # {"enabled": false, "muted": false, "blocked": false}
```

実機の停止距離と範囲は公開されていないので、値は推定です。LiDAR の高さは 10.5 cm なので、
それより低い障害物は見えません。

### トーチライト

![暗い部屋で kachaka-api からトーチを点灯。シミュレータの画面と前後カメラの画像](docs/torch.gif)

kachaka-api には前と後ろのトーチライトがあり、`SetFrontTorchIntensity` /
`SetBackTorchIntensity`（0〜255、0 で消灯）で明るさを変えます。実機のどこに付いているかは
公開されておらず、URDF にもフレームがありません。シミュレータではカメラを照らすものと仮定し、
前後カメラの位置にカメラの向きのスポットライトを置いています。Python クライアントにはラッパーがないので、
gRPC のスタブを直接呼びます。

```python
from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2

client = KachakaApiClient("127.0.0.1:26400")
client.stub.SetFrontTorchIntensity(pb2.SetFrontTorchIntensityRequest(intensity=255))
```

- 実機には明るさを読み出す RPC がありません。シミュレータは `sim/torch`（latched の JSON
  `{"front", "back"}`）に出します
- 既定のシーンは明るいので、トーチの効果はほとんど見えません。`--dome-light 0` で
  デフォルトの部屋の環境光を消せます（Isaac Sim のシーンは独自の照明を持っています）
- 明るさ・照射角・色は推定です（`config/robot.yaml` の `torch`）

### シミュレータのリセット

シミュレータ専用のサービス `sim/reset` で、シミュレータを再起動せずに開始時の状態へ戻せます。

![シェルフを牽引中に sim/reset：ロボットとシェルフが開始時の位置に戻る様子](docs/sim_reset.gif)

```bash
ros2 service call /kachaka/sim/reset std_srvs/srv/SetBool "{data: false}"   # data: true なら SLAM の地図も消す
```

- ロボットはスポーン位置に静止状態で戻ります。牽引中のシェルフは切り離し、すべてのシェルフをホームに戻します
- オドメトリはスポーン位置へのジャンプを誤差なしの移動として扱うので、SLAM の地図と map フレームはそのまま使えます。
  `data: true` のときは slam_toolbox が地図を作り直し、オドメトリもゼロから始め直します
- バッテリーは初期値に戻り、トーチは消え、緊急停止は解除され、前方障害物停止は
  `config/robot.yaml` の設定に戻ります
- `command_server` は実行中・待機中のコマンドを結果を出さずに破棄し、コストマップを消去して、
  パラメータの設定（自動充電）に戻ります
- gRPC サーバはコマンド履歴・最後の結果・エラー・追加した音声・スピーカー音量を初期化します。
  `navigation.launch.py` のときは AMCL にロボットが開始位置に戻ったことを伝えます
- リセット後 1 秒間は `cmd_vel` を無視します。まだ指令を送っているコントローラがロボットを
  動かしてしまわないようにするためです

`tests/` の ROS チェックは最初にリセットします（`--no-reset` で省略、`--clear-map` で地図も消去）。
`python3 /tests/sim_reset.py` 単体でもリセットできます。

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

![シェルフから離れたあと、シェルフを回り込んで反対側から再びドッキングし、元の場所まで牽引する様子](docs/shelf_return.gif)

*続けて RETURN_SHELF：置いたシェルフはコストマップに全体が載る（LiDAR に映るのは脚だけ）ので、Nav2 はそれを避けて経路を引く。*

部屋にはカチャカシェルフが 1 台（`S01`、ホーム `S01_home`）と、ほかの場所が 1 つ（`L01`）あります。
ロボットは前進でシェルフの下に入り、ピンを上げてキャスター付きのシェルフを牽引します
（最高 0.5 m/s、[速度上限](#速度上限)）。
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

### 速度上限

| | 単体 | シェルフ牽引中 |
|---|---|---|
| 手動: `manual_control/cmd_vel`、`set_robot_velocity` | 0.3 m/s | 0.3 m/s *（仮定）* |
| 自律: Nav2、kachaka-api のコマンド（`sim/autonomous/cmd_vel`） | 0.8 m/s *（推定）* | 0.5 m/s *（推定）* |
| 旋回（両方） | 1.57 rad/s | 1.57 rad/s（Nav2: 0.98 rad/s） |

![1.0 m/s の指令に対し、シェルフ牽引中は manual_control/cmd_vel で 0.30 m/s、sim/autonomous/cmd_vel で 0.50 m/s、単体では 0.30 m/s と 0.80 m/s で頭打ちになる様子](docs/speed.gif)

実機のブリッジは手動の cmd_vel を 0.3 m/s・1.57 rad/s にスケールします。カチャカの仕様の最高速度は
単体 0.8 m/s、シェルフ付き 0.5 m/s で、実機で測るまで（#4）シミュレータは自律走行にこの値を使います。
実機が牽引中に手動の上限も下げるかは分かっていません（牽引時の 0.5 m/s はどのみち 0.3 m/s より上です）。

自律走行の指令はシミュレータ専用の topic `sim/autonomous/cmd_vel` で届きます。`nav_bridge` は
`use_sim_time` が true のとき Nav2 の `/cmd_vel` をここへ転送し（実機では `manual_control/cmd_vel`）、
`command_server` は MOVE_FORWARD や ROTATE_IN_PLACE などをこの topic で動かします。
`manual_control/cmd_vel` に出した指令は誰からでも 0.3 m/s のままです。シェルフをドックしている間は
`command_server` が Nav2 のコントローラに 0.5 m/s の `speed_limit` を送り（MPPI は旋回速度も同じ比率で下げ、
1.57 → 0.98 rad/s）、アンドックで解除します。上限は `config/robot.yaml` の `drive.max_linear_vel`、
`drive.max_linear_vel_auto`、`drive.max_linear_vel_towing` で、`tests/speed_limit_check.py` が測ります。

### 物体検出

![その場で旋回しながら、前方カメラに映る人・シェルフ・充電器の枠、ラベル、スコア、距離を表示](docs/object_detection.gif)

*`scripts/detection_demo.py`: その場で 1 回転し、`GetObjectDetection` と前方カメラの画像から描画。*

実機は前方カメラの画像を検出ネットワークにかけます。シミュレータは代わりにレンダラーから物体を取ります。
各ピクセルのインスタンス ID と距離から、どの人・シェルフ・充電器が映っているかがわかります。
物体は見えているピクセルだけから検出するので、人の手前に箱があれば、実際の検出器と同じく
人の枠はその分だけ小さくなります。

- `roi`: 見えているピクセルを囲む枠。前方カメラの画素単位（640×480）で、±2 px のノイズ付き
- `distance_median`: 見えているピクセルの距離の中央値
- `score`: 見えている面積が大きいほど高い（150 px で 0.5、20000 px 以上で 0.95）。少しノイズ付き
- 5 m より遠いものと 150 px より小さいものは検出しません。各フレームで 2 % の物体を見逃します
- 結果は前方カメラの画像ごと（10 Hz）に出て、画像と同じタイムスタンプが付きます

実機の検出距離・スコア・見逃し率は公開されていないので、これらの値は推定です
（`config/robot.yaml` の `object_detection`）。ドアは検出しません。

`GetObjectDetectionFeatures` は前方カメラの画像ごとに、実機の検出器と同じ形の特徴マップを返します。
[kachaka-transfer-learning](https://github.com/pf-robotics/kachaka-transfer-learning)
で学習する `predictor.onnx` の入力で、`p4_out`（1×96×14×25）、`p5_out`（1×96×7×13）、
`p6_out`（1×96×4×7）です。実機のネットワークは公開されていないので、別のネットワークで作ります。
画像を 224×398 に縮小して torchvision の ImageNet 学習済み MobileNetV3-Large に通し、
ストライド 16 と 32 のブロックを固定の乱数 1×1 畳み込みで 96 チャンネルにし、`p6_out` は
ストライド 2 の max pool です（`kachaka_isaac_sim/detection_features.py`）。名前はデモノートブックが
`_out` を外して ONNX の入力名にしていることからの推定です。シミュレータの特徴量で学習したヘッドは
シミュレータでは動きますが実機では動きません（逆も同じ）。初回の呼び出しで重み（22 MB）を
torch hub のキャッシュにダウンロードします。`config/robot.yaml` の `api.detection_features: false`
で特徴量を止められます（以前と同じく空）。

部屋には人が 2 人います（`config/robot.yaml` の `people.room`）。Isaac Sim 同梱のキャラクター
（`people.characters`、腕を下ろして立ったまま）です。アセットサーバーに届かないときはカプセルの人形に
なります。`--person X Y YAW` で人を置き、`--no-people` でなくせます。Isaac Sim のシーンには既定では人はいません。

LiDAR と Nav2 には人のコライダーが見えます（[`legs.py`](kachaka_isaac_sim/legs.py)）。太もも・すねごとの
カプセル（太さ 0.13 m、推定）と、0.75 m より上の上半身を覆う見えない箱（0.28 × 0.5 m）です。LiDAR は床から
0.105 m の高さを走査するので、人は 0.2 m 離れた 2 つの小さな弧（すね）として映ります。歩いている人の脚は
歩容に合わせて振れ、弧が前後に入れ替わり、振り出し中に浮いた脚は一瞬スキャンから消えます。大きさは
`people.height` に比例します。

![歩く 3 人のキャラクターと人形。赤は脚のコライダー、下は LiDAR スキャン：1 人につき 2 つの弧が前後に入れ替わる](docs/person_legs.gif)

結果は `GetObjectDetection` と、実機ブリッジと同じ ROS 2 の topic
`/kachaka/object_detection/result`（`kachaka_interfaces/ObjectDetectionListStamped`）に出ます。
シミュレータは `sim/object_detection` に JSON を出し、`bridge.launch.py` の
`object_detection_relay` がそれを変換します。

### 段差と衝突

実機のカチャカは前方に段差センサーを 2 つ持っています。床が見えなくなると停止してコマンドを
キャンセルし、電源ボタンが押されるまでエラー **21052「段差を検知」** と **21051「一時停止中」**
で一時停止します。シミュレータも同じように動きます。

- 前キャスターのすぐ前から下向きのレイを 2 本飛ばし、物理ステップごと（120 Hz）に床を確認します
  （位置と検出距離は推定。`config/robot.yaml` の `cliff`）。
- 新しく段差を見つけると車輪をすぐ止め、全コマンドをキャンセルし、速度指令を無視します。
  `GetError` は `[21051, 21052]` を返し、新しいコマンドは 21051 で失敗します。
- 電源ボタンは `sim/power_button`（`std_srvs/Trigger`）です。kachaka-api の `RestartRobot` と
  `sim/reset` でも復帰します。センサーがまだ段差の上にある間は前進だけを止め、後退と旋回はできます。

![段差の手前で 21051/21052 を出して一時停止し、電源ボタンで再開、後退した後、前方停止を切って北の壁に衝突（sim/collision に 1 件）](docs/cliff.gif)

標準のワールドは平らです。`--drop X Y SX SY` で (X, Y) を中心に SX × SY m の穴を床に開けます
（深さ 0.15 m、`drop.depth`）。複数指定できます。

```bash
scripts/run_sim.sh --drop 1.0 0.0 0.4 0.8   # スポーン位置の 0.8 m 東に段差
ros2 topic echo /kachaka/sim/cliff          # {"enabled": true, "paused": true, "over": [true, true]}
ros2 topic echo /kachaka/sim/errors         # [21051, 21052]
ros2 service call /kachaka/sim/power_button std_srvs/srv/Trigger
```

実機のバンパーは公表されていないため、既定では衝突しても止まりません。本体（車輪・キャスター・床を除く）
の PhysX 接触レポートを数え、衝突ごとに `sim/collision` へ `{"stamp", "count", "with", "pose"}`
を出します（シナリオの評価指標などに）。同じ物体との接触は、0.5 秒離れてから再び数えます。
`collision.stop: true` にすると段差と同じように停止し、電源ボタンまでシミュレータのエラー −18 を
出します（推定の挙動）。`sim/reset` で回数は 0 に戻ります。

### ドア

`--door X Y YAW` で (X, Y) を中心とする出入口にドアを置きます。出入口は YAW の向き（壁に沿って x）です。
出入口の両側の壁は別に `--wall X0 Y0 X1 Y1` で足します（どちらも複数可）。ドアはキネマティックな板で、
LiDAR に映り、閉じている間は Nav2 がコストマップに載せて迂回します。進路にあるものは押しのけます
（安全センサーで止まりません）。

![壁で部屋を仕切り、開いたドアを東へ通過。戻るときにドアが閉まり、Nav2 は北端のすき間を回る。ドアが再び開くとそこを通る](docs/door.gif)

- `--door-kind sliding|hinged`（`door.kind`）: 引き戸は壁に沿って動き、開き戸は出入口の −x 側を軸に 90° 回ります。
- `--door-width`（0.8 m）、`--door-closed`（閉じた状態で開始）、`--door-cycle OPEN_S CLOSED_S`
  （最初のコマンドまで一定周期で開閉）。速さ・厚さ・高さは `config/robot.yaml` の `door` にあります。
- ドアは指定順に D01、D02、... と名付けます。`sim/doors/<id>/set_open`（`std_srvs/SetBool`）で開閉し、
  `sim/doors`（latched の JSON）に各ドアの `id`、`kind`、`pose`（map 座標系）、`width`、
  `state`（open、closed、opening、closing）、`opening`（0〜1）、`cycling`、`panel`（パネル中心の
  map 座標系の `pose` と `size` [長さ, 厚さ, 高さ]）を出します。状態が変わったときと、パネルが
  動いている間は 0.1 秒ごとに配信します。`sim/reset` でドアは開始時の状態に戻ります。
- RViz では `door_markers` ノードが `sim/doors` から配信する `/door_markers`（MarkerArray）で
  パネルを表示します。閉は紫、開閉中は黄色、開は薄い灰色で、id と状態付きです（[RViz の色](#rviz-の色)）。

**自動ドア**（#63）は、オフィス・店舗・病院によくあるように自分で開きます。レイアウトのドアに
`auto: true` を付けるか、`--door` のドアに `--door-auto` を付けます。自動ドアは引き戸で、出入口の両側に
センサ範囲があります。範囲は壁に沿って `width + 2 * margin`、両側にそれぞれ奥行き `range` の長方形です。
範囲内にロボット（真値）または動く人・箱（[動く人と箱](#動く人と箱)）がいる間は開き、範囲が空いてから
`hold` 秒後に閉まります。閉まる途中で何かが入ると開き直します。

![壁で部屋を仕切り、通り道は自動ドアだけ。ロボットが東へ通り抜けると後ろでドアが閉まる。閉じたドアの前まで行くと近づくにつれてドアが開き、西へ通って戻る](docs/auto_door.gif)

- 設定は `config/robot.yaml` の `door.auto` です。`range` 1.0 m、`margin` 0.3 m、`hold` 2.0 秒
  （実際のドアセンサ（マイクロ波・赤外線）は製品ごとに違うため、いずれも仮定値）と `movers`
  （true なら動く人や箱でも開く）。レイアウトのドアごとに上書きできます:
  `auto: {range: 1.5, hold: 3.0, movers: false}`。
- 自動ドアは開き戸にできず、`cycle` も付けられません。
- `sim/doors/<id>/set_open` で強制的に開閉できます。その後は `sim/reset` までセンサを無視します。
  `sim/doors` には `auto`（センサがドアを動かしている間 true）と、自動ドアなら `sensor`
  （出入口を中心とする範囲の `size` [壁に沿って, 壁を横切って]）が入ります。
- RViz ではセンサ範囲を床の薄いオレンジの長方形で表示し、ドアのラベルに「(auto)」を付けます。
- Nav2 には閉じた自動ドアも他のドアと同じ障害物に見えるので、離れた所からはそこを通る経路を作れません。
  通り道がその出入口だけなら、ドアが開くほど近づくまで向こう側のゴールは失敗することがあります（未検証）。
  `tests/ros_auto_door_check.py` のように、まずドアの前まで行かせてください。

```bash
# x = 0.6 の壁、y = -0.6 にドア、北端にすき間（layouts/room_door.yaml）
scripts/run_sim.sh --no-shelves --no-people --wall 0.6 -2.0 0.6 -1.0 --wall 0.6 -0.2 0.6 1.2 --door 0.6 -0.6 1.5708
scripts/run_sim.sh --layout layouts/room_door.yaml   # 同じ
ros2 service call /kachaka/sim/doors/D01/set_open std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /kachaka/sim/doors   # {"doors": [{"id": "D01", "state": "closing", ...}]}
# 同じ壁ですき間なし、自動ドア（layouts/room_auto_door.yaml）
scripts/run_sim.sh --layout layouts/room_auto_door.yaml
```

### エレベーター

レイアウトに `floors` と `elevator` があると、部屋をフロアごとに x 方向へ 10 m ずつずらして並べ
（フロア k の座標は 1 階の座標 + k × 10 m）、どのフロアにも壁の出入口の奥に同じかごを置きます。
かごにはフロアごとに引き戸があります。`sim/elevator/go`（`std_msgs/String`、フロア名）でかごを呼ぶと、
ドアが閉まり、1 フロアあたり `ride` 秒待ってから、かごの中のもの（ロボット、シェルフ）を行き先フロアの
かごへ移し、そこでドアが開きます。オドメトリはこの移動を見ません。ロボットが出入口にいる間は
ドアセンサーのように開き直します。`sim/elevator`（latched の JSON）に `floors`、`floor`（かごの階）、
`target`、`state`（open、closing、riding、opening）、`door`、`robot_floor`、`robot_in_car` を出します。
`sim/reset` でかごは開始階に戻ります。フロアごとに床の色を変え、床にフロア名を描いています
（見た目だけで、センサーには映りません）。乗ったことがビューポートで分かります。

![layouts/elevator.yaml: 1 階で Nav2 がかごに入り、2 階へ移動、2 階の地図を読み込んで 2 階の壁の先へ出る](docs/elevator.gif)

```bash
python3 scripts/make_maps.py layouts/elevator.yaml   # maps/elevator_1F.yaml, maps/elevator_2F.yaml
scripts/run_sim.sh --layout layouts/elevator.yaml
ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/elevator_1F.yaml
ros2 topic pub --once /kachaka/sim/elevator/go std_msgs/msg/String "{data: 2F}"
ros2 topic echo /kachaka/sim/elevator   # {"floor": "2F", "state": "open", "robot_floor": "2F", ...}
```

フロアごとに別の保存地図で走ります。`scripts/make_maps.py` がレイアウトから地図を描きます（壁、かご、
動かない箱。ドアは開いた状態）。各フロアの map 座標系は開始位置をそのフロアへずらしたものなので、
かごの中の位置はどのフロアの地図でも同じ座標です。乗った後は次のフロアの地図を読み込みます。
そのとき乗る前の map 上の位置（TF の `map` → `base_footprint`。`/amcl_pose` は 0.25 m か 0.2 rad
動いたときしか出ないので遅れることがあります）を一緒に渡します（kachaka-api の `SwitchMap` の `initial_pose`、
または `sim/map/load` の `"pose"`）。command_server は地図を読み込み、AMCL にその位置を与え、
向きまで合ったのを確かめてから応答します（AMCL は新しい地図で初期化し直すので、別に送った
`/initialpose` は失われることがあります）。`tests/ros_elevator_check.py` は 1 階でかごに入り、2 階へ移動し、
地図を切り替えて 2 階のゴールまで走ります。

実機がどう乗るかは建物側の仕組みによります。三菱電機ビルソリューションズと Preferred Robotics の実証では、
カチャカが建物のプラットフォーム（Ville-feuille）に依頼し、Ville-feuille がかごを呼んで行き先階を登録し、
乗るかごと乗り降りのタイミングをカチャカに伝えます
（[プレスリリース、2024-09-28](https://www.excite.co.jp/news/article/Prtimes_2024-09-28-115855-30/)）。
戸田建設の TODA BUILDING では、同社が公開 API で作ったシステムでカチャカプロがエレベーターを使い
フロア間で郵便物を運んでいます（[LOGISTICS TODAY、2026-07-17](https://www.logi-today.com/978771)）。
kachaka-api にはエレベーターの呼び出しがないため `sim/elevator/go` はシミュレータ専用で、かごの大きさ
（1.4 × 1.4 m）、ドア（0.9 m）、時間は推定です（`config/robot.yaml` の `elevator`）。

#### フロアを縦に積む

`elevator` に `stack: vertical` を書くと、フロアを 3 m（推定）ずつ縦に積み、かごの床を台にして
ロボット（とシェルフ）を昇降路に沿って運びます。テレポートはしません。レイアウトの書き方は同じ
（フロア k は x + k × 10 m）なので同じ保存地図が使え、各フロアの map 座標系は同じ x, y の開始位置です。
`sim/elevator` には `stack` と `robot_z`（1 階からの高さ）が加わります。上の階にはドアと動く人・箱を
置けません。かごの向きは 90° の倍数にします。

![layouts/elevator_vertical.yaml: 同じ走行をフロアを縦に積んで。カメラはロボットを追って 2 階の床を抜けて上がる](docs/elevator_vertical.gif)

```bash
python3 scripts/make_maps.py layouts/elevator.yaml
scripts/run_sim.sh --layout layouts/elevator_vertical.yaml
ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/elevator_1F.yaml
```

### レイアウトファイル

`--layout FILE` で YAML ファイルを読み、ロボットの開始位置と、充電器・シェルフ・
ロケーション・人・箱・壁・ドア・段差の配置をワールドに重ねます。どのワールドとも
組み合わせられます。例は [`layouts/`](layouts) にあります: `room.yaml`、
`room_door.yaml` と `room_auto_door.yaml`（上のドアの構成）、`room_movers.yaml`（[動く人と箱](#動く人と箱)）、Isaac Sim のシーン用の `office.yaml`、
`hospital.yaml`、`warehouse.yaml`、`simple_room.yaml`。

![layouts/room.yaml を 4 つのシードで起動。固定の物体はそのままで、Shelf 2・人・緑の箱がそれぞれの範囲内で引かれる](docs/layout.gif)

```yaml
world: room                 # --world が優先
seed: 7                     # 下の範囲指定用
clearance: 0.15             # ランダム配置の物体どうしの間隔 [m]（既定 0.1）
spawn: [0.0, 0.0, 0.0]      # x, y, yaw (m, rad)
charger: [-2.27, 0.0, 0.0]  # null なら充電器なし
shelves:
  - {id: S01, name: Shelf 1, pose: [1.3, -1.2, 0.0]}
  - {id: S02, name: Shelf 2, pose: [[0.6, 2.0], [0.3, 1.5], [-0.3, 0.3]]}  # 範囲指定
locations:
  - {id: L01, name: Kitchen, pose: [0.5, 1.0, 1.5708]}
people: [[2.0, 1.5, -2.5]]
boxes:
  - {name: crate, pose: [0.2, -1.6, 0.4], size: [0.5, 0.35, 0.45], color: [0.45, 0.55, 0.35]}
walls: [[0.6, -2.0, 0.6, -1.0]]              # x0, y0, x1, y1
doors:
  - {pose: [0.6, -0.6, 1.5708], kind: sliding, width: 0.8, open: true, cycle: [10, 5]}
  - {pose: [0.6, 1.0, 1.5708], auto: true}    # 自動ドア: ロボットが近づくと開く
drops: [[1.0, 0.0, 0.4, 0.8]]                # x, y, x 方向の大きさ, y 方向の大きさ
```

- 書かなかったキーはワールドの既定のまま（部屋には充電器、シェルフ 1 台、ロケーション 1 つ、
  人 2 人）。書いたキーは置き換えるので、`shelves: []` ならシェルフなし、`charger: null`
  なら充電器なしです。
- 姿勢のどの数値も範囲 `[lo, hi]` にでき、シードで値を引きます。ランダム配置の物体は
  他の物体（部屋ではその壁も）から `clearance` 離れるまで引き直します。`--layout-seed N`
  はファイルのシードより優先し、どちらもなければシードを選んで表示します
  （`[kachaka] layout layouts/room.yaml (seed 7)`）。同じ配置を再現できます。
- シェルフとロケーションは kachaka-api（`GetShelves`、`GetLocations`）とダッシュボードに
  渡るので、`MOVE_TO_LOCATION` やシェルフのコマンドで使えます。
- コマンドラインのオプションも効きます。`--shelf`、`--charger`、`--person`、`--no-*`、
  `--x/--y/--yaw` はレイアウトの値を置き換え、`--wall`、`--door`、`--drop` は追加します。
- 起動後、各物体がシーンと重なっていないかを調べ、壁や家具にめり込んでいれば
  `[kachaka] layout warning: ...` を表示します。

```bash
scripts/run_sim.sh --layout layouts/room.yaml --layout-seed 3
scripts/run_sim.sh --layout layouts/office.yaml   # --world office にもなる
```

### 動く人と箱

レイアウトファイルの人と箱は動かせます。動くものはキネマティックな剛体で、物理ステップごとに
動かすので、LiDAR・カメラ・ToF カメラ・物体検出にはその時の位置で見えます。レイアウトで
指定しない限り何も動きません。例は [`layouts/room_movers.yaml`](layouts/room_movers.yaml) です。

![layouts/room_movers.yaml: 2 人と台車が部屋を横切る中、Nav2 が触れずに 4 つのゴールへ着く](docs/movers.gif)

```yaml
people:
  - {pose: [0.8, -1.6, 1.5708], route: [[0.8, 1.6]], pause: 2.0}   # 往復
  - {pose: [0.0, 1.2, 0.0], wander: [[-0.1, 0.2], [0.6, 1.6]], speed: 0.5}  # x の範囲, y の範囲
boxes:
  - {name: cart, pose: [-1.6, -0.3, 0.0], size: [0.5, 0.4, 0.8],
     route: [[-0.5, -0.3]], speed: 0.3, pause: 3.0, appear: [25, 8]}
```

- `route`: 経由点を順に歩き、開始位置に戻るのを繰り返します。
  `wander`: 範囲内のランダムな点へ歩きます（レイアウトのシードで引きます）。両方は指定できません。
- `speed`（m/s。既定は `config/robot.yaml` の `movers.person_speed` 0.7 と
  `movers.box_speed` 0.4）、`pause`（経由点ごとに止まる秒数）、`appear: [いる, いない]`
  （秒。しばらくいて、いなくなる。周期のどこから始まるかはシードで引きます）。
- 次の経由点の方を向いてから歩きます。ロボットが行く手のすぐ先にいる間は待ち、ロボットの
  真上には現れません。`movers.give_way` 秒（3）待つと直前の経由点へ引き返すか、別の場所へ
  ぶらつきに行きます（人がすぐそばにいるとロボットが離れられないことがあるため）。それ以外では
  止まらないので、ロボットの側でよける必要があります。
- 人は歩くときに脚（脚のコライダーも）を、キャラクターは腕も振ります。
- `sim/reset` で開始位置に戻ります。
- 正解位置は `sim/movers`（シミュレータのみ、ground truth と同じ 20 Hz）に出ます。JSON で、
  動くものごとに `id`、`path`（USD プリム）、`kind`（`person` か `box`）、`present`、
  `pose` [x, y, yaw]（world 座標系、`sim/ground_truth` と同じ）と `map_pose`（map 座標系。
  いない間はどちらも `null`）、`velocity` [vx, vy]（world 座標系）、`radius`（占有半径）、
  `size` [x, y, z]、`blocked`（ロボット待ち）、`gap`（占有円とロボット本体の間の距離 m）、
  最後にロボットに一番近いもの `closest` が入ります。ドアは含みません（`sim/doors`、[ドア](#ドア)）。
- RViz では `mover_markers` ノードが `sim/movers` から配信する `/mover_markers`（MarkerArray）で
  表示します。人は円柱、箱は箱で、名前と速度の矢印付き、ロボット待ちの間は赤です。

```bash
scripts/run_sim.sh --layout layouts/room_movers.yaml
python3 /tests/ros_nav_check.py --no-collisions   # SLAM + Nav2 と一緒に: ゴール到達、衝突なし、最接近距離
ros2 topic echo /kachaka/sim/movers   # {"stamp": 12.3, "frame_id": "world", "movers": [...], "closest": {"id": "P01", "gap": 0.41}}
```

### シナリオランナー

`scripts/run_scenarios.py` はシナリオをヘッドレスでまとめて実行し、数値で報告します。
パラメータ変更の比較に使います。シナリオファイル（[`scenarios/`](scenarios)）には、ワールドかレイアウトファイル
（その上に spawn・shelves・people などレイアウトのキーも書けます）、ステップ（Nav2 のゴール・kachaka-api の呼び出し・待機）、
実行回数とシードを書きます:

```yaml
layout: ../layouts/room_movers.yaml
repeat: 3
steps:
  - goal: [1.6, -0.3, 3.1416]          # Nav2 NavigateToPose、ワールド座標
  - api: move_shelf                    # KachakaApiClient の任意のメソッド
    args: {shelf_name_or_id: S01, location_name_or_id: L01}
  - api: move_to_pose                  # x, y, yaw はワールド座標
    args: {x: 0.0, y: 0.0, yaw: 0.0}
  - wait: 2.0
```

```bash
python3 scripts/run_scenarios.py scenarios/room_nav.yaml scenarios/room_movers.yaml scenarios/room_api.yaml
python3 scripts/run_scenarios.py scenarios/room_movers.yaml --nav2-params my_nav2.yaml --label tuned
python3 scripts/run_scenarios.py --report build/scenarios/*/     # 過去の結果を並べて表示
```

シナリオごとにシミュレータ（シナリオのレイアウトで `run_sim.sh --headless`）と ROS イメージの SLAM + Nav2 を起動し、
実行回数だけステップを実行して（間に `sim/reset` と SLAM 地図のクリア）、両方を終了します。
`restart: true` なら実行ごとに起動し直し、シードは seed + 実行番号になります。
`--nav2-params` で別の Nav2 パラメータファイルを使います。`--backend fake` は Isaac Sim の代わりに
`scripts/fake_sim.py` を使います（GPU 不要。既定の部屋のみで人と箱はなし）。CI は `scenarios/fake_room.yaml` をこれで実行します。

実行ごとの指標: 成功（全ステップ。goal と move_to_pose はグラウンドトゥルースで 0.35 m 以内に止まることも条件）、
シミュレーション時間、走行距離（グラウンドトゥルース）、衝突（`sim/collision`、相手も記録）、
LiDAR の点と車体の外形との最小距離、Nav2 のリカバリー回数（NavigateToPose のフィードバック。kachaka-api の移動も含む）。
結果は `build/scenarios/<日時>[-label]/` に `runs.csv`、`runs.json`（ステップごとの結果つき）、`summary.csv`、
`summary.txt` とログを書きます。時間と距離は成功した実行の平均です:

```
scenario     success  time [s]     path [m]      collisions  min clear [m]  recoveries
-----------  -------  -----------  ------------  ----------  -------------  ----------
room_nav     3/3      48.8 ± 0.5   10.97 ± 0.13  0 (0 runs)  0.113          0.0
room_movers  2/3      52.8 ± 1.1   11.09 ± 0.04  0 (0 runs)  0.011          6.7
room_api     2/2      111.8 ± 2.0  15.71 ± 0.36  0 (0 runs)  0.000          1.0
```

RTX 2080 Ti のホストでの 1 回分です。room_movers の失敗した 1 回はスタート地点から動けていません。
人が前を横切る間、最初のゴールまでの経路が見つからず、リカバリー 20 回で諦めました。
シェルフの下に入るステップ（move_shelf、return_shelf）では、ドッキングするまでシェルフの脚が車体のすぐ横にあるため
最小距離はほぼ 0 になります。`runs.json` にはステップごとの最小距離とリカバリー回数も入っています。

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

### RViz の色

| 色 | 表示 | トピック |
|---|---|---|
| 赤い点 | LiDAR のスキャン | `/kachaka/lidar/scan` |
| 青い線 | Nav2 のグローバルプラン | `/plan` |
| 緑の枠 | ロボットのフットプリント | `/local_costmap/published_footprint` |
| 濃い灰色 / 青 | シェルフ / 牽引中のシェルフ | `/shelf_markers` |
| オレンジの円柱 / 茶色の箱 | 動く人 / 箱（ロボット待ちの間は赤）、緑の矢印は速度 | `/mover_markers` |
| 紫 / 黄色 / 薄い灰色のパネル | ドア 閉 / 開閉中 / 開 | `/door_markers` |
| 床の薄いオレンジの長方形 | 自動ドアのセンサ範囲 | `/door_markers` |

コストマップ（既定では非表示）は RViz の `costmap` 配色です。

## シミュレータのオプション

```bash
scripts/run_sim.sh --world office        # Isaac Sim のシーン: office, hospital, warehouse, simple_room
scripts/run_sim.sh --world /path/to.usd  # 任意の USD シーン
scripts/run_sim.sh --x 1.0 --y 0.5 --yaw 1.57
scripts/run_sim.sh --layout layouts/room.yaml   # レイアウトファイル（--layout-seed N）、レイアウトファイルの節を参照
scripts/run_sim.sh --no-cameras          # LiDAR とオドメトリだけでよいとき（速い）
scripts/run_sim.sh --dome-light 0        # 暗いシーン（環境光。既定 800）。トーチを見たいときなど
scripts/run_sim.sh --render-hz 20        # 描画・カメラのループ周期（「性能」を参照）
scripts/run_sim.sh --charger X Y YAW     # 充電器の位置（既定: 部屋の西の壁際）
scripts/run_sim.sh --no-charger
scripts/run_sim.sh --shelf X Y YAW       # シェルフを置く（複数可。デフォルトは部屋に 1 台）
scripts/run_sim.sh --no-shelves
scripts/run_sim.sh --person X Y YAW      # 人を置く（複数可。デフォルトは部屋に 2 人）
scripts/run_sim.sh --no-people
scripts/run_sim.sh --drop X Y SX SY      # 床に穴を開ける（複数可。「段差と衝突」を参照）
scripts/run_sim.sh --wall X0 Y0 X1 Y1    # 壁（複数可。「ドア」を参照）
scripts/run_sim.sh --door X Y YAW        # ドア（複数可。--door-kind、--door-width、--door-closed、--door-cycle、--door-auto）
scripts/run_sim.sh --api-port 26400      # kachaka-api の gRPC ポート（既定 26400。0 で無効）
scripts/run_sim.sh --config config/degraded.yaml   # センサーを劣化させる（「センサーの劣化」を参照）
scripts/run_sim.sh --help
```

ロボットのパラメータ（速度上限、センサーの解像度と周期、ノイズ）は
[`config/robot.yaml`](config/robot.yaml) にあります。Preferred Robotics が公開していない値には
*(est.)* と書いてあります。設定ファイルは `base` で別のファイルを指定し、変える
キーだけを書くこともできます（`config/degraded.yaml` がその例です）。

### センサーの劣化

既定ではセンサーはほぼ理想的です。`config/robot.yaml` の `degradation` で
センサーを悪くして、SLAM・AMCL・Nav2 がどこまで耐えるかを試せます。既定ではすべて
無効で、値は試験用のつまみであり、実機の測定値ではありません。

| センサー | キー | 効果 |
|---|---|---|
| LiDAR | `invisible`、`dark`、`dark_range` | 透過するプリム（ガラス）、近くでしか見えないプリム（黒い面）のパスのパターン |
| | `dropout`、`noise_per_m` | 何も返さないレイ、距離に比例して増えるノイズ |
| オドメトリ | `slip_areas` | オドメトリ（fused とホイール）が動きの `1 + slip` 倍を読む床の長方形 |
| | `distance_scale_error`、`turn_scale_error` | 全域のスケール誤差 |
| カメラ | `noise`、`exposure_jitter`、`exposure_time` | 画素ノイズ、ゆらぐ露出、旋回中の水平ブレ（前後の RGB） |
| IMU | `gyro_bias(_walk)`、`accel_bias(_walk)` | 起動時に決まり、その後ゆらぐバイアス。ジャイロ z のバイアスは姿勢も回す |

LiDAR は衝突形状を見るので、`invisible` と `dark` は材質ではなくプリムを指定します
（`["/World/Room/Glass*"]`）。オドメトリのランダムなドリフトを増やすには
`odometry.fused_noise_*` を大きくします。[`config/degraded.yaml`](config/degraded.yaml)
はほとんどを有効にします:

```bash
scripts/run_sim.sh --config config/degraded.yaml --person 0.0 1.0 0.0
python3 /tests/ros_degradation_check.py   # ROS 2 コンテナで: それぞれの効果が出ているか
```

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
| `manual_control/cmd_vel` | `geometry_msgs/Twist`（購読）、最高 0.3 m/s | |
| `goal_pose` | `geometry_msgs/PoseStamped`（購読）: MOVE_TO_POSE | |
| `odometry/odometry` | `nav_msgs/Odometry` | 20 Hz |
| `wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | 20 Hz |
| `lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `imu/imu` | `sensor_msgs/Imu` | 100 Hz |
| `front_camera/image_raw`、`back_camera/image_raw` | `sensor_msgs/Image`（rgb8、640×480）+ `camera_info` | 10 Hz |
| `tof_camera/image_raw` | `sensor_msgs/Image`（16UC1、mm）+ `camera_info` | 10 Hz |
| `object_detection/result` | `kachaka_interfaces/ObjectDetectionListStamped`、[物体検出](#物体検出)を参照（`object_detection_relay`） | 10 Hz |
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
| `sim/autonomous/cmd_vel` | `geometry_msgs/Twist`（購読）、`nav_bridge` と `command_server` からの自律走行の指令、最高 0.8 m/s（シミュレータのみ、[速度上限](#速度上限)） | |
| `sim/speech` | `std_msgs/String`、SPEAK と `tts_on_success` のテキスト（シミュレータのみ） | |
| `sim/front_stop/state` | `std_msgs/String`、前方障害物停止の状態 JSON `{"enabled", "muted", "blocked"}`（シミュレータのみ、latched、変化時） | |
| `sim/cliff` | `std_msgs/String`、段差センサーの状態 JSON `{"enabled", "paused", "over"}`（シミュレータのみ、latched、変化時） | |
| `sim/collision` | `std_msgs/String`、本体の衝突ごとの JSON `{"stamp", "count", "with", "pose"}`（シミュレータのみ） | |
| `sim/errors` | `std_msgs/String`、`GetError` が返すエラーコードの JSON リスト（シミュレータのみ、latched、変化時） | |
| `sim/power_button` | `std_srvs/Trigger` サービス: 段差や衝突による一時停止から復帰（シミュレータのみ） | |
| `sim/object_detection` | `std_msgs/String`、前方カメラの画像ごとの検出結果の JSON（シミュレータのみ） | 10 Hz |
| `sim/torch` | `std_msgs/String`、トーチの明るさ 0〜255 の JSON `{"front", "back"}`（シミュレータのみ、latched） | |
| `sim/reset` | `std_srvs/SetBool` サービス、開始時の状態に戻す。`data: true` で SLAM の地図も消す（シミュレータのみ、[シミュレータのリセット](#シミュレータのリセット)を参照） | |
| `sim/reset/event` | `std_msgs/String`、リセットのたびに JSON `{"id", "clear_map"}`（シミュレータのみ） | |
| `sim/movers` | `std_msgs/String`、JSON `{"stamp", "frame_id", "movers": [...], "closest"}`: 動く人と箱の正解位置（シミュレータのみ、20 Hz、[動く人と箱](#動く人と箱)を参照） | |
| `sim/doors` | `std_msgs/String`、JSON `{"doors": [...]}`: 各ドアの状態（シミュレータのみ、latched、変化時とパネルが動いている間は 0.1 秒ごと、[ドア](#ドア)を参照） | |
| `sim/doors/<id>/set_open` | `std_srvs/SetBool` サービス: ドアを開ける（`true`）/閉める（シミュレータのみ） | |
| `sim/front_stop/set_enabled`、`sim/front_stop/mute` | `std_srvs/SetBool` サービス。前方障害物停止の ON/OFF（アプリの設定に相当）と、コマンド実行中の一時無効化（`command_server` が使う）（シミュレータのみ） | |
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
このとき `nav_bridge` は Nav2 の指令を `manual_control/cmd_vel` に送り、実機のブリッジは 0.3 m/s で頭打ちにします。
`nav2_params.yaml` の MPPI の `vx_max` と velocity smoother の `max_velocity`（シミュレータ向けに 0.8 m/s）を合わせて下げてください。

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
| ロボット情報 | `GetRobotSerialNumber`、`GetRobotVersion`、`IsReady`、`GetError`、`GetRobotErrorCodeJson`、`GetRobotPose`、`GetBatteryInfo` | `GetError` は −14（非常停止）、21051 + 21052（段差で一時停止）、−18（`collision.stop` 時の衝突）を返す。[段差と衝突](#段差と衝突)を参照 |
| センサー | `GetRosImu`、`GetRosOdometry`、`GetRosWheelOdometry`、`GetRosLaserScan`、`Get{Front,Back,Tof}CameraRos{CameraInfo,Image,CompressedImage}`、`GetStaticTransform`、`GetDynamicTransform` | |
| 地図 | `GetPngMap`、`GetMapList`、`GetCurrentMapId`、`LoadMapPreview`、`ExportMap`、`ImportMap`、`ImportImageAsMap` | [地図](#地図)を参照 |
| 地図（保存済み地図） | `SwitchMap`、`SetRobotPose` | `navigation.launch.py`（map_server と AMCL）が必要。SLAM 中は −16 で失敗。`inherit_docking_state_and_docked_shelf` はカチャカ Pro 専用（−17） |
| コマンド | `StartCommand`、`CancelCommand`、`GetCommandState`、`GetLastCommandResult`、`Proceed`、`GetHistoryList` | 履歴は直近 100 件。シミュレータを止めると消える |
| ショートカット | `GetShortcuts`、`StartShortcutCommand` | `config/robot.yaml`（`api.shortcuts`）で定義 |
| レイアウト | `GetLocations`、`GetLocationsDigest`、`GetShelves`、`GetShelvesDigest`、`GetMovingShelfId`、`ResetShelfPose` | `ResetShelfPose` はシェルフをホームに戻す。ロボットが運搬中なら失敗 |
| 操作 | `SetManualControlEnabled`、`GetManualControlEnabled`、`SetRobotVelocity`、`SetAutoHomingEnabled`、`GetAutoHomingEnabled`、`SetSpeakerVolume`、`GetSpeakerVolume` | |
| 安全 | `SetEmergencyStop`、`RestartRobot` | 非常停止で車輪を止め、全コマンドをキャンセルし、`RestartRobot` で解除するまで新しいコマンドを拒否（−14）。`RestartRobot` は段差や衝突による一時停止からも復帰する。シミュレータ自体は再起動しない |
| サウンド | `GetSoundList`、`PlaySound`、`StopSound`、`AddSound`、`DeleteSound` | 音声なし。サウンド名を `sim/sound` に出す。追加したサウンドはシミュレータを止めると消える |
| トーチ | `SetFrontTorchIntensity`、`SetBackTorchIntensity` | 各カメラの位置のスポットライト（[トーチライト](#トーチライト)を参照）。値は 0〜255 に丸め、`sim/torch` に出します |
| 物体検出 | `GetObjectDetection`、`GetObjectDetectionFeatures` | シーンの人・シェルフ・充電器。特徴量は実機の検出器ではなく ImageNet の MobileNetV3 から（[物体検出](#物体検出)を参照） |
| 何もしない | `ActivateLaserScan` | |

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

### asyncio 版クライアント

asyncio 版のクライアント（`kachaka_api.aio.KachakaApiClient`）も使えます。クライアントはイベントループの中で作ってください。
コマンドは `wait_for_completion=False` を付けない限り完了まで待ちます。コマンドの実行中もストリームは値を返し続けます。

```python
import asyncio
from kachaka_api.aio import KachakaApiClient

async def main():
    client = KachakaApiClient("127.0.0.1:26400")
    await client.update_resolver()

    async def watch():
        async for pose in client.robot_pose.stream():
            print(pose.x, pose.y, pose.theta)

    watcher = asyncio.create_task(watch())
    print(await client.move_to_location("L01"))
    watcher.cancel()

asyncio.run(main())
```

`scripts/aio_demo.py` は、これにコマンドの状態の表示も加えた例です。

タイムスタンプ（`stamp_nsec`）は実機と同じく壁時計の時刻です（デモノートブックは `time.time()` と
比べます）。ROS トピックはシミュレーション時間（`/clock`）のままです。サーバは各スタンプを、
それ以前で最新の `/clock` の時点での壁時計とシミュレーション時間の差で変換します
（`kachaka_isaac_sim/wall_clock.py`）。同じシミュレーション時刻はどのストリームでも同じ壁時計の時刻になり、
順序も保たれます。シミュレーションが実時間より遅いときは、スタンプは壁時計より遅れ、`/clock` ごとに追いつきます。

クライアントのストリームはすべてシミュレータで動きます: `robot_pose`, `ros_odometry`, `ros_wheel_odometry`, `ros_imu`, `ros_laser_scan`, `front_camera_ros_image`, `front_camera_ros_compressed_image`, `object_detection`, `object_detection_features`, `png_map`, `command_state`, `last_command_result`, `shelves`, `moving_shelf_id`, `locations`, `map_list`, `current_map_id`, `history_list`, `auto_homing_enabled`, `manual_control_enabled`, `error`。
サーバのカーソル方式のロングポーリングに従います。

- センサーと位置のストリームは配信レートで値を返します
- `png_map` は slam_toolbox が地図を配信するたびに値を返します
- そのほかは、まず現在の値を返し、その後は値が変わったときだけ返します。
  `shelves` と `locations` が変わるのはレイアウトが変わったときだけで、シェルフを牽引しても変わりません。
  牽引中のシェルフは `moving_shelf_id` か `get_shelves()` で取得してください

`set_*_callback` の形でも使えます。kachaka-api 3.17.5.0 では `set_*_callback(None)` でコールバックは止まりますが、
その後クライアントライブラリが `TypeError: 'NoneType' object is not callable`
（"Task exception was never retrieved"）を 1 回出力します。動作に影響はありません。
また、止めた直後に別のコールバックを登録すると古いループも動き続け、コールバックが 1 つの値につき 2 回呼ばれます。
登録し直すのは次の値が届いてからにしてください。どちらもクライアント側のバグです（kachaka-api 3.18.1 でも同じ）。

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

### Web ダッシュボード

![Web ダッシュボードでシェルフをキッチンへ運び、戻し、充電器に帰る様子](docs/dashboard.gif)

地図上にロボットを表示し、コマンドを送れる小さな Web ページです。ホスト上の別プロセスで、
kachaka-api だけを使うので ROS は不要で、実機のカチャカにもつながります。既定では起動しないので、
シミュレータとは別に起動します。

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python scripts/dashboard.py
# http://127.0.0.1:8080/ を開く
# オプション: --target <ロボット>:26400（既定 127.0.0.1:26400）、他のマシンから開くなら --host 0.0.0.0、
# --port 8080
```

- **地図**: ロボットの地図（`GetPngMap`。SLAM で広がるよう 10 秒ごとに再取得）、ロボット、シェルフ
  （運搬中は濃く表示）、充電器、場所。ホイールで拡大縮小、ドラッグで移動します。kachaka-api には
  経路を返す RPC がないので、青い線は計画経路ではなく、ページが受け取った位置の軌跡です。
- **状態**: バッテリーと充電状態、位置、運搬中のシェルフ、実行中のコマンド、直前の結果、発生中の
  エラー（説明は `get_robot_error_code` から）。その下にコマンド履歴を表示します。
- **コマンド**: 場所への移動、シェルフを場所へ運ぶ、シェルフを戻す、シェルフを切り離す、充電器に戻る、
  「Goal on map」（地図上の点をクリックしてドラッグで向きを決め、`move_to_pose` で送る）、キャンセル。
  ほかの kachaka-api クライアントと同じく、Nav2 を含む ROS 2 側が必要です。

ログインはありません。ページを開ければ誰でもロボットを動かせるので、信頼できるネットワーク以外では
既定の `--host 127.0.0.1` のままにしてください。

### デモノートブック

kachaka-api のデモノートブック（[pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)
の `python/demos/*.ipynb`）はシミュレータで動きます。`scripts/demo_notebooks.sh` はこれらを
ヘッドレスで実行します。シミュレータを起動し、SLAM の地図を保存して Nav2 をその地図で起動し直し、
各ノートブックを新しいカーネルで、クライアントの接続先を `127.0.0.1:26400` にして実行します
（ノートブックの既定の接続先はロボット本体のアドレスです）。所要時間は約 10 分で、ROS 2 イメージと
uv が必要です。

```bash
scripts/demo_notebooks.sh                    # すべて
scripts/demo_notebooks.sh speak robot_pose   # 一部だけ
```

実行後のノートブックは `build/demo_notebooks/` に保存されます。終わらないループ（カメラ、スキャン、
エラーのストリーム）は一定時間で止め、何か表示されていれば合格です。HTTP サーバーのノートブックには、
動いている間にリクエストを送ります。kachaka-api 3.17.5.0 での結果:

| ノートブック | 結果 | 備考 |
|---|---|---|
| get_front_camera | OK | |
| get_front_camera_continuous | OK | |
| undistort | OK | シミュレータのカメラには歪みがないので、2 枚の画像は同じ |
| feature_matching | OK | カメラの前に何もかざさないので、一致点は少ない |
| qrcode | OK | シミュレータの部屋には QR コードがない |
| get_object_detection | OK | |
| get_imu | OK | グラフはウィジェットなので、JupyterLab 上でのみ描画される |
| get_laser_scan | OK | |
| error_code | OK | ロボットではなくシミュレータのエラーコードを表示 |
| speak | OK | 音は出ない。テキストはログと `sim/speech` に出る |
| template | OK | |
| teleop, command_gui | OK | ボタンやフォームは作られるが、ヘッドレスでは誰も押さない |
| plot_map_robot_lidar | OK | アニメーションはウィジェットなので、JupyterLab 上でのみ描画される |
| http_server | OK | `/run` が `{"success":true}` を返す |
| url_kachaka_api | OK | シリアル番号、位置、カメラ画像を HTTP で取得 |
| follow_path | OK | |
| robot_pose | OK | `set_robot_pose` で `get_robot_pose` の位置が変わる |
| switch_map | OK | 2 つ目の保存地図に切り替えて戻す |
| install_libraries | スキップ | pip install のみ |
| save_object_detection_features | OK | `p4`・`p5`・`p6` と画像を保存。特徴量は実機の検出器ではなくシミュレータのもの |
| run_custom_object_detection | OK | 公式と同じ入力を持つ代用の `predictor.onnx`（固定の枠 1 つ）で実行。下記参照 |

シミュレータで独自の検出器を動かすには、save_object_detection_features で画像と特徴量を保存し、
kachaka-transfer-learning で学習した `predictor.onnx` を run_custom_object_detection と同じディレクトリに置きます。
ノートブックは 1280×720 の画像（実機のカメラと推定）に合わせて枠を拡大します（`scale_h = 720.0 / 224`、
`scale_w = 1280.0 / 398`）。シミュレータのカメラは 640×480 なので `480.0 / 224` と `640.0 / 398`
にしてください。上の確認は、特徴量が公式の入力に合うことだけを確かめる代用モデル
（`tests/demo_notebooks_check.py` の `stand_in_predictor`）を使っています。

同じノートブックをブラウザの JupyterLab で動かした様子です（`scripts/record_demo.sh notebooks sensors|control|map|detection` で録画。録画スクリプトがボタンやプロットをクリックし、センサのストリーム中はロボットを旋回・前後進させます）。

![ロボットを旋回させながら、カメラ連続取得・物体検出・LiDAR・IMU・歪み補正・QR コード・特徴点マッチングのノートブックを実行](docs/notebooks_sensors.gif)

![teleop のボタン、コマンド GUI の MoveToPose と ReturnHome、speak、楕円に沿う follow_path、ライブ地図プロットのクリックでロボットを移動](docs/notebooks_control.gif)

![保存地図上の Nav2 を表示する RViz と、姿勢の取得・設定、地図切り替え、エラーコード表、リクエストに応答する HTTP サーバのノートブック](docs/notebooks_map.gif)

![ロボットを旋回させながら save_object_detection_features で画像と特徴量を 3 回保存し、run_custom_object_detection で代用 predictor.onnx の固定枠をカメラ映像に描画](docs/notebooks_detection.gif)

`get_png_map` が返す地図はグレースケールのため、`plot_map_robot_lidar` では matplotlib の既定のカラーマップ（黄色）で描かれます。実機の PNG 形式とは照合していません。

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

### シミュレータなし

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

### シミュレータあり

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

上の GIF は `scripts/record_demo.sh` で録画しています（充電器は `scripts/record_demo.sh dock`、シェルフは `shelf`、前方障害物での停止は `front_stop`、JupyterLab でのデモノートブックは `notebooks sensors|control|map`。ホストに Chromium が必要）。トーチライトは `scripts/record_torch_demo.sh docs/torch.gif`、Web ダッシュボードは `scripts/record_dashboard_demo.py docs/dashboard.gif`（シミュレータ・Nav2・ダッシュボードを起動し、ホストに Chromium が必要）で、`--scenario reset` を付けると `docs/sim_reset.gif` を録画します。物体検出は `scripts/detection_demo.py --out docs/object_detection.gif`（kachaka-api のみ使用）で録画します。

## ライセンス

Apache License 2.0。[LICENSE](LICENSE) と [NOTICE](NOTICE) を参照してください。
カチャカの開発元である Preferred Robotics, Inc. とは関係のない非公式プロジェクトです。

ロボットモデルは [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)（Apache-2.0）、
シェルフのモデルは [GAI-313/kachaka_shelf_description](https://github.com/GAI-313/kachaka_shelf_description)（MIT）
のものです。どちらも `scripts/fetch_assets.py` でダウンロードし、このリポジトリには含めていません。
