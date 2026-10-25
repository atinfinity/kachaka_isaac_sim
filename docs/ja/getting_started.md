# はじめに

[English](../getting_started.md) · [README](../../README.ja.md)

このページでは、リポジトリを clone したところから、シミュレータ上のカチャカで地図を作り、Nav2 でゴールまで走らせ、
公式の kachaka-api Python クライアントから命令を送るところまでを順に進めます。各手順の最後に、次へ進む前に見えているはずのものを書いてあります。

![slam_toolbox が地図を作りながら、Nav2 がシミュレータのカチャカを走らせる](../images/demo.gif)

## 1. マシンの確認

- Ubuntu 24.04、x86_64。
- Isaac Sim 6.1 が対応する NVIDIA RTX GPU とドライバ（RTX 2080 Ti、ドライバ 595 で確認）。`nvidia-smi` に GPU が出ること。
- [uv](https://docs.astral.sh/uv/)、git、Docker と Compose（`docker compose version` が `sudo` なしで動くこと）。
- 空きディスク約 40 GB。`.venv` の Isaac Sim が約 34 GB、ROS 2 のイメージが約 4 GB です。

## 2. インストール

```bash
git clone https://github.com/atinfinity/kachaka_isaac_sim.git
cd kachaka_isaac_sim

uv sync                                  # Isaac Sim 6.1 を .venv に入れる（大きなダウンロード）
uv run python scripts/fetch_assets.py    # 公式のカチャカとシェルフの記述を third_party/ に取得
uv run python scripts/build_usd.py       # 公式の URDF とメッシュからロボットの USD を作る

docker compose -f docker/compose.yaml build   # ROS 2 Jazzy のイメージ: slam_toolbox、Nav2、RViz、teleop
```

ロボットとシェルフのモデルは元のリポジトリからダウンロードするもので、このリポジトリには含めていません（[ライセンス](../../README.ja.md#ライセンス)を参照）。

## 3. シミュレータを起動する

```bash
scripts/run_sim.sh               # Isaac Sim のウィンドウあり
scripts/run_sim.sh --headless    # ウィンドウなし
```

`scripts/run_sim.sh` は [NVIDIA Isaac Sim のライセンス](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html)への同意
（`OMNI_KIT_ACCEPT_EULA=YES`）を設定します。初回はシェーダのコンパイルで数分かかり、2 回目以降は 30 秒以内に起動します。次の行が出たら準備完了です:

```
[kachaka] running: ROS namespace '/kachaka', world 'room', charger at (-2.27, 0.0, 0.0), kachaka-api on port 26400
```

既定のワールドは壁に囲まれた部屋で、中央にロボット、西の壁際に充電器、カチャカシェルフ（`S01`）が 1 台と人が 2 人います。
シミュレータは起動したままにして、以降は別のターミナルで操作します。

## 4. キーボードで動かす

```bash
docker compose -f docker/compose.yaml run --rm teleop
```

そのターミナルにフォーカスを置いて、`i` で前進、`,` で後退、`j` と `l` で旋回、`k` で停止します。
手動操作は実機のブリッジと同じく 0.3 m/s が上限で、障害物の手前ではロボットがブレーキをかけます。Ctrl+C で終了します。

## 5. 地図を作って Nav2 でゴールを送る

```bash
xhost +local:                    # ログインごとに 1 回: コンテナから RViz を開けるようにする
NAV=true docker compose -f docker/compose.yaml up slam
```

RViz が開き、LiDAR の点（赤）とロボットの周りに広がっていく地図が表示されます。ツールバーの **2D Goal Pose** を押し、
地図上でクリックしてドラッグするとゴールと向きが決まり、Nav2 が経路（青い線）を引いてそこまで走ります。
部屋全体が地図に載るまで走らせてください。表示のほかの要素は [RViz の色](../../README.ja.md#rviz-の色)の表にあります。

## 6. 地図を保存してその上で走る

`slam` を動かしたまま、3 つ目のターミナルで地図を保存します:

```bash
docker compose -f docker/compose.yaml run --rm shell \
    ros2 run nav2_map_server map_saver_cli -f /maps/map
```

`maps/map.yaml` と `maps/map.pgm` ができます。`slam` を Ctrl+C で止めてから、保存した地図で自己位置推定とナビゲーションを起動します:

```bash
docker compose -f docker/compose.yaml up navigation    # maps/name.yaml を使うときは MAP=name
```

AMCL は地図の原点、つまり地図を作ったときのスポーン位置から始まります。ロボットがほかの場所にいるときは、RViz の **2D Pose Estimate** で位置を教えてください。

## 7. Python から kachaka-api を使う

シミュレータは実機と同じく、ポート 26400 で kachaka-api の gRPC API を提供します。`slam`（`NAV=true`）か `navigation` を動かした状態で、
ホストから公式クライアントを試せます。ホスト側に ROS は要りません:

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python
```

```python
from kachaka_api import KachakaApiClient

client = KachakaApiClient("127.0.0.1:26400")
print(client.get_robot_pose())
client.update_resolver()            # 場所とシェルフの名前を読み込む
client.move_to_location("L01")      # 既定の部屋の「Kitchen」
client.move_shelf("S01", "L01")     # シェルフの下に潜り、キッチンまで運ぶ
client.return_shelf("S01")
client.return_home()                # 充電器に戻る
```

実機のカチャカ向けに書いたコードも、接続先をロボットのアドレスから `127.0.0.1:26400` に変えるだけで動きます。
ROS 2 のコードは実機のブリッジと同じ `/kachaka` 以下のトピックを使えます（[ROS 2 インターフェース](ros2_interface.md)）。

## 8. 動作確認

```bash
scripts/smoke_test.sh            # ヘッドレス: トピック、TF、走行、段差センサー、2 台のロボット
scripts/smoke_test.sh --nav      # さらに SLAM、Nav2、ドッキング、シェルフ、コマンド（時間がかかる）
```

スクリプトが自分でシミュレータを起動するので、先に手元のシミュレータを止めてください。終了コード 0 ならすべて合格です。
ほかの確認方法は[テスト](testing.md)にあります。

## 次に読むもの

- ほかのワールド: `scripts/run_sim.sh --world office`（`hospital`、`warehouse`、`kitchen_l_shape` なども）、USD ファイル、
  占有格子地図から作る壁。[シミュレータのオプション](simulator_options.md)を参照。
- 自分の部屋: シェルフ、場所、人、箱、壁、ドアをレイアウトファイルで置く。[ワールドとレイアウト](worlds.md)を参照。
- ロボットの機能: コマンド、ドッキング、シェルフ、バッテリー、トーチ、物体検出。[ロボットの機能](robot.md)を参照。
- ナビゲーションをまとめて走らせて指標をとる: [シナリオランナー](scenario_runner.md)。
- kachaka-api を使う Web ダッシュボードとデモ用ノートブック: [kachaka-api (gRPC)](kachaka_api.md)。

## うまくいかないとき

- **RViz が開かない**（`could not connect to display`）。デスクトップのセッションで `xhost +local:` を実行し、
  コンテナを起動するターミナルで `DISPLAY` が設定されているか確かめてください。
- **ROS 2 側からトピックが見えない。** コンテナはホストネットワークを使うので、シミュレータと同じマシンで、
  同じ `ROS_DOMAIN_ID`（既定は 0）で動かす必要があります。
  `docker compose -f docker/compose.yaml run --rm shell ros2 topic list` で確かめられます。
- **自作の ROS 2 ノードでカメラ画像が欠ける、届かない。** Fast DDS のソケットバッファを大きくする必要があります:
  `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304`（コンテナでは設定済み）。[性能](performance.md)を参照。
- **人が灰色のカプセルになる。** 人のキャラクターは NVIDIA のアセットサーバーから読み込みます。
  ネットワークにつながらないとマネキンになり、シミュレータが `Isaac Sim asset server not reachable` と出力します。
- **実時間より遅い。** LiDAR とオドメトリだけでよければ `--no-cameras`、または `--render-hz` を下げてください。[性能](performance.md)を参照。
