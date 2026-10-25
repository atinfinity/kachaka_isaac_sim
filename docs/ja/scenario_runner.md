# シナリオランナー

[English](../scenario_runner.md) · [README](../../README.ja.md)

`scripts/run_scenarios.py` はシナリオをヘッドレスでまとめて実行し、数値で報告します。
パラメータ変更の比較に使います。シナリオファイル（[`scenarios/`](../../scenarios)）には、ワールドかレイアウトファイル
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
