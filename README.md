# obstacle_zone_tools（ROS 2 Humble / Ubuntu 22.04）

RViz上でmapフレームの四角を移動・回転・拡縮し、ロボットの自己位置が領域内にあればNav2の`obstacle_layer.enabled`を`false`、領域外なら`true`に切り替える独立パッケージです。waypoint_toolsへの変更は不要です。

## launchの役割

| launch | 起動するもの | 役割 |
|---|---|---|
| `edit.launch.py` | 地図配信・地図の自動有効化・編集ノード・RViz | 四角を配置・編集しYAML保存 |
| `run.launch.py` | 実行ノードのみ | 保存した四角と自己位置でobstacle_layerを制御 |

## 動作

- 四角は長方形です。幅と高さを同じ値にすれば正方形になります。
- 自己位置はTFの`map → base_link`の並進成分(x, y)です。footprint全体ではなく、base_link原点の点で判定します。
- 四角を回転しても判定します。複数領域はOR判定で、すべての領域を出ると再有効化します。境界上は内側です。
- デフォルトは10 Hz判定です。TFの取得、サービス応答、costmap更新による遅延があります。
- TF取得失敗、1秒以上古いTFの場合は`true`を要求します。サービス応答がない場合は復帰の成立を保証できず、状態を未確認として表示します。
- `exit_margin: 0.05`なら、入った四角から5 cm以上出るまで無効を保ち、境界の往復切り替えを抑えます。厳密に境界で戻す場合はデフォルトの`0.0`を使います。
- 各対象ノードの現在値を約2秒ごとに確認し、別操作で値が変わった場合やNav2の再起動後も設定し直します。応答待ちの変更要求には別の変更を重ねません。応答が永久に返らない場合は再起動・手動復帰が必要です。

## 1. インストール

ZIPを展開して、`obstacle_zone_tools`ディレクトリをROS 2ワークスペースの`src`へ置きます。以下は`~/ros2_ws`を例にしています。

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-interactive-markers ros-humble-tf2-ros \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager ros-humble-rviz2 python3-yaml
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select obstacle_zone_tools
source ~/ros2_ws/install/setup.bash
```

実行する各ターミナルで、ROS 2とこのワークスペースをsourceしてください。

## 2. 四角を配置する（編集のみ）

編集用launchがmap_server・地図用lifecycle_manager・編集ノード・RVizを起動します。Nav2の走行系や自己位置推定を起動する必要はありません。自己位置TFの取得やobstacle_layerへの変更要求も行いません。

`map`には、地図画像ではなく、その画像のパス・解像度・原点を記した地図YAMLを指定します。地図YAMLとそこから参照する画像が必要です。別のmap_serverやNav2が動いている場合は、編集前に終了して/mapの二重配信を避けてください。

```bash
ros2 launch obstacle_zone_tools edit.launch.py \
  map:=/home/ros/maps/map.yaml \
  zones_file:=/home/ros/obstacle_zones.yaml
```

`/home/ros/obstacle_zones.yaml`がなければ空の状態で起動します。自分のユーザー名に合わせてパスを変更してください。

RVizは`map`をFixed Frameにし、Map・InteractiveMarkers・MarkerArrayと操作ツールを設定済みで開きます。地図が画面外の場合はMoveCameraでパン・ズームしてください。設定を手動で行う場合:

1. `Global Options → Fixed Frame`を`map`に設定する。
2. `Add → InteractiveMarkers`を追加し、`Update Topic`を`/obstacle_zones/update`にする。
3. `Add → MarkerArray`を追加し、`Topic`を`/obstacle_zones/rectangles`にする。
4. 上部ツールバーの`Publish Point`で地図の配置したい位置をクリックする。ツールがなければ`+`から`rviz_default_plugins/PublishPoint`を追加する。Topicは`/clicked_point`。
5. `Interact`ツールに戻し、青い四角をドラッグして移動する。
6. 回転用リングをドラッグして向きを変える。
7. 右上側にある黄色い小四角をドラッグして幅・高さを変更する。中心位置は固定したまま、両側を対称に拡縮する。
8. 青い四角を右クリックして`Duplicate`（複製）、`Delete`（削除）、`Save YAML`（保存）を選ぶ。

追加・移動・回転・拡縮はメモリ上の領域に即座に反映されます。自動保存は行わないので、終了前に保存してください。数値を正確に設定したい場合はYAMLを編集して再読み込みできます。

すべての四角を削除した場合も、サービスから保存できます。

```bash
ros2 service call /obstacle_zone_node/save std_srvs/srv/Trigger '{}'
ros2 service call /obstacle_zone_node/reload std_srvs/srv/Trigger '{}'
```

編集終了後はCtrl+Cで終了します。

## 3. 自己位置による切り替えを起動する

Nav2と自己位置推定を起動し、`map → odom → base_link`がつながった状態で実行します。

```bash
ros2 launch obstacle_zone_tools run.launch.py \
  zones_file:=/home/ros/obstacle_zones.yaml
```

デフォルトの切り替え対象は`/local_costmap/local_costmap`のみです。実行中の四角はRVizのMarkerArrayで表示できます。

実行用launchは保存済み領域の読み込み・自己位置判定・obstacle_layerの制御だけを行い、地図配信・RViz・Interactive Marker編集は起動しません。地図と自己位置推定は通常のロボット側構成を使用します。編集用launchを終了してから実行用launchを起動してください。

旧`zones.launch.py`、`edit_mode`、`enable_switch`は廃止しました。編集と実行はlaunchまたは実行ファイルで固定され、モード変更の設定はありません。古いパッケージディレクトリを削除してから新版を配置してください。以前のビルドがある場合はワークスペースで`rm -rf build/obstacle_zone_tools install/obstacle_zone_tools`を実行してから再ビルドしてください（通常の非merge-install構成）。新しいターミナルでsourceし直してください。領域YAMLの形式は変わりません。

ロボットの原点が`base_footprint`の場合:

```bash
ros2 launch obstacle_zone_tools run.launch.py \
  zones_file:=/home/ros/obstacle_zones.yaml robot_frame:=base_footprint
```

rosbagやシミュレーションの時刻を使う場合は`use_sim_time:=true`も付けます。

## 4. local / global両方を切り替える

`config/switch_params.yaml`を別の編集可能な場所にコピーし、パスと対象を設定します。

```yaml
obstacle_zone_node:
  ros__parameters:
    zones_file: /home/ros/obstacle_zones.yaml
    frame_id: map
    robot_frame: base_link
    target_nodes:
      - /local_costmap/local_costmap
      - /global_costmap/global_costmap
    layer_parameter: obstacle_layer.enabled
    check_rate: 10.0
    tf_max_age: 1.0
    exit_margin: 0.0
    use_sim_time: false
```

```bash
ros2 launch obstacle_zone_tools run.launch.py \
  zones_file:=/home/ros/obstacle_zones.yaml \
  params_file:=/home/ros/switch_params.yaml
```

`run.launch.py`の`zones_file`、`robot_frame`、`exit_margin`、`use_sim_time`はlaunch引数がparams_fileより優先されます。対象ノードなどはparams_fileで設定します。

両対象は別々のサービスなので同時に確定するとは限りません。globalにobstacle_layerがない場合はlocalだけ指定してください。名前空間付きのロボットでは`/robot1/local_costmap/local_costmap`など実際のノード名を使います。

Nav2の`plugins`で使っているレイヤー名が`obstacles`なら、`layer_parameter: obstacles.enabled`としてください。正しいキーは`enabled`で、`enable`ではありません。起動設定パラメータはこのノードの再起動で反映してください。領域YAMLはreloadサービスで反映できます。

## 5. YAML形式

```yaml
frame_id: map
zones:
  - name: zone_1
    x: 5.0
    y: 2.0
    yaw: 0.0
    width: 3.0
    height: 2.0
```

`x,y`は中心座標[m]、`yaw`は回転角[rad]、`width,height`は四角のローカルX/Y方向の全幅[m]です。正のyawは反時計回りです。幅・高さの最小値は0.1 mです。名前は重複不可で英数字・`_`・`-`を使用でき、`__size`で終わる名前は予約済みです。空の領域は`zones: []`です。サンプルは`config/zones_example.yaml`にあります。

## 6. 動作確認

```bash
ros2 node list
ros2 param get /local_costmap/local_costmap obstacle_layer.enabled
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /obstacle_zones/status
```

`status`の`inside_zones`は内側と判定した領域名、`desired_enabled`は要求値です。`targets`の`enabled`は各ノードから確認した値、`pending`は応答待ち、`error`は失敗理由です。要求値だけで変更成功と判断しないでください。

最初に編集のみで四角の移動・回転・拡縮・保存・再起動後の復元を確認し、次に実行用launchで内側`false`、外側`true`を確認してください。最後に対象環境でRVizのcostmapを確認してください。

通常のCtrl+C / SIGTERMでは最大4秒、`true`への復帰を試みます。強制終了、プロセスクラッシュ、通信断では復帰が保証されません。必要な場合は次のコマンドで復帰できます。

```bash
ros2 param set /local_costmap/local_costmap obstacle_layer.enabled true
# globalも対象にした場合:
ros2 param set /global_costmap/global_costmap obstacle_layer.enabled true
```

## 切り替えの意味と制限

領域は**切り替えのトリガー**です。ロボットが四角に入ると、対象costmapのobstacle_layer全体を無効にします。「四角の中にある障害物セルだけを削除する」処理ではありません。

HumbleのObstacleLayerは`enabled`を動的に変更できますが、`false`にしても、すでに統合されたコストや内部の障害物データを直ちに消去する処理ではありません。更新範囲や他レイヤーにより既存コストが残る場合があり、再有効化時にも内部の古い障害物が影響し得ます。本実装はcostmap全体のクリアを自動実行しません。**「falseになった」と「障害物コストが消えた」は別々に確認してください。**残留が問題になる場合は、対象レイヤーの消去と全範囲更新を追加する設計が必要です。

`static_layer`、`elevation_layer`、prestop、collision_monitorなどは、この実装では変更しません。これらのコストや停止処理が原因なら、obstacle_layerの切り替えだけで走行可能になるとは限りません。

## 検証状況

ROS非依存の幾何判定、YAML入出力、非同期パラメータ変更のテストダブルによる検証とPython構文確認を実施しました。作成環境にROS 2 / RViz / Nav2がないため、Humble実機でのビルド、マーカー操作、サービス通信、costmap表示は未検証です。

```bash
cd ~/ros2_ws/src/obstacle_zone_tools
python3 -m unittest discover -s test -v
```

## 参照

- waypoint_toolsのInteractive Marker実装: https://github.com/TakumiIshiguro/waypoint_tools/blob/master/waypoint_tools/interactive_waypoints.py
- リポジトリ: https://github.com/TakumiIshiguro/waypoint_tools
- Nav2 Humble ObstacleLayer: https://api.nav2.org/nav2-humble/html/obstacle__layer_8cpp_source.html

このパッケージのコードはApache-2.0ライセンスで提供します。
