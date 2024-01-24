# CoRE_AutoRobot_2024_sample
Core 2024 自動ロボットの動作確認のために、実行委員会が使用したプログラムです。

## 想定している実行環境
- Core 2024 向け自動ロボット
- PC
  - x86_64 CPU
  - NVIDIA社製GPUと、その最新のドライバ
  - Ubuntu22.04
  - ROS2 Humble
- DualSense(PS5のコントローラ)

DualSense <--USB--> PC <---Ethernet---> RaspberryPi <--CAN/GPIO--> 自動ロボット

## 初期確認
次の手順に従い、ロボットの動作を確認してください。  
すぐに動きを止めたい時は電源スイッチを切るしかないため、予め位置を確認してください。  
想定通りに動かない場合、電気的・機械的に不具合が無いかを確認してください。または、実行委員会にご連絡ください。

**この作業の間は、ボールを装填しないでください**

- 配線をして、ロボットの電源を入れる
- PCで`ros2 topic list`を実行し、gpio_nodeやservo0などのRaspberryPiが出力するTopicが出ているかを確認する。
  - 出なければ接続設定等を確認して出るようにする
- 砲台をそれぞれの軸の限界まで回して、フォトインタラプタのTopicがtrueを出力するか確認する。
  - Yaw回転・右端
    - `ros2 topic echo /gpio_node/in0`
  - Yaw回転・左端
    - `ros2 topic echo /gpio_node/in1`
  - Pitch回転・下端
    - `ros2 topic echo /gpio_node/in2`
  - Pitch回転・上端
    - `ros2 topic echo /gpio_node/in3`
- サーボモーターの動作を確認する。
  - ボール押し出し部(hammer)
    - 押す
      - `ros2 topic pub /servo0/degree std_msgs/msg/Float64 data:\ 85.0`
    - 引く
      - `ros2 topic pub /servo0/degree std_msgs/msg/Float64 data:\ 0.0`
  - ボールかきまぜ部(mazemaze)
    - 時計回り
      - `ros2 topic pub /servo1/degree std_msgs/msg/Float64 data:\ 0.0`
    - 反時計回り
      - `ros2 topic pub /servo1/degree std_msgs/msg/Float64 data:\ 270.0`
- ローラーの動作を確認する。射出方向に回っていることを確認する。
  - 左ローラー
    - 回す
      - `ros2 topic pub /can_node/c620_0/target_current std_msgs/msg/Float64 data:\ 1.0`
    - 止める
      - `ros2 topic pub /can_node/c620_0/target_current std_msgs/msg/Float64 data:\ 0.0`
  - 右ローラー
    - 回す
      - `ros2 topic pub /can_node/c620_1/target_current std_msgs/msg/Float64 data:\ -1.0`
    - 止める
      - `ros2 topic pub /can_node/c620_1/target_current std_msgs/msg/Float64 data:\ 0.0`
- Pitch回転の動作を確認する。(動きを確認したら速やかにCtrl-Cで出力を止めて、素早く停止の命令を入力し送る)
  - 上に
    - `ros2 topic pub /can_node/gm6020_0/target_volt std_msgs/msg/Int64 data:\ -5000`
  - 下に
    - `ros2 topic pub /can_node/gm6020_0/target_volt std_msgs/msg/Int64 data:\ 5000`
  - 停止
    - `ros2 topic pub /can_node/gm6020_0/target_volt std_msgs/msg/Int64 data:\ 0`
- Yaw回転の動作を確認する。(動きを確認したら速やかにCtrl-Cで出力を止めて、素早く停止の命令を入力し送る)
  - 右に
    - `ros2 topic pub /can_node/gm6020_1/target_volt std_msgs/msg/Int64 data:\ -3000`
  - 左に
    - `ros2 topic pub /can_node/gm6020_1/target_volt std_msgs/msg/Int64 data:\ 3000`
  - 停止
    - `ros2 topic pub /can_node/gm6020_1/target_volt std_msgs/msg/Int64 data:\ 0`
- Pitch回転の原点を確認
  - 現在の値を出力する
    - `ros2 topic echo /can_node/gm6020_0/degree`
  - 前方に向けた時の数値を記録する
    - 例として、サンプルプログラムを試した機体ではおよそ120°。
- Yaw回転の原点を確認
  - 現在の値を出力する
    - `ros2 topic echo /can_node/gm6020_0/degree`
  - 前方に向けた時の数値を記録する
    - 例として、サンプルプログラムを試した機体ではおよそ295°。

## サンプルプログラムの実行
## 環境構築
- x86_64系のCPUをNVIDIAのGPUを搭載したPCを用意する
- Ubuntu 22.04をインストールする
- NVIDIAのドライバをインストールする。`nvidia-smi`が動作すればだいたいOK。
- [ROS2 Humbleをインストール](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)する
- PS5コントローラをPCに接続し、`/dev/input/js0`に出てくることを確認する
- RealSenseを接続する

## サンプルプログラムのセットアップ
```
cd /path/to/ros2_ws/src
git clone https://github.com/scramble-robot/CoRE_AutoRobot_2024_sample.git
cd CoRE_AutoRobot_2024_sample
rosdep install -i --from-paths .
```

### パラメータ調節
- PitchとYaw回転の原点を設定する
  - corejp_sample/corejp_sample_controller/corejp_sample_controller/turret.py を開く
  - Pitch回転の原点を入力する
    - `self.offset_pitch = math.radians(120)`となっている部分の120の部分を、確認した原点の値に置き換える
  - Yaw回転の原点を入力する
    - `self.offset_yaw = math.radians(295)`となっている部分の295の部分を、確認した原点の値に置き換える
- 最初の動作確認では、ボールを入れないことを推奨

### ビルド
```
cd /path/to/ros2_ws/
colcon build --symlink-install
source ./install/setup.bash
```

### 起動
- 砲口をおよそ前方に向ける
- `ros2 launch corejp_sample_commander ros2 launch corejp_sample_commander all.launch.py` を実行する
- 起動しても、ロボットが動き出すことは無い
- 初回起動時はYOLOのweightをダウンロードするために少し時間がかかる

### 手動操作
- PID制御を有効化する
  - **L1**を押し続けると、各モーターに制御が入り、砲口は原点を向く
  - ここで異常な振動や原点以外への移動が見られた場合は素早く**L1**を離し、制御を止める。
  - 以降の作業は全て**L1**を<u>押したまま</u>実行する。作業の合間に離すのは構わない。
- ローラーを回す
  - **R2**を押している間、ローラーが回る。また、押し加減で速度が変化する。
- 砲口の向きを変える
  - **右スティック**の上下左右が、それぞれPitchとYawに対応している。
- ボールかきまぜ部を動作させる
  - 矢印の左(**←**)ボタンを押すと、かき混ぜる動作をする。
  - 押すたびにプログラムされた動作が走るため、ボタンを離してもすぐに止まることは無い。
  - ここはL1を押さなくても動く
- ボール押し出し部を動作させる
  - **R1**ボタンを押すと、ボール押し出しとかき混ぜの動作をする。
  - 押すたびにプログラムされた動作が走るため、ボタンを離してもすぐに止まることは無い。
  - ここもL1を押さなくても動く
- ボールを飛ばす
  - 弾倉にボールを入れる
  - **←**ボタンでかき混ぜて、じゃばらにボールを入れる。
  - **L1**で制御を入れながら**L2**でローラーを回しながら**R1**でボールを押し出すとボールが飛んでいく。
  - ボールがローラーで詰まる場合は、**R1**の押し加減が足りず速度足りていないため。もう一度、よく押して飛ばす。

### 連続した射出のサンプル
ボールを色々な方向に自動で飛ばします。

- 機体の周りにボールが当たってマズいものが無いかを確認する
- 矢印ボタンの下(**↓**)を押して、Sequenceモードにする。押しても特に分かる変化はない。
- **L1**と共に**L2**を押し続ける。押し続けている間、２秒おきにボールを色々な方向に飛ばす。
- プログラムされた動作を全て完了すると、正面を向いて止まる。
- **L2**ボタンを離すと止まる。

### ターゲットの追尾サンプル
RealSenseのデータを使って照準を合わせ、ボールを飛ばします。

- 的を用意する
  - 簡単のため、YoloV8のサンプルに含まれる`stop sign`を追尾するプログラムとしている。
  - 道路標識及び信号に関するウィーン条約の八角形の一時停止標識を紙に印刷する
    - 実行委員会でのテストでは[Wikipediaのデータ](https://commons.wikimedia.org/wiki/File:Vienna_Convention_road_sign_B2a.svg)をラベルシールに印刷し、ダメージパネルに貼り付けた。
    - 日本の一時停止標識は認識されない。
  - 道路標識及び信号に関するウィーン条約で規定された八角形の一時停止標識が部屋に存在しないことを確認する
    - 主に、印刷に使ったPCのディスプレイに表示されたままの標識に向けてボールが飛んでいく悲劇を避けたい
- 矢印ボタンの上(**↑**)を押して、Trackingモードにする。押しても特に分かる変化はない。
- **L1**と共に**L2**を押し続ける。視界に的があれば追尾し、ある程度そちらの方向に向くとボールを飛ばす。
  - 実行委員会でのテストでは50cm程度の距離で動作を確認した。
- **L2**ボタンを離すと止まる。

## 本サンプルプログラムが簡単のため技術的に無視している部分について
- 前述の通り、ダメージパネルを認識するプログラムではない。複数のターゲットへの考慮も無い。
- PS5コントローラの操作が無いと動かない構成になっている。もちろん、本番では試合中の操作は認められていない。
  - サンプル動かしたいけど持っていない方は、大変申し訳無いが調達するかプログラムをお手持ちのJoyコン向けに改変して頂きたい・・・`corejp_sample/corejp_sample_commander/corejp_sample_commander/joy.py`だけ変更すれば使えるはず。
- PID制御をPC側で回している。何をどう考えてもRaspberryPi側で回したほうが良い。
- 異常を検知して停止するような機能が無い。特に、司令が一定時間送られなければ不具合があったとして安全に停止するなどといった構造が無い。暴走を避けるためには必須の機能。
- カメラの取り付け位置情報は使っていない。砲口の位置にカメラがある想定になっている。RealSenseの取り付け位置は下でテストしたが、上でもある程度動くはず。
- RealSenseのDepth情報は使っていない。距離に応じてボールを打ち上げる機能は無いので、離れていくとボールは的の下を通過する。
