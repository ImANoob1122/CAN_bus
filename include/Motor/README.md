# ロボマスモーター制御
ロボマスモーター制御用のクラスです。

## Usage
インクルード
```bash
#include "CAN.h"
#include "Motor.hpp"
```
このクラスにはCANをインクルードすることも必須です。

### 
モーター登録
```bash
CAN can(PB_8, PB_9); // CANピンの定義
int motor_id = 1;
Motor mortor(can, motor_id);
```
can,motor_id,mortorなどの名前は何でも構いません。

mortor_idの変数は接続されているモーターのIDの番号を確認してください。IDに関する詳細はC620の説明書を確認。

### 
速度送信
```bash
int16_t currents = 500;
mortor.set_current(currents);
```
この関数で送信しているのはトルク電流です。RoboMaster C620コントローラーでは、CANメッセージを通じてモーターの出力トルクを制御します。この電流値はモーターに加えるトルクの大きさを表し、-16384から16384の範囲で設定できます。

### 
フィードバックのアップデート
```bash
motor.update_feedback();
```
モーターから送信されるフィードバックデータ（角度、速度、トルク、温度）を受信して更新します。

フィードバックの取得
```
### 

