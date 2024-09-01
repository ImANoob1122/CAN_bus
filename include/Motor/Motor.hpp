#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "mbed.h"
#include "CAN.h"

class Motor {
public:
    // コンストラクタ
    Motor(CAN &can, int motor_id);

    // デストラクタ
    ~Motor();

    // 速度コマンドを送信するメソッド
    void set_current(int16_t current);

    // フィードバックデータを更新するメソッド
    void update_feedback();

    // PID制御で目標速度を設定するメソッド
    void set_target_speed(int16_t target_speed);

    // PIDパラメータを自動調整するメソッド
    void auto_tune_pid();

    // 現在のPIDパラメータを出力するメソッド
    void print_pid_parameters() const;

    // フィードバックデータを取得するメソッド
    int16_t get_speed() const;
    int16_t get_torque() const;
    int32_t get_angle() const;
    int8_t get_temperature() const;

    // 制御ループを停止するメソッド
    void stop_control_loop();

    // 制御ループを再開するメソッド
    void start_control_loop();

    // PIDパラメータを設定するメソッド
    void set_pid_parameters(float kp, float ki, float kd);

    // PID内部変数をリセットするメソッド
    void reset_pid();

private:
    CAN &can_interface; // CANインターフェースの参照
    int motor_id;       // モーターのCAN ID

    // フィードバックデータ
    int16_t speed;
    int16_t torque;
    int32_t cumulative_angle; // 累積角度
    int16_t last_angle; // 前回の角度値
    int8_t temperature;

    // PID制御用のパラメータ
    int16_t target_speed;
    float kp, ki, kd;
    float integral;
    int16_t prev_error;

    // 制御ループがアクティブかどうかのフラグ
    bool control_loop_active;
    rtos::Mutex control_mutex;

    // PID制御で計算されたトルク電流を取得
    int16_t calculate_pid_output();

    // 制御ループ
    void control_loop();

    // スレッドとタイマー
    rtos::Thread control_thread;

    // 制御ループを呼び出すための静的メソッド
    static void call_control_loop(Motor *instance);
};

#endif // MOTOR_HPP
