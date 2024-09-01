#include "Motor.hpp"
#include <cmath>

// コンストラクタ
Motor::Motor(CAN &can, int motor_id)
    : can_interface(can), motor_id(motor_id), speed(0), torque(0),
      cumulative_angle(0), last_angle(0), temperature(0),
      target_speed(0), kp(1.0f), ki(0.0f), kd(0.0f), integral(0.0f), prev_error(0),
      control_thread(osPriorityNormal, 1024, nullptr, "MotorControlThread"),
      control_loop_active(false) {
    // スレッドで制御ループを開始
    control_thread.start(callback(Motor::call_control_loop, this));
}

Motor::~Motor() {
    stop_control_loop();
    control_thread.terminate();
}

void Motor::set_current(int16_t current) {
    char data[8] = {0};
    data[0] = (current >> 8) & 0xFF; // 電流の上位バイト
    data[1] = current & 0xFF;        // 電流の下位バイト
    // 他のモーターの電流を0にする（必要に応じて調整）
    data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = 0;

    CANMessage msg(0x200, data, 8);
    if (!can_interface.write(msg)) {
        printf("CAN send failed for Motor ID: 0x%X\n", motor_id);
    }
}

void Motor::update_feedback() {
    CANMessage msg;
    while (can_interface.read(msg)) {
        if (msg.id == (0x200 + motor_id)) {
            int16_t current_angle = (static_cast<int16_t>(msg.data[0]) << 8) | msg.data[1];
            speed = (static_cast<int16_t>(msg.data[2]) << 8) | msg.data[3];
            torque = (static_cast<int16_t>(msg.data[4]) << 8) | msg.data[5];
            temperature = msg.data[6];

            // 累積角度の更新
            int16_t delta_angle = current_angle - last_angle;
            if (delta_angle > 32767) {
                delta_angle -= 65536; // 反時計回り
            } else if (delta_angle < -32767) {
                delta_angle += 65536; // 時計回り
            }
            cumulative_angle += delta_angle;
            last_angle = current_angle;
        }
    }
}

void Motor::set_target_speed(int16_t target_speed) {
    this->target_speed = target_speed;
}

int16_t Motor::get_speed() const {
    return speed;
}

int16_t Motor::get_torque() const {
    return torque;
}

int32_t Motor::get_angle() const {
    return cumulative_angle;
}

int8_t Motor::get_temperature() const {
    return temperature;
}

int16_t Motor::calculate_pid_output() {
    int16_t error = target_speed - speed;
    integral += error * 0.01f; // 0.01秒間隔を想定
    float derivative = (error - prev_error) / 0.01f;

    float output = kp * error + ki * integral + kd * derivative;

    prev_error = error;

    // 出力をトルク電流の範囲にクリップ (-16384 ～ 16384)
    if (output > 16384.0f) output = 16384.0f;
    if (output < -16384.0f) output = -16384.0f;

    return static_cast<int16_t>(output);
}

void Motor::control_loop() {
    while (true) {
        control_mutex.lock();
        bool active = control_loop_active;
        control_mutex.unlock();
        
        if (active) {
            update_feedback();  // フィードバックデータを更新
            int16_t pid_output = calculate_pid_output();
            set_current(pid_output);  // 計算されたPID出力をトルク電流として設定
        }
        ThisThread::sleep_for(10ms); // 10ms周期
    }
}

void Motor::call_control_loop(Motor *instance) {
    instance->control_loop();
}

void Motor::stop_control_loop() {
    control_mutex.lock();
    control_loop_active = false;
    control_mutex.unlock();
    reset_pid(); // 必要に応じてPID内部変数をリセット
}

void Motor::start_control_loop() {
    reset_pid(); // ループ開始時にPID内部変数をリセット
    control_mutex.lock();
    control_loop_active = true;
    control_mutex.unlock();
}

void Motor::set_pid_parameters(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void Motor::reset_pid() {
    integral = 0.0f;
    prev_error = 0;
}

void Motor::auto_tune_pid() {
    stop_control_loop();

    // 初期の設定
    float ku = 0.0f; // 臨界ゲイン
    float tu = 0.0f; // オシレーション周期
    bool oscillating = false;
    float current_kp = 1.0f;
    const float increase_factor = 1.1f;

    set_pid_parameters(current_kp, 0.0f, 0.0f);

    // オシレーションを検出するための制御ループ
    for (int i = 0; i < 100; ++i) {
        set_target_speed(1000); // 目標速度を設定
        start_control_loop();

        ThisThread::sleep_for(500ms); // 少し待つ

        if (!oscillating && (get_speed() > 100 || get_speed() < -100)) {
            // オシレーションの開始を検出
            oscillating = true;
            ku = current_kp;
            tu = 0.5f; // 初期値 (サンプル時間)
        }

        if (oscillating && (get_speed() < 10 && get_speed() > -10)) {
            // オシレーションが停止したら終わり
            break;
        }

        // ゲインを増加
        current_kp *= increase_factor;
        set_pid_parameters(current_kp, 0.0f, 0.0f);
    }

    // ゲインが設定されなかった場合の安全処理
    if (ku == 0.0f || tu == 0.0f) {
        set_pid_parameters(1.0f, 0.0f, 0.0f); // デフォルトのゲイン
    } else {
        // Ziegler-Nichols法に基づく最適ゲインを設定
        float tuned_kp = 0.6f * ku;
        float tuned_ki = 2.0f * tuned_kp / tu;
        float tuned_kd = tuned_kp * tu / 8.0f;
        set_pid_parameters(tuned_kp, tuned_ki, tuned_kd);
    }

    start_control_loop();
}

PIDGains Motor::get_pid_parameters() const {
    return {kp, ki, kd};
}
