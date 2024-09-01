#include "Motor.hpp"

Motor::Motor(CAN &can, int motor_id)
    : can_interface(can), motor_id(motor_id), speed(0), torque(0),
      cumulative_angle(0), last_angle(0), temperature(0),
      target_speed(0), kp(1.0f), ki(0.1f), kd(0.01f), integral(0.0f), prev_error(0),
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

void Motor::rotate_to_angle(int32_t target_angle, int16_t torque, int16_t angle_tolerance) {
    // 制御ループを停止
    stop_control_loop();

    update_feedback(); // まず現在の角度を取得

    // 現在の角度が目標角度からの許容誤差内であるかを確認
    while (abs(cumulative_angle - target_angle) > angle_tolerance) {
        if (cumulative_angle < target_angle) {
            set_current(torque);  // 正方向に回転
        } else {
            set_current(-torque); // 逆方向に回転
        }
        update_feedback(); // 角度を再度確認
        ThisThread::sleep_for(10ms); // 10ms待機
    }

    // 目標角度に達したらトルクをゼロにしてモーターを停止
    set_current(0);

    // 制御ループを再開
    start_control_loop();
}
