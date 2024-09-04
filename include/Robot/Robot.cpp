#include "Robot.hpp"
#include <cmath>

// コンストラクタ
Robot::Robot(Motor &left_motor, Motor &right_motor, float wheel_diameter, float wheel_base)
    : left_motor(left_motor), right_motor(right_motor), wheel_diameter(wheel_diameter), wheel_base(wheel_base) {}

// ロボット全体の速度を設定するメソッド
void Robot::set_speed(int16_t left_speed, int16_t right_speed) {
    left_motor.set_target_speed(left_speed);
    right_motor.set_target_speed(right_speed);
    left_motor.start_control_loop();
    right_motor.start_control_loop();
}

// ロボット全体の目標角度まで回転するメソッド
void Robot::rotate_to_angle(int32_t left_target_angle, int32_t right_target_angle, int16_t torque, int16_t angle_tolerance) {
    left_motor.rotate_to_angle(left_target_angle, torque, angle_tolerance);
    right_motor.rotate_to_angle(right_target_angle, torque, angle_tolerance);
}

// ロボットを特定の時間前進または後退させるメソッド
void Robot::move_for_duration(int16_t speed, uint32_t duration_ms) {
    set_speed(speed, speed);
    ThisThread::sleep_for(duration_ms);
    set_speed(0, 0); // 停止
}

// ロボットを回転させるメソッド
void Robot::turn(int16_t speed, int32_t duration_ms) {
    set_speed(speed, -speed); // 左右のモーターを逆方向に動かすことで回転
    ThisThread::sleep_for(duration_ms);
    set_speed(0, 0); // 停止
}

// 角度を指定してタイヤの回転角度を計算する関数
int32_t Robot::calculate_wheel_rotation(float angle) {
    // ロボットが指定した角度だけ回転するために必要なタイヤの回転角度を計算
    float rotation_distance = (M_PI * wheel_base * (angle / 360.0f)); // タイヤが進む距離
    int32_t wheel_rotation = static_cast<int32_t>((rotation_distance / (M_PI * wheel_diameter)) * 360.0f); // 必要なタイヤの回転角度
    return wheel_rotation;
}

// ロボット本体を指定角度回転させるメソッド
void Robot::rotate_robot_by_angle(float Turning_Radius, float turning_angle, float distance_to_wheel, int16_t robot_speed, int16_t angle_tolerance, bool Clockwise) {
    
}
