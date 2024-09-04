#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "mbed.h"
#include "Motor.hpp"

class Robot {
public:
    // コンストラクタ
    Robot(Motor &left_motor, Motor &right_motor, float wheel_diameter, float wheel_base);

    // ロボット全体の速度を設定するメソッド
    void set_speed(int16_t left_speed, int16_t right_speed);

    // ロボット全体の目標角度まで回転するメソッド
    void rotate_to_angle(int32_t left_target_angle, int32_t right_target_angle, int16_t torque, int16_t angle_tolerance = 5);

    // ロボットを特定の時間前進または後退させるメソッド
    void move_for_duration(int16_t speed, uint32_t duration_ms);

    // ロボットを回転させるメソッド
    void turn(int16_t speed, int32_t duration_ms);

    // ロボット本体を指定角度回転させるメソッド
    void rotate_robot_by_angle(float Turning_Radius, float turning_angle, float distance_to_wheel, int16_t robot_speed, int16_t angle_tolerance = 5, bool Clockwise = true);

private:
    Motor &left_motor;
    Motor &right_motor;
    float wheel_diameter;
    float wheel_base;

    // 角度を指定してタイヤの回転角度を計算する関数
    int32_t calculate_wheel_rotation(float angle);
};

#endif // ROBOT_HPP
