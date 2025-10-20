//
// Created by zhuxinyue on 2025/10/3.
//

#ifndef TIMER_M3508_MOTOR_H
#define TIMER_M3508_MOTOR_H
#include <stdint.h>
#include <pid.h>

class M3508_Motor {
private:
    const float ratio_;
    float angle_ = 0.f;
    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float rotate_speed_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;
    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;

    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

public:
    explicit M3508_Motor(const float ratio):
        ratio_(ratio) {};
    void canRxMsgCallback(const uint8_t rx_data[8]);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
};

#endif //TIMER_M3508_MOTOR_H