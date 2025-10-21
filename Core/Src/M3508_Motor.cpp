//
// Created by zhuxinyue on 2025/10/3.
//

#include "M3508_Motor.h"
#include <algorithm>
#include <cmath>

extern uint8_t stop_flag;

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float output;
    output = out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
    return output;
}

M3508_Motor::M3508_Motor(const float ratio, const float Kt):
    ratio_(ratio),
    Kt_(Kt),
    spid_(10.0f, 0.1f, 0.2f, 100.0f, 100.0f),
    ppid_(5.0f, 0.0f, 0.1f, 100.0f, 100.0f),
    target_angle_(0.0f),
    fdb_angle_(0.0f),
    target_speed_(0.0f),
    fdb_speed_(0.0f),
    feedforward_speed_(0.0f),
    feedforward_intensity_(0.0f),
    output_intensity_(0.0f),
    control_method_(SPEED) {}

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    ecd_angle_ = (rx_data[0] << 8) | rx_data[1];
    rotate_speed_ = (rx_data[2] << 8) | rx_data[3];
    current_ = (rx_data[4] << 8) | rx_data[5];
    temp_ = rx_data[6];

    ecd_angle_ = linearMapping(ecd_angle_, 0, 8191, 0, 360);

    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180.0f) {
        delta_ecd_angle_ -= 360.0f;
    } else if (delta_ecd_angle_ < -180.0f) {
        delta_ecd_angle_ += 360.0f;
    }
    last_ecd_angle_ = ecd_angle_;

    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
}

void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = POSITION_SPEED;
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = SPEED;
}

void M3508_Motor::SetIntensity(float intensity) {
    output_intensity_ = intensity;
    control_method_ = TORQUE;
}

void M3508_Motor::handle() {
    if (stop_flag) {
        output_intensity_ = 0;
        return;
    }

    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_;

    switch (control_method_) {
        case TORQUE:
            // 直接使用设定强度
            break;

        case SPEED:
            // 内环速度PID
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_);
            output_intensity_ += feedforward_intensity_;
            break;

        case POSITION_SPEED:
            float gravity_compensation = FeedforwardIntensityCalc(fdb_angle_);
            // 外环位置PID得到目标速度
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            // 内环速度PID得到输出
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_);
            output_intensity_ += feedforward_intensity_ + gravity_compensation;
            break;
    }

    output_intensity_ = std::clamp(output_intensity_, -spid_.out_max_, spid_.out_max_);
}

float M3508_Motor::FeedforwardIntensityCalc(float current_angle) {
    const float mass = 0.5f;
    const float arm_length = 0.05524f;
    const float g = 9.8f;

    float gravity_torque = mass * g * arm_length * sinf(current_angle * 3.14159f / 180.0f);
    float motor_torque = gravity_torque / ratio_;
    float feedforward_current = motor_torque / Kt_;

    return feedforward_current;
}

void M3508_Motor::GetCurrentData(uint8_t tx_data[8]) {
    int16_t current_to_send = static_cast<int16_t>(output_intensity_ * 16384.0f / 20.0f);
    for (int i = 0; i < 8; i++) {
        tx_data[i] = 0;
    }
    tx_data[0] = (current_to_send >> 8) & 0xFF;
    tx_data[1] = current_to_send & 0xFF;
}