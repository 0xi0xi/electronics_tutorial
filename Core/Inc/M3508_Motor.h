//
// Created by zhuxinyue on 2025/10/3.
//

#ifndef TIMER_M3508_MOTOR_H
#define TIMER_M3508_MOTOR_H

#include "cinttypes"

class M3508_Motor{
private:
    const float ratio_;
    float angle_ = 0.f;
    float delta_angle_ =0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float rotate_speed_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;

    // // 添加编码器相关变量
    // uint16_t ecd_ = 0;
    // uint16_t last_ecd_ = 0;
    // int16_t speed_rpm_ = 0;
    // int16_t given_current_ = 0;
    // uint8_t temperate_ = 0;

public:
    explicit M3508_Motor(const float ratio) : ratio_(ratio){};
    void canRxMsgCallback(const uint8_t rx_data[8]);
    // // 获取电机状态的公共接口
    // float getAngle() const { return angle_; }
    // float getSpeed() const { return rotate_speed_; }
    // float getCurrent() const { return current_; }
    // float getTemperature() const { return temp_; }
    // float getDeltaAngle() const { return delta_angle_; }
    //
    // // 重置角度（可选）
    // void resetAngle() { angle_ = 0.f; last_ecd_angle_ = ecd_angle_; }
};

#endif //TIMER_M3508_MOTOR_H