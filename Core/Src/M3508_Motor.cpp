//
// Created by zhuxinyue on 2025/10/3.
//

#include "M3508_Motor.h"

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    // // 解析CAN数据
    // ecd_ = (rx_data[0] << 8) | rx_data[1];              // 编码器值
    // speed_rpm_ = (rx_data[2] << 8) | rx_data[3];        // 转速
    // given_current_ = (rx_data[4] << 8) | rx_data[5];    // 电流
    // temperate_ = rx_data[6];                            // 温度
    //
    // // 计算角度（编码器值转换为角度）
    // ecd_angle_ = (ecd_ * 360.0f) / 8191.0f;  // M3508编码器范围0-8191
    //
    // // 计算角度变化量，处理过零情况
    // delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    // if (delta_ecd_angle_ > 180.0f) {
    //     delta_ecd_angle_ -= 360.0f;
    // } else if (delta_ecd_angle_ < -180.0f) {
    //     delta_ecd_angle_ += 360.0f;
    // }
    //
    // // 更新总角度（考虑减速比）
    // angle_ += delta_ecd_angle_ / ratio_;
    // delta_angle_ = delta_ecd_angle_ / ratio_;
    //
    // // 更新上一次的角度
    // last_ecd_angle_ = ecd_angle_;
    //
    // // 更新其他状态
    // rotate_speed_ = speed_rpm_;
    // current_ = given_current_;
    // temp_ = temperate_;
}