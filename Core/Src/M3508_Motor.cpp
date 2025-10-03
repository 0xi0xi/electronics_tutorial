//
// Created by zhuxinyue on 2025/10/3.
//

#include "M3508_Motor.h"

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    float output;
    output = out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
    return output;
}

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    ecd_angle_ = (rx_data[0] << 8) | rx_data[1];
    rotate_speed_ = (rx_data[2] << 8) | rx_data[3];
    current_ = (rx_data[4] << 8) | rx_data[5];
    temp_ = rx_data[6];

    ecd_angle_ = linearMapping(ecd_angle_, 0, 8191, 0, 360);

    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180.0f)
    {
        delta_ecd_angle_ -= 360.0f;
    }
    else if (delta_ecd_angle_ < -180.0f)
    {
        delta_ecd_angle_ += 360.0f;
    }
    last_ecd_angle_ = ecd_angle_;

    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
}