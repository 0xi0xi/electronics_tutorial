//
// Created by zhuxinyue on 2025/10/20.
//
#include <pid.h>
#include <algorithm>

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    out_max_ = out_max;
    d_filter_k_ = d_filter_k;
    output_ = 0.0f;
    ref_ = 0.0f;
    fdb_ = 0.0f;
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
}

void PID::reset() {
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
}


float PID::calc(float ref, float fdb) {
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;

    pout_ = kp_ * err_; //比例项
    err_sum_ += err_; //积分项
    err_sum_ = std::clamp(err_sum_, -i_max_, i_max_); //积分限幅
    iout_ = ki_ * err_sum_;
    dout_ = err_ - last_err_;
    dout_ = d_filter_k_ * last_dout_ + (1 - d_filter_k_) * dout_; //带一阶低通滤波的微分项

    output_ = pout_ + iout_ + dout_;
    output_ = std::clamp(output_, -out_max_, output_); //总输出与限幅

    last_err_ = err_;
    last_dout_ = dout_; //更新状态

    return output_;
}