//
// Created by zhuxinyue on 2025/10/2.

#include "main.h"
#include "tim.h"
#include "can.h"
#include "M3508_Motor.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mail_box_;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
extern uint8_t stop_flag;

M3508_Motor Motor(19.2f, 0.3f);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == htim6.Instance) {
        Motor.handle();
        Motor.GetCurrentData(tx_data);

        tx_header.StdId = 0x200;
        tx_header.DLC = 8;

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x201) {
            Motor.canRxMsgCallback(rx_data);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY_Pin) {
        static uint32_t last_tick = 0;
        uint32_t now = HAL_GetTick();

        if (now - last_tick < 200)
            return;
        last_tick = now;

        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
            stop_flag = !stop_flag;
        }
    }
}