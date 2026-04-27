//
// Created by xuhao on 2026/3/24.
//
#include "config.h"
#include "main.h"
#include "stm32g4xx_hal.h"

// User Public Variables
uint8_t uart3_rx_buf[256];
uint8_t pwm_buffer[PWM_BUFFER_SIZE];

uint16_t adc1_buf[16];
uint16_t adc2_buf[16];
uint16_t adc3_buf[16];
uint16_t adc4_buf[16];
uint16_t adc5_buf[16];

// User Functions
void CAN_filter_config(FDCAN_HandleTypeDef* hfdcan){
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_CHASSIS_CMD;
    sFilterConfig.FilterID2 = CAN_ID_CHASSIS_CMD;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}
