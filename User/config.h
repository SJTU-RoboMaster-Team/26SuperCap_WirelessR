//
// Created by xuhao on 2026/3/24.
//
#ifndef INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H
#define INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H

#include "stm32g4xx_hal.h"

#define PWM_BUFFER_SIZE         250

#define TIMER_CAN_CTRL_LOOP                 TIM6
#define TIMER_CAN_CTRL_LOOP_HANDLER         htim6
#define TIMER_BUZZ_LED_CTRL_LOOP            TIM17
#define TIMER_BUZZ_LED_CTRL_LOOP_HANDLER    htim17
#define TIMER_BUZZ_PWM                      TIM2
#define TIMER_CHANNEL_BUZZ_PWM              TIM_CHANNEL_1
#define TIMER_BUZZ_PWM_HANDLER              htim2
#define TIMER_4201_CTRL_LOOP                TIM5
#define TIMER_4201_CTRL_LOOP_HANDLER        htim5
#define TIMER_4201_ASK_LOOP                 TIM7
#define TIMER_4201_ASK_LOOP_HANDLER         htim7

#define FDCAN_HANDLER                       hfdcan3
#define USART_HANDLER                       huart3

#endif //INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H