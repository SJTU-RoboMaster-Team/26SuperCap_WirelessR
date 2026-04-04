//
// Created by xuhao on 2026/3/24.
//
#ifndef INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H
#define INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H

#include "stm32g4xx_hal.h"
// Peripherals
#define PWM_BUFFER_SIZE                     250

#define TIMER_CAN_CTRL_LOOP                 TIM6
#define TIMER_HANDLER_CAN_CTRL_LOOP         htim6
#define TIMER_BUZZ_WS2812_CTRL_LOOP         TIM17
#define TIMER_HANDLER_BUZZ_WS2812_CTRL_LOOP htim17
#define TIMER_BUZZ_PWM                      TIM2
#define TIMER_CHANNEL_BUZZ_PWM              TIM_CHANNEL_1
#define TIMER_HANDLER_BUZZ_PWM              htim2
#define TIMER_WS2812_SEND                   TIM8
#define TIMER_HANDLER_WS2812_SEND           htim8
#define TIMER_CHANNEL_WS2812_SEND           TIM_CHANNEL_1
#define TIMER_4201_CTRL_LOOP                TIM5
#define TIMER_HANDLER_4201_CTRL_LOOP        htim5
#define TIMER_4201_ASK_LOOP                 TIM7
#define TIMER_HANDLER_4201_ASK_LOOP         htim7
#define HRTIMER_BBM                         HRTIM1
#define HRTIMER_HANDLER_BBM                 hhrtim1

#define FDCAN                               FDCAN1
#define FDCAN_HANDLER                       hfdcan3

#define USART_REFEREE                       USART3
#define USART_HANDLER_REFEREE               huart3

#define I2C_HANDLER_4201                    hi2c1

#define GPIO_PORT_EN_4201                   GPIOC
#define GPIO_PIN_EN_4201                    GPIO_PIN_13

// WR_Parameters
#define WR_MODE                             0 // 0:CV MODE 1:CP MODE
#define WR_MAX_VOUT                         26.f
#define WR_MAX_IOUT                         5.8f
#define WR_CP_TARGET                        120
#define WR_CP_PI_KP                         0.f
#define WR_CP_PI_KI                         1e-3
#define WR_CV_TARGET                        6.f
#define WR_MIN_VIN                          15.f
#endif //INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H