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

#define FDCAN                               FDCAN3
#define FDCAN_HANDLER                       hfdcan3

#define USART_REFEREE                       USART3
#define USART_HANDLER_REFEREE               huart3

#define I2C_HANDLER_4201                    hi2c1

#define GPIO_PORT_EN_4201                   GPIOC
#define GPIO_PIN_EN_4201                    GPIO_PIN_13

// WR_Parameters
#define WR_MODE                             1 // 0:CV MODE 1:CP MODE
#define WR_MAX_VOUT                         26.f
#define WR_MAX_IOUT                         5.8f
#define WR_CP_TARGET                        120
#define WR_CP_PI_KP                         0.f
#define WR_CP_PI_KI                         1e-3
#define WR_CV_TARGET                        6.f
#define WR_MIN_VIN                          15.f

// WR_Charge_Manager_Parameters
#define WR_PWR_LIMIT                        120.0f // 无线充最大功率
#define WR_CHARGE_VIN_UPPER_THRESHOLD       15.0f  // 无线充激活上阈值 (V)
#define WR_CHARGE_VIN_LOWER_THRESHOLD       12.0f  // 无线充退出下阈值 (V)
#define WR_CHARGE_TARGET_VOLTAGE            26.0f  // 无线充模式下超电目标电压 (V)
#define WR_CHARGE_MAX_CURRENT               5.8f   // 无线充最大充电电流 (A)
#define WR_CHARGE_ENERGY_PER_BONUS          1.0f   // 每 N(J) 能量差触发一次 bonus
#define WR_CHARGE_BONUS_PER_JOULE           8      // 每次 bonus 增加的底盘能量
// main_control_loop 被 HRTIM master 以 25kHz 触发，wcm.handle() 与之同频
#define MAIN_CONTROL_LOOP_FREQ_HZ           25000.0f
#define WR_CHARGE_LOOP_DT                   (1.0f / MAIN_CONTROL_LOOP_FREQ_HZ)
#endif //INC_26_SUPERCAP_WIRELESSR_V2_CONFIG_H