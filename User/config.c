//
// Created by xuhao on 2026/3/24.
//
#include "config.h"
#include "stm32g4xx_hal.h"

uint8_t uart3_rx_buf[256];
uint8_t uart1_rx_buf[256];
uint8_t pwm_buffer[PWM_BUFFER_SIZE];

uint16_t adc1_buf[16];
uint16_t adc2_buf[16];
uint16_t adc3_buf[16];
uint16_t adc4_buf[16];
uint16_t adc5_buf[16];