/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   WS2812 Driver Implementation
  ******************************************************************************
  */

#include "../Inc/WS2812.h"
#include <string.h> // memset

/* 私有变量 */
static TIM_HandleTypeDef *ws_htim;
static uint32_t ws_channel;

// LED 颜色缓存 (RGB格式)
static WS2812_Color_t led_buffer[WS2812_NUM_LEDS];

// DMA 发送缓存 (PWM 占空比数据)
// 每个 LED 需要 24 位数据。为了产生 Reset 信号，通常最后保持总线为低即可。
// 数组大小: 24 * N
static uint16_t pwm_buffer[WS2812_NUM_LEDS * 24 + 1]; // +1 为了安全终止

// 标志位：是否正在发送
static volatile uint8_t is_sending = 0;

/* * PWM 占空比计算 (基于 Timer ARR)
 * 在 Init 函数中会自动根据 htim->Init.Period 计算
 * 0码: ~30% duty cycle
 * 1码: ~60% duty cycle
 */
static uint16_t pwm_0_code = 0;
static uint16_t pwm_1_code = 0;

/**
  * @brief 初始化函数
  */
void WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    ws_htim = htim;
    ws_channel = channel;

    // 根据 Timer 的自动重装载值 (ARR) 计算 0码和 1码的占空比
    // WS2812B 手册要求:
    // 0码高电平: 220ns ~ 380ns (取 300ns)
    // 1码高电平: 580ns ~ 1us   (取 650ns，确保 > 580ns)
    // 整个周期: 1.25us (800kHz)

    // 假设 Timer 已配置为 800kHz (周期 1.25us)
    // ARR 值对应 1.25us
    uint32_t period = ws_htim->Init.Period;

    // 计算 CCR 值
    // 300ns / 1250ns = 0.24
    // 650ns / 1250ns = 0.52
    pwm_0_code = (uint16_t)(period * 0.24f);
    pwm_1_code = (uint16_t)(period * 0.52f);

    // 初始化 buffer 为全 0 (熄灭)
    WS2812_Clear();
}

/**
  * @brief 设置单个像素颜色
  */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= WS2812_NUM_LEDS) return;
    led_buffer[index].R = r;
    led_buffer[index].G = g;
    led_buffer[index].B = b;
}

/**
  * @brief 设置所有像素
  */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        WS2812_SetPixel(i, r, g, b);
    }
}

/**
  * @brief 清除所有像素
  */
void WS2812_Clear(void) {
    memset(led_buffer, 0, sizeof(led_buffer));
}

/**
  * @brief 将颜色数据转换为 PWM 波形数据并启动 DMA
  */
void WS2812_Send(void) {
    if (is_sending) return; // 如果正在发送，忽略本次请求（或改为等待）

    uint32_t buffer_idx = 0;

    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        // WS2812B 发送顺序: GRB (高位在先)
        uint32_t grb = ((uint32_t)led_buffer[i].G << 16) |
                       ((uint32_t)led_buffer[i].R << 8)  |
                       led_buffer[i].B;

        for (int j = 23; j >= 0; j--) {
            if ((grb >> j) & 0x01) {
                pwm_buffer[buffer_idx] = pwm_1_code;
            } else {
                pwm_buffer[buffer_idx] = pwm_0_code;
            }
            buffer_idx++;
        }
    }

    // 最后一个设为 0，确保结束时为低电平
    pwm_buffer[buffer_idx] = 0;

    is_sending = 1;

    // 启动 DMA 传输
    // 注意：这里发送长度为 buffer_idx，不包括 reset 信号的时间
    // Reset 信号由 DMA 传输结束后的空闲低电平时间产生
    HAL_TIM_PWM_Start_DMA(ws_htim, ws_channel, (uint32_t *)pwm_buffer, buffer_idx);
}

/**
  * @brief DMA 完成回调
  * 必须在 stm32g4xx_it.c 或 main.c 的 HAL_TIM_PWM_PulseFinishedCallback 中调用
  */
void WS2812_DMA_Callback(TIM_HandleTypeDef *htim) {
    if (htim == ws_htim) {
        // 停止 PWM DMA 生成
        HAL_TIM_PWM_Stop_DMA(ws_htim, ws_channel);

        // 关键点：手册要求 Reset 时间 > 280us
        // 此时 PWM 停止，GPIO 默认为低电平（如果配置正确）
        // 我们利用软件延时或标记来保证这段时间
        // 在这里我们仅清除标志位，实际 Reset 时间由下一次调用 Send 的间隔决定
        // 如果需要强制等待，可以在 Send 函数开头加入延时判断

        is_sending = 0;
    }
}