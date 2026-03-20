/**
  ******************************************************************************
  * @file    ws2812.h
  * @author  xhr
  * @brief   WS2812B-B/W Driver for STM32 HAL (PWM + DMA)
  * Based on datasheet: WS2812.pdf
  ******************************************************************************
  */

#ifndef __WS2812_H__
#define __WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==============================================================================
 * 用户配置区 (User Configuration)
 * ============================================================================== */

// LED 灯珠总数量
#define WS2812_NUM_LEDS    1

// 使用普通的 PWM + DMA 模式 (占用内存 = LED数 * 24 * 2 字节)
#define WS2812_USE_DOUBLE_BUFFER 0

/* ==============================================================================
 * 时序参数定义 (Timing Constraints based on Datasheet)
 * ============================================================================== */
// 假设 PWM 周期 (ARR) 对应 1.25us (800kHz)
// 具体的 CCR 值需要在 ws2812.c 中根据 Timer 时钟频率计算或手动校准

/* ==============================================================================
 * 颜色结构体
 * ============================================================================== */
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} WS2812_Color_t;

/* ==============================================================================
 * 函数原型
 * ============================================================================== */

/**
 * @brief 初始化 WS2812 驱动
 * @param htim: 指向定时器句柄 (e.g., &htim1)
 * @param channel: 定时器通道 (e.g., TIM_CHANNEL_1)
 */
void WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置单个 LED 的颜色 (不立即更新显示，需调用 Send)
 * @param index: LED 索引 (0 ~ WS2812_NUM_LEDS-1)
 * @param r: Red 亮度 (0-255)
 * @param g: Green 亮度 (0-255)
 * @param b: Blue 亮度 (0-255)
 */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置所有 LED 为同一颜色
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清除所有 LED (熄灭)
 */
void WS2812_Clear(void);

/**
 * @brief 将缓存数据发送到 LED 条 (DMA传输)
 * @note  发送完毕后会产生 >280us 的复位信号
 */
void WS2812_Send(void);

/**
 * @brief DMA 传输完成回调 (需要在 HAL_TIM_PWM_PulseFinishedCallback 中调用)
 */
void WS2812_DMA_Callback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */