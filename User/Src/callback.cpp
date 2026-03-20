#include "../Inc/callback.h"
#include "../Inc/MP4201.h"
#include "../Inc/timer.h"
#include "../Inc/modulator.h"

extern "C" {
extern TIM_HandleTypeDef htim5; // 无线充控制频率 2kHz
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7; // ASK负载调制频率 4kHz
extern HRTIM_HandleTypeDef hhrtim1;
MP4201_Handle_t mp4201_dev; // 全局 MP4201 设备句柄
FreqMeter modulator_loop(1);
Modulator load_modulator(modulator_loop);

// 定时器回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim5) {
        // 以 2kHz 的频率进行一轮采集 & 控制
        // 如果上一次 DMA 更新还没完成，就跳过这次
        MP4201_StartDMAUpdate(&mp4201_dev);
    } else if (htim == &htim7) {
        // ASK 负载调制
        load_modulator.handle();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == mp4201_dev.hi2c) {
        MP4201_OnRxCplt(&mp4201_dev);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == mp4201_dev.hi2c) {
        MP4201_OnTxCplt(&mp4201_dev);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == mp4201_dev.hi2c) {
        MP4201_OnError(&mp4201_dev);
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef* hhrtim, uint32_t TimerIdx) {
    (void)hhrtim;
    (void)TimerIdx;
}
}