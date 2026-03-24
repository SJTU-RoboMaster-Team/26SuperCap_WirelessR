//
// Created by yixin on 2024/1/18.
//
#include "main.h"
#include "SuperCap/control_loop.h"
#include "SuperCap/types.h"
#include "SuperCap/math_tool/my_math.h"
#include "SuperCap/referee_comm.h"
#include "SuperCap/music/music_player.h"
#include "SuperCap/led_indicator.h"
#include "SuperCap/lcd/lcd_comm.h"
#include "SuperCap/ws2812/ws2812.h"
#include "callback.h"
#include "SuperCap/task_manager/timer.h"
#include "SuperCap/adc_sample.h"
#include "config.h"
#include "WirelessR/Inc/MP4201.h"
#include "WirelessR/Inc/timer.h"
#include "WirelessR/Inc/modulator.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern IWDG_HandleTypeDef hiwdg;

extern uint8_t uart1_rx_buf[256];
extern uint8_t uart3_rx_buf[256];

uint8_t header;
uint32_t size;

ADC_Buffer adc_buffer{};
ADC_Freq adc_freq{};
ADC_Tick adc_tick{};
ADC_Tick adc_last_tick{};

RefereeComm referee;

extern LCDComm lcd_comm;
extern WS2812 ws2812;

MusicPlayer music_player = MusicPlayer(htim2);
extern LED led;

RefreshFlag refresh_flag = { HAL_ERROR, false };

//AverageFilterInt adc_v_in;
//AverageFilterInt adc_i_in;
//AverageFilterInt adc_i_o;
//AverageFilterInt adc_v_c;
//AverageFilterInt adc_i_c;

extern uint16_t adc1_buf[16];
extern uint16_t adc2_buf[16];
extern uint16_t adc3_buf[16];
extern uint16_t adc4_buf[16];
extern uint16_t adc5_buf[16];

ADC_SAMPLE adc_v_in(8, adc1_buf);
ADC_SAMPLE adc_i_in(8, adc2_buf);
ADC_SAMPLE adc_i_o(8, adc3_buf);
ADC_SAMPLE adc_v_c(8, adc4_buf);
ADC_SAMPLE adc_i_c(8, adc5_buf);
FreqMeter control_loop(1);
FreqMeter can_loop(1);

uint32_t uint_sample_times;

uint8_t refresh_counter;

MP4201_Handle_t mp4201_dev; // 全局 MP4201 设备句柄
FreqMeter modulator_loop(1);
Modulator load_modulator(modulator_loop);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIMER_CAN_CTRL_LOOP) {
        // 100 Hz
        CAN_control_loop();
        can_loop.update();
    } else if (htim->Instance == TIMER_BUZZ_LED_CTRL_LOOP) {
        // 100 Hz
        music_player.handle();
        led.handle();
        lcd_comm.handle();
        refresh_flag.tick_ = HAL_GetTick();
        // 250ms iwdg; 100ms refresh
        refresh_counter++;
        if (refresh_flag.isAllflagtrue() && refresh_counter >= 10) {
            HAL_IWDG_Refresh(&hiwdg);
            refresh_flag.reset();
        }
    } else if (htim->Instance == TIMER_4201_CTRL_LOOP) {
        // 2kHz
        MP4201_StartDMAUpdate(&mp4201_dev);
    }else if (htim->Instance == TIMER_4201_ASK_LOOP) {
        //4kHz
        load_modulator.handle();
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef* hhrtim, uint32_t TimerIdx) {
    // 25000Hz
    if (hhrtim->Instance == HRTIM1) {
        if (TimerIdx == HRTIM_TIMERINDEX_MASTER) {
            main_control_loop();
            control_loop.update();
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // V in
        // 滤波不好用
        //        adc_buffer.v_in = lp1.update(HAL_ADC_GetValue(&hadc1));
        // adc_buffer.v_in = HAL_ADC_GetValue(&hadc1);
        // EventStopA(0);
        // EventStartA(0);
        //        uint_sample_times++;
        adc_buffer.v_in = adc_v_in.cal_average();
        adc_v_in.freq_meter_.update();
    } else if (hadc->Instance == ADC2) {
        // I in
        // adc_buffer.i_in = HAL_ADC_GetValue(&hadc2) - 32767;
        adc_buffer.i_in = adc_i_in.cal_average() - 2047;
        adc_i_in.freq_meter_.update();
    } else if (hadc->Instance == ADC3) {
        // I out
        // adc_buffer.i_o = HAL_ADC_GetValue(&hadc3) - 32767;
        adc_buffer.i_o = adc_i_o.cal_average() - 2047;
        adc_i_o.freq_meter_.update();
    } else if (hadc->Instance == ADC4) {
        // V cap
        // adc_buffer.v_c = HAL_ADC_GetValue(&hadc4);
        adc_buffer.v_c = adc_v_c.cal_average();
        adc_v_c.freq_meter_.update();
    } else if (hadc->Instance == ADC5) {
        // I cap
        // adc_buffer.i_c = HAL_ADC_GetValue(&hadc5) - 32767;
        adc_buffer.i_c = adc_i_c.cal_average() - 2047;
        adc_i_c.freq_meter_.update();
    }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//    if (hadc->Instance == ADC1){
//        // V in
//        // 滤波不好用
////        adc_buffer.v_in = lp1.update(HAL_ADC_GetValue(&hadc1));
//        adc_buffer.v_in = adc_v_in.update(HAL_ADC_GetValue(&hadc1));
//    } else if (hadc->Instance == ADC2){
//        // I in
//        adc_buffer.i_in = adc_i_in.update((int32_t)HAL_ADC_GetValue(&hadc2) - 32767);
//    } else if (hadc->Instance == ADC3){
//        // I out
//        adc_buffer.i_o = adc_i_o.update((int32_t)HAL_ADC_GetValue(&hadc3) - 32767);
//    } else if (hadc->Instance == ADC4){
//        // V out
//        adc_buffer.v_c = adc_v_c.update(HAL_ADC_GetValue(&hadc4));
//    } else if (hadc->Instance == ADC5){
//        // I cap
//        adc_buffer.i_c = adc_i_c.update(HAL_ADC_GetValue(&hadc5) - 32767);
//    }
//}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim8) {
        ws2812.pwm_send_callback();
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        static uint8_t buffer[256];
        for (int i = 0; i < Size; ++i) {
            buffer[i] = uart3_rx_buf[i];
            //            uart5_rx_buf[i] = 0;
        }

        referee.rxCallback(buffer, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, sizeof(uart3_rx_buf));
    } else if (huart->Instance == USART1) {
        static uint8_t buffer[256];
        for (int i = 0; i < Size; ++i) {
            buffer[i] = uart1_rx_buf[i];
        }
        lcd_comm.rxCallback(buffer, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_buf, sizeof(uart1_rx_buf));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART3) {
        // Handle UART5 error

        // Reset the peripheral and the DMA
        HAL_UART_DeInit(huart);
        HAL_DMA_DeInit(huart->hdmarx); // If you're using DMA

        // Reconfigure GPIOs if necessary

        // Reinitialize the peripheral and the DMA
        HAL_UART_Init(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, sizeof(uart3_rx_buf)); // If you're using DMA
    } else if (huart->Instance == USART1) {
        // Handle LPUART1 error

        // Reset the peripheral and the DMA
        HAL_UART_DeInit(huart);
        HAL_DMA_DeInit(huart->hdmarx); // If you're using DMA

        // Reconfigure GPIOs if necessary

        // Reinitialize the peripheral and the DMA
        HAL_UART_Init(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_buf, sizeof(uart1_rx_buf)); // If you're using DMA
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
