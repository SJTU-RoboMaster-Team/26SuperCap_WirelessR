//
// Created by yixin on 2024/1/18.
//
#include "callback.h"
#include "SuperCap/adc_sample.h"
#include "SuperCap/can_task.h"
#include "SuperCap/control_loop.h"
#include "SuperCap/lcd/lcd_comm.h"
#include "SuperCap/led_indicator.h"
#include "SuperCap/math_tool/my_math.h"
#include "SuperCap/music/music_player.h"
#include "SuperCap/power_manager.h"
#include "SuperCap/referee_comm.h"
#include "SuperCap/task_manager/timer.h"
#include "SuperCap/types.h"
#include "SuperCap/ws2812/ws2812.h"
#include "WirelessR/Inc/MP4201.h"
#include "WirelessR/Inc/modulator.h"
#include "WirelessR/Inc/timer.h"
#include "config.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern IWDG_HandleTypeDef hiwdg;

extern uint8_t uart3_rx_buf[256];

uint8_t header;
uint32_t size;

ADC_Buffer adc_buffer {};
ADC_Freq adc_freq {};
ADC_Tick adc_tick {};
ADC_Tick adc_last_tick {};

RefereeComm referee;

extern LCDComm lcd_comm;
extern WS2812 ws2812;
extern PowerManager pm;

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

static uint8_t fdcan_rx_data[8];
static uint8_t fdcan_dlc_to_bytes(uint32_t dlc);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIMER_CAN_CTRL_LOOP) {
        // 100 Hz
        CAN_control_loop();
        can_loop.update();
    } else if (htim->Instance == TIMER_BUZZ_WS2812_CTRL_LOOP) {
        // 100 Hz
        music_player.handle();
        led.handle();
        lcd_comm.handle();
        refresh_flag.tick_ = HAL_GetTick();
        // 1000ms iwdg; 100ms refresh
        refresh_counter++;
        if (refresh_flag.isAllflagtrue() && refresh_counter >= 10) {
            HAL_IWDG_Refresh(&hiwdg);
            refresh_flag.reset();
        }
    } else if (htim->Instance == TIMER_4201_CTRL_LOOP) {
        // 2kHz
        MP4201_StartDMAUpdate(&mp4201_dev);
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef* hhrtim, uint32_t TimerIdx) {
    if (hhrtim->Instance == HRTIMER_BBM) {
        if (TimerIdx == HRTIM_TIMERINDEX_MASTER) {
            // 25kHz
            main_control_loop();
            control_loop.update();
        } else if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A) {
            // 2kHz
            load_modulator.handle();
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
    if (htim->Instance == TIMER_WS2812_SEND) {
        ws2812.pwm_send_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART_REFEREE) {
        static uint8_t buffer[256];
        for (int i = 0; i < Size; ++i) {
            buffer[i] = uart3_rx_buf[i];
        }

        referee.rxCallback(buffer, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, sizeof(uart3_rx_buf));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART_REFEREE) {
        // Handle UART3 error

        // Reset the peripheral and the DMA
        HAL_UART_DeInit(huart);
        HAL_DMA_DeInit(huart->hdmarx); // If you're using DMA

        // Reconfigure GPIOs if necessary

        // Reinitialize the peripheral and the DMA
        HAL_UART_Init(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, sizeof(uart3_rx_buf)); // If you're using DMA
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan->Instance == FDCAN && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef rx_header;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, fdcan_rx_data) != HAL_OK) {
            return;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME
            || rx_header.Identifier != CAN_ID_CHASSIS_CMD
            || fdcan_dlc_to_bytes(rx_header.DataLength) < sizeof(ChassisCmd))
        {
            return;
        }
        ChassisCmd cmd {};
        // cmd.desired_power_limit = fdcan_rx_data[0];
        cmd.desired_power_limit = 0;
        pm.set_chassis_cmd(cmd);
    }
}

static uint8_t fdcan_dlc_to_bytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:
            return 0;
        case FDCAN_DLC_BYTES_1:
            return 1;
        case FDCAN_DLC_BYTES_2:
            return 2;
        case FDCAN_DLC_BYTES_3:
            return 3;
        case FDCAN_DLC_BYTES_4:
            return 4;
        case FDCAN_DLC_BYTES_5:
            return 5;
        case FDCAN_DLC_BYTES_6:
            return 6;
        case FDCAN_DLC_BYTES_7:
            return 7;
        case FDCAN_DLC_BYTES_8:
            return 8;
        default:
            return 0;
    }
}
