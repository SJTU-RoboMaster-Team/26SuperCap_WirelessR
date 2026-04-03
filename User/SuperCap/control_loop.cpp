//
// Created by yixin on 2024/1/18.
//

#include "control_loop.h"
#include "buck_boost_manager.h"
#include "calibrator_estimator.h"
#include "can_task.h"
#include "flash_memory_manager.h"
#include "half_bridge.h"
#include "lcd/lcd_comm.h"
#include "led_indicator.h"
#include "math_tool/my_math.h"
#include "music/music_player.h"
#include "power_manager.h"
#include "referee_comm.h"
#include "types.h"
#include "ws2812/ws2812.h"
#include <cstdio>
#include "../callback.h"
#include "adc_sample.h"
#include "../config.h"

static uint32_t tick_40us;

extern HalfBridge left, right;

extern ADC_Buffer adc_buffer;
extern MusicPlayer music_player;
//extern LED led;

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim8;

PID vout_pid(0.005, 0, 0, 0, 0.01);
PID iout_pid(0.005, 0.0, 0.0, 0.02, 0.01); // 10uH 其软无比
//PID iout_pid(0.09, 0.05, 0.1, 0.02, 0.01);  // 10uH
//PID iout_pid(0.012, 0.100, 0.5, 0.1, 0.015);  // 10uH

//PID iout_pid(0.006, 0.080, 0.5, 0.1, 0.010);  // 10uH


// PID cap_pwr_pid(0.0003, 0, 0.000, 0, 0.05);
PID cap_pwr_pid(0.0003, 0, 0.05, 0, 0.05);
PID chassis_pwr_pid(0.008, 0, 0.01, 0, 10);
// PID referee_buffer_pid(1.5f, 0.0007, 0.00, 10.0f/0.0007f, 150);
PID referee_buffer_pid(1.5f, 0.0002, 0.01, 10000.0f, 150);


BuckBoostManager bbm(left, right, vout_pid, iout_pid);
PowerManager pm(bbm, cap_pwr_pid, chassis_pwr_pid, referee_buffer_pid);

FlashMemoryManager flash;

Calibrator cali_estimator;

LCDComm lcd_comm(&huart3);

WS2812 ws2812(&TIMER_HANDLER_WS2812_SEND,TIM_CHANNEL_1, 1);

LED led = LED(&ws2812);

extern RefereeComm referee;

CapMsg msg;

extern RefreshFlag refresh_flag;
// this loop will be run at 100 Hz
void CAN_control_loop() {
    // msg.input_voltage = (uint16_t) (bbm.filtered_v_i.val*1000);
    // msg.tick_ = refresh_flag.tick_/1000;
    msg.cap_voltage = (uint16_t)(bbm.filtered_v_c.val * 1000);
    msg.chassis_P = pm.filtered_in_pwr.val;
    // msg.cap_state = (uint8_t) pm.state;
    // msg.chassis_voltage = ((uint8_t)(bbm.v_i*1000.0f)) /100;
    CAN_send_msg((uint8_t*)&msg, 8);
}

/*
 this loop will be run at 10000 Hz
 no matter testcode or not, the loop ends up with bbm.handle()
 so loop_flag is written in end of bbm.handle()
 when referee disconnected, loop_flag is set true to keep program on
*/
float set_voltage = 6; // for debug purpose
void main_control_loop() {
    static uint32_t loop_cnt = 0;
    loop_cnt++;

    static bool first = true;
    if (first) {
        ws2812.start_circular();
        first = false;
        auto s = flash.read_calibrate_parameter();
        if (cali_estimator.judge_if_reasonable(s)) {
            cali_estimator.load_calibrate_parameter(s);
        }
    }

    bbm.update_adc_data(
        cali_estimator.calibrate_in_voltage((float)adc_buffer.v_in * V_IN_TO_FLOAT) - 0.1f,
        cali_estimator.calibrate_cap_voltage((float)adc_buffer.v_c * V_C_TO_FLOAT) - 0.2f,
        cali_estimator.calibrate_in_current((float)adc_buffer.i_in * I_IN_TO_FLOAT),
        cali_estimator.calibrate_out_current((float)adc_buffer.i_o * I_O_TO_FLOAT),
        cali_estimator.calibrate_cap_current((float)adc_buffer.i_c * I_C_TO_FLOAT)
    );

    tick_40us++;

    // bbm.stop();
    // bbm.set_voltage_loop(set_voltage);
    // bbm.start();
    // bbm.handle();
    // return;
    const bool in_calibrate_mode        = (pm.mode == PowerManager::CALIBRATE);
    const bool referee_connected        = referee.connected();
    const bool referee_disconnect_alarm = !in_calibrate_mode && !referee_connected && tick_40us > 75000;
    const bool cap_over_voltage_alarm   = (bbm.v_c > 29.0f);

    if (!in_calibrate_mode) {
        if (referee_connected) {
            if (referee.robot_status_.current_HP <= 0 || referee.robot_status_.power_management_chassis_output == false) {
                bbm.safe_disable();
                bbm.stop();
                pm.stop();
            } else {
                bbm.unlock();
            }
        } else {
            // Keep output strategy alive on referee disconnect.
            bbm.unlock();
        }
    }

    if (referee_disconnect_alarm) {
        pm.err_code |= 1;
        refresh_flag.loop_flag_ = true;
    } else {
        pm.err_code &= ~1;
    }

    if (cap_over_voltage_alarm) {
        // out v too high
        led.set_rgb(100, 0, 0);
        pm.err_code |= 2;
        bbm.stop();
    } else {
        pm.err_code &= ~2;
    }

    // Warning priority: over-voltage > referee disconnect.
    if (cap_over_voltage_alarm) {
        music_player.start_warning(L3);
    } else if (referee_disconnect_alarm) {
        music_player.start_warning(H5);
    } else {
        music_player.stop_warning();
    }

    // for debug purpose
    //    static bool first = false;
    //    if (!first) {
    //        first = true;
    //        left.set_duty(0.05f);
    //        right.set_duty(0.05f);
    //        bbm.set_current_loop(bbm.v_c,0);
    //    }

    // for debug purpose
    //    left.handle();
    //    right.handle();
    pm.handle();
    bbm.handle();

    // led 刷新频率不用很快
    if (loop_cnt % 5000 == 0) {
        switch (pm.state) {
            case PowerManager::CHARGING:
                led.set_rgb(0, 255, 0);
                break;
            case PowerManager::DISCHARGING:
                led.set_rgb(0, 0, 255);
                //            music_player.start_warning(H1);
                break;
            case PowerManager::IDLE:
                led.set_rgb(255, 255, 255);
                //            music_player.stop();
                break;
            case PowerManager::ERR:
                led.set_rgb(255, 0, 0);
                break;
        }
    }
}
