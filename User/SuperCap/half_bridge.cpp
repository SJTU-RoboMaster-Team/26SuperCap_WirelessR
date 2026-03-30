//
// Created by yixin on 2024/1/18.
//

#include "half_bridge.h"

extern HRTIM_HandleTypeDef hhrtim1;

HalfBridge left(HalfBridge::TIM_C), right(HalfBridge::TIM_B);

void HalfBridge::start() {
    //// safety
    //    return;

    if (!enable) {
        enable = true;
        if (tim == TIM_C) {
            HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);
        } else {
            HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        }
    }
}

void HalfBridge::stop() {
    enable = false;
    if (tim == TIM_C) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                     HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);
    } else {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                     HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    }
}

void HalfBridge::read_config() {
    //// TODO
    // read the config generated automatically by cubeMX
}

void HalfBridge::handle() {
    if (last_state && !enable) {
        stop();
    } else if (!last_state && enable) {
        start();
    }

    if (enable) {
        if (update_by_uint) {
            duty = (float)out_compare_val / (float)period;
        } else {
            out_compare_val = (uint16_t)((float)period * duty);
        }
        // set the output compare
        if (tim == TIM_C) { ////todo
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, out_compare_val);
        } else {
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, out_compare_val);
        }
    }

    last_state = enable;
}

void HalfBridge::set_duty(float d) {
    update_by_uint = false;
    if (d <= 0)d = 0;
    duty = d;
}

void HalfBridge::set_duty(uint16_t cmp) {
    update_by_uint = true;
    out_compare_val = cmp;
}


void HalfBridge::add_duty(float d) {
    update_by_uint = false;
    duty += d;
    if (duty > 0.95)duty = 0.95;
    if (duty < 0)duty = 0;
}

void HalfBridge::add_duty(int16_t add_on) {
    update_by_uint = true;
    out_compare_val += add_on;
}

void HalfBridge::force_larger_than(float d) {
    update_by_uint = false;
    if (duty < d) {
        duty = d;
    }
}

void HalfBridge::force_larger_than(uint16_t threshold) {
    update_by_uint = true;
    if (out_compare_val < threshold) {
        out_compare_val = threshold;
    }
}

void HalfBridge::force_smaller_than(float d) {
    update_by_uint = false;
    if (duty > d) {
        duty = d;
    }
}

void HalfBridge::force_smaller_than(uint16_t threshold) {
    update_by_uint = true;
    if (out_compare_val > threshold) {
        out_compare_val = threshold;
    }
}