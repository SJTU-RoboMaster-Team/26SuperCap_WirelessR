//
// Created by zhu_y on 2025/11/18.
//
#include "../Inc/modulator.h"
#include "stm32g4xx_hal.h"
#include "main.h"

void Modulator::stop() {
    modulator_state_ = STOP;
    // ensure ASK output is low when stopped
    HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, GPIO_PIN_RESET);
}

void Modulator::start() {
    modulator_state_ = START;
}

Modulator::ModulatorStatus Modulator::set_next_frame() {
    // encoder update from power management first (an encoded data frame), then transfer the data frame to modulator.data_frame_
    encoder_.handle();
    data_frame_ = encoder_.frame_;
    return ModulatorStatus::OK;
}

void Modulator::handle() {
    // If modulation is stopped, keep pin low
    if (modulator_state_ == STOP) {
        HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, GPIO_PIN_RESET);
        return;
    }

    if (!half_phase_) {
        // 第一次调用 or 帧起始
        if (first_) {
            index_ = 0;
            first_ = false;
            ModulatorStatus status = set_next_frame();
            if (status != OK) {
                stop();
                reset();
                return;
            }
        } else if (index_ >= FRAME_LEN) {
            index_ = 0;
            ModulatorStatus status = set_next_frame();
            if (status != OK) {
                stop();
                reset();
                return;
            }
        }

        start();

        // 下周期前半段电平= !上周期后半段电平
        next_level_state_ = (last_level_state_ == LOW) ? HIGH : LOW;
        // 移位决定发送 bit
        uint32_t bit = (data_frame_ >> (FRAME_LEN - 1 - index_)) & 0x01u;

        if (bit == 0) {
            // 本周期中不翻转
            revert_this_period_ = false;
            last_level_state_ = next_level_state_;
        } else {
            // 本周期翻转: 前半周期 next_level_state_, 后半周期 !next_level_state_
            revert_this_period_ = true;
            // 周期结束时，电平为 !next_level_state_, 根据此更新 last_level_state_
            last_level_state_ = (next_level_state_ == HIGH) ? LOW : HIGH;
        }

        // Apply level for the first half of this period immediately
        HAL_GPIO_WritePin(ASK_GPIO_Port,
                          ASK_Pin,
                          (next_level_state_ == HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        index_++;

        // 更新频率计
        if (index_ == FRAME_LEN) {
            freq_meter_.update();
        }

        // 准备进入下半周期
        half_phase_ = true;
    } else {
        // second half of period: produce second-half level according to revert flag
        if (revert_this_period_) {
            // second half level is opposite of next_level_state_
            HAL_GPIO_WritePin(ASK_GPIO_Port,
                              ASK_Pin,
                              (next_level_state_ == HIGH) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        } else {
            // keep same level as first half
            HAL_GPIO_WritePin(ASK_GPIO_Port,
                              ASK_Pin,
                              (next_level_state_ == HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // 回到上半周期
        half_phase_ = false;
    }
}