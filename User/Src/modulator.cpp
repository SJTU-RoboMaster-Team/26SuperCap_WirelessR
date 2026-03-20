#include "../Inc/modulator.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include "../Inc/ask_task.h"

extern MP4201_Handle_t mp4201_dev;

void Modulator::stop() {
    modulator_state_ = STOP;
    HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, GPIO_PIN_RESET);
}

void Modulator::start() {
    modulator_state_ = START;
}

void Modulator::set_next_frame() {
    uint16_t packed = wpt_msg_update();

    // 解包到 wpt_msg_
    wpt_msg_.wpt_power_ = (uint8_t)(packed & 0xFFu);
    wpt_msg_.switch_flag_ = ((packed >> 8) & 0x01u) ? true : false;

    encoder_.data_ = ((uint16_t)wpt_msg_.switch_flag_ << 8) | (uint16_t)wpt_msg_.wpt_power_;
    encoder_.data_ &= 0x01FFu; // 9-bit data

    // 奇偶校验
    encoder_.parity_ = 0;
    for (uint8_t i = 0; i < ASK_DATA_LEN; ++i) {
        encoder_.parity_ ^= (encoder_.data_ >> i) & 0x1u;
    }

    // 组装帧: START(9) | DATA(9) | PARITY(1) | STOP(1)
    encoder_.frame_ = ((uint32_t)ASK_START_SIGN << (ASK_DATA_LEN + 1 + 1)) | ((uint32_t)encoder_.data_ << 2) | ((
        uint32_t)encoder_.parity_ << 1) | (uint32_t)ASK_STOP_SIGN;

    // 传递
    data_frame_ = encoder_.frame_;
}

void Modulator::handle() {
    if (modulator_state_ == STOP) {
        HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, GPIO_PIN_RESET);
        return;
    }

    if (!half_phase_) {
        if (first_) {
            index_ = 0;
            first_ = false;
            set_next_frame();
        } else if (index_ >= FRAME_LEN) {
            index_ = 0;
            set_next_frame();
        }

        start();
        first_level_state_ = (second_level_state_ == LOW) ? HIGH : LOW;
        uint32_t bit = (data_frame_ >> (FRAME_LEN - 1 - index_)) & 0x01u;

        if (bit == 0) {
            revert_this_period_ = false;
            second_level_state_ = first_level_state_;
        } else {
            revert_this_period_ = true;
            second_level_state_ = (first_level_state_ == HIGH) ? LOW : HIGH;
        }

        HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, (first_level_state_ == HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        index_++;
        if (index_ == FRAME_LEN) {
            freq_meter_.update();
        }

        half_phase_ = true;
    } else {
        if (revert_this_period_) {
            HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, (first_level_state_ == HIGH) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(ASK_GPIO_Port, ASK_Pin, (first_level_state_ == HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
        half_phase_ = false;
    }
}

uint16_t Modulator::wpt_msg_update() {
    uint8_t power = (uint8_t)(mp4201_dev.power_meas / 150.f * 255);
    uint8_t sw = mp4201_dev.wpt_switch_flag_ ? 1 : 0;
    return ((uint16_t)sw << 8) | (uint16_t)power;
}