//
// Created by zhu_y on 2025/11/18.
// Refactored by xhr for 4kHz GPIO load modulation & Class consolidation
//

#include "../Inc/modulator.h"

// 引入外部 MP4201 设备句柄获取数据
extern MP4201_Handle_t mp4201_dev;
extern PowerManager pm;

// 1. 获取 MP4201 原始数据
void Modulator::update_wpt_msg() {
    wpt_msg_.switch_flag_ = mp4201_dev.wpt_switch_flag_ = pm.chassis_cmd_.desired_power_limit;
    switch (encoder_.frametype_) {
        // MSB of payload indicates what the message represents. Bit 0 stands for power, while bit 1 stands for voltage
        case Encoder::POWER: {
            wpt_msg_.payload_ = (uint8_t)(mp4201_dev.power_meas / 150.f * 127) | 0x00;
            encoder_.frametype_ = Encoder::VOLTAGE;
            break;
        }
        case Encoder::VOLTAGE: {
            wpt_msg_.payload_ = (uint8_t)(mp4201_dev.vin_meas / 50.f * 127) | 0x80;
            encoder_.frametype_ = Encoder::POWER;
            break;
        }
        default:
            break;
    }
}

// 2. 将数据组装为带有起始位、校验位、停止位的 20 位数据帧
void Modulator::encode() {
    update_wpt_msg();

    memcpy(&encoder_data_, &wpt_msg_, sizeof(WPTMsg));
    encoder_data_ &= 0x01FF; // 只取低9位

    parity_ = 0x0;
    for (uint8_t i = 0; i < PAYLOAD_LEN; i++) {
        parity_ ^= (encoder_data_ >> i) & 0x1; // 异或每一位计算奇偶校验
    }

    // 拼接成完整的 20 位帧: [Start(9bit)] + [Data(9bit)] + [Parity(1bit)] + [Stop(1bit)]
    data_frame_ = (START_SIGN << (PAYLOAD_LEN + 1 + 1)) | (encoder_data_ << 2) | (parity_ << 1) | STOP_SIGN;
}

// 3. 设置下一帧
Modulator::ModulatorStatus Modulator::set_next_frame() {
    encode();
    return ModulatorStatus::OK;
}

void Modulator::start() {
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
}

void Modulator::stop() {
    modulator_state_ = STOP;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TA1);
}

void Modulator::handle() {
    if (index_ == FRAME_LEN || first_) {
        index_ = 0;
        first_ = false;
        if (set_next_frame() != OK) {
            stop();
            return;
        }
    }

    start();

    switch (last_level_state_) {
        // end with low, start with high
        case LOW: {
            next_level_state_ = HIGH;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 0);
            break;
        }
        // end with high, start with low
        case HIGH: {
            next_level_state_ = LOW;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, 0);
            break;
        }
    }
    switch ((data_frame_ >> (FRAME_LEN - 1 - index_)) & 0x01) { // modulation output is the same sequence as data_frame_
        case 0: {
            if (next_level_state_ == HIGH) {
                __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, MODULATION_PERIOD + 1);
                // don't revert level this period
                last_level_state_ = HIGH;
            } else {
                __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, MODULATION_PERIOD + 1);
                // don't revert level this period
                last_level_state_ = LOW;
            }
            break;
        }
        case 1: {
            if (next_level_state_ == HIGH) {
                __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, MODULATION_PERIOD / 2);
                // revert level this period
                last_level_state_ = LOW;
            } else {
                __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, MODULATION_PERIOD / 2);
                // revert level this period
                last_level_state_ = HIGH;
            }
            break;
        }
    }
    index_++;
    // data_frame_ >>= 1;
    if (index_ == FRAME_LEN) {
        freq_meter_.update();
    }
}