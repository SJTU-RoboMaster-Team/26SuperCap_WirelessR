//
// Created by zhu_y on 2025/11/18.
// Refactored by xhr for 4kHz GPIO load modulation & Class consolidation
//

#include "../Inc/modulator.h"
#include <cstring>

// 引入外部 MP4201 设备句柄获取数据
extern MP4201_Handle_t mp4201_dev;

// 1. 获取 MP4201 原始数据
void Modulator::update_wpt_msg() {
    wpt_msg_.switch_flag_ = mp4201_dev.wpt_switch_flag_;
    wpt_msg_.wpt_power_ = (uint8_t)(mp4201_dev.power_meas / 150.f * 255);
}

// 2. 将数据组装为带有起始位、校验位、停止位的 20 位数据帧
void Modulator::encode() {
    update_wpt_msg();

    memcpy(&encoder_data_, &wpt_msg_, sizeof(WPTMsg));
    encoder_data_ &= 0x01FF; // 只取低9位

    parity_ = 0x0;
    for (uint8_t i = 0; i < DATA_LEN; i++) {
        parity_ ^= (encoder_data_ >> i) & 0x1; // 异或每一位计算奇偶校验
    }

    // 拼接成完整的 20 位帧: [Start(9bit)] + [Data(9bit)] + [Parity(1bit)] + [Stop(1bit)]
    data_frame_ = (START_SIGN << (DATA_LEN + 1 + 1)) | (encoder_data_ << 2) | (parity_ << 1) | STOP_SIGN;
}

// 3. 设置下一帧
Modulator::ModulatorStatus Modulator::set_next_frame() {
    encode();
    return ModulatorStatus::OK;
}

void Modulator::start() {
    modulator_state_ = BUSY;
    reset();
}

void Modulator::stop() {
    modulator_state_ = STOP;
    // 停止时拉低负载调制管脚，避免消耗额外功率
    HAL_GPIO_WritePin(MOD_GPIO_PORT, MOD_PIN, GPIO_PIN_RESET);
}

// 核心调制函数：4kHz 调用（250us 周期）
void Modulator::handle() {
    if (modulator_state_ == STOP) {
        return;
    }

    if (first_) {
        first_ = false;
        bit_index_ = 0;
        half_bit_ = 0;
        if (set_next_frame() != OK) {
            stop();
            return;
        }
    }

    /*
     * QI 协议双相标记编码 (Biphase Mark Coding)
     * 基带速率 2kbps，即 1个 Bit 周期为 500us
     * handle() 以 4kHz 运行，即每 250us 触发一次 (代表半个 Bit 周期)
     */

    if (half_bit_ == 0) {
        // [阶段 1: 位的开始 (0ms)]
        // 在每一位的开始，必须翻转一次电平
        HAL_GPIO_TogglePin(MOD_GPIO_PORT, MOD_PIN);
        half_bit_ = 1;
    } else {
        // [阶段 2: 位的中间 (250us)]
        uint8_t current_bit = (data_frame_ >> (FRAME_LEN - 1 - bit_index_)) & 0x01;

        if (current_bit == 1) {
            // 如果数据位是 '1'，位中间再翻转一次
            HAL_GPIO_TogglePin(MOD_GPIO_PORT, MOD_PIN);
        }
        // 如果数据位是 '0'，位中间保持原电平，不做翻转动作

        // 进入下一位
        half_bit_ = 0;
        bit_index_++;

        // 一帧 (20 bits) 发送完成判定
        if (bit_index_ == FRAME_LEN) {
            bit_index_ = 0;
            freq_meter_.update();

            // 直接准备并装载下一帧。如果需要包间空闲期(Preamble gap)，可在这里引入额外状态延时
            if (set_next_frame() != OK) {
                stop();
            }
        }
    }
}