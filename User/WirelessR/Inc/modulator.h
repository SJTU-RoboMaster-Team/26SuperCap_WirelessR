//
// Created by zhu_y on 2025/11/18.
// Refactored for 4kHz GPIO load modulation & Class consolidation
//

#ifndef LOAD_MODULATION_H
#define LOAD_MODULATION_H

#include "main.h"
#include "../Inc/timer.h"
#include "../Inc/MP4201.h"

#define FRAME_LEN (8+1+9+1+1) // 20 bits
#define DATA_LEN 9
#define START_SIGN 0x1FE
#define STOP_SIGN 0x00

#ifndef MOD_GPIO_PORT
#define MOD_GPIO_PORT GPIOA
#define MOD_PIN       GPIO_PIN_8
#endif

// 整合进来的 WPTMsg 结构体
struct WPTMsg {
    uint8_t wpt_power_ = 0; // LSB of uint16_t
    bool switch_flag_ = false;  // on: 0x01, off: 0x00
}__attribute__((packed));

class Modulator {
public:
    enum ModulatorState {
        START,
        STOP,
        BUSY,
        IDLE,
        LOCK
    } modulator_state_ = BUSY;

    enum ModulatorStatus {
        OK,
        ERROR,
        TIMEOUT
    };

    // 构造函数移除了 Encoder 参数，因为已经内部整合
    Modulator(FreqMeter &freq_meter) : freq_meter_(freq_meter) {
        bit_index_ = 0;
        half_bit_ = 0;
        first_ = true;
        modulator_state_ = START;
        data_frame_ = 0;
    }

    FreqMeter &freq_meter_;

    // 状态控制变量
    uint8_t bit_index_;    // 当前进行到一帧中的第几个位 (0 ~ FRAME_LEN-1)
    uint8_t half_bit_;     // 当前是前半周期还是后半周期 (0: 前半部分, 1: 后半部分)
    uint32_t data_frame_ = 0;
    bool first_;

    // 内部编码相关变量
    WPTMsg wpt_msg_ {};
    uint16_t encoder_data_ = 0;
    uint8_t parity_ = 0;

    void start();
    void stop();
    void handle(); // 必须以 4kHz 的频率放入定时器中断调用

    void reset() {
        first_ = true;
        bit_index_ = 0;
        half_bit_ = 0;
        data_frame_ = 0;
        last_pin_state_ = GPIO_PIN_RESET;
    }

private:
    GPIO_PinState last_pin_state_ = GPIO_PIN_RESET;

    void update_wpt_msg();
    void encode();
    ModulatorStatus set_next_frame();
};

#endif //LOAD_MODULATION_H