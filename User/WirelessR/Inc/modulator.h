//
// Created by zhu_y on 2025/11/18.
// Refactored for 4kHz GPIO load modulation & Class consolidation
//

#ifndef LOAD_MODULATION_H
#define LOAD_MODULATION_H

#include "../Inc/MP4201.h"
#include "../Inc/timer.h"
#include "../../SuperCap/power_manager.h"
#include "main.h"
#include "hrtim.h"

#define FRAME_LEN (8 + 1 + 9 + 1 + 1) // 20 bits
#define DATA_LEN 9
#define START_SIGN 0x1FE
#define STOP_SIGN 0x00
#define MODULATION_PERIOD 42500

#ifndef MOD_GPIO_PORT
    #define MOD_GPIO_PORT GPIOA
    #define MOD_PIN GPIO_PIN_8
#endif

// 整合进来的 WPTMsg 结构体
struct WPTMsg {
    uint8_t payload_ = 0;    // LSB of uint16_t
    bool switch_flag_ = false; // on: 0x01, off: 0x00
} __attribute__((packed));

class Modulator {
public:
    // symbolize state of last level, which means the level in a new period should be !level_state_ (revert level each period event)
    enum LevelState {
        LOW,
        HIGH
    };
    LevelState last_level_state_; // level of the second half of last period
    LevelState next_level_state_; // level of the first half of this period

    enum ModulatorState { START, STOP, BUSY, IDLE, LOCK } modulator_state_ = BUSY;

    enum ModulatorStatus { OK, ERROR, TIMEOUT };

    struct Encoder {
        enum Frametype { POWER, VOLTAGE } frametype_;
    } encoder_;

    // 构造函数移除了 Encoder 参数，因为已经内部整合
    Modulator(FreqMeter& freq_meter): freq_meter_(freq_meter) {
        first_ = true;
        modulator_state_ = START;
        data_frame_ = 0;
    }

    FreqMeter& freq_meter_;

    // 状态控制变量
    uint8_t index_;
    uint32_t data_frame_ = 0;
    bool first_;

    // 内部编码相关变量
    WPTMsg wpt_msg_ {};
    uint16_t encoder_data_ = 0;
    uint8_t parity_ = 0;

    void start();
    void stop();
    void handle();
    void reset() {
        first_ = true;
        index_ = 0;
        data_frame_ = 0;
        last_level_state_ = LOW;
        next_level_state_ = LOW;
    }

private:

    void update_wpt_msg();
    void encode();
    ModulatorStatus set_next_frame();
};

#endif //LOAD_MODULATION_H