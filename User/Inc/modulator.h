//
// Created by zhu_y on 2025/11/18.
//

#ifndef LOAD_MODULATION_H
#define LOAD_MODULATION_H

#include "main.h"
#include "encoder.h"
#include "../Inc/timer.h"

#define MODULATION_PERIOD 42500
#define FRAME_LEN (8+1+9+1+1)

class Modulator {
public:
    // 每个周期的逻辑电平
    // 0: 周期中不翻转 1: 周期中翻转
    // 每个周期结束后必须翻转
    enum LevelState {
        LOW,
        HIGH
    };
    LevelState last_level_state_; // level of the second half of last period
    LevelState next_level_state_; // level of the first half of this period

    enum ModulatorState {
        START,
        STOP,
        BUSY,
        IDLE,
        LOCK
    } modulator_state_ = IDLE;

    enum ModulatorStatus {
        OK,
        ERROR,
        TIMEOUT
    };

    Modulator(Encoder &encoder, FreqMeter &freq_meter):encoder_(encoder), freq_meter_(freq_meter) {
        index_ = 0;
        last_level_state_ = LOW;
        next_level_state_ = LOW;
        first_ = true;
        revert_this_period_ = false;
        half_phase_ = false; // start at first-half expected
    }

    Encoder &encoder_;
    FreqMeter &freq_meter_;
    uint8_t index_;
    uint32_t data_frame_ = 0;
    bool first_;

    // flag to indicate whether this full period requires a mid-period level revert
    bool revert_this_period_ = false;

    // phase flag: false => first half of period; true => second half of period
    bool half_phase_ = false;

    void start();
    void stop();
    void handle();        // unified handler called at half-period rate (e.g., TIM6 4kHz)
    void reset() {
        first_ = true;
        index_ = 0;
        data_frame_ = 0;
        last_level_state_ = LOW;
        next_level_state_ = LOW;
        revert_this_period_ = false;
        half_phase_ = false;
    }

    ModulatorStatus set_next_frame();
};

#endif //LOAD_MODULATION_H
