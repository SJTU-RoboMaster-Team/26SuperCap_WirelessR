//
// Created by zhu_y on 2025/11/18.
//

#ifndef LOAD_MODULATION_H
#define LOAD_MODULATION_H

#include "main.h"
#include "../Inc/timer.h"

#define MODULATION_PERIOD 42500

// Encoding definitions
#define ASK_START_SIGN 0x1FE
#define ASK_STOP_SIGN 0x00
#define ASK_DATA_LEN 9
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
    LevelState second_level_state_; // 后半周期电平
    LevelState first_level_state_; // 前半周期电平

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

    // 内嵌类 Encoder
    class Encoder {
    public:
        enum EncoderState {
            START,
            STOP,
            BUSY,
            IDLE,
            FINISH
        } encoder_state_ = IDLE;

        Encoder() { encoder_state_ = STOP; frame_ = 0; }

        uint16_t data_ = 0;
        uint32_t frame_ = 0;
        uint8_t parity_ = 0;

        void reset() { frame_ = 0; }
        // void handle(); // will be implemented in modulator.cpp if needed or left unused
    } encoder_;

    // 内嵌结构体 WPT message type
    struct WPTMsg {
        uint8_t wpt_power_ = 0; // LSB
        bool switch_flag_ = false; // MSB
    } wpt_msg_;

    explicit Modulator(FreqMeter &freq_meter):freq_meter_(freq_meter) {
        index_ = 0;
        second_level_state_ = LOW;
        first_level_state_ = LOW;
        first_ = true;
        revert_this_period_ = false;
        half_phase_ = false;
    }

    FreqMeter &freq_meter_;
    uint8_t index_;
    uint32_t data_frame_ = 0;
    bool first_;

    // 周期中是否翻转标志
    bool revert_this_period_ = false;

    // 周期中标志
    bool half_phase_ = false;

    void start();
    void stop();
    void handle();        // 循环中调用
    void reset() {
        first_ = true;
        index_ = 0;
        data_frame_ = 0;
        second_level_state_ = LOW;
        first_level_state_ = LOW;
        revert_this_period_ = false;
        half_phase_ = false;
        encoder_.reset();
    }
    uint16_t wpt_msg_update();
    void set_next_frame();
};

#endif //LOAD_MODULATION_H
