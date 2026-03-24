//
// Created by yixin on 2024/6/2.
//

#ifndef SUPER_CAP_TIMER_H
#define SUPER_CAP_TIMER_H

#include "main.h"

class Timer {
public:
    Timer(){};

    void start_counter(uint32_t tick){
        this_tick = tick;
        mode = COUNTER;
    }

    enum MODE{
        TIMER,
        COUNTER
    }mode;

private:
    int32_t this_tick = 0;
    int32_t overflow = 0;

};

class FreqMeter {
public:
    uint32_t tick_;
    uint32_t last_tick_;
    uint32_t freq_;
    uint32_t times_;
    uint16_t times_for_each_;

    FreqMeter(uint8_t tfe = 1):times_for_each_(tfe){};

    void update() {
        tick_ = HAL_GetTick();
        if (tick_ - last_tick_ >= 1000) {
            last_tick_ = tick_;
            freq_ = times_;
            times_ = 0;
        } else times_ += times_for_each_;
    }
};

#endif //SUPER_CAP_TIMER_H
