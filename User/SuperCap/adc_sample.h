//
// Created by zhu_y on 2025/7/3.
//

#ifndef ADC_SAMPLE_H
#define ADC_SAMPLE_H

#include "task_manager/timer.h"

struct ADC_Buffer{
    uint16_t  v_in = 0;
    uint16_t  v_c = 0;
    int16_t i_in = 0;
    int16_t i_o = 0;
    int16_t i_c = 0;
};

class ADC_SAMPLE {
public:
    FreqMeter freq_meter_;
    uint16_t avg_;
    uint16_t times_;
    uint16_t* buf_;

    uint16_t cal_average() {
        uint32_t sum = 0;
        for (uint16_t i = 0; i < times_; i++) {
            sum += buf_[i];
        }
        avg_ = sum / times_;
        return avg_;
    }

    ADC_SAMPLE(uint16_t times, uint16_t* buf):avg_(0),times_(times),buf_(buf) {
        freq_meter_.times_for_each_ = times;
    }
};


#endif //ADC_SAMPLE_H
