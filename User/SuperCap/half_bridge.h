//
// Created by yixin on 2024/1/18.
//

#ifndef SUPER_CAP_HALF_BRIDGE_H
#define SUPER_CAP_HALF_BRIDGE_H

#include "main.h"

class HalfBridge {
public:
    // 通道H1L2
    enum TIM {
        TIM_C, // 左桥C，右桥B
        TIM_B,
    } tim;


    HalfBridge(TIM t) : tim(t) {

    };

    void init() {};

    void handle();

    void read_config();

    void start();

    void stop();


    void set_duty(float d);

    void set_duty(uint16_t cmp);

    void add_duty(float d);

    void add_duty(int16_t add_on);

    void force_larger_than(float d);

    void force_larger_than(uint16_t threshold);

    void force_smaller_than(float d);

    void force_smaller_than(uint16_t threshold);

    HRTIM_HandleTypeDef *hhrtim = nullptr;

    uint8_t dead_time_rising = 200, dead_time_falling = 200;
    uint16_t period = PWM_PERIOD;
    uint16_t    out_compare_val = PWM_PERIOD + 1;

    float duty = 1.1f;
    bool update_by_uint = false;
    bool enable = false;
    bool last_state = false;
};

#endif //SUPER_CAP_HALF_BRIDGE_H
