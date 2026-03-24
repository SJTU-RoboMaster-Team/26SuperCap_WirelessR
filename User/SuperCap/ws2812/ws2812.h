//
// Created by yixin on 2023/12/18.
//

#ifndef G474_DEV_WS2812_H
#define G474_DEV_WS2812_H

#include "stdint.h"
#include "main.h"

#define WS2812_0 4
#define WS2812_1 7
#define WS2812_RESET 0
#define WS2812_RESET_TIME 50

class WS2812 {
private:
    TIM_HandleTypeDef *htim = nullptr;
    uint32_t channel = TIM_CHANNEL_1;

    uint16_t led_num = 0;
    uint8_t *led_data = nullptr;

    uint16_t displayed_num = 0; // nums of led displayed

public:
    enum Mode {
        ERR,
        STOP,
        RUNNING_ONCE,
        FINISHED,
        CIRCULAR,
    } mode = STOP;

    explicit WS2812(TIM_HandleTypeDef* htim,uint32_t channel,  uint16_t led_num);

    // only update led once
    void start_once() {
        if (mode != STOP) return;
        mode = RUNNING_ONCE;
        displayed_num = 0;
        send_buffer();
    }

    // circular update all the led
    void start_circular() {
        if (mode != STOP) return;
        mode = CIRCULAR;
        displayed_num = 0;
        send_buffer();
    }

    // set the pixel info of one led
    void set_one_led(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b) {
        if (led_index >= led_num) return;
        led_data[led_index * 3] = g;
        led_data[led_index * 3 + 1] = r;
        led_data[led_index * 3 + 2] = b;
    }

    uint8_t *get_led_data() {
        return led_data;
    }


    void send_buffer();

    void pwm_send_callback();

    void clear(){
        for (int i = 0; i < led_num; ++i) {
            led_data[i*3] = 0;
            led_data[i*3+1] = 0;
            led_data[i*3+2] = 0;
        }
    };
};

#endif //G474_DEV_WS2812_H
