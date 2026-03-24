//
// Created by yixin on 2024/3/3.
//

#include "led_indicator.h"

void LED::handle() {
    if(!enable || ws2812 == nullptr){
        return;
    }
    tick++;

    uint16_t m = tick % period;
    if (m == 0){
        ws2812->set_one_led(0,color.r,color.g,color.b);
    }else if (m> on_tick){
        ws2812->set_one_led(0,0,0,0);
    }
}
