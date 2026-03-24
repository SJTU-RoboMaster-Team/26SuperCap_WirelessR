//
// Created by yixin on 2024/3/3.
//

#ifndef SUPER_CAP_LED_INDICATOR_H
#define SUPER_CAP_LED_INDICATOR_H

#include "stm32g4xx.h"
#include "ws2812/ws2812.h"

class LED{
public:
    LED(WS2812* ws2812) : ws2812(ws2812) {};

    void handle();

    void set_r(uint8_t r){
        color.r = r;
    };
    void set_g(uint8_t g){
        color.g = g;
    };
    void set_b(uint8_t b){
        color.b = b;
    };

    void set_rgb(uint8_t r, uint8_t g, uint8_t b){
        set_r(r);
        set_g(g);
        set_b(b);
    }

    void start(){
        enable = true;
    };

    void stop(){
        enable = false;
    };

private:
    WS2812* ws2812 = nullptr;

    struct {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
    } color;

    bool enable = true;

    uint32_t tick = 0; // 100hz
    uint32_t period = 100;
    uint32_t on_tick = 20;

};

#endif //SUPER_CAP_LED_INDICATOR_H
