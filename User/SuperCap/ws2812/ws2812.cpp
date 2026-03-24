//
// Created by yixin on 2023/12/18.
//

#include <cstdlib>
#include <cstring>
#include "ws2812.h"
#include "main.h"
#include "../../config.h"

extern uint8_t pwm_buffer[PWM_BUFFER_SIZE];


void WS2812::send_buffer() {
    if(led_num ==0 || led_data == nullptr)return;

    uint16_t this_display_num = (led_num - displayed_num) > 8 ? 8 : (led_num - displayed_num);

    uint16_t color_idx = 0;
    uint16_t bit_idx = 0;

    while (color_idx<this_display_num*3) {
        uint8_t color = led_data[displayed_num*3+color_idx];
        for (int i = 0; i < 8; i++) {
            if (color & 0x80) {
                pwm_buffer[bit_idx] = WS2812_1;
            } else {
                pwm_buffer[bit_idx] = WS2812_0;
            }
            color <<= 1;
            bit_idx++;
            if (bit_idx>=PWM_BUFFER_SIZE) break;
        }
        color_idx++;
    }

    if(displayed_num+this_display_num>=led_num){
        while (bit_idx<PWM_BUFFER_SIZE) {
            pwm_buffer[bit_idx] = WS2812_RESET;
            bit_idx++;
        }
        HAL_TIM_PWM_Start_DMA(htim, channel, (uint32_t *)pwm_buffer,PWM_BUFFER_SIZE);
    } else{
        HAL_TIM_PWM_Start_DMA(htim, channel, (uint32_t *)pwm_buffer,8*24);

    }


    displayed_num += this_display_num;
}

void WS2812::pwm_send_callback() {
    if (mode == RUNNING_ONCE) {
        if (displayed_num >= led_num) {
            mode = FINISHED;
            HAL_TIM_PWM_Stop_DMA(htim, channel);
            return;
        } else{
            send_buffer();
        }
    } else if (mode == CIRCULAR) {
        if (displayed_num >= led_num) {
            displayed_num = 0;
        }
        send_buffer();
    }

}

WS2812::WS2812(TIM_HandleTypeDef* htim,uint32_t channel,  uint16_t led_num_):htim(htim),channel(channel),led_num(led_num_){
//        led_data = new uint8_t[led_num * 3]; // g, r, b
    if (htim == nullptr){
        mode = ERR;
        return;
    }
    if (led_num + 7 % 8 >=5){
        // for some reason, for led nums like 8x+6 or 7 or 8 can't work properly, just make sure these nums won't appear
        led_num+=3;
    }
    auto tmp = calloc(led_num*3, sizeof(uint8_t));
    if (tmp == nullptr) {
        mode = ERR;
        return;
    }
    led_data = (uint8_t *) tmp;
}
