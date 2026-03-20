//
// Created by zhu_y on 2025/11/19.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

#define START_SIGN 0x1FE
#define STOP_SIGN 0x00
#define DATA_LEN 9

/*
Data Frame Format:
| Start Bit (9 bit) | Data (9 bits) | Parity Bit (1 bit) | Stop Bit (1 bit) |
start: 0x1FE (8*1 + 0)
stop: 0x0
 */
class Encoder {
public:
    enum EncoderState {
        START,
        STOP,
        BUSY,
        IDLE,
        FINISH
    } encoder_state_ = IDLE;

    Encoder() {
        encoder_state_ = STOP;
        frame_ = 0;
    }

    // uint8_t index_;
    // uint8_t data_buffer_[PACKAGE_LEN];
    uint16_t data_ = 0;
    uint32_t frame_ = 0;
    uint8_t parity_ = 0;
    const uint16_t start_ = START_SIGN;
    const uint8_t stop_ = STOP_SIGN;
    void reset() {
        frame_ = 0;
    }

    void handle();
};


#endif //ENCODER_H