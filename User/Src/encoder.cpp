//
// Created by zhu_y on 2025/11/19.
//
#include "../Inc/encoder.h"
#include "../Inc/ask_task.h"
#include <cstring>

extern WPTMsg wpt_msg;

// 准备所有新数据
void Encoder::handle() {
    wpt_msg_update();
    memcpy(&data_, &wpt_msg, sizeof(WPTMsg));
    data_ &= 0x01FF; // 只取低9位
    parity_ = 0x00;
    for (uint8_t i = 0; i < DATA_LEN; i++) {
        parity_ ^= (data_ >> i) & 0x1; // 异或每一位
    }
    frame_ = (START_SIGN << (DATA_LEN + 1 + 1)) | (data_ << 2) | (parity_ << 1) | STOP_SIGN;
    encoder_state_ = BUSY;
}