//
// Created by zhu_y on 2025/11/21.
//

#ifndef ASK_TASK_H
#define ASK_TASK_H
#include "../Inc/MP4201.h"

void wpt_msg_update();

// memcpy() to uint16_t => switch_flag_<<8|wpt_power_
struct WPTMsg {
    uint8_t wpt_power_; // LSB of uint16_t
    bool switch_flag_; // on: 0x01, off: 0x00
}__attribute__((packed));

#endif //ASK_TASK_H
