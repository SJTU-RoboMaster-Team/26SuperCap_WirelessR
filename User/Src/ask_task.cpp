//
// Created by zhu_y on 2025/11/21.
//
#include "../Inc/ask_task.h"
#include "../Inc/MP4201.h"

extern MP4201_Handle_t mp4201_dev;
WPTMsg wpt_msg = {3,true};
//111111110000000000000
//1111 1111 0000 0001 1110
// 1111 1111 0100 0000 1110
void wpt_msg_update() {
    wpt_msg.switch_flag_ = mp4201_dev.wpt_switch_flag_;
    wpt_msg.wpt_power_ = (uint8_t)(mp4201_dev.power_meas / 150.f * 255);
}