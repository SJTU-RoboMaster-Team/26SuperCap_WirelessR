//
// Created by yixin on 2024/1/18.
//

#ifndef SUPER_CAP_CAN_TASK_H
#define SUPER_CAP_CAN_TASK_H

#include "main.h"

void CAN_send_msg(const uint8_t *data, uint32_t len);


enum CapMsgType {
    CAP_VOLTAGE,
    WPT_VOLTAGE,
};

struct CapMsg {
    // uint16_t tick_;
    volatile uint16_t under_voltage_flag;
    // uint16_t cap_voltage;
    volatile uint16_t payload;
    volatile float chassis_P;
}__attribute__((packed));

// 底盘→超电控制包 (CAN ID 0x102)
// desired_power_limit > 0：请求进入无线充覆盖模式（具体功率限制由超电内部决定）
// desired_power_limit <= 0：取消覆盖，恢复裁判系统功率限制
struct ChassisCmd {
    uint8_t desired_power_limit; // 无线充覆盖请求标志（> 0 进入无线充，== 0 退出）
}__attribute__((packed));
#endif //SUPER_CAP_CAN_TASK_H