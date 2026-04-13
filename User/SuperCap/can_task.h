//
// Created by yixin on 2024/1/18.
//

#ifndef SUPER_CAP_CAN_TASK_H
#define SUPER_CAP_CAN_TASK_H

#include "main.h"

void CAN_send_msg(const uint8_t *data, uint32_t len);


struct CapMsg {
    // uint16_t tick_;
    uint16_t under_voltage_flag;
    uint16_t cap_voltage;
    float chassis_P;
}__attribute__((packed));

// 底盘→超电控制包 (CAN ID 0x102)
// desired_power_limit > 0：底盘请求该功率上限，超出裁判系统限制的部分由超电放电补偿
// desired_power_limit = 0：取消覆盖，恢复裁判系统功率限制
struct ChassisCmd {
    float desired_power_limit; // 底盘请求的功率上限 (W)
}__attribute__((packed));
#endif //SUPER_CAP_CAN_TASK_H
