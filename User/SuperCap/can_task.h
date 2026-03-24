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
#endif //SUPER_CAP_CAN_TASK_H
