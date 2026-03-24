//
// Created by yixin on 2024/1/18.
//

#ifndef SUPER_CAP_CALLBACK_H
#define SUPER_CAP_CALLBACK_H

#include "main.h"

class RefreshFlag {
public:
    HAL_StatusTypeDef can_flag_;
    bool loop_flag_;
    uint32_t tick_;

    RefreshFlag(HAL_StatusTypeDef can_flag, bool loop_flag): can_flag_(can_flag), loop_flag_(loop_flag), tick_(0) {}
    void reset() {
        loop_flag_ = false; // 只监控control_loop是否进入死循环
    };
    bool isAllflagtrue() {
        // HAL_OK = 0X00U
        if (!can_flag_ && loop_flag_) return true;
        else return false;
    }
};
#endif //SUPER_CAP_CALLBACK_H
