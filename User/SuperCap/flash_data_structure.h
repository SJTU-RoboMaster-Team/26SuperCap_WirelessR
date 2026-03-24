//
// Created by yixin on 2024/5/1.
//

#ifndef SUPER_CAP_FLASH_DATA_STRUCTURE_H
#define SUPER_CAP_FLASH_DATA_STRUCTURE_H

#include "stm32g4xx_hal.h"

#define PID_FLASH_STORAGE_SOF 0x3A3B
#define USER_PARAMETER_FLASH_STORAGE_SOF 0x6020
#define CALIBRATE_PARAMETER_FLASH_STORAGE_SOF 0xEA02

struct PID_FLASH_STORAGE {
    uint16_t SOF = PID_FLASH_STORAGE_SOF;
    uint16_t id = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float i_max = 0;
    float out_max = 0;
};

struct USER_PARAMETER_FLASH_STORAGE {
    uint16_t SOF = USER_PARAMETER_FLASH_STORAGE_SOF;
    uint16_t id = 0;
    float max_releasing_pwr = 0;
    float target_buffer_pwr = 0;
};

struct CALIBRATE_PARAMETER_FLASH_STORAGE {
    uint16_t SOF = CALIBRATE_PARAMETER_FLASH_STORAGE_SOF;
    uint16_t id = 0;
    float in_voltage_offset = 0;
    float in_voltage_scale = 1;

    float in_current_offset = 0;
    float in_current_scale = 1;

    float out_current_offset = 0;
    float out_current_scale = 1;

    float cap_voltage_offset = 0;
    float cap_voltage_scale = 1;

    float cap_current_offset = 0;
    float cap_current_scale = 1;

    uint32_t reserved;

};

#endif //SUPER_CAP_FLASH_DATA_STRUCTURE_H
