//
// Created by yixin on 2024/1/19.
//

#ifndef SUPER_CAP_TYPES_H
#define SUPER_CAP_TYPES_H

#include "stm32g474xx.h"

struct ADC_Freq {
    uint32_t v_in;
    uint32_t v_c;
    uint32_t i_in;
    uint32_t i_o;
    uint32_t i_c;
};

struct ADC_Tick {
    uint32_t v_in;
    uint32_t v_c;
    uint32_t i_in;
    uint32_t i_o;
    uint32_t i_c;
};

#define V_IN_TO_FLOAT 1.0f / 4096.0f * 3.0f * 20.0f
#define V_C_TO_FLOAT 1.0f / 4096.0f * 3.0f * 20.0f
#define I_IN_TO_FLOAT 1.0f / 4096.0f * 3.0f / 50.0f * 1000.0f
#define I_O_TO_FLOAT 1.0f / 4096.0f * 3.0f / 50.0f * 1000.0f
#define I_C_TO_FLOAT 1.0f / 4096.0f * 3.0f / 50.0f * 1000.0f


#endif //SUPER_CAP_TYPES_H
