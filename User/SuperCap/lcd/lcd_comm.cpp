//
// Created by yixin on 2024/5/7.
//

#include <cstring>
#include <cstdlib>
#include "lcd_comm.h"
#include "../power_manager.h"

extern PowerManager pm;



void LCDComm::handle() {
    tick++;

    switch (state) {

        case NOT_CONNECTED:
            break;
        case CONNECT_TO_CALIBRATION_MODE:
            state = WAIT_FOR_START_CALI_CMD;
            show_start_cali_button();
            break;
        case WAIT_FOR_START_CALI_CMD:
            break;
        case START_CALI:
            hide_start_button();
            state = CALI_GOING_ON;
            break;
        case CALI_GOING_ON:
            pm.set_calibrate_mode();
            state = CALI_DONE_ASK_STORAGE;
            break;
        case CALI_DONE_ASK_STORAGE:
            show_parameter();
            state = IDLE;
            break;
        case IDLE:
            break;
    }

}

void LCDComm::rxCallback(uint8_t *data, uint32_t size) {
    if (data[0] == 0xff) {

    } else if (data[0] == 0x50) {
        if (size==2 && data[1] == 0xcc){
            state = CONNECT_TO_CALIBRATION_MODE;
            send_connected();
        }
    } else if (data[0] == 0x5a){
        if (size == 2 && data[1] == 0x33){
            state = START_CALI;

        }
    }
}


void LCDComm::send_connected() {
    static uint8_t command[21];
    memcpy(command,"tc.txt=\"connected\"",18);
    command[18] = 0xff;
    command[19] = 0xff;
    command[20] = 0xff;
    HAL_UART_Transmit_IT(huart,command, sizeof(command));
}

void LCDComm::show_start_cali_button() {
    static uint8_t command[12];
    memcpy(command,"bsc.y=190",9);
    command[9] = 0xff;
    command[10] = 0xff;
    command[11] = 0xff;
    HAL_UART_Transmit_IT(huart,command, 12);
}

void LCDComm::hide_start_button() {
    static uint8_t command[12];
    memcpy(command,"bsc.y=300",9);
    command[9] = 0xff;
    command[10] = 0xff;
    command[11] = 0xff;
    HAL_UART_Transmit_IT(huart,command, 12);
}

void LCDComm::show_save_parameter_button(){
    static uint8_t command[14];
    memcpy(command,"bsave.y=300",11);
    command[11] = 0xff;
    command[12] = 0xff;
    command[13] = 0xff;
    HAL_UART_Transmit_IT(huart,command, 14);
}

void LCDComm::show_parameter(){

    uint8_t show_text[8+(4*2)*5];
    static uint8_t command[sizeof show_text + 3];

    memcpy(show_text,"tc.txt=\"",8);
    memcpy(show_text+8,"123 456 ",8);
    memcpy(show_text+8+8,"123 456 ",8);
    memcpy(show_text+8+8*2,"123 456 ",8);
    memcpy(show_text+8+8*3,"123 456 ",8);
    memcpy(show_text+8+8*4,"123 456\"",8);
    command[sizeof show_text] = 0xff;
    command[sizeof show_text+1] = 0xff;
    command[sizeof show_text+2] = 0xff;
    HAL_UART_Transmit_IT(huart,command, sizeof command);
}

