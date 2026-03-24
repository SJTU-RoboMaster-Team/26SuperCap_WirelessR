//
// Created by yixin on 2024/5/7.
//

#ifndef SUPER_CAP_LCD_COMM_H
#define SUPER_CAP_LCD_COMM_H

#include "main.h"

enum LCD_CONNECTION_STATE{
    NOT_CONNECTED,
    CONNECT_TO_CALIBRATION_MODE,
    WAIT_FOR_START_CALI_CMD,
    START_CALI,
    CALI_GOING_ON,
    CALI_DONE_ASK_STORAGE,
    IDLE,
};


class LCDComm{
public:
    LCDComm(UART_HandleTypeDef* huart):huart(huart){};

    void handle();

    void rxCallback(uint8_t *data, uint32_t size);

private:

    void send_connected();
    void show_start_cali_button();
    void hide_start_button();
    void show_save_parameter_button();
    void show_parameter();


    uint32_t tick = 0;

    LCD_CONNECTION_STATE state = NOT_CONNECTED;

    UART_HandleTypeDef* huart;
};

#endif //SUPER_CAP_LCD_COMM_H
