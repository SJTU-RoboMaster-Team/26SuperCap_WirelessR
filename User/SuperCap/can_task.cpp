//
// Created by yixin on 2024/1/18.
//
#include "../config.h"
#include "can_task.h"
#include "../callback.h"

extern FDCAN_HandleTypeDef hfdcan3;

uint8_t CAN1_rx_data[8];
extern RefreshFlag refresh_flag;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan->Instance == FDCAN) {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
            FDCAN_RxHeaderTypeDef RxHeader;
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN1_rx_data);
        }
    }
}


void CAN_send_msg(const uint8_t *data, uint32_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    for (int i = 0; i < len; ++i) {
        TxData[i] = data[i];
    }

    // TxHeader.Identifier = 0x301; // 25国赛舵轮英雄
    TxHeader.Identifier = 0x103; // 其他
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    refresh_flag.can_flag_ = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}