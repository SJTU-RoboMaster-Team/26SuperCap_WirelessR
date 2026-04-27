//
// Created by yixin on 2024/1/20.
//

#ifndef SUPER_CAP_REFEREE_COMM_H
#define SUPER_CAP_REFEREE_COMM_H

#include "stm32g4xx_hal.h"
#include "protocol.h"

class RefereeComm {
public:

    // Init UART receive 初始化，打开UART接收
    void init(void);

    // Handle data, check connection 处理数据，检查连接状态
    void handle(void);

    // Data transmit 数据发送
    void txMsg(void);

    // Data receive callback 接收中断
    void rxCallback(uint8_t *data, uint32_t size);

    // UART端口校验
//    bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

//    UART_HandleTypeDef* getHuart() const;

    int32_t unpack_one_pack(uint8_t *data, uint32_t size);

    bool connected() {
        if (HAL_GetTick() - connect_info.last_recv_tick > 1000) {
            connect_info.connected = false;
        }
        return connect_info.connected;
    }

    power_parameter_t power_parameter{};

    power_parameter_t get_parameter() const{
        return power_parameter;
    }

public:

    game_status_t game_status_;      // 比赛状态数据
    game_result_t game_result_;      // 比赛结果数据
    game_robot_HP_t game_robot_HP_;  // 机器人血量数据

    event_data_t event_data_;            // 场地事件数据
    referee_warning_t referee_warning_;  // 裁判警告数据
    dart_info_t dart_info_;              // 飞镖发射相关数据

    robot_status_t robot_status_;  // 机器人性能体系数据
    power_heat_data_t power_heat_data_;  // 实时底盘缓冲能量和射击热量数据
    robot_pos_t robot_pos_;              // 机器人位置数据
    buff_t buff_;                        // 机器人增益和底盘能量数据
    hurt_data_t hurt_data_;              // 伤害状态数据
    shoot_data_t shoot_data_;            // 实时射击数据
    projectile_allowance_t projectile_allowance_;  // 允许发弹量
    rfid_status_t rfid_status_;                    // 机器人RFID模块状态
    dart_client_cmd_t dart_client_cmd_;            // 飞镖选手端指令数据
    ground_robot_position_t ground_robot_position_;  // 地面机器人位置数据
    radar_mark_data_t radar_mark_data_;              // 雷达标记进度数据
    sentry_info_t sentry_info_;  // 哨兵自主决策信息同步
    radar_info_t radar_info_;    // 雷达自主决策信息同步

    // todo: interactive data
    custom_robot_data_t custom_robot_data_;  // 自定义控制器交互数据(另外实现)
    map_command_t map_command_;              // 选手端小地图交互数据
    uint32_t map_command_receive_tick_ = 0;
    remote_control_t remote_control_;  // 键鼠遥控数据(todo，文档混乱)
    map_robot_data_t map_robot_data_;  // 选手端小地图接收雷达数据
    custom_client_data_t
            custom_client_data_;  // 自定义控制器与选手端交互数据(todo，文档混乱)
    map_data_t map_data_;        // 选手端小地图接收哨兵数据
    custom_info_t custom_info_;  // 选手端小地图接收机器人数据
    robot_custom_data_t robot_custom_data_;  // 自定义控制器接收机器人数据

private:
    UART_HandleTypeDef *huart_;

    struct {
        uint8_t sof;
        uint16_t data_len;
        uint8_t seq;
        uint16_t cmd_id;
        uint8_t *data;
    } frame;

    uint16_t crc_err_count;
    uint16_t recv_cnt;

//    struct Tx_t {
//        uint8_t buf[REFEREE_TX_BUF_SIZE];
//        RefereeCommFrame_t frame;
//        uint32_t pack_size;
//    } tx_;
//
//    struct Rx_t {
//        Rx_t(void) : fifo(buf, REFEREE_RX_BUF_SIZE) {}
//
//        uint8_t byte[1];
//        uint8_t buf[REFEREE_RX_BUF_SIZE];
//        FIFOBuffer fifo;
//        RefereeCommFrame_t frame;
//        uint32_t expect_size;
//    } rx_;

    struct {
        uint32_t last_recv_tick;
        bool connected;
    } connect_info;

    enum MessageUnpackStep_e {
        WAIT,
        DATA_LEN,
        SEQ,
        HEADER_CRC8,
        CMD_ID,
        DATA,
        PACK_CRC16,
        READ_DATA,
    } unpack_step_;

    enum UnpackError_e {
        NO_ERROR,
        DATA_LEN_OUTRANGE,
        HEADER_CRC_FAIL,
        ID_UNDEFINED,
        DATA_LEN_MISMATCH,
        PACK_CRC_FAIL,
    } unpack_error_;

    bool apply_frame(uint16_t cmd_id, const uint8_t *data, uint16_t data_len);
};


#endif //SUPER_CAP_REFEREE_COMM_H
