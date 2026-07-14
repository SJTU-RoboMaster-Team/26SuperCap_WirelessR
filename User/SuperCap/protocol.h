/**
 ******************************************************************************
 * @file    referee_protocol.cpp/h
 * @brief   Referee communication(UART) protocol. 裁判系统通信协议
 ******************************************************************************
 * @version V2.0.0
 * @date 2026.06.26
 * @protocol
 * 1. UART config 串口配置
 *    波特率115200，8位数据位，1位停止位，无硬件流控，无校验位
 * 2. frame format 数据帧格式
 *    header(5-byte) + cmd_id(2-byte) + data(n-byte) + tail(2-byte, CRC16)
 *    header: SOF(1-byte, 0xA5) + data_len(2-byte) + seq(1-byte) + CRC8(1-byte)
 *    data: n-byte
 *    tail: CRC16(2-byte, 整包校验)
 ******************************************************************************
 * Copyright (c) 2025 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

#define REFEREE_COMM_FRAME_MAX_SIZE (5 + 2 + 127 + 2)
#define REFEREE_COMM_SOF 0xA5  // start of frame

// 裁判系统通信帧头
typedef struct RefereeCommFrameHeader {
    uint8_t sof;
    uint16_t data_len;
    uint8_t seq;
    uint8_t crc8;
} __packed RefereeCommFrameHeader_t;

// 裁判系统通信一帧数据中data以外的内容
typedef struct RefereeCommFrame {
    RefereeCommFrameHeader_t header;
    uint16_t cmd_id;
    // uint8_t data[n];
    uint16_t crc16;
} __packed RefereeCommFrame_t;

// cmd_id
typedef enum RefereeCMDID {
    GAME_STATUS_ID = 0x0001,
    GAME_RESULT_ID = 0x0002,
    GAME_ROBOT_HP_ID = 0x0003,
    EVENT_DATA_ID = 0x0101,
    REFEREE_WARNING_ID = 0x0104,
    DART_INFO_ID = 0x0105,
    ROBOT_STATUS_ID = 0x0201,
    POWER_HEAT_DATA_ID = 0x0202,
    ROBOT_POS_ID = 0x0203,
    BUFF_ID = 0x0204,
    HURT_DATA_ID = 0x0206,
    SHOOT_DATA_ID = 0x0207,
    PROJECTILE_ALLOWANCE_ID = 0x0208,
    RFID_STATUS_ID = 0x0209,
    DART_CLIENT_CMD_ID = 0x020A,
    GROUND_ROBOT_POSITION_ID = 0x020B,
    RADAR_MARK_DATA_ID = 0x020C,
    SENTRY_INFO_ID = 0x020D,
    RADAR_INFO_ID = 0x020E,
    ROBOT_INTERACTION_DATA_ID = 0x0301,
    CUSTOM_ROBOT_DATA_ID = 0x0302,
    MAP_COMMAND_ID = 0x0303,
    REMOTE_CONTROL_ID = 0x0304,
    MAP_ROBOT_DATA_ID = 0x0305,
    CUSTOM_CLIENT_DATA_ID = 0x0306,
    MAP_DATA_ID = 0x0307,
    CUSTOM_INFO_ID = 0x0308,
    ROBOT_CUSTOM_DATA_ID = 0x0309,
} RefereeCMDID_e;

// 0x0001 11 比赛状态数据，固定以1Hz频率发送 服务器→全体机器人
typedef struct {
    // 比赛类型
    // • 1：RoboMaster 机甲大师超级对抗赛
    // • 2：RoboMaster 机甲大师高校单项赛
    // • 3：ICRA RoboMaster 高校人工智能挑战赛
    // • 4：RoboMaster 机甲大师高校联盟赛3V3 对抗
    // • 5：RoboMaster 机甲大师高校联盟赛步兵对抗
    uint8_t game_type : 4;
    // 当前比赛阶段
    // • 0：未开始比赛
    // • 1：准备阶段
    // • 2：十五秒裁判系统自检阶段
    // • 3：五秒倒计时
    // • 4：比赛中
    // • 5：比赛结算中
    uint8_t game_progress : 4;
    // 当前阶段剩余时间，单位：秒
    uint16_t stage_remain_time;
    // UNIX 时间，当机器人正确连接到裁判系统的NTP 服务器后生效
    uint64_t SyncTimeStamp;
} __packed game_status_t;

// 0x0002 1 比赛结果数据，比赛结束触发发送 服务器→全体机器人
typedef struct {
    // • 0：平局
    // • 1：红方胜利
    // • 2：蓝方胜利
    uint8_t winner;
} __packed game_result_t;

// 0x0003 20 机器人血量数据，固定以3Hz频率发送 服务器→全体机器人
typedef struct {
    // 己方1号英雄机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t ally_1_robot_HP;
    // 己方2号工程机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t ally_2_robot_HP;
    // 己方3号步兵机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t ally_3_robot_HP;
    // 己方4号步兵机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t ally_4_robot_HP;
    // 己方全队总伤害与对方全队总伤害之差
    int16_t damage_difference;
    // 己方7号哨兵机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t ally_7_robot_HP;
    // 己方前哨站血量
    uint16_t ally_outpost_HP;
    // 己方基地血量
    uint16_t ally_base_HP;
    // 对方前哨站血量
    uint16_t enemy_outpost_HP;
    // 对方基地血量
    uint16_t enemy_base_HP;
} __packed game_robot_HP_t;

// 0x0101 4 场地事件数据，固定以1Hz频率发送 服务器→己方全体机器人
typedef struct {
    // 0：未占领/未激活
    // 1：已占领/已激活
    // • bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领
    // • bit 1：己方与兑换区重叠的补给区占领状态，1为已占领
    // • bit 2：己方补给区的占领状态，1为已占领（仅 RMUL适用）
    // • bit 3：己方小能量机关的激活状态，1为已激活
    // • bit 4：己方大能量机关的激活状态，1为已激活
    // • bit 5-6：己方中央高地的占领状态，1为被己方占领，2为被对方占领
    // • bit 7-8：己方梯形高地的占领状态，1为已占领
    // • bit 9-17：对方飞镖最后一次击中己方前哨站或基地的时间
    //     （0-420，开局默认为0）
    // • bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
    //   • 1 为击中前哨站，
    //   • 2 为击中基地固定目标，
    //   • 3 为击中基地随机固定目标，
    //   • 4 为击中基地随机移动目标
    // • bit 21-22：中心增益点的占领状态，（仅RMUL适用）
    //   • 0 为未被占领，
    //   • 1 为被己方占领，
    //   • 2 为被对方占领，
    //   • 3 为被双方占领。
    // • bit 23-31：保留位
    uint32_t event_data;
} __packed event_data_t;

// 0x0104 3 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送
// 服务器→被判罚方全体机器人
typedef struct {
    // 己方最后一次受到判罚的等级：
    // • 1：双方黄牌
    // • 2：黄牌
    // • 3：红牌
    // • 4：判负
    uint8_t level;
    // • 己方最后一次受到判罚的违规机器人ID。
    //   （如红1机器人ID为1，蓝1机器人ID为101）
    // • 判负和双方黄牌时，该值为0
    uint8_t offending_robot_id;
    // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为0。）
    uint8_t count;
} __packed referee_warning_t;

// 0x0105 3 飞镖发射相关数据，固定以1Hz频率发送 服务器→己方全体机器人
typedef struct {
    // 己方飞镖发射剩余时间，单位：秒
    uint8_t dart_remaining_time;
    // bit 0-2：
    //   最近一次己方飞镖击中的目标，开局默认为0，
    //   • 1 为击中前哨站，
    //   • 2 为击中基地固定目标，
    //   • 3 为击中基地随机固定目标，
    //   • 4 为击中基地随机移动目标
    // bit 3-5：
    //   对方最近被击中的目标累计被击中计次数，开局默认为0，至多为4
    // bit 6-7：
    //   飞镖此时选定的击打目标，
    //   • 开局默认或未选定/选定前哨站时为0，
    //   • 选中基地固定目标为1，
    //   • 选中基地随机固定目标为2，
    //   • 选中基地随机移动目标为3
    // bit 8-15：保留
    uint16_t dart_info;
} __packed dart_info_t;

// 0x0201 13 机器人性能体系数据，固定以10Hz频率发送 主控模块→对应机器人
typedef struct {
    // 本机器人ID
    uint8_t robot_id;
    // 机器人等级
    uint8_t robot_level;
    // 机器人当前血量
    uint16_t current_HP;
    // 机器人血量上限
    uint16_t maximum_HP;
    // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_cooling_value;
    // 机器人射击热量上限
    uint16_t shooter_barrel_heat_limit;
    // 机器人底盘功率上限
    uint16_t chassis_power_limit;
    // 机器人射击初速度上限
    float bullet_speed_limit;
    // gimbal口输出，0为无输出，1为 24V输出
    uint8_t power_management_gimbal_output : 1;
    // chassis口输出，0为无输出，1为24V输出
    uint8_t power_management_chassis_output : 1;
    // shooter口输出，0为无输出，1为24V输出
    uint8_t power_management_shooter_output : 1;
} __packed robot_status_t;

// 0x0202 16 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送
// 主控模块→对应机器人
typedef struct {
    // 保留位
    uint16_t reserved_1;
    // 保留位
    uint16_t reserved_2;
    // 保留位
    float reserved_3;
    // 缓冲能量（单位：J）
    uint16_t buffer_energy;
    // 第1个17mm发射机构的射击热量
    uint16_t shooter_17mm_1_barrel_heat;
    // 第2个17mm发射机构的射击热量
    uint16_t shooter_17mm_2_barrel_heat;
    // 42mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;
} __packed power_heat_data_t;

// 0x0203 16 机器人位置数据，固定以1Hz频率发送 主控模块→对应机器人
typedef struct {
    // 本机器人位置x坐标，单位：m
    float x;
    // 本机器人位置y坐标，单位：m
    float y;
    // 本机器人测速模块的朝向，单位：度。正北为0度
    float angle;
} __packed robot_pos_t;

// 0x0204 7 机器人增益和底盘能量数据，固定以3Hz频率发送 服务器→对应机器人
typedef struct {
    // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint8_t recovery_buff;
    // 机器人射击热量冷却倍率（直接值，值为5表示5倍冷却）
    uint8_t cooling_buff;
    // 机器人防御增益（百分比，值为50表示50%防御增益）
    uint8_t defence_buff;
    // 机器人负防御增益（百分比，值为30表示-30%防御增益）
    uint8_t vulnerability_buff;
    // 机器人攻击增益（百分比，值为50表示50%攻击增益）
    uint16_t attack_buff;
    // bit 0-4：机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例，
    //          仅在机器人剩余能量小于50%时反馈，其余默认反馈0x32。
    // • bit 0：在剩余能量≥50%时为1，其余情况为0
    // • bit 1：在剩余能量≥30%时为1，其余情况为0
    // • bit 2：在剩余能量≥15%时为1，其余情况为0
    // • bit 3：在剩余能量≥5%时为1，其余情况为0
    // • bit 4：在剩余能量≥1%时为1，其余情况为0
    uint8_t remaining_energy;
} __packed buff_t;

// 0x0206 1 伤害状态数据，伤害发生后发送 主控模块→对应机器人
// 注：0x0206 的受伤害情况为机器人裁判系统本地判定，即时发送，但实际是否受到对
//    应伤害受规则条例影响，请以服务器最终判定为准。
typedef struct {
    // 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
    // 该4 bit组成的数值为装甲模块或测速模块的ID编号；
    // 当其他原因导致扣血时，该数值为0
    uint8_t armor_id : 4;
    // 血量变化类型
    // • 0：装甲模块被弹丸攻击导致扣血
    // • 1：裁判系统重要模块离线导致扣血
    // • 2：射击初速度超限导致扣血
    // • 3：射击热量超限导致扣血
    // • 4：底盘功率超限导致扣血
    // • 5：装甲模块受到撞击导致扣血
    uint8_t HP_deduction_reason : 4;
} __packed hurt_data_t;

// 0x0207 7 实时射击数据，弹丸发射后发送 主控模块→对应机器人
typedef struct {
    // 弹丸类型：
    // • 1：17mm弹丸
    // • 2：42mm弹丸
    uint8_t bullet_type;
    // 发射机构ID：
    // • 1：第1个17mm发射机构
    // • 2：第2个17mm发射机构
    // • 3：42mm发射机构
    uint8_t shooter_number;
    // 弹丸射速（单位：Hz）
    uint8_t launching_frequency;
    // 弹丸初速度（单位：m/s）
    float initial_speed;
} __packed shoot_data_t;

// 0x0208 6 允许发弹量，固定10Hz频率发送 服务器→己方英雄、步兵、哨兵、空中机器人
typedef struct {
    // 17mm弹丸允许发弹量
    uint16_t projectile_allowance_17mm;
    // 42mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;
    // 剩余金币数量
    uint16_t remaining_gold_coin;
} __packed projectile_allowance_t;

// 0x0209 4 机器人RFID模块状态，固定以3Hz频率发送
// 服务器→己方装有RFID模块的机器人
typedef struct {
    // bit 位值为1/0的含义：是否已检测到该增益点RFID
    // • bit 0：己方基地增益点
    // • bit 1：己方中央高地增益点
    // • bit 2：对方中央高地增益点
    // • bit 3：己方梯形高地增益点
    // • bit 4：对方梯形高地增益点
    // • bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    // • bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    // • bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    // • bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    // • bit 9：己方地形跨越增益点（中央高地下方）
    // • bit 10：己方地形跨越增益点（中央高地上方）
    // • bit 11：对方地形跨越增益点（中央高地下方）
    // • bit 12：对方地形跨越增益点（中央高地上方）
    // • bit 13：己方地形跨越增益点（公路下方）
    // • bit 14：己方地形跨越增益点（公路上方）
    // • bit 15：对方地形跨越增益点（公路下方）
    // • bit 16：对方地形跨越增益点（公路上方）
    // • bit 17：己方堡垒增益点
    // • bit 18：己方前哨站增益点
    // • bit 19：己方与兑换区不重叠的补给区/RMUL补给区
    // • bit 20：己方与兑换区重叠的补给区
    // • bit 21：己方大资源岛增益点
    // • bit 22：对方大资源岛增益点
    // • bit 23：中心增益点（仅 RMUL 适用）
    // • bit 24-31：保留
    // 注：所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID卡，对应值也为0。
    uint32_t rfid_status;
} __packed rfid_status_t;

// 0x020A 6 飞镖选手端指令数据，固定以3Hz频率发送 服务器→己方飞镖机器人
typedef struct {
    // 当前飞镖发射站的状态：
    // • 1：关闭
    // • 2：正在开启或者关闭中
    // • 0：已经开启
    uint8_t dart_launch_opening_status;
    // 保留位
    uint8_t reserved;
    // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0。
    uint16_t target_change_time;
    // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0。
    uint16_t latest_launch_cmd_time;
} __packed dart_client_cmd_t;

// 0x020B 40 地面机器人位置数据，固定以1Hz频率发送 服务器→己方哨兵机器人
typedef struct {
    // 己方英雄机器人位置x轴坐标，单位：m
    float hero_x;
    // 己方英雄机器人位置y轴坐标，单位：m
    float hero_y;
    // 己方工程机器人位置x轴坐标，单位：m
    float engineer_x;
    // 己方工程机器人位置y轴坐标，单位：m
    float engineer_y;
    // 己方3号步兵机器人位置x轴坐标，单位：m
    float standard_3_x;
    // 己方3号步兵机器人位置y轴坐标，单位：m
    float standard_3_y;
    // 己方4号步兵机器人位置x轴坐标，单位：m
    float standard_4_x;
    // 己方4号步兵机器人位置y轴坐标，单位：m
    float standard_4_y;
    // 保留位
    float reserved_1;
    // 保留位
    float reserved_2;
} __packed ground_robot_position_t;

// 0x020C 2 雷达标记进度数据，固定以1Hz频率发送 服务器→己方雷达机器人
typedef struct {
    // 对方机器人：在对应机器人被标记进度≥100时发送1，被标记进度<100时发送0。
    // 己方机器人：在对应机器人，被标记进度≥50时发送1，被标记进度<50时发送0。
    // • bit 0：对方1号英雄机器人易伤情况
    // • bit 1：对方2号工程机器人易伤情况
    // • bit 2：对方3号步兵机器人易伤情况
    // • bit 3：对方4号步兵机器人易伤情况
    // • bit 4：对方空中机器人特殊标识情况
    // • bit 5：对方哨兵机器人易伤情况
    // • bit 6：己方1号英雄机器人特殊标识情况
    // • bit 7：己方2号工程机器人特殊标识情况
    // • bit 8：己方3号步兵机器人特殊标识情况
    // • bit 9：己方4号步兵机器人特殊标识情况
    // • bit 10：己方空中机器人特殊标识情况
    // • bit 11：己方哨兵机器人特殊标识情况
    // • bit 12：对方空中机器人是否被己方雷达激光瞄准
    // • bit 13：对方空中机器人当前是否处于被反制状态
    // • bit 14：己方空中机器人是否被对方雷达激光瞄准
    // • bit 15：己方空中机器人当前是否处于被反制状态
    uint16_t mark_progress;
} __packed radar_mark_data_t;

typedef struct {
    // bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0，在
    //   哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的
    //   允许发弹量值。
    // bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为0，在哨兵
    //   机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允
    //   许发弹量的次数。
    // bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为0，在哨兵机器人
    //   成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数。
    // bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，
    //   否则为0。
    // bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，
    //   否则为0。
    // bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
    // bit 31：保留。
    uint32_t sentry_info;
    // bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0。
    // bit 1-11：队伍17mm允许发弹量的剩余可兑换数。
    // bit 12-13：哨兵当前姿态，1为进攻姿态，2为防御姿态，3为移动姿态。
    // bit 14：己方能量机关是否能够进入正在激活状态，1为当前可激活。
    // bit 15：哨兵当前姿态是否为强化姿态。
    uint16_t sentry_info_2;
    // bit 0-7：哨兵进攻姿态弱化前剩余可持续时长（单位秒，下同）
    // bit 8-15：哨兵防御姿态弱化前剩余可持续时长
    // bit 16-23：哨兵移动姿态弱化前剩余可持续时长
    // bit 24-31：保留位
    // bit 32-39：哨兵强化进攻姿态剩余可持续时长
    // bit 40-47：哨兵强化防御姿态剩余可持续时长
    // bit 48-55：哨兵强化移动姿态剩余可持续时长
    // bit 56-63：保留位
    uint64_t sentry_info_3;
} __packed sentry_info_t;

// 0x020E 1 雷达自主决策信息同步，固定以1Hz频率发送 服务器→己方雷达机器人
typedef struct {
    // bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为0，
    //   数值为雷达拥有触发双倍易伤的机会，至多为2
    // bit 2：对方是否正在被触发双倍易伤
    // • 0：对方未被触发双倍易伤
    // • 1：对方正在被触发双倍易伤
    // bit 3-7：保留
    uint8_t radar_info;
} __packed radar_info_t;

// 机器人间交互数据

// 0x0301 118 机器人交互数据，发送方触发发送，频率上限为30Hz
typedef struct {
    // 需为开放的子内容ID
    uint16_t data_cmd_id;
    // 需与自身ID匹配，ID编号详见附录
    uint16_t sender_id;
    // 接收者ID
    // • 仅限己方通信
    // • 需为规则允许的多机通讯接收者
    // • 若接收者为选手端，则除雷达系统外，仅可发送至发送者对应的选手端。
    //   雷达系统可发送至任意己方选手端。
    // • ID编号详见附录
    uint16_t receiver_id;
    // 内容数据段，x最大为112
    // uint8_t user_data[x];
} __packed robot_interaction_data_t;

// 机器人ID（机器人->机器人）
typedef enum RobotID {
    RED_HERO = 0x0001,
    RED_ENGINEER = 0x0002,
    RED_STANDARD3 = 0x0003,
    RED_STANDARD4 = 0x0004,
    RED_STANDARD5 = 0x0005,
    RED_AERIAL = 0x0006,
    RED_SENTRY = 0x0007,
    RED_DART = 0x0008,
    RED_RADAR = 0x0009,
    BLUE_HERO = 0x0101,
    BLUE_ENGINEER = 0x0102,
    BLUE_STANDARD3 = 0x0103,
    BLUE_STANDARD4 = 0x0104,
    BLUE_STANDARD5 = 0x0105,
    BLUE_AERIAL = 0x0106,
    BLUE_SENTRY = 0x0107,
    BLUE_DART = 0x0108,
    BLUE_RADAR = 0x0109,
} RobotID_e;

// 客户端ID（机器人->ui）
typedef enum ClientID {
    RED_HERO_CLIENT = 0x0101,
    RED_ENGINEER_CLIENT = 0x0102,
    RED_STANDARD3_CLIENT = 0x1003,
    RED_STANDARD4_CLIENT = 0x1004,
    RED_STANDARD5_CLIENT = 0x1005,
    RED_AERIAL_CLIENT = 0x0106,
    BLUE_HERO_CLIENT = 0x0165,
    BLUE_ENGINEER_CLIENT = 0x0166,
    BLUE_STANDARD3_CLIENT = 0x0167,
    BLUE_STANDARD4_CLIENT = 0x0168,
    BLUE_STANDARD5_CLIENT = 0x0169,
    BLUE_AERIAL_CLIENT = 0x016A,
} ClientID_e;

// UI

// 客户端删除图形
typedef struct {
    uint8_t operate_tpye;
    uint8_t layer;
} __packed interaction_layer_delete_t;

// 图形数据
typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} __packed interaction_figure_t;

// 客户端绘制二个图形
typedef struct {
    interaction_figure_t interaction_figure[2];
} __packed interaction_figure_2_t;

// 客户端绘制五个图形
typedef struct {
    interaction_figure_t interaction_figure[5];
} __packed interaction_figure_3_t;

// 客户端绘制七个图形
typedef struct {
    interaction_figure_t interaction_figure[7];
} __packed interaction_figure_4_t;

// 客户端绘制字符
typedef struct {
    interaction_figure_t grapic_data_struct;
    uint8_t data[30];
} __packed client_custom_character_t;

// 0x0302 30 自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz
// 自定义控制器→选手端图传连接的机器人 图传链路
typedef struct {
    // 自定义数据
    // uint8_t data[x];
} __packed custom_robot_data_t;

// 0x0303 15 选手端小地图交互数据，选手端触发发送
// 选手端点击→服务器→发送方选择的己方机器人
typedef struct {
    // 目标位置x轴坐标，单位m
    float target_position_x;
    // 目标位置y轴坐标，单位m
    float target_position_y;
    // 云台手按下的键盘按键通用键值
    uint8_t cmd_keyboard;
    // 对方机器人ID
    uint8_t target_robot_id;
    // 信息来源ID
    uint16_t cmd_source;
} __packed map_command_t;

// 0x0304 12 键鼠遥控数据，固定30Hz频率发送
// 选手端→选手端图传连接的机器人 图传链路
typedef struct {
    // 鼠标x轴移动速度，负值标识向左移动
    int16_t mouse_x;
    // 鼠标y轴移动速度，负值标识向下移动
    int16_t mouse_y;
    // 鼠标滚轮移动速度，负值标识向后滚动
    int16_t mouse_z;
    // 鼠标左键是否按下：0为未按下；1为按下
    int8_t left_button_down;
    // 鼠标右键是否按下：0为未按下，1为按下
    int8_t right_button_down;
    // 键盘按键信息，每个bit对应一个按键，0为未按下，1为按下：
    // • bit 0：W键
    // • bit 1：S 键
    // • bit 2：A 键
    // • bit 3：D 键
    // • bit 4：Shift 键
    // • bit 5：Ctrl 键
    // • bit 6：Q 键
    // • bit 7：E 键
    // • bit 8：R 键
    // • bit 9：F 键
    // • bit 10：G 键
    // • bit 11：Z 键
    // • bit 12：X 键
    // • bit 13：C 键
    // • bit 14：V 键
    // • bit 15：B 键
    uint16_t keyboard_value;
    // 保留位
    uint16_t reserved;
} __packed remote_control_t;

// 0x0305 24 选手端小地图接收雷达数据，频率上限为5Hz 雷达→服务器→己方所有选手端
typedef struct {
    // 对方英雄机器人x位置坐标，单位：cm
    uint16_t opponent_hero_position_x;
    // 对方英雄机器人y位置坐标，单位：cm
    uint16_t opponent_hero_position_y;
    // 对方工程机器人x位置坐标，单位：cm
    uint16_t opponent_engineer_position_x;
    // 对方工程机器人y位置坐标，单位：cm
    uint16_t opponent_engineer_position_y;
    // 对方3号步兵机器人x位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_x;
    // 对方3号步兵机器人y位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_y;
    // 对方4号步兵机器人x位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_x;
    // 对方4号步兵机器人y位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_y;
    // 对方 6 号空中机器人 x 位置坐标, 单位：cm
    uint16_t opponent_aerial_position_x;
    // 对方 6 号空中机器人 y 位置坐标, 单位：cm
    uint16_t opponent_aerial_position_y;
    // 对方哨兵机器人x位置坐标，单位：cm
    uint16_t opponent_sentry_position_x;
    // 对方哨兵机器人y位置坐标，单位：cm
    uint16_t opponent_sentry_position_y;
    // 己方英雄机器人x位置坐标，单位：cm
    uint16_t ally_hero_position_x;
    // 己方英雄机器人y位置坐标，单位：cm
    uint16_t ally_hero_position_y;
    // 己方工程机器人x位置坐标，单位：cm
    uint16_t ally_engineer_position_x;
    // 己方工程机器人y位置坐标，单位：cm
    uint16_t ally_engineer_position_y;
    // 己方3号步兵机器人x位置坐标，单位：cm
    uint16_t ally_infantry_3_position_x;
    // 己方3号步兵机器人y位置坐标，单位：cm
    uint16_t ally_infantry_3_position_y;
    // 己方4号步兵机器人x位置坐标，单位：cm
    uint16_t ally_infantry_4_position_x;
    // 己方4号步兵机器人y位置坐标，单位：cm
    uint16_t ally_infantry_4_position_y;
    // 己方 6 号空中机器人 x 位置坐标, 单位：cm
    uint16_t ally_aerial_position_x;
    // 己方 6 号空中机器人 y 位置坐标, 单位：cm
    uint16_t ally_aerial_position_y;
    // 己方哨兵机器人x位置坐标，单位：cm
    uint16_t ally_sentry_position_x;
    // 己方哨兵机器人y位置坐标，单位：cm
    uint16_t ally_sentry_position_y;
} __packed map_robot_data_t;

// 0x0306 8 自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz
// 自定义控制器→选手端
typedef struct {
    // 键盘键值：
    // • bit 0-7：按键1键值
    // • bit 8-15：按键2键值
    uint16_t key_value;
    // 鼠标X轴像素位置
    uint16_t x_position : 12;
    // 鼠标左键状态
    uint16_t mouse_left : 4;
    // 鼠标Y轴像素位置
    uint16_t y_position : 12;
    // 鼠标右键状态
    uint16_t mouse_right : 4;
    // 保留位
    uint16_t reserved;
} __packed custom_client_data_t;

// 0x0307 105 选手端小地图接收哨兵数据，频率上限为1Hz
// 哨兵/半自动控制机器人→对应操作手选手端
typedef struct {
    // 1：到目标点攻击
    // 2：到目标点防守
    // 3：移动到目标点
    uint8_t intention;
    // 路径起点x轴坐标，单位：dm
    uint16_t start_position_x;
    // 路径起点y轴坐标，单位：dm
    uint16_t start_position_y;
    // 路径点x轴增量数组，单位：dm
    int8_t delta_x[49];
    // 路径点y轴增量数组，单位：dm
    int8_t delta_y[49];
    // 发送者ID
    uint16_t sender_id;
} __packed map_data_t;

// 0x0308 34 选手端小地图接收机器人数据，频率上限为3Hz 己方机器人→己方选手端
typedef struct {
    // 发送者的ID
    uint16_t sender_id;
    // 接收者的ID
    uint16_t receiver_id;
    // 字符。以utf-16格式编码发送，支持显示中文。
    uint8_t user_data[30];
} __packed custom_info_t;

// 0x0309 30 自定义控制器接收机器人数据，频率上限为10Hz
// 己方机器人→对应操作手选手端连接的自定义控制器
typedef struct {
    // 自定义数据
    // uint8_t data[x];
} __packed robot_custom_data_t;

struct power_parameter_t{
//    uint16_t chassis_voltage = 24;
//    uint16_t chassis_current = 0;
//    float chassis_power = 50;
    uint16_t buffer_energy = 40;
    uint16_t chassis_power_limit = 50;
};


#endif  // REFEREE_PROTOCOL_H