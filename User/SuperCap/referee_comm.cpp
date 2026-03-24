//
// Created by yixin on 2024/1/20.
//

#include "referee_comm.h"
#include "math_tool/my_math.h"


void RefereeComm::init(void) {

}

void RefereeComm::handle(void) {

}

void RefereeComm::txMsg(void) {

}

void RefereeComm::rxCallback(uint8_t * data,uint32_t size) {
    // crc check
    auto length_left = (int32_t)size;

    // 空闲中断可能会几个包连在一起，所以这里手动拆分一下
    while (length_left >0){
        length_left = length_left - unpack_one_pack(data+(size-length_left),length_left);
    }

}

int32_t RefereeComm::unpack_one_pack(uint8_t *data, uint32_t size) {
    frame.sof = *data;
    frame.data_len = *(uint16_t*)(data + 1);
    frame.seq = *(data+3);
    frame.cmd_id = *(uint16_t*)(data+5);
    frame.data = data+7;

    if(!(CRC16_Verify(data,frame.data_len+9))){
        crc_err_count++;
        return 256;
    } else{
        recv_cnt++;
    }
    unpack_error_ = NO_ERROR;

    if (frame.cmd_id == GAME_STATUS_ID) {
        memcpy(&game_status_, frame.data , frame.data_len);
    } else if (frame.cmd_id == GAME_RESULT_ID) {
        memcpy(&game_result_, frame.data, frame.data_len);
    } else if (frame.cmd_id == GAME_ROBOT_HP_ID) {
        memcpy(&game_robot_HP_, frame.data, frame.data_len);
    } else if (frame.cmd_id == EVENT_DATA_ID) {
        memcpy(&event_data_, frame.data, frame.data_len);
    } else if (frame.cmd_id == REFEREE_WARNING_ID) {
        memcpy(&referee_warning_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == DART_INFO_ID) {
        memcpy(&dart_info_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == ROBOT_STATUS_ID) {
        memcpy(&robot_status_, frame.data, frame.data_len);
        power_parameter.chassis_power_limit = robot_status_.chassis_power_limit;
    } else if (frame.cmd_id == POWER_HEAT_DATA_ID) {
        memcpy(&power_heat_data_, frame.data,
               frame.data_len);
//        power_parameter.chassis_power = power_heat_data_.chassis_power;
//        power_parameter.chassis_voltage = power_heat_data_.chassis_voltage;
//        power_parameter.chassis_current = power_heat_data_.chassis_current;
        power_parameter.buffer_energy = power_heat_data_.buffer_energy;
    } else if (frame.cmd_id == ROBOT_POS_ID) {
        memcpy(&robot_pos_, frame.data, frame.data_len);
    } else if (frame.cmd_id == BUFF_ID) {
        memcpy(&buff_, frame.data, frame.data_len);
    } else if (frame.cmd_id == HURT_DATA_ID) {
        memcpy(&hurt_data_, frame.data, frame.data_len);
    } else if (frame.cmd_id == SHOOT_DATA_ID) {
        memcpy(&shoot_data_, frame.data, frame.data_len);
    } else if (frame.cmd_id == PROJECTILE_ALLOWANCE_ID) {
        memcpy(&projectile_allowance_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == RFID_STATUS_ID) {
        memcpy(&rfid_status_, frame.data, frame.data_len);
    } else if (frame.cmd_id == DART_CLIENT_CMD_ID) {
        memcpy(&dart_client_cmd_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == GROUND_ROBOT_POSITION_ID) {
        memcpy(&ground_robot_position_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == RADAR_MARK_DATA_ID) {
        memcpy(&radar_mark_data_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == MAP_COMMAND_ID) {
        memcpy(&map_command_, frame.data, frame.data_len);
    } else if (frame.cmd_id == REMOTE_CONTROL_ID) {
        memcpy(&remote_control_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == MAP_ROBOT_DATA_ID) {
        memcpy(&map_robot_data_, frame.data,
               frame.data_len);
    } else if (frame.cmd_id == CUSTOM_CLIENT_DATA_ID) {
        memcpy(&custom_client_data_, frame.data,
               frame.data_len);
    } else {
        unpack_error_ = ID_UNDEFINED;
    }

//    if(unpack_error_ == NO_ERROR){
        connect_info.last_recv_tick = HAL_GetTick();
        connect_info.connected = true;
//    }

    return frame.data_len+9;
}
