//
// Created by yixin on 2024/1/20.
//

#include "referee_comm.h"
#include "math_tool/my_math.h"

namespace {
constexpr uint32_t REFEREE_FRAME_HEADER_LEN = 5U;
constexpr uint32_t REFEREE_CMD_ID_LEN = 2U;
constexpr uint32_t REFEREE_CRC16_LEN = 2U;
constexpr uint32_t REFEREE_FRAME_MIN_LEN =
        REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_CRC16_LEN;

uint16_t read_u16_le(const uint8_t *data) {
    uint16_t value;
    memcpy(&value, data, sizeof(value));
    return value;
}
}

void RefereeComm::init(void) {

}

void RefereeComm::handle(void) {

}

void RefereeComm::txMsg(void) {

}

void RefereeComm::rxCallback(uint8_t *data, uint32_t size) {
    uint32_t offset = 0;

    // 分多个包
    while (offset < size) {
        int32_t consumed = unpack_one_pack(data + offset, size - offset);
        if (consumed <= 0) {
            break;
        }
        offset += (uint32_t)consumed;
    }
}

int32_t RefereeComm::unpack_one_pack(uint8_t *data, uint32_t size) {
    if (data == nullptr || size == 0U) {
        return 0;
    }

    if (data[0] != REFEREE_COMM_SOF) {
        unpack_error_ = HEADER_CRC_FAIL;
        return 1;
    }

    if (size < REFEREE_FRAME_HEADER_LEN) {
        unpack_error_ = DATA_LEN_OUTRANGE;
        return (int32_t)size;
    }

    if (!CRC8_Verify(data, REFEREE_FRAME_HEADER_LEN)) {
        crc_err_count++;
        unpack_error_ = HEADER_CRC_FAIL;
        return 1;
    }

    const uint16_t data_len = read_u16_le(data + 1);
    const uint32_t frame_len = (uint32_t)data_len + REFEREE_FRAME_MIN_LEN;
    const uint32_t max_data_len = REFEREE_COMM_FRAME_MAX_SIZE - REFEREE_FRAME_MIN_LEN;

    if (data_len > max_data_len || frame_len > REFEREE_COMM_FRAME_MAX_SIZE) {
        unpack_error_ = DATA_LEN_OUTRANGE;
        return 1;
    }

    if (size < frame_len) {
        unpack_error_ = DATA_LEN_OUTRANGE;
        return (int32_t)size;
    }

    if (!CRC16_Verify(data, frame_len)) {
        crc_err_count++;
        unpack_error_ = PACK_CRC_FAIL;
        return 1;
    }

    frame.sof = data[0];
    frame.data_len = data_len;
    frame.seq = data[3];
    frame.cmd_id = read_u16_le(data + 5);
    frame.data = data + 7;

    recv_cnt++;
    unpack_error_ = NO_ERROR;

    if (apply_frame(frame.cmd_id, frame.data, frame.data_len)) {
        connect_info.last_recv_tick = HAL_GetTick();
        connect_info.connected = true;
    }

    return (int32_t)frame_len;
}

bool RefereeComm::apply_frame(uint16_t cmd_id, const uint8_t *data, uint16_t data_len) {
#define COPY_IF_SIZE_MATCH(dst)                         \
    do {                                                \
        if (data_len != sizeof(dst)) {                  \
            unpack_error_ = DATA_LEN_MISMATCH;          \
            return false;                               \
        }                                               \
        memcpy(&(dst), data, sizeof(dst));              \
    } while (0)

    switch (cmd_id) {
        case GAME_STATUS_ID:
            COPY_IF_SIZE_MATCH(game_status_);
            break;
        case GAME_RESULT_ID:
            COPY_IF_SIZE_MATCH(game_result_);
            break;
        case GAME_ROBOT_HP_ID:
            COPY_IF_SIZE_MATCH(game_robot_HP_);
            break;
        case EVENT_DATA_ID:
            COPY_IF_SIZE_MATCH(event_data_);
            break;
        case REFEREE_WARNING_ID:
            COPY_IF_SIZE_MATCH(referee_warning_);
            break;
        case DART_INFO_ID:
            COPY_IF_SIZE_MATCH(dart_info_);
            break;
        case ROBOT_STATUS_ID:
            COPY_IF_SIZE_MATCH(robot_status_);
            power_parameter.chassis_power_limit = robot_status_.chassis_power_limit;
            break;
        case POWER_HEAT_DATA_ID:
            COPY_IF_SIZE_MATCH(power_heat_data_);
//            power_parameter.chassis_power = power_heat_data_.chassis_power;
//            power_parameter.chassis_voltage = power_heat_data_.chassis_voltage;
//            power_parameter.chassis_current = power_heat_data_.chassis_current;
            power_parameter.buffer_energy = my_math::limit(power_heat_data_.buffer_energy, (uint16_t)0, (uint16_t)60);
            break;
        case ROBOT_POS_ID:
            COPY_IF_SIZE_MATCH(robot_pos_);
            break;
        case BUFF_ID:
            COPY_IF_SIZE_MATCH(buff_);
            break;
        case HURT_DATA_ID:
            COPY_IF_SIZE_MATCH(hurt_data_);
            break;
        case SHOOT_DATA_ID:
            COPY_IF_SIZE_MATCH(shoot_data_);
            break;
        case PROJECTILE_ALLOWANCE_ID:
            COPY_IF_SIZE_MATCH(projectile_allowance_);
            break;
        case RFID_STATUS_ID:
            COPY_IF_SIZE_MATCH(rfid_status_);
            break;
        case DART_CLIENT_CMD_ID:
            COPY_IF_SIZE_MATCH(dart_client_cmd_);
            break;
        case GROUND_ROBOT_POSITION_ID:
            COPY_IF_SIZE_MATCH(ground_robot_position_);
            break;
        case RADAR_MARK_DATA_ID:
            COPY_IF_SIZE_MATCH(radar_mark_data_);
            break;
        case MAP_COMMAND_ID:
            COPY_IF_SIZE_MATCH(map_command_);
            map_command_receive_tick_ = HAL_GetTick();
            break;
        case REMOTE_CONTROL_ID:
            COPY_IF_SIZE_MATCH(remote_control_);
            break;
        case MAP_ROBOT_DATA_ID:
            COPY_IF_SIZE_MATCH(map_robot_data_);
            break;
        case CUSTOM_CLIENT_DATA_ID:
            COPY_IF_SIZE_MATCH(custom_client_data_);
            break;
        default:
            unpack_error_ = ID_UNDEFINED;
            return false;
    }

    unpack_error_ = NO_ERROR;
    return true;

#undef COPY_IF_SIZE_MATCH
}
