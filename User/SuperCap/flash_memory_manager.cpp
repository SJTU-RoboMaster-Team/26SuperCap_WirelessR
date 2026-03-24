//
// Created by yixin on 2024/4/30.
//

#include "flash_memory_manager.h"
#include "buck_boost_manager.h"
extern BuckBoostManager bbm;
void FlashMemoryManager::handle() {}

void load_one_pid_param(storage_status& status, uint32_t start_addr, PID_FLASH_STORAGE& pfs) {
    pfs = *(PID_FLASH_STORAGE*)(start_addr);
    status.loaded = true;
    if (pfs.SOF != PID_FLASH_STORAGE_SOF) {
        status.err = true;
        status.err_code |= 1;
    }
}

PID_FLASH_STORAGE FlashMemoryManager::read_pid_param(PID_CATEGORY c) {
    PID_FLASH_STORAGE pfs;
    switch (c) {
        case VOUT:
            load_one_pid_param(vout_pid_status, VOUT_PID_START_ADDR, pfs);
            vout_pid_flash_storage = pfs;
            break;
        case IOUT:
            load_one_pid_param(iout_pid_status, IOUT_PID_START_ADDR, pfs);
            iout_pid_flash_storage = pfs;
            break;
        case CAP_PWR:
            load_one_pid_param(cap_pwr_pid_status, CAP_PWR_PID_START_ADDR, pfs);
            cap_pwr_pid_flash_storage = pfs;
            break;
        case CHASSIS_PWR:
            load_one_pid_param(chassis_pwr_pid_status, CHASSIS_PWR_PID_START_ADDR, pfs);
            chassis_pwr_pid_flash_storage = pfs;
            break;
        case REFEREE_BUFFER:
            load_one_pid_param(referee_buffer_pid_status, REFREE_BUFFER_PID_START_ADDR, pfs);
            referee_buffer_pid_flash_storage = pfs;
            break;
    }
    return pfs;
}

void FlashMemoryManager::save_pid_param(PID_CATEGORY c, const PID_FLASH_STORAGE pfs) {
    if (bbm.is_working()) return;
    switch (c) {
        case VOUT:
            vout_pid_status.saved = true;
            vout_pid_flash_storage = pfs;
            break;
        case IOUT:
            iout_pid_status.saved = true;
            iout_pid_flash_storage = pfs;
            break;
        case CAP_PWR:
            cap_pwr_pid_status.saved = true;
            cap_pwr_pid_flash_storage = pfs;
            break;
        case CHASSIS_PWR:
            chassis_pwr_pid_status.saved = true;
            chassis_pwr_pid_flash_storage = pfs;
            break;
        case REFEREE_BUFFER:
            referee_buffer_pid_status.saved = true;
            referee_buffer_pid_flash_storage = pfs;
            break;
    }
}

void FlashMemoryManager::flash_pid_param() {
    clear_all(PID_START_ADDR);
    write((uint32_t*)&vout_pid_flash_storage, sizeof(PID_FLASH_STORAGE) / 8, VOUT_PID_START_ADDR);
    write((uint32_t*)&iout_pid_flash_storage, sizeof(PID_FLASH_STORAGE) / 8, IOUT_PID_START_ADDR);
    write((uint32_t*)&cap_pwr_pid_flash_storage, sizeof(PID_FLASH_STORAGE) / 8, CAP_PWR_PID_START_ADDR);
    write((uint32_t*)&chassis_pwr_pid_flash_storage, sizeof(PID_FLASH_STORAGE) / 8, CHASSIS_PWR_PID_START_ADDR);
    write((uint32_t*)&referee_buffer_pid_flash_storage, sizeof(PID_FLASH_STORAGE) / 8, REFREE_BUFFER_PID_START_ADDR);
}

CALIBRATE_PARAMETER_FLASH_STORAGE FlashMemoryManager::read_calibrate_parameter() {
    CALIBRATE_PARAMETER_FLASH_STORAGE cpfs;
    cpfs = *(CALIBRATE_PARAMETER_FLASH_STORAGE*)(CALIBRATE_PARAMETER_START_ADDR);
    calibrate_parameter_status.loaded = true;
    if (cpfs.SOF != CALIBRATE_PARAMETER_FLASH_STORAGE_SOF) {
        calibrate_parameter_status.err = true;
        calibrate_parameter_status.err_code |= 1;
    }
    calibrate_parameter_flash_storage = cpfs;

    return cpfs;
}

void FlashMemoryManager::save_calibrate_parameter(CALIBRATE_PARAMETER_FLASH_STORAGE cpfs) {
    if (bbm.is_working()) return;
    calibrate_parameter_status.saved = true;
    calibrate_parameter_flash_storage = cpfs;
}

void FlashMemoryManager::flash_calibrate_parameter() {
    clear_all(CALIBRATE_PARAMETER_START_ADDR);
    write((uint32_t*)&calibrate_parameter_flash_storage,
          sizeof(CALIBRATE_PARAMETER_FLASH_STORAGE) / 8,
          CALIBRATE_PARAMETER_START_ADDR);
}