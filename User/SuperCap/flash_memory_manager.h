//
// Created by yixin on 2024/4/30.
//

#ifndef SUPER_CAP_FLASH_MEMORY_MANAGER_H
#define SUPER_CAP_FLASH_MEMORY_MANAGER_H


#include "stm32g4xx_hal.h"
#include "flash_data_structure.h"

// current program size
#define PROGRAM_SIZE 0x3000

// For stm32g474RET6  512kb
// For 128kb, erase page should be modified but actually should not.
#define PARAMETER_START_ADDR 0x0801C000
#define PARAMETER_END_ADDR 0x0801DFFF
// PID * 5
#define PID_START_ADDR 0x0801C000
#define VOUT_PID_START_ADDR 0x0801C000
#define IOUT_PID_START_ADDR 0x0801C100
#define CAP_PWR_PID_START_ADDR 0x0801C200
#define CHASSIS_PWR_PID_START_ADDR 0x0801C300
#define REFREE_BUFFER_PID_START_ADDR 0x0801C400


#define CALIBRATE_PARAMETER_START_ADDR 0x0801C800

#define USER_PARAMETER_START_ADDR 0x0801D000


struct storage_status{
    bool empty = true;
    bool loaded = false;
    bool saved = false;
    bool err = false;
    uint8_t err_code = 0;
};

enum PID_CATEGORY{
    VOUT,
    IOUT,
    CAP_PWR,
    CHASSIS_PWR,
    REFEREE_BUFFER,
};

//// data to store:

// 1. calibrate_parameter
// 2. serial number
// 3. erase times
// 4. odometer

// for odometer, use 4 bytes to store



//// README!!!!!!!
//// Parameter stored in flash, address start from 0x0801E000, thus program size should not exceed 120k!!!
//// This is specially designed for stm32g474RET6 and this program, for other chips or usage, please modify the code!
class FlashMemoryManager {
public:
    FlashMemoryManager()= default;

    void handle();

    void load_all_pid_param(){
        read_pid_param(VOUT);
        read_pid_param(IOUT);
        read_pid_param(CAP_PWR);
        read_pid_param(CHASSIS_PWR);
        read_pid_param(REFEREE_BUFFER);
    };
    PID_FLASH_STORAGE read_pid_param(PID_CATEGORY c);
    void save_pid_param(PID_CATEGORY c, PID_FLASH_STORAGE pfs);
    void flash_pid_param();

    void load_calibrate_parameter(){
        read_calibrate_parameter();
    }
    CALIBRATE_PARAMETER_FLASH_STORAGE read_calibrate_parameter();
    void save_calibrate_parameter(CALIBRATE_PARAMETER_FLASH_STORAGE cpfs);
    void flash_calibrate_parameter();


    // size must be multiple of 8, or n * 64 bits
    // because we use double word to write
    static void write(const uint32_t *pointer_to_data, uint32_t size,uint32_t start_addr){
        if (!addr_legal(start_addr) || !addr_legal(start_addr + size * 8)){
            return;
        }

        HAL_FLASH_Unlock();

        if(!judge_if_empty((uint32_t*)start_addr, size)){
            FLASH_EraseInitTypeDef erase;
            erase.TypeErase = FLASH_TYPEERASE_PAGES;
            erase.Banks = FLASH_BANK_1;
//        erase.Page = 60 - 2 -2;
            erase.Page = address_to_page(start_addr);
            // 1 page = 2k
            // page 60 = 120k = 122880 = 0x1e000
            erase.NbPages = 1;
            uint32_t page_error;
            HAL_FLASHEx_Erase(&erase, &page_error);
        }

        // size = number of 64 bits double word
        for (uint32_t i = 0; i < size; ++i) {
            uint64_t data64 = (uint64_t)((uint64_t) pointer_to_data[i * 2 + 1] << 32) | (uint64_t)pointer_to_data[i * 2];
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr + i * 8, data64);
        }
        // 1e000 == 122880 = 120k
        HAL_FLASH_Lock();
    }


    static void clear_all(uint32_t start_addr) {
        if (!addr_legal(start_addr)) {
            return;
        }
        HAL_FLASH_Unlock();
        FLASH_EraseInitTypeDef erase;
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = FLASH_BANK_1;
        erase.Page = address_to_page(start_addr);
        erase.NbPages = 1;
        uint32_t page_error;
        HAL_FLASHEx_Erase(&erase, &page_error);
    }

    // size must be multiple of 8, or n * 64 bits
    // because we use double word to write
    static bool judge_if_empty(const uint32_t *data, uint32_t size) {
        for (int i = 0; i < size*2; i++) {
            if (data[i] != 0xFFFFFFFF) {
                return false;
            }
        }
        return true;
    }

    static uint32_t address_to_page(uint32_t address){
        return (address - 0x08000000) / 2048;
    }

    static bool addr_legal(uint32_t addr){
        return addr >= PARAMETER_START_ADDR && addr < PARAMETER_END_ADDR;
    }


private:


    storage_status vout_pid_status;
    storage_status iout_pid_status;
    storage_status cap_pwr_pid_status;
    storage_status chassis_pwr_pid_status;
    storage_status referee_buffer_pid_status;

    PID_FLASH_STORAGE vout_pid_flash_storage;
    PID_FLASH_STORAGE iout_pid_flash_storage;
    PID_FLASH_STORAGE cap_pwr_pid_flash_storage;
    PID_FLASH_STORAGE chassis_pwr_pid_flash_storage;
    PID_FLASH_STORAGE referee_buffer_pid_flash_storage;


    storage_status calibrate_parameter_status;

    CALIBRATE_PARAMETER_FLASH_STORAGE calibrate_parameter_flash_storage;

    storage_status user_parameter_status;
};






#endif //SUPER_CAP_FLASH_MEMORY_MANAGER_H
