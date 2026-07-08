//
// Created by xuhao on 2026/4/6.
//

#ifndef WIRELESS_CHARGE_MANAGER_H
#define WIRELESS_CHARGE_MANAGER_H

#include "MP4201.h"
#include "modulator.h"
#include "../../SuperCap/power_manager.h"
#include "../../config.h"

#define WCM_ONE_MILLISECOND MAIN_CONTROL_LOOP_FREQ_HZ / 1000
/**
 * @brief 无线充电管理器
 *
 * 功能：
 *   1. 检测无线充端口电压，达到阈值后触发无线充模式
 *   2. 无线充模式下，整套设备完全由无线充提供能量，并给超电充电
 *   3. 能量统计：超电输入能量与底盘输出能量之差每达到 1J，底盘能量 +8
 */
class WirelessChargeManager {
public:
    WirelessChargeManager(MP4201_Handle_t& mp4201, PowerManager& pm, Modulator& modulator)
        : mp4201_(mp4201), pm_(pm), modulator_(modulator) {}

    /**
     * @brief 主循环，在 main_control_loop 中调用 (25kHz)
     */
    void handle();

    // ---- 状态查询 ----
    bool is_transmitter_detected() const { return transmitter_detected_; }
    uint32_t get_chassis_energy_bonus() const { return chassis_energy_bonus_; }
    float get_energy_accumulator() const { return energy_accumulator_; }

    // ---- 配置 ----
    struct Config {
        float transmitter_detector_vin_upper_threshold = WR_CHARGE_TRANSMITTER_DETECT_VIN_UPPER_THRESHOLD;  // 无线充激活上阈值 (V)
        float transmitter_detector_vin_lower_threshold = WR_CHARGE_TRANSMITTER_DETECT_VIN_LOWER_THRESHOLD;  // 无线充退出下阈值 (V)
        float charge_allow_vin_upper_threshold = WR_CHARGE_CHARGE_ALLOWANCE_VIN_UPPER_THRESHOLD;  // 无线充激活上阈值 (V)
        float charge_allow_vin_lower_threshold = WR_CHARGE_CHARGE_ALLOWANCE_VIN_LOWER_THRESHOLD;  // 无线充退出下阈值 (V)
        float charge_target_voltage = WR_CHARGE_TARGET_VOLTAGE;     // 超电目标充电电压 (V)
        float max_charge_current = WR_CHARGE_MAX_CURRENT;           // 最大充电电流 (A)
        float energy_per_bonus = WR_CHARGE_ENERGY_PER_BONUS;        // 每多少 J 能量差获得一次 bonus
        uint32_t bonus_per_joule = WR_CHARGE_BONUS_PER_JOULE;       // 每次 bonus 增加的底盘能量
        float dt = WR_CHARGE_LOOP_DT;                               // 控制周期 (s)，由主控制环频率派生
    } config;

    // ---- 实时数据 (供外部调试/显示) ----
    float wireless_in_power = 0;     // 无线充输入功率 (W)
    float supercap_in_energy = 0;    // 累计超电输入能量 (J)
    float chassis_out_energy = 0;    // 累计底盘输出能量 (J)
    uint32_t chassis_energy_bonus_ = 0; // 累计底盘能量 bonus
    bool mp4201_started_ = false;
private:
    MP4201_Handle_t& mp4201_;
    PowerManager& pm_;
    Modulator& modulator_;

    // 无线充电压检测 (施密特触发器)
    SchmittTrigger<float> transmitter_detector_smtrg_{config.transmitter_detector_vin_upper_threshold, config.transmitter_detector_vin_lower_threshold, false};
    SchmittTrigger<float> charge_allow_smtrg_{config.charge_allow_vin_upper_threshold, config.charge_allow_vin_lower_threshold, false};
    bool transmitter_detected_ = false;
    bool charge_allowed_ = false;

    // 能量积分器
    float energy_accumulator_ = 0;   // (超电输入 - 底盘输出) 能量差累积 (J)

    void enter_wireless_mode();
    void exit_wireless_mode();
    void energy_accounting();
};

#endif // WIRELESS_CHARGE_MANAGER_H
