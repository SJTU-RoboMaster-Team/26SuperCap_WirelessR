//
// Created by yixin on 2024/1/19.
//

#ifndef SUPER_CAP_POWER_MANAGER_H
#define SUPER_CAP_POWER_MANAGER_H

#include "math_tool/my_math.h"
#include "buck_boost_manager.h"
#include "protocol.h"
#include "flash_data_structure.h"
#include "can_task.h"

class PowerManager {
public:
    PowerManager(BuckBoostManager& bbm, PID& cap_pwr_pid, PID& chassis_pwr_pid, PID& referee_buffer_pid,PID&wireless_recharge_pid):
        cap_pwr_pid(cap_pwr_pid),
        chassis_pwr_pid(chassis_pwr_pid),
        referee_buffer_pid(referee_buffer_pid),
        wireless_recharge_pid(wireless_recharge_pid),
        bbm(bbm) {};

    void start() { bbm.start(); }

    void stop() { bbm.stop(); }

    void handle();

    void load_parameter(power_parameter_t para) { power_parameter = para; }

    void load_flash_parameter(const PID_FLASH_STORAGE& cap_pwr_pid_flash_storage,
                              const PID_FLASH_STORAGE& chasiss_pwr_pid_flash_storage,
                              const PID_FLASH_STORAGE& referee_buffer_pid_flash_storage);

    void set_calibrate_mode() { mode = CALIBRATE; }

    // 运行状态
    enum State {
        CHARGING = 0,
        DISCHARGING = 1,
        IDLE = 2,
        ERR = 3,
    } state = IDLE;

    // 控制模式
    enum Mode {
        BUFFER = 0,
        CUSTOM = 2,
        CALIBRATE = 3,
        STAND_BY = 4
    } mode = BUFFER;

    // 功率相关变量 (W)
    float cap_pwr = 0; // + charging, - releasing
    float in_minus_out_pwr = 0;
    float target_cap_pwr = 0;

    // 功率限制变量
    float max_releasing_pwr = -800.0f;
    float max_charging_pwr = 50.0f;

    float in_pwr = 0;
    float out_pwr = 0;
    float power_limit = 0; // 裁判系统限制的底盘功率

    float compensation = 0; // 功率补偿量

    // float target_in_pwr = 0;
    // float target_out_pwr = 0;
    // float chassis_pwr = 0;
    // float target_chassis_pwr = 0;

    // 欠压保护逻辑
    uint32_t under_voltage_thres = 125000; // 计数阈值
    uint32_t under_voltage_tick = 0;

    int32_t target_buffer = 40; // 目标缓冲能量

    struct ChargeConfig {
        float full_charge_target_v = 26.0f;
        float full_charge_upper_threshold = 26.000f; // 停止充电电压
        float full_charge_lower_threshold = 25.500f; // 恢复充电电压
        float allowed_max_releasing_pwr = -800.0f;   // 最大放电功率
    } charge_parameter;

    uint8_t err_code = 0;
    power_parameter_t power_parameter{};

    bool wireless_active = false; // 由 WirelessChargeManager 设置

    // CAN 命令接口：底盘请求进入无线充覆盖模式
    void set_chassis_cmd(const ChassisCmd& cmd) {
        chassis_cmd_ = cmd;
    }

    // 无线充覆盖是否激活
    [[nodiscard]] bool is_wireless_override_active() const {
        return chassis_cmd_.desired_power_limit;
    }

    // 获取压缩后的状态码
    [[nodiscard]] uint8_t get_mode_and_state() const { return mode << 4 | state; }

    // 功率滤波器
    LowPassFilter filtered_out_pwr = LowPassFilter(0, 0.05);
    LowPassFilter filtered_in_pwr = LowPassFilter(0, 0.05);
    LowPassFilter filtered_cap_pwr = LowPassFilter(0, 0.05);

private:
    PID& cap_pwr_pid;
    PID& chassis_pwr_pid;
    PID& referee_buffer_pid;
    PID& wireless_recharge_pid;
    BuckBoostManager& bbm;

    // CAN 命令
    ChassisCmd chassis_cmd_{};
    // 用于检测 wireless_active 下降沿（离开无线充区时清空无线相关状态）
    bool last_wireless_active_ = false;

    // 无线充覆盖：检测 wireless_discharge_ok_smtrg 上升沿（充满时自动退出覆盖）
    bool last_wireless_discharge_ok_ = true;

    // 用于状态判断的迟滞比较
    SchmittTrigger<float> cap_full_smtrg{ charge_parameter.full_charge_upper_threshold,
                                          charge_parameter.full_charge_lower_threshold, false };
    SchmittTrigger<float> cap_limit_60power_smtrg{ 17.000f, 15.000f, false };
    SchmittTrigger<float> cap_limit_30power_smtrg{ 14.000f, 12.000f, false };
    SchmittTrigger<float> cap_low_smtrg{ 11.500f, 10.000f, false };
    SchmittTrigger<float> wireless_discharge_ok_smtrg{ 25.500f, 16.000f, true };// 无线充电模式下放电截止：降至 16V 停止放电，充回 25.5V 后恢复
    SchmittTrigger<uint16_t> buffer_far_from_target{ 7, 3, false };

    FollowTrendEstimator follow_trend_estimator = FollowTrendEstimator(3.0f, 400);

    bool full_charge = false;
    bool empty = false;

    void buffer_mode();
    void compute_buffer_compensation();
    void compute_and_apply_target_power();
    void handle_wireless_override(); // 无线充覆盖逻辑（功率限制、放电截止、自动退出）
    void clear_wireless_state();     // wireless_active 下降沿时调用，清空所有无线充相关状态
    void calibrate_mode();
    void check_cap_status();
};

#endif //SUPER_CAP_POWER_MANAGER_H
