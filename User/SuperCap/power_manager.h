//
// Created by yixin on 2024/1/19.
//

#ifndef SUPER_CAP_POWER_MANAGER_H
#define SUPER_CAP_POWER_MANAGER_H

#include "math_tool/my_math.h"
#include "buck_boost_manager.h"
#include "protocol.h"
#include "flash_data_structure.h"

class PowerManager {
public:
    PowerManager(BuckBoostManager& bbm, PID& cap_pwr_pid, PID& chassis_pwr_pid, PID& referee_buffer_pid):
        bbm(bbm),
        cap_pwr_pid(cap_pwr_pid),
        chassis_pwr_pid(chassis_pwr_pid),
        referee_buffer_pid(referee_buffer_pid) {};

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
        QUICK_CHARGE = 1,
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

    // 获取压缩后的状态码
    uint8_t get_mode_and_state() const { return mode << 4 | state; }

    // 功率滤波器
    LowPassFilter filtered_out_pwr = LowPassFilter(0, 0.05);
    LowPassFilter filtered_in_pwr = LowPassFilter(0, 0.05);
    LowPassFilter filtered_cap_pwr = LowPassFilter(0, 0.05);

private:
    PID& cap_pwr_pid;
    PID& chassis_pwr_pid;
    PID& referee_buffer_pid;
    BuckBoostManager& bbm;

    // 用于状态判断的迟滞比较
    SchmittTrigger<float> cap_full_smtrg{ charge_parameter.full_charge_upper_threshold,
                                          charge_parameter.full_charge_lower_threshold, false };
    SchmittTrigger<float> cap_limit_60power_smtrg{ 17.000f, 15.000f, false };
    SchmittTrigger<float> cap_limit_30power_smtrg{ 14.000f, 12.000f, false };
    SchmittTrigger<float> cap_low_smtrg{ 11.500f, 10.000f, false };
    SchmittTrigger<uint16_t> buffer_far_from_target{ 7, 3, false };

    FollowTrendEstimator follow_trend_estimator = FollowTrendEstimator(3.0f, 400);

    bool full_charge = false;
    bool empty = false;

    void buffer_mode();
    void compute_buffer_compensation();   // buffer_mode 子步骤：计算 buffer 能量补偿量
    void compute_and_apply_target_power(); // buffer_mode 子步骤：计算并下发目标功率
    void calibrate_mode();
    void check_cap_status();
};

#endif //SUPER_CAP_POWER_MANAGER_H