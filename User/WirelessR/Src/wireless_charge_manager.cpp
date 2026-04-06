//
// Created by xuhao on 2026/4/6.
//

#include "../Inc/wireless_charge_manager.h"

void WirelessChargeManager::handle() {
    // 1. 读取无线充输入电压，更新施密特触发器
    float vin = mp4201_.vin_meas;
    vin_trigger_.upper_threshold = config.vin_upper_threshold;
    vin_trigger_.lower_threshold = config.vin_lower_threshold;
    bool vin_ok = vin_trigger_.update(vin);

    // 2. 模式切换
    if (vin_ok && !wireless_active_) {
        enter_wireless_mode();
    } else if (!vin_ok && wireless_active_) {
        exit_wireless_mode();
    }

    // 3. 更新实时功率
    wireless_in_power = mp4201_.vout_meas * mp4201_.iout_meas; //TODO: 替换成滤波后的值

    // 4. 无线充模式下的能量统计
    if (wireless_active_) {
        energy_accounting();
    }

    // 5. 同步标志位给 PM
    pm_.wireless_active = wireless_active_;
}

void WirelessChargeManager::enter_wireless_mode() {
    wireless_active_ = true;

    // 仅控制 MP4201：启用并设为恒功率 120W
    // BuckBoost 由 PM 的 QUICKCHARGE 模式继续管理，不在此处操作
    MP4201_Enable(&mp4201_, 1);
    MP4201_SetCP_Target(&mp4201_, 120,0, 0.005);
}

void WirelessChargeManager::exit_wireless_mode() {
    wireless_active_ = false;

    // 关闭 MP4201
    MP4201_Enable(&mp4201_, 0);

    // 重置能量累积器
    energy_accumulator_ = 0;
    supercap_in_energy = 0;
    chassis_out_energy = 0;
}

void WirelessChargeManager::energy_accounting() {
    // 无线充模式能量统计：
    //   输入 = 无线充提供的功率 (MP4201)
    //   输出 = BuckBoost 向 chassis 输出的功率
    //   差值累积达到阈值后，给 chassis 发放能量 bonus
    float wr_in_pwr = wireless_in_power;                // 无线充输入功率
    float chassis_pwr = pm_.filtered_out_pwr.val;       // BuckBoost → chassis 输出功率

    // 累积能量 (J = W * s)
    supercap_in_energy += wr_in_pwr * config.dt;
    chassis_out_energy += chassis_pwr * config.dt;

    // 能量差累积 (仅正向，即无线充输入多于底盘输出时才累积)
    float delta_energy = (wr_in_pwr - chassis_pwr) * config.dt;
    if (delta_energy > 0) {
        energy_accumulator_ += delta_energy;
    }

    // 每积累 1J，底盘能量 +8
    while (energy_accumulator_ >= config.energy_per_bonus) {
        energy_accumulator_ -= config.energy_per_bonus;
        chassis_energy_bonus_ += config.bonus_per_joule;
    }
}
