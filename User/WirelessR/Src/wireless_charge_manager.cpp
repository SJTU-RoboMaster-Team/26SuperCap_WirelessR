//
// Created by xuhao on 2026/4/6.
//

#include "../Inc/wireless_charge_manager.h"

volatile bool transmitter_detected;
volatile bool charging_allowance;

void WirelessChargeManager::handle() {
    //    读取无线充输入电压，更新施密特触发器
    //    I2C 瞬断时 MP4201_MarkDisconnected 会把 vin_meas 清 0，
    //    直接喂给 schmitt 会立刻触发退出。此时保持上一次的状态，等待 I2C 恢复。
    if (mp4201_.is_connected) {
        mp4201_.lost_connection_tick = 0;
        float vin = mp4201_.vin_meas;
        transmitter_detected = transmitter_detector_smtrg_.update(vin);
        charging_allowance = charge_allow_smtrg_.update(vin);
    } else {
        mp4201_.lost_connection_tick++;
        if (mp4201_.lost_connection_tick >= 200 * WCM_ONE_MILLISECOND) { // 100ms timeout
            transmitter_detected = false;                                // force turn off
            charging_allowance = false;
        } else {
            // 通信掉线期间维持原状态 (由 vin_trigger_.state 保存)
            transmitter_detected = transmitter_detector_smtrg_.state;
            charging_allowance = charge_allow_smtrg_.state;
        }
    }

    // 模式切换
    if (transmitter_detected && !transmitter_detected_) {
        enter_wireless_mode();
    } else if (!transmitter_detected && transmitter_detected_) {
        exit_wireless_mode();
    }

    // 更新实时功率
    wireless_in_power = mp4201_.vout_meas * mp4201_.iout_meas; //TODO: 替换成滤波后的值

    // 同步标志位给 PM
    pm_.transmitter_detected_ = transmitter_detected_;
    if (!transmitter_detected_) {
        charging_allowance = false;
    }
    pm_.charge_allowed_ = charge_allowed_ = charging_allowance;
    // main charging loop
    // 将启动 MP4201 权限转移给 PM
    if (pm_.allow_wireless_mp4201_start()) {
        if (!mp4201_started_) {
            MP4201_SetCP_Target(&mp4201_, 120, 0, 0.005f);
            MP4201_Enable(&mp4201_, 1);
            mp4201_started_ = true;
        }
    } else {
        if (mp4201_started_) {
            MP4201_Enable(&mp4201_, 0);
            mp4201_started_ = false;
        }
    }

    // 无线充模式下的能量统计
    if (charge_allowed_) {
        energy_accounting();
    }
}

void WirelessChargeManager::enter_wireless_mode() {
    // 仅设置标志位，无线充在线，默认关闭4201
    transmitter_detected_ = true;

    MP4201_Enable(&mp4201_, 0);

    // 负载调制器默认启动
}

void WirelessChargeManager::exit_wireless_mode() {
    transmitter_detected_ = false;

    MP4201_Enable(&mp4201_, 0);

    mp4201_started_ = false;
    // 不停止负载调制器
    // 不清零energy_accounting
}

void WirelessChargeManager::energy_accounting() {
    // 无线充模式能量统计：
    //   输入 = 无线充提供的功率 (MP4201)
    //   输出 = BuckBoost 向 chassis 输出的功率
    //   差值累积达到阈值后，给 chassis 发放能量 bonus
    float wr_in_pwr = wireless_in_power;          // 无线充输入功率
    float chassis_pwr = pm_.filtered_out_pwr.val; // BuckBoost → chassis 输出功率

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
