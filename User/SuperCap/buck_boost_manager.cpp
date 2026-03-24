//
// Created by yixin on 2024/1/19.
//

#include <cmath>
#include "buck_boost_manager.h"
#include "../callback.h"

#include <ios>

extern RefreshFlag refresh_flag;

// 随着电容端电压逐渐增大，左桥下mos占空比逐渐减小，右桥下mos占空比逐渐增大
void BuckBoostManager::handle() {
    // 安全检查
    if (!safety_check()) {
        stop();
        return;
    }

    // 限制目标电流
    target_current = my_math::limit_abs(target_current, config.limits.max_target_current);

    // 确定工作模式(Buck/Boost/Buck-Boost/Off)
    mode_switcher();

    // 软启动逻辑
    if (mode == OFF) {
        start_up_info.starting = false;
        start_up_info.started = false;
        start_up_info.time_since_last_start = 0;
    }

    if (last_mode == OFF && mode != OFF) {
        start_up_info.starting = true;
    }

    if (start_up_info.starting) {
        start_up_info.time_since_last_start++;
        if (start_up_info.time_since_last_start > start_up_info.start_time_threshold) {
            start_up_info.started = true;
            start_up_info.starting = false;
        }
    }

    // 执行不同控制模式
    switch (control_mode) {
        case V_LOOP:
            v_loop_control();
            break;
        case I_LOOP:
            i_loop_control();
            break;
        // case V_OUT_I_IN:
        //     //            stop();
        //     break;
        // case I_OUT_V_IN:
        //     //            stop();
        //     break;
        case OPEN_LOOP:
        default:
            break;
    }

    // 更新历史状态
    last_mode = mode;

    // 驱动底层半桥
    left_bridge.handle();
    right_bridge.handle();

    refresh_flag.loop_flag_ = true;
}

bool BuckBoostManager::safety_check() {
    // 检查硬件过压
    if (v_c > config.thresholds.max_cap_voltage) return false;
    // 检查硬件欠压
    if (v_i < config.thresholds.min_working_voltage) return false;

    return true;
}

void BuckBoostManager::switch_mode_reset() {
    // 模式切换时，先让硬件进入安全状态，并重置 PID 积分项
    right_bridge.start();
    left_bridge.start();
    vpid.reset();
    ipid.reset();
}

void BuckBoostManager::mode_switcher() {
    if (!enable) {
        mode = OFF;
        return;
    }

    // 状态机迟滞比较
    // 防止频繁跳变，切换模式
    // ReSharper disable once CppDFAUnreachableCode
    switch (mode) {
        case OFF:
            // 启动时，根据电压差决定初始模式
            if (v_i < target_voltage + config.thresholds.buck_to_buckboost_gap) {
                // 输入电压不足以维持纯 Buck，进入 Buck-Boost 模式
                mode = BUCKBOOST;
            } else {
                // 输入电压足够高，进入 Buck 模式
                mode = BUCK;
            }
            break;

        case BUCK:
            // Vin 降低或 Target 升高，导致压差不足 -> 切入 BuckBoost
            if (v_i < target_voltage + config.thresholds.buck_to_buckboost_gap) {
                mode = BUCKBOOST;
            }
            break;

        case BOOST:
            // Vin 升高或 Target 降低，导致压差不足以维持纯 Boost -> 切入 BuckBoost
            // Boost 模式下 Vout > Vin，所以这里是 target > Vin + gap
            if (target_voltage < v_i + config.thresholds.boost_to_buckboost_gap) {
                mode = BUCKBOOST;
            }
            break;

        case BUCKBOOST:
            if (target_voltage < v_i - config.thresholds.buckboost_exit_gap_buck) {
                // Vout 显著低于 Vin -> 切入 Buck
                mode = BUCK;
            } else if (target_voltage > v_i) {
                // Vout 显著高于 Vin -> 切入 Boost
                mode = BOOST;
            }
            break;

        default:
            break;
    }
}

void BuckBoostManager::v_loop_control() {
    float pid_out = vpid.update(target_voltage, v_c);

    switch (mode) {
        case BUCK:
            // vout = vin * (1-D)
            if (last_mode == BUCK) {
                // Vout > Target -> PID 输出为负 -> Duty 变大（下管导通时间增加）
                // duty for left bridge lower mos
                // when v_out > target
                // pid output a negative value, and we want duty to be larger, so there is a minus sign
                left_bridge.add_duty(-pid_out);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                // 前馈
                left_bridge.set_duty(1.0f - target_voltage / v_i);
                right_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BOOST:
            if (last_mode == BOOST) {
                right_bridge.add_duty(pid_out);
                right_bridge.force_larger_than(config.limits.min_duty);
            } else {
                // 前馈
                // Vout = Vin / (1-D) -> D =  1- Vin/Vout
                right_bridge.set_duty(1.0f - v_i / target_voltage);
                left_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BUCKBOOST:
            if (last_mode == BUCKBOOST) {
                // 四开关同时工作，按比例分配 PID 输出
                right_bridge.add_duty(0.5f * pid_out);
                // 限制占空比，防止直通
                right_bridge.force_smaller_than(config.limits.max_buckboost_duty); ////todo 参数可调
                right_bridge.force_larger_than(config.limits.min_duty);

                left_bridge.add_duty(-0.3f * pid_out);
                left_bridge.force_smaller_than(config.limits.max_buckboost_duty); ////todo
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                right_bridge.set_duty(0.24f); // 默认值
                left_bridge.set_duty(0.15f);
                switch_mode_reset();
            }
            break;

        case OFF:
            left_bridge.stop();
            right_bridge.stop();
            break;
    }
}

void BuckBoostManager::i_loop_control() {
    // suppose the voltage will be the target voltage

    // 启动阶段限制电流，防止冲击
    // ReSharper disable once CppDFAConstantConditions
    if (start_up_info.starting) {
        target_current = my_math::limit_abs(target_current, 5.0f);
    }

    float pid_out = ipid.update(target_current, i_c);

    switch (mode) {
        case BUCK:
            if (last_mode == BUCK) {
                // duty for left bridge lower mos
                // when i_cap > target
                // pid output a negative value, and we want duty to be larger, so there is a minus sign
                left_bridge.add_duty(-0.3f * pid_out);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                // 前馈
                debug.v_just_read = v_i;
                debug.calc_res = 1 - target_voltage / v_i;
                left_bridge.set_duty(1.0f - target_voltage / v_i);
                right_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BOOST:
            if (last_mode == BOOST) {
                right_bridge.add_duty(pid_out);
                right_bridge.force_larger_than(config.limits.min_duty);
            } else {
                // 前馈
                right_bridge.set_duty(1.0f - v_i / target_voltage);
                left_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BUCKBOOST:
            if (last_mode == BUCKBOOST) {
                // PID 分配
                right_bridge.add_duty(0.15f * pid_out);
                right_bridge.force_smaller_than(config.limits.max_buckboost_duty);
                right_bridge.force_larger_than(config.limits.min_duty);

                // 启动时的特殊限幅
                // ReSharper disable once CppDFAConstantConditions
                float left_limit = start_up_info.starting ? 0.12f : config.limits.max_buckboost_duty;
                left_bridge.add_duty(-0.2f * pid_out);
                left_bridge.force_smaller_than(left_limit);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                if (last_mode == OFF) {
                    // 软启动逻辑
                    // 现在的问题是刚启动电流放出的太厉害
                    // 从左往右看，如果要电容充电，那就升高电压，boost，放电则需要降低电压
                    // 所以如果启动电流放的太厉害，那就代表boost太弱，应该更多boost以及更少buck
                    // 反映到下面的代码就是左半桥初始占空比小一点，算出来的升高ratio大一点
                    float ratio = v_c / (v_i * 0.8f);
                    float right_d = 1.0f - 1.0f / ratio;
                    if (right_d < 0) right_d = 0.1f;
                    if (right_d > 0.8f) right_d = 0.5f;

                    left_bridge.set_duty(0.12f);
                    right_bridge.set_duty(right_d);
                } else {
                    right_bridge.add_duty(0.12f);
                    left_bridge.add_duty(0.12f);
                }
                ipid.reset();
                switch_mode_reset();
            }
            break;

        case OFF:
            left_bridge.stop();
            right_bridge.stop();
            break;
    }
}

// void BuckBoostManager::v_out_i_in_control() {
//
// }
//
// void BuckBoostManager::i_out_v_in_control() {
//
// }