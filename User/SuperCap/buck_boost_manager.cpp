//
// Created by yixin on 2024/1/19.
//

#include <cmath>
#include "buck_boost_manager.h"
#include "../callback.h"

extern RefreshFlag refresh_flag;

namespace {
float clamp_duty(float duty, float min_duty, float max_duty) {
    return my_math::limit(duty, min_duty, max_duty);
}
}

void BuckBoostManager::handle() {
    if (!safety_check()) {
        stop();
        return;
    }

    target_current = my_math::limit_abs(target_current, config.limits.max_target_current);
    mode_switcher();

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

    switch (control_mode) {
        case V_LOOP:
            v_loop_control();
            break;
        case I_LOOP:
            i_loop_control();
            break;
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

float BuckBoostManager::get_mode_reference_voltage() const {
    if (control_mode == V_LOOP) {
        return target_voltage;
    }
    return filtered_v_c.val;
}

BuckBoostManager::Mode BuckBoostManager::decide_target_mode(float mode_v_in, float mode_v_ref) const {
    switch (mode) {
        case OFF:
            if (mode_v_in < mode_v_ref + config.thresholds.buck_to_buckboost_gap) {
                return BUCKBOOST;
            }
            return BUCK;

        case BUCK:
            if (mode_v_in < mode_v_ref + config.thresholds.buck_to_buckboost_gap) {
                return BUCKBOOST;
            }
            return BUCK;

        case BOOST:
            if (mode_v_ref < mode_v_in + config.thresholds.boost_to_buckboost_gap) {
                return BUCKBOOST;
            }
            return BOOST;

        case BUCKBOOST:
            if (mode_v_ref < mode_v_in - config.thresholds.buckboost_exit_gap_buck) {
                return BUCK;
            }
            if (mode_v_ref > mode_v_in + config.thresholds.buckboost_exit_gap_boost) {
                return BOOST;
            }
            return BUCKBOOST;

        default:
            return mode;
    }
}

void BuckBoostManager::mode_switcher() {
    if (!enable) {
        mode = OFF;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
        return;
    }

    const float mode_v_in = filtered_v_i.val;
    const float mode_v_ref = get_mode_reference_voltage();
    const Mode target_mode = decide_target_mode(mode_v_in, mode_v_ref);

    if (target_mode == mode) {
        pending_mode = OFF;
        pending_mode_ticks = 0;
        if (mode_stable_ticks < UINT32_MAX) {
            mode_stable_ticks++;
        }
        return;
    }

    if (mode != OFF && mode_stable_ticks < switching_config.min_mode_hold_ticks) {
        pending_mode = OFF;
        pending_mode_ticks = 0;
        if (mode_stable_ticks < UINT32_MAX) {
            mode_stable_ticks++;
        }
        return;
    }

    if (mode == OFF) {
        mode = target_mode;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
        return;
    }

    if (pending_mode != target_mode) {
        pending_mode = target_mode;
        pending_mode_ticks = 1;
        return;
    }

    if (pending_mode_ticks < UINT32_MAX) {
        pending_mode_ticks++;
    }

    if (pending_mode_ticks >= switching_config.confirm_ticks) {
        mode = target_mode;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
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
                left_bridge.set_duty(clamp_duty(1.0f - target_voltage / v_i,
                                                config.limits.min_duty,
                                                config.limits.max_duty));
                right_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BOOST:
            if (last_mode == BOOST) {
                right_bridge.add_duty(pid_out);
                right_bridge.force_larger_than(config.limits.min_duty);
            } else {
                const float ff_duty = clamp_duty(1.0f - v_i / target_voltage,
                                                 config.limits.min_duty,
                                                 config.limits.max_duty);
                if (last_mode == BUCKBOOST) {
                    right_bridge.set_duty(clamp_duty(right_bridge.duty,
                                                     config.limits.min_duty,
                                                     config.limits.max_duty));
                } else {
                    right_bridge.set_duty(ff_duty);
                }
                left_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BUCKBOOST:
            if (last_mode == BUCKBOOST) {
                right_bridge.add_duty(0.5f * pid_out);
                right_bridge.force_smaller_than(config.limits.max_buckboost_duty);
                right_bridge.force_larger_than(config.limits.min_duty);

                left_bridge.add_duty(-0.3f * pid_out);
                left_bridge.force_smaller_than(config.limits.max_buckboost_duty);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                if (last_mode == BOOST) {
                    right_bridge.set_duty(clamp_duty(right_bridge.duty,
                                                     config.limits.min_duty,
                                                     config.limits.max_buckboost_duty));
                    left_bridge.set_duty(config.limits.min_duty);
                } else if (last_mode == BUCK) {
                    left_bridge.set_duty(clamp_duty(left_bridge.duty,
                                                    config.limits.min_duty,
                                                    config.limits.max_buckboost_duty));
                    right_bridge.set_duty(config.limits.min_duty);
                } else {
                    right_bridge.set_duty(0.24f);
                    left_bridge.set_duty(0.15f);
                }
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
    if (start_up_info.starting) {
        target_current = my_math::limit_abs(target_current, 5.0f);
    }

    float pid_out = ipid.update(target_current, i_c);

    switch (mode) {
        case BUCK:
            if (last_mode == BUCK) {
                left_bridge.add_duty(-0.3f * pid_out);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                debug.v_just_read = v_i;
                debug.calc_res = 1.0f - target_voltage / v_i;
                left_bridge.set_duty(clamp_duty(1.0f - target_voltage / v_i,
                                                config.limits.min_duty,
                                                config.limits.max_duty));
                right_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BOOST:
            if (last_mode == BOOST) {
                right_bridge.add_duty(pid_out);
                right_bridge.force_larger_than(config.limits.min_duty);
            } else {
                const float ff_duty = clamp_duty(1.0f - v_i / target_voltage,
                                                 config.limits.min_duty,
                                                 config.limits.max_duty);
                if (last_mode == BUCKBOOST) {
                    right_bridge.set_duty(clamp_duty(right_bridge.duty,
                                                     config.limits.min_duty,
                                                     config.limits.max_duty));
                } else {
                    right_bridge.set_duty(ff_duty);
                }
                left_bridge.set_duty(config.limits.min_duty);
                switch_mode_reset();
            }
            break;

        case BUCKBOOST:
            if (last_mode == BUCKBOOST) {
                right_bridge.add_duty(0.15f * pid_out);
                right_bridge.force_smaller_than(config.limits.max_buckboost_duty);
                right_bridge.force_larger_than(config.limits.min_duty);

                float left_limit = start_up_info.starting ? 0.12f : config.limits.max_buckboost_duty;
                left_bridge.add_duty(-0.2f * pid_out);
                left_bridge.force_smaller_than(left_limit);
                left_bridge.force_larger_than(config.limits.min_duty);
            } else {
                if (last_mode == OFF) {
                    float ratio = v_c / (v_i * 0.8f);
                    float right_d = 1.0f - 1.0f / ratio;
                    if (right_d < 0) right_d = 0.1f;
                    if (right_d > 0.8f) right_d = 0.5f;

                    left_bridge.set_duty(0.12f);
                    right_bridge.set_duty(right_d);
                } else if (last_mode == BOOST) {
                    right_bridge.set_duty(clamp_duty(right_bridge.duty,
                                                     config.limits.min_duty,
                                                     config.limits.max_buckboost_duty));
                    left_bridge.set_duty(config.limits.min_duty);
                } else if (last_mode == BUCK) {
                    left_bridge.set_duty(clamp_duty(left_bridge.duty,
                                                    config.limits.min_duty,
                                                    config.limits.max_buckboost_duty));
                    right_bridge.set_duty(config.limits.min_duty);
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
