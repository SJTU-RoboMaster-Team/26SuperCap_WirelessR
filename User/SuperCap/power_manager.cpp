//
// Created by yixin on 2024/1/19.
//
#include "power_manager.h"
#include "types.h"
#include "referee_comm.h"
#include "calibrator_estimator.h"
#include "adc_sample.h"
#include "can_task.h"

extern ADC_Buffer adc_buffer;
extern RefereeComm referee;
extern Calibrator cali_estimator;
extern CapMsg msg;

void PowerManager::load_flash_parameter(const PID_FLASH_STORAGE& cap_pwr_pid_flash_storage,
                                        const PID_FLASH_STORAGE& chasiss_pwr_pid_flash_storage,
                                        const PID_FLASH_STORAGE& referee_buffer_pid_flash_storage) {
    cap_pwr_pid.load_flash_parameter(cap_pwr_pid_flash_storage);
    chassis_pwr_pid.load_flash_parameter(chasiss_pwr_pid_flash_storage);
    referee_buffer_pid.load_flash_parameter(referee_buffer_pid_flash_storage);
}

void PowerManager::handle() {
    // 功率计算与滤波
    cap_pwr = bbm.filtered_i_c.val * bbm.filtered_v_c.val;
    out_pwr = bbm.filtered_i_o.val * bbm.filtered_v_i.val;
    in_pwr = bbm.filtered_i_i.val * bbm.filtered_v_i.val;

    filtered_out_pwr.update(out_pwr);
    filtered_in_pwr.update(in_pwr);

    // 如果硬件没工作或处于电压环，认为电容功率为0
    if (!bbm.is_working() || bbm.is_voltage_control_loop()) {
        filtered_cap_pwr.update(0);
    } else {
        filtered_cap_pwr.update(cap_pwr);
    }
    //    filtered_cap_pwr.val = filtered_in_pwr.val - filtered_out_pwr.val;
    in_minus_out_pwr = filtered_in_pwr.val - filtered_out_pwr.val;

    // 更新裁判系统发送的功率上限与缓冲能量
    if (referee.connected()) {
        target_buffer = 40;
        power_parameter = referee.get_parameter();
        power_limit = max_charging_pwr = power_parameter.chassis_power_limit;
    }else {
        // 裁判系统断联！！！
        target_buffer = 60;
        power_parameter.buffer_energy = 60;
        power_limit = max_charging_pwr = power_parameter.chassis_power_limit = 50;
    }

    //    calibrate_info.in_minus_out_minus_cap = in_minus_out_pwr - filtered_cap_pwr.val;
    // if (my_math::abs(in_minus_out_pwr) > 10) {
    //        calibrate_info.in_minus_out_over_cap_pwr = in_minus_out_pwr / filtered_cap_pwr.val;
    // }
    //    chassis_pwr = referee.power_heat_data_.chassis_power;

    // 状态检查
    check_cap_status();

    // 更新跟随趋势 (超前/滞后)
    follow_trend_estimator.update(target_cap_pwr - filtered_cap_pwr.val); // lagging和leading的标准是相对给电容组充电而言

    // 确定当前充放电状态(UI)
    if (!bbm.is_working() || full_charge) {
        state = IDLE;
    } else {
        if (cap_pwr > 1.0f) state = CHARGING;
        else if (cap_pwr < -1.0f) state = DISCHARGING;
        else state = IDLE;
    }

    // 执行控制模式
    switch (mode) {
        case BUFFER:
            buffer_mode();
            break;
        case QUICK_CHARGE:
            // TODO: 实现快充逻辑
            break;
        case CALIBRATE:
            calibrate_mode();
        default:
            break;
    }

    // 错误检测
    if (bbm.v_i < 15.0f) {
        err_code |= 4; // 输入欠压
        bbm.stop();
    } else {
        err_code &= ~4;
    }
}

void PowerManager::check_cap_status() {
    cap_full_smtrg.update(bbm.filtered_v_c.val);
    cap_limit_60power_smtrg.update(bbm.filtered_v_c.val);
    cap_limit_30power_smtrg.update(bbm.filtered_v_c.val);
    cap_low_smtrg.update(bbm.filtered_v_c.val);

    // 将施密特触发器状态同步到标志位
    full_charge = cap_full_smtrg.state;   // 电压 ≥ 26V 后置位，≤ 25.5V 复位
    empty       = cap_low_smtrg.state;    // 电压 ≤ 10V 后置位，≥ 11.5V 复位

    // 根据电容电压分级限制最大放电功率
    if (!cap_limit_30power_smtrg.state) {
        max_releasing_pwr = -30.0f;        // 电压极低 (<12V)：严格限制放电
    } else if (!cap_limit_60power_smtrg.state) {
        max_releasing_pwr = -60.0f;        // 电压偏低 (<15V)：中等限制
        msg.under_voltage_flag = 0x01;
        under_voltage_tick = 0;
    } else {
        max_releasing_pwr = charge_parameter.allowed_max_releasing_pwr;  // 电压正常：不限制
    }
}

void PowerManager::buffer_mode() {
    //
    // 尽量保证底盘功率 < 裁判系统限制
    // 利用多余功率给电容充电
    // 功率不足时电容放电补充
    // 保持 buffer 能量在安全水位

    int32_t buffer_minus_target = power_parameter.buffer_energy - target_buffer;

    /* 策略1：计算Compensation */
    // 大功率输出时，提升cap端输出权重，减少buffer充电补偿
    if (buffer_minus_target >= 0 && filtered_out_pwr.val > 180) {
        buffer_minus_target = (int16_t)((float)buffer_minus_target * 0.25);
    }

    // 0. + 抬高从chassis口出来的用电量; - 减少; +-指的是cap端的充放电
    // 1. 如果缓冲能量低了，则稍微降低功率，把缓冲能量补回去
    // 2. 如果底盘功率高了，则加大电容输出
    // 3. 如果底盘功率低了，就充电
    // 4. 如果底盘功率低，而且满电了，就不充电
    // 5. 不论如何最终以裁判系统为准
    if (bbm.is_working()) {
        if (filtered_out_pwr.val > 250) {
            referee_buffer_pid.kp = 6.5;
        } else if (filtered_out_pwr.val > 150) {
            referee_buffer_pid.kp = 3.5;
        } else {
            referee_buffer_pid.kp = 1.5;
        }
        buffer_far_from_target.update(buffer_minus_target); // 3, 7
        referee_buffer_pid.update(0, (float)buffer_minus_target);

        // 如果未满电 或者 Buffer 极低 (<60J)，允许补偿充电
        if (!full_charge || power_parameter.buffer_energy < 60) {
            // compensation < 0: 减少从 chassis 取电 (放电/少充)
            // compensation > 0: 增加从 chassis 取电 (充电)
            // PID 输出负表示 buffer 太多 -> 需要增加取电 -> compensation 正
            if (buffer_far_from_target.state == true) {
                compensation = -referee_buffer_pid.get_output(); // compensation是电容组充电补偿,+充电,-放电
            } else {
                compensation = -referee_buffer_pid.get_iout();
            }
        } else {
            compensation = 0;
        }
    } else {
        referee_buffer_pid.reset();
        compensation = 0;
    }

    /* 策略2: 启停 */
    bool over_power = filtered_out_pwr.val > power_limit;
    bool buffer_low = power_parameter.buffer_energy < 60;
    bool power_surplus = filtered_out_pwr.val < power_limit;

    if ((over_power && !empty) || buffer_low) {
        // 需要放电 或者 Buffer 危急 -> 启动
        bbm.start();
    } else if (full_charge) {
        // 满电且功率充足 -> 待机，转为恒压模式维持电压
        compensation = 0;
        cap_pwr_pid.reset();
        target_cap_pwr = 0;
        bbm.set_voltage_loop(charge_parameter.full_charge_target_v);
        return;
    }

    if (power_surplus && !full_charge) {
        // 有富余功率且没满电 -> 启动充电
        bbm.start();
    } else if (empty) {
        // 没电 -> 停机
        bbm.stop();
        compensation = 0;
        cap_pwr_pid.reset();
        bbm.set_current_loop(bbm.v_c, 0); // 零电流待机
    }

    if (full_charge) {
        // 满电不允许继续充电
        target_cap_pwr = my_math::limit_max(target_cap_pwr, 0);
    }

    /* 计算最终目标功率 */
    if (bbm.is_working()) {
        // 目标电容功率 = 裁判限制 - 当前底盘实际消耗 + 补偿
        float pwr_gap = power_limit - filtered_out_pwr.val + compensation;

        // 平滑控制：如果反应滞后，融合历史值
        if (follow_trend_estimator.trend == FollowTrendEstimator::LAGGING) {
            float smoothed_input = pwr_gap * 0.3f + filtered_cap_pwr.val * 0.7f;
            target_cap_pwr += chassis_pwr_pid.update(smoothed_input, target_cap_pwr);
        } else {
            target_cap_pwr += chassis_pwr_pid.update(pwr_gap, target_cap_pwr);
        }
        //        target_cap_pwr = power_limit + compensation - out_pwr;

        // 限幅
        if (bbm.start_up_info.starting) {
            // 启动阶段充电限幅，放电不限幅，快速建立母线电压
            target_cap_pwr = my_math::limit(target_cap_pwr, charge_parameter.allowed_max_releasing_pwr, 100);
        } else {
            target_cap_pwr = my_math::limit(target_cap_pwr, max_releasing_pwr, max_charging_pwr);
        }

        if (empty) {
            target_cap_pwr = 10.0f;
        }

        // I = P/U PID
        float target_i = bbm.target_current + cap_pwr_pid.update(target_cap_pwr, filtered_cap_pwr.val);
        if (follow_trend_estimator.trend == FollowTrendEstimator::LAGGING && bbm.get_mode() == BuckBoostManager::BUCK &&
            state == CHARGING) {
            bbm.upgrade();
            cap_pwr_pid.reset();
            target_i = target_cap_pwr / bbm.filtered_v_c.val;
        }
        target_i = my_math::limit_abs(target_i, 25.0f);
        bbm.set_current_loop(bbm.filtered_v_c.val, target_i);
    }
}

void PowerManager::calibrate_mode() {
    if (cali_estimator.calibrate_step != Calibrator::I_CAP_STEP_LOAD) {
        bbm.stop();
    }

    cali_estimator.filtered_in_voltage.update((float)adc_buffer.v_in * V_IN_TO_FLOAT);
    cali_estimator.filtered_cap_voltage.update((float)adc_buffer.v_c * V_C_TO_FLOAT);
    cali_estimator.filtered_in_current.update((float)adc_buffer.i_in * I_IN_TO_FLOAT);
    cali_estimator.filtered_out_current.update((float)adc_buffer.i_o * I_O_TO_FLOAT);
    cali_estimator.filtered_cap_current.update((float)adc_buffer.i_c * I_C_TO_FLOAT);

    cali_estimator.start_calibrate();
    cali_estimator.handle();

    if (cali_estimator.calibrate_step == Calibrator::I_CAP_STEP_LOAD) {
        bbm.unlock();
        bbm.start();
        bbm.set_voltage_loop(18);
    }

    if (cali_estimator.calibrate_step == Calibrator::DONE) {
        //        mode = BUFFER;
        mode = STAND_BY;
    }
}