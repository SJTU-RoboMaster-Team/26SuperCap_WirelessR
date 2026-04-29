//
// Created by yixin on 2024/1/19.
//
#include "power_manager.h"
#include "types.h"
#include "referee_comm.h"
#include "calibrator_estimator.h"
#include "adc_sample.h"
#include "can_task.h"
#include "../config.h"
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
        target_buffer = 40;
        power_parameter.buffer_energy = 40;
        power_limit = max_charging_pwr = power_parameter.chassis_power_limit = 50;
    }

    // 状态检查
    check_cap_status();

    // 无线充覆盖逻辑（功率限制、放电截止、自动退出、断联清空）
    handle_wireless_override();

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

    switch (mode) {
        case BUFFER:
            buffer_mode();
            break;
        case CALIBRATE:
            calibrate_mode();
            break;
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
    // 更新全部电压迟滞比较器
    cap_full_smtrg.update(bbm.filtered_v_c.val);
    cap_limit_60power_smtrg.update(bbm.filtered_v_c.val);
    cap_limit_30power_smtrg.update(bbm.filtered_v_c.val);
    cap_low_smtrg.update(bbm.filtered_v_c.val);
    wireless_discharge_ok_smtrg.update(bbm.filtered_v_c.val);

    // 将施密特触发器状态同步到标志位
    full_charge = cap_full_smtrg.state;   // 电压 ≥ 26V 后置位，≤ 25.5V 复位
    empty       = !cap_low_smtrg.state;    // 电压 ≤ 10V 后置位，≥ 11.5V 复位

    // 根据电容电压分级限制最大放电功率
    if (!cap_limit_30power_smtrg.state) {
        max_releasing_pwr = -30.0f;        // 电压极低 (<12V)：严格限制放电
        msg.under_voltage_flag = 0x01;
    } else if (!cap_limit_60power_smtrg.state) {
        max_releasing_pwr = -60.0f;        // 电压偏低 (<15V)：中等限制
        msg.under_voltage_flag = 0x01;
        under_voltage_tick = 0;
    } else {
        max_releasing_pwr = charge_parameter.allowed_max_releasing_pwr;  // 电压正常：不限制
        msg.under_voltage_flag = 0x00;
    }
}

    // Buffer Mode 目标：
void PowerManager::buffer_mode() {
    //   底盘实际功率 ≤ 裁判系统功率上限
    //   利用超级电容吸收/释放功率差额，维护 buffer 能量在安全水位

    // 阶段1：计算 buffer 能量补偿量 (compensation)
    compute_buffer_compensation();

    // 阶段2：启停决策
    const bool over_power    = (filtered_out_pwr.val > power_limit);
    const bool buffer_low    = (power_parameter.buffer_energy < 60);
    const bool power_surplus = (filtered_out_pwr.val < power_limit);

    if ((over_power && !empty) || buffer_low) {
        // 需要放电补充 或 buffer 危急 → 启动
        bbm.start();
    } else if (full_charge) {
        // 满电且功率充足 → 转恒压浮充，退出功率控制
        compensation   = 0.0f;
        cap_pwr_pid.reset();
        target_cap_pwr = 0.0f;
        bbm.set_voltage_loop(charge_parameter.full_charge_target_v);
        return;
    }

    if (power_surplus && !full_charge) {
        // 有富余功率且未满电 → 启动充电
        bbm.start();
    } else if (empty) {
        // 电容接近耗尽 → 停机，零电流待机
        bbm.stop();
        compensation = 0.0f;
        cap_pwr_pid.reset();
        bbm.set_current_loop(bbm.v_c, 0.0f);
        // 不 return：is_working() = false，功率控制阶段会自然跳过
    }

    // 满电时禁止充电（允许放电补给底盘）
    if (full_charge) {
        target_cap_pwr = my_math::limit_max(target_cap_pwr, 0.0f);
    }

    // 阶段3：目标功率计算（仅在 DC/DC 运行时执行）
    if (bbm.is_working()) {
        compute_and_apply_target_power();
    }
}

void PowerManager::compute_buffer_compensation() {
    if (!bbm.is_working()) {
        referee_buffer_pid.reset();
        compensation = 0.0f;
        return;
    }

    // buffer_surplus > 0: buffer 有余量; < 0: buffer 不足
    int32_t buffer_surplus = power_parameter.buffer_energy - target_buffer;

    // 大功率输出时降低 buffer 补偿权重，优先保障电容放电响应
    if (buffer_surplus > 0 && filtered_out_pwr.val > 180.0f) {
        buffer_surplus = (int32_t)((float)buffer_surplus * 0.25f);
    }

    // 动态 PID 增益：底盘功率越大，响应越激进
    if      (filtered_out_pwr.val > 250.0f) referee_buffer_pid.kp = 6.5f;
    else if (filtered_out_pwr.val > 150.0f) referee_buffer_pid.kp = 3.5f;
    else                                     referee_buffer_pid.kp = 1.5f;

    // 施密特触发器判断偏差是否超出死区（阈值 3J/7J）
    buffer_far_from_target.update(buffer_surplus);
    referee_buffer_pid.update(0.0f, (float)buffer_surplus);

    // 仅在未满电或 buffer 危急时计算补偿
    if (!full_charge || power_parameter.buffer_energy < 60) {
        // PID ref=0, fdb=buffer_surplus，输出符号说明：
        //   buffer_surplus < 0 (buffer不足) → PID输出 > 0 → compensation < 0 → 电容多放电
        //   buffer_surplus > 0 (buffer过多) → PID输出 < 0 → compensation > 0 → 电容多充电
        // 偏差大时用完整输出，接近目标时仅用积分项以减少抖动
        compensation = (buffer_far_from_target.state)
                     ? (-referee_buffer_pid.get_output())
                     : (-referee_buffer_pid.get_iout());
    } else {
        compensation = 0.0f;
    }

    if (is_wireless_override_active()) {
        compensation = my_math::limit_max(compensation, 0.0f); // 无线充模式下只允许负补偿促放电，不允许正补偿抵消放电
    }
}

void PowerManager::compute_and_apply_target_power() {
    // 期望功率差 = 功率上限 - 底盘实际功率 + buffer 补偿
    //   pwr_gap > 0: 功率有余 → 充电
    //   pwr_gap < 0: 功率不足 → 放电
    float pwr_gap = power_limit - filtered_out_pwr.val + compensation;

    // 滞后补偿：若电容功率跟随存在滞后，融合当前测量值以平滑控制
    if (follow_trend_estimator.trend == FollowTrendEstimator::LAGGING) {
        float smoothed = pwr_gap * 0.3f + filtered_cap_pwr.val * 0.7f;
        target_cap_pwr += chassis_pwr_pid.update(smoothed, target_cap_pwr);
    } else {
        target_cap_pwr += chassis_pwr_pid.update(pwr_gap, target_cap_pwr);
    }

    // 目标功率限幅
    if (bbm.start_up_info.starting) {
        // 启动阶段：充电限幅，放电不限（快速建立母线电压）
        target_cap_pwr = my_math::limit(target_cap_pwr,
                                        charge_parameter.allowed_max_releasing_pwr, 100.0f);
    } else {
        target_cap_pwr = my_math::limit(target_cap_pwr, max_releasing_pwr, max_charging_pwr);
    }

    // 电容耗尽时请求微量充电，维持系统可用
    if (empty) {
        target_cap_pwr = 10.0f;
    }

    // 功率外环 → 电流内环：target_i = 当前设定 + PID(P_target, P_actual)
    float target_i = bbm.target_current + cap_pwr_pid.update(target_cap_pwr, filtered_cap_pwr.val);

    // 若控制严重滞后且处于 Buck 充电模式，升级至 BuckBoost 以提升响应带宽
    if (follow_trend_estimator.trend == FollowTrendEstimator::LAGGING
        && bbm.get_mode() == BuckBoostManager::BUCK
        && state == CHARGING) {
        bbm.upgrade();
        cap_pwr_pid.reset();
        target_i = target_cap_pwr / bbm.filtered_v_c.val;
    }

    bbm.set_current_loop(bbm.filtered_v_c.val, my_math::limit_abs(target_i, 25.0f));
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

void PowerManager::handle_wireless_override() {
    // 退出路径共三条：
    //   1. wireless_active 下降沿（离开充电区）→ clear_wireless_state()
    //   2. 充满上升沿（16V→25.5V 循环完成）→ 自动清零 desired_power_limit
    //   3. 主机主动下发 desired_power_limit = 0（如出发后临时取消）
    // 若主机下发命令但最终未去充电区，超电侧不做超时处理，由主机负责取消。

    // 检测 wireless_active 下降沿（离开充电区）
    if (!wireless_active && last_wireless_active_) {
        clear_wireless_state();
    }
    last_wireless_active_ = wireless_active;

    // 若未激活覆盖，恢复正常功率限制
    if (!is_wireless_override_active()) {
        return;
    }

    // ─── 以下逻辑仅在覆盖激活时执行 ───

    // 1. 覆盖功率限制（由底盘通过 CAN 命令指定，超电内部决定具体功率值）
    power_limit = max_charging_pwr = 5.0f;

    // 2. 无线充电模式下放电截止：电容放至 16V 后禁止继续放电
    if (!wireless_discharge_ok_smtrg.state) {
        max_releasing_pwr = 0.0f;
    }

    // 3. 检测 wireless_discharge_ok_smtrg 上升沿（充满时自动退出覆盖）
    if (wireless_discharge_ok_smtrg.state && !last_wireless_discharge_ok_) {
        chassis_cmd_.desired_power_limit = 0; // 一次充放电循环完成，清除标志位，退出覆盖
    }
    last_wireless_discharge_ok_ = wireless_discharge_ok_smtrg.state;
}

void PowerManager::clear_wireless_state() {
    chassis_cmd_.desired_power_limit = 0;
    wireless_discharge_ok_smtrg.state = true; // 复位为"允许放电"初始状态
    last_wireless_discharge_ok_ = true;
}
