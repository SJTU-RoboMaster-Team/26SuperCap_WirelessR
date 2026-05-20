//
// Created by yixin on 2024/1/19.
//

#ifndef SUPER_CAP_BUCK_BOOST_MANAGER_H
#define SUPER_CAP_BUCK_BOOST_MANAGER_H


#include "half_bridge.h"
#include "./math_tool/my_math.h"

class BuckBoostManager {
public:
    struct Config {
        struct Thresholds {
            float min_working_voltage = 5.0f;      // 最小工作电压
            float max_cap_voltage = 30.0f;         // 最大电容电压
            float buck_to_buckboost_gap = 2.0f;    // Buck -> BuckBoost 切换阈值 (Vin - Vtarget)
            float boost_to_buckboost_gap = 3.5f;   // Boost -> BuckBoost 切换阈值 (Vtarget - Vin)
            float buckboost_exit_gap_buck = 3.0f;  // BuckBoost -> Buck 切换阈值
            float buckboost_exit_gap_boost = 5.0f; // BuckBoost -> Boost 切换阈值
        } thresholds;

        struct Limits {
            float min_duty = 0.05f;           // 最小占空比（死区保护/自举电容充电）
            float max_duty = 0.95f;           // 最大占空比
            float max_buckboost_duty = 0.48f; // Buck-Boost 模式下的单管最大占空比限制 todo 可调
            float max_target_current = 25.0f; // 最大目标电流
        } limits;
    } config;

    struct SwitchingConfig {
        uint32_t min_mode_hold_ticks = 250;
        uint32_t confirm_ticks = 20;
    } switching_config;

    BuckBoostManager(HalfBridge& left_bridge, HalfBridge& right_bridge, PID& vpid, PID& ipid):
        left_bridge(left_bridge),
        right_bridge(right_bridge),
        vpid(vpid),
        ipid(ipid) {};

    // 更新ADC数据
    void update_adc_data(float v_in, float v_out, float i_in, float i_out, float i_cap) {
        this->v_i = v_in;
        this->v_c = v_out;
        this->i_i = i_in;
        this->i_o = i_out;
        this->i_c = i_cap;

        // 更新滤波器
        filtered_i_c.update(i_c);
        filtered_v_c.update(v_c);
        filtered_v_i.update(v_i);
        filtered_i_i.update(i_i);
        filtered_i_o.update(i_o);
    }

    // 主控制函数
    void handle();

    void start() {
        if (safe_disable_lock) return;
        enable = true;
    }

    void stop() {
        enable = false;
        mode = OFF;
        last_mode = OFF;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
        left_bridge.stop();
        right_bridge.stop();
    }

    void safe_disable() {
        stop();
        safe_disable_lock = true;
    }

    void unlock() {
        safe_disable_lock = false;
    }


    bool is_working() {
        return mode != OFF;
    }

    bool is_enabled() {
        return enable;
    }

    bool is_voltage_control_loop() {
        return control_mode == V_LOOP;
    }

    void set_voltage_loop(float v) {
        target_voltage = v;
        control_mode = V_LOOP;
    };

    void set_current_loop(float v, float i) {
        target_current = i;
        target_voltage = v;
        control_mode = I_LOOP;
    };

    float v_i = 24, v_c = 0, i_i = 0, i_o = 0, i_c = 0;

    float target_voltage = 18; // V
    float target_current = 0;  // A

    LowPassFilter filtered_v_c = LowPassFilter(0.f, 0.01);
    LowPassFilter filtered_i_c = LowPassFilter(0.f, 0.01);
    LowPassFilter filtered_v_i = LowPassFilter(0.f, 0.01);
    LowPassFilter filtered_i_i = LowPassFilter(0.f, 0.01);
    LowPassFilter filtered_i_o = LowPassFilter(0.f, 0.01);

    enum Mode {
        BUCK,
        BOOST,
        BUCKBOOST,
        OFF,
    };

    struct {
        uint8_t starting = 0;
        bool started = false;
        uint32_t time_since_last_start = 0;
        uint32_t start_time_threshold = 25000; // 25000/25000s
    } start_up_info;


    void upgrade() {
        if (mode == BUCK)
            mode = BUCKBOOST;
        else if (mode == BUCKBOOST)
            mode = BOOST;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
    };

    void downgrade() {
        if (mode == BOOST)
            mode = BUCKBOOST;
        else if (mode == BUCKBOOST)
            mode = BUCK;
        pending_mode = OFF;
        pending_mode_ticks = 0;
        mode_stable_ticks = 0;
    };

    Mode get_mode() {
        return mode;
    }

private:
    HalfBridge &left_bridge, &right_bridge;
    PID& vpid;
    PID& ipid;

    Mode mode = OFF, last_mode = OFF;

    enum ControlMode {
        OPEN_LOOP,
        V_LOOP,
        I_LOOP,
        V_OUT_I_IN,
        I_OUT_V_IN,
    } control_mode = I_LOOP;

    volatile bool enable = false;
    bool safe_disable_lock = false;

    struct Debug {
        float v_just_read;
        float calc_res;
    } debug{};

    bool auto_mode_switcher = false;
    uint32_t mode_stable_ticks = 0;
    uint32_t pending_mode_ticks = 0;
    Mode pending_mode = OFF;

    void switch_mode_reset(); // 模式切换时的重置逻辑
    void mode_switcher();    // 状态机逻辑
    void v_loop_control();   // 电压环控制
    void i_loop_control();   // 电流环控制
    bool safety_check();     // 安全检查
    [[nodiscard]] Mode decide_target_mode(float mode_v_in, float mode_v_ref) const;
    [[nodiscard]] float get_mode_reference_voltage() const;
    // void v_out_i_in_control();
    // void i_out_v_in_control();

};

#endif //SUPER_CAP_BUCK_BOOST_MANAGER_H
