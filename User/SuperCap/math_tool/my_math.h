//
// Created by yixin on 2024/1/18.
//

#ifndef SUPER_CAP_MY_MATH_H
#define SUPER_CAP_MY_MATH_H

#include "stm32g474xx.h"
#include "../flash_data_structure.h"

class PID {
public:
    PID(float kp, float ki, float kd, float i_max, float out_max) :
            kp(kp), ki(ki), kd(kd), i_max(i_max), out_max(out_max) {
        err = 0;
        err_sum = 0;
        last_err = 0;
        pout = 0;
        iout = 0;
        dout = 0;
        output = 0;
    }


    float update(float ref_, float fdb_);
    float get_output() { return output; }
    float get_iout() { return iout; }
    float get_pout() { return pout; }
    float get_dout() { return dout; }


    void reset();

    void load_flash_parameter(const PID_FLASH_STORAGE& flash_storage){
        if (flash_storage.SOF!=PID_FLASH_STORAGE_SOF)return;
        id = flash_storage.id;
        kp = flash_storage.kp;
        ki = flash_storage.ki;
        kd = flash_storage.kd;
        i_max = flash_storage.i_max;
        out_max = flash_storage.out_max;
        reset();
    };

    PID_FLASH_STORAGE save_flash_parameter(uint16_t id_) const{
        PID_FLASH_STORAGE flash_storage;
        flash_storage.id = id_;
        flash_storage.kp = kp;
        flash_storage.ki = ki;
        flash_storage.kd = kd;
        flash_storage.i_max = i_max;
        flash_storage.out_max = out_max;
        return flash_storage;
    };

    float kp, ki, kd;
    float i_max, out_max;
private:
    float ref{}, fdb{};
    float err, err_sum, last_err;
    float pout, iout, dout;
    float output;
    uint16_t id{0};

};

namespace my_math {
    float limit(float value, float min, float max);

    float limit_abs(float value, float max);

    float limit_max(float value, float max);

    float limit_min(float value, float min);

    float abs(float val);

    bool near(float val, float target, float threshold);

    bool near_percentage(float val, float target, float percentage);
}

class LowPassFilter {
public:
    float val = 0;
    float rate = 0.01;

    LowPassFilter(float v, float rate) : val(v), rate(rate) {};

    float update(float v) {
        val = v * rate + val * (1 - rate);
        return val;
    }
};

template<typename T>
class SchmittTrigger {
public:
    SchmittTrigger(T upper_threshold, T lower_threshold, bool state) : upper_threshold(upper_threshold),
                                                                    lower_threshold(lower_threshold), state(state) {};

    T upper_threshold, lower_threshold;
    bool state;

    bool update(T v) {
        if (!state && v > upper_threshold) {
            state = true;
            return state;
        }
        if (state && v < lower_threshold) {
            state = false;
            return state;
        }
        return state;
    }

};


/** allow user to find that the fdb cannot follow ref well
 *
 */
class FollowTrendEstimator {
public:
    FollowTrendEstimator(float threshold, uint16_t cnt_threshold) : threshold(threshold),
                                                                    count_threshold(cnt_threshold) {
    };

    enum Trend : int8_t {
        LAGGING = 1,
        NORMAL = 0,
        LEADING = 2
    } trend = NORMAL, last_tick_trend = NORMAL, this_trend = NORMAL;

    uint32_t cnt = 0;

    float threshold = 10;
    uint32_t count_threshold = 50;

    Trend update(float ref_minus_fdb) {
        if (ref_minus_fdb > threshold) {
            this_trend = LAGGING;
        } else if (ref_minus_fdb < -threshold) {
            this_trend = LEADING;
        } else {
            this_trend = NORMAL;
        }
        if (this_trend != last_tick_trend) {
            cnt = 0;
        } else {
            cnt++;
        }
        if (cnt > count_threshold) {
            trend = last_tick_trend;
        }
        last_tick_trend = this_trend;
        return trend;
    }
};

class AverageFilter {
public:
    explicit AverageFilter(uint16_t size) : size(size) {
        buffer = new float[size];
        if (buffer == nullptr) {
            valid = false;
            return;
        }
        valid = true;
        for (int i = 0; i < size; i++) {
            buffer[i] = 0;
        }
    }

    float update(float v) {
        if (!valid){
            return 0;
        }
        sum -= buffer[head];
        buffer[head] = v;
        sum += v;
        head = (head + 1) % size;
        val = sum / (float )size;
        write_times++;
        return val;
    }

    float get() const {
        if (!valid){
            return 0;
        }
        return val;
    }

    uint32_t get_write_times() const{
        return write_times;
    }

    void reset(){
        if (!valid)return;
        for (int i = 0; i < size; ++i) {
            buffer[i] =0;
        }
        head = 0;
        sum = 0;
        val = 0;
        write_times = 0;
    }
private:
    float *buffer;
    uint16_t size;
    uint16_t head = 0;
    float sum = 0;
    bool valid = false;
    float val = 0;
    uint32_t write_times = 0;
};

class AverageFilterInt {
public:
    explicit AverageFilterInt() {
        if (buffer == nullptr) {
            valid = false;
            return;
        }
        valid = true;
        for (int i = 0; i < size; i++) {
            buffer[i] = 0;
        }
    }

    int32_t update(int32_t v) {
        if (!valid){
            return 0;
        }
        sum -= buffer[head];
        buffer[head] = v;
        sum += v;
        head = (head + 1) % size;
        val = sum / size;
        write_times++;
        return val;
    }

    int32_t get() const {
        if (!valid){
            return 0;
        }
        return val;
    }

    uint32_t get_write_times() const{
        return write_times;
    }

    void reset(){
        if (!valid)return;
        for (int i = 0; i < size; ++i) {
            buffer[i] =0;
        }
        head = 0;
        sum = 0;
        val = 0;
        write_times = 0;
    }
private:
    int32_t buffer[20]{};
    uint16_t size = 20;
    uint16_t head = 0;
    int32_t sum = 0;
    bool valid = false;
    int32_t val = 0;
    uint32_t write_times = 0;
};

class LinearRegressionModel {
public:
    LinearRegressionModel() {
        size = 0;
        sum_x = 0;
        sum_y = 0;
        sum_x2 = 0;
        sum_y2 = 0;
    }


    void regress(){
        if (size<=0)return;
        Sxx = sum_x2 - sum_x * sum_x / (float )size;
        Syy = sum_y2 - sum_y * sum_y / (float )size;
        Sxy = sum_xy - sum_x * sum_y / (float )size;
        a = Sxy / Sxx;
        float x_mean = sum_x / (float )size;
        float y_mean = sum_y / (float )size;
        b = y_mean - a * x_mean;
    }

    void evaluate(){
        if (size<=0)return;
        SST = Syy;
        SSE = Syy - Sxy * Sxy / Sxx;
        R2 = 1 - SSE / SST;
    }

    void add_point(float x,float y){
        size++;
        sum_x += x;
        sum_y += y;
        sum_xy += x*y;
        sum_x2 += x*x;
        sum_y2 += y*y;
    }

    // y = a * x + b
    float get_a() const {
        return a;
    }

    // y = a * x + b
    float get_b() const {
        return b;
    }

    float get_r2() const {
        return R2;
    }

    void reset(){
        size = 0;
        sum_x = 0;
        sum_y = 0;
        sum_x2 = 0;
        sum_y2 = 0;
        sum_xy = 0;
        SST = 0;
        SSE = 0;
        SSR = 0;
        R2 = 0;
    }

private:
    uint32_t size{};
    float a{},b{};
    float Sxx{},Syy{},Sxy{};
    float sum_x{},sum_y{},sum_xy{};
    float sum_x2{},sum_y2{};
    float SST{},SSE{},SSR{};
    float R2{};
};

class LinearCalibrator {
public:
    LinearCalibrator(float k, float b) : k(k), b(b) {};
    float k=1, b=0;

    float val(float v) {
        return k * v + b;
    }
};

/**
 ******************************************************************************
 * @file    crc.cpp/h
 * @brief   CRC calculation. CRC计算
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "stm32g4xx_hal.h"

// CRC8 Calculate. CRC8计算
uint8_t CRC8_Calc(uint8_t *data, uint32_t len);

// CRC8 verify. CRC8校验函数
uint8_t CRC8_Verify(uint8_t *data, uint32_t len);

// Append CRC8 to the end of data. 添加CRC8到数据的结尾
void CRC8_Append(uint8_t *data, uint32_t len);

// CRC16 Calculate. CRC16计算
uint16_t CRC16_Calc(uint8_t *data, uint32_t len);

// CRC16 verify. CRC16校验函数
uint8_t CRC16_Verify(uint8_t *data, uint32_t len);

// Append CRC16 to the end of data. 添加CRC16到数据的结尾
void CRC16_Append(uint8_t *data, uint32_t len);


#endif //SUPER_CAP_MY_MATH_H
