/**
  ******************************************************************************
  * @file    mp4201.c
  * @brief   MP4201 DMA-based Async Driver Implementation with PI Control
  * Optimized for Wireless Charging & Supercapacitor Load
  ******************************************************************************
  */

#include "../Inc/MP4201.h"
#include "../../config.h"
#include "stm32g4xx_hal.h"

// define static debug member for C++ wrapper
#ifdef __cplusplus
float MP4201::feedForwardDebug = 0.0f;
#endif

// 私有辅助函数
static HAL_StatusTypeDef MP4201_WriteReg_Blocking(MP4201_Handle_t* dev, uint8_t reg, uint16_t data);
static void MP4201_RunControlAlgorithm(MP4201_Handle_t* dev);

/**
  * @brief  初始化 (阻塞式)
  */
HAL_StatusTypeDef MP4201_Init(MP4201_Handle_t* dev, I2C_HandleTypeDef* hi2c) {
    dev->hi2c = hi2c;
    dev->devAddr = MP4201_ADDR_DEFAULT;
    dev->state = MP4201_STATE_IDLE;
    dev->controlMode = MP4201_MODE_CV;
    dev->isMaxIout = false;
    dev->isCVphase = false;
    dev->calculated_vout_cmd = 0;
    dev->calculated_iout_cmd = 0;
    dev->errorCount = 0;
    dev->wpt_switch_flag_ = false;
    dev->is_connected = false;

    // 初始化 PI
    dev->pi.Kp = 0.0f;
    dev->pi.Ki = 0.01f;
    dev->pi.integral_sum = 0.0f;

    // 1. 硬件激活 (优化：避免重试时反复产生 50ms 延时)
    if (HAL_GPIO_ReadPin(GPIO_PORT_EN_4201, GPIO_PIN_EN_4201) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(GPIO_PORT_EN_4201, GPIO_PIN_EN_4201, GPIO_PIN_SET);
        HAL_Delay(50); // 仅首次冷启动时需要等待
    }

    // 2. 检查设备 (Ping) - 将超时时间从 100ms 缩短到 10ms，提升非阻塞轮询速度
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->devAddr, 3, 10) != HAL_OK) {
        return HAL_ERROR;
    }

    dev->is_connected = true;
    return HAL_OK;
}

/**
  * @brief  基础硬件配置 (软关机、方向、控制环路等)，连接成功后仅执行一次
  */
HAL_StatusTypeDef MP4201_ConfigBasic(MP4201_Handle_t* dev) {
    if (!dev->is_connected)
        return HAL_ERROR;

    // 软关机 (Soft Shutdown) - 必须先关机才能改方向
    uint8_t stop_cmd = 0x00;
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->devAddr, REG_OPERATION, I2C_MEMADD_SIZE_8BIT, &stop_cmd, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(200); // 这是必要的放电延时，由于它仅在连接成功的时刻执行一次，所以可以接受阻塞

    // 修改方向与模式
    uint8_t ctrl1_val = 0;
    // 即使读取失败，我们也给一个安全的默认值
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->devAddr, REG_MFR_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 100) != HAL_OK) {
        ctrl1_val = 0x08; // 读失败时的安全默认值 (Bit 3=1)
    }

    ctrl1_val |= (1 << 7); // Bit 7: VIN -> OUT (DIR)
    ctrl1_val |= (1 << 3); // Bit 3: 强制置 1 (Internal Feedback Mode)
    ctrl1_val |= (1 << 2); // Bit 2: CC Mode (Current Limit Mode)

    if (HAL_I2C_Mem_Write(dev->hi2c, dev->devAddr, REG_MFR_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // 更新本地状态并返回
    dev->isEnabled = 0;

    return HAL_OK;
}

/**
  * @brief  初始化 (带重试机制，非阻塞状态机模式)
  * @param  dev: 设备句柄
  * @param  hi2c: I2C句柄
  * @param  retries: 尝试次数
  * @param  delay_ms: 失败后的延时时间 (ms)
  * @retval HAL_OK(成功), HAL_ERROR(彻底失败), HAL_BUSY(还在重试等待间隔中)
  */
HAL_StatusTypeDef MP4201_Init_Retry(MP4201_Handle_t* dev, I2C_HandleTypeDef* hi2c, uint8_t retries, uint32_t delay_ms) {
    static uint8_t current_retry = 0;
    static uint32_t last_try_time = 0;
    if (dev->is_connected != false) { return HAL_OK; }
    // 如果已经达到最大重试次数，则返回彻底失败
    if (current_retry >= retries) {
        return HAL_ERROR;
    }

    // 利用 HAL_GetTick() 进行非阻塞的时间判断
    if (HAL_GetTick() - last_try_time >= delay_ms || last_try_time == 0) {
        last_try_time = HAL_GetTick(); // 更新时间戳
        current_retry++;

        if (MP4201_Init(dev, hi2c) == HAL_OK) {
            current_retry = 0; // 连接成功后清零状态机，以便未来热插拔恢复
            last_try_time = 0;
            return HAL_OK; // 成功
        }
    }

    // 如果还没到下一次重试时间，或者刚重试完但失败了，立刻返回 BUSY
    return HAL_BUSY;
}

/**
  * @brief  配置安全参数 (阻塞式)
  */
HAL_StatusTypeDef MP4201_ConfigureMaximum(MP4201_Handle_t* dev, float maxVout, float maxIout) {
    dev->maxVout = maxVout;
    dev->maxIout = maxIout;
    return HAL_OK;
}

/**
  * @brief  配置无线充电输入端保护
  */
HAL_StatusTypeDef MP4201_ConfigWirelessInput(MP4201_Handle_t* dev, float minVin) {
    // 1. 开启 VIN Regulation 功能 (MFR_OVP_CTRL bit 7)
    uint8_t config_byte = 0b11011011; // 1000 1000

    if (HAL_I2C_Mem_Write(dev->hi2c, dev->devAddr, REG_MFR_OVP_CTRL, I2C_MEMADD_SIZE_8BIT, &config_byte, 1, 100) !=
        HAL_OK) {
        return HAL_ERROR;
    }

    // 2. 设置阈值电压 (VIN_REG_THLD)
    uint16_t vin_reg_val = (uint16_t)(minVin * SCALE_VOUT_SET_LSB_INV); // 系数与 VOUT 相同 (0.02V/LSB)
    return MP4201_WriteReg_Blocking(dev, REG_VIN_REG_THLD, vin_reg_val);
}

HAL_StatusTypeDef MP4201_Enable(MP4201_Handle_t* dev, uint8_t enable) {
    uint8_t data = enable ? 0x80 : 0x00;
    dev->isEnabled = enable;
    dev->pi.integral_sum = 0.0f;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c,
                                                 dev->devAddr,
                                                 REG_OPERATION,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 &data,
                                                 1,
                                                 100);
    return status;
}

void MP4201_SetCV_Target(MP4201_Handle_t* dev, float voltage) {
    if (voltage > dev->maxVout)
        voltage = dev->maxVout;
    dev->controlMode = MP4201_MODE_CV;
    dev->targetVal = voltage;

    uint16_t iout_reg = (uint16_t)(dev->maxIout * SCALE_ILIM_SET_LSB_INV);
    MP4201_WriteReg_Blocking(dev, REG_IOUT_OC_FAULT_LIMIT, iout_reg);
}

void MP4201_SetCC_Target(MP4201_Handle_t* dev, float current) {
    if (current > dev->maxIout)
        current = dev->maxIout;
    dev->controlMode = MP4201_MODE_CC;
    dev->targetVal = current;
}

void MP4201_SetCP_Target(MP4201_Handle_t* dev, float power, float Kp, float Ki) {
    dev->controlMode = MP4201_MODE_CP;
    dev->targetVal = power;
    dev->pi.Kp = Kp;
    dev->pi.Ki = Ki;

    // 1. 积分器初始化：从当前实际电流开始积分，实现无扰切换
    if (dev->iout_meas > 0.1f) {
        dev->pi.integral_sum = dev->iout_meas;
    } else {
        // 如果刚启动没电流，给一个初始小电流（例如 0.1A）起步
        dev->pi.integral_sum = 0.1f;
    }

    // 2. 先把电压设为 26V (满充电压)
    // 这样 MP4201 会试图输出 26V，然后进入动态调节的 CC 模式
    uint16_t vout_reg = (uint16_t)(dev->maxVout * SCALE_VOUT_SET_LSB_INV);
    MP4201_WriteReg_Blocking(dev, REG_VOUT_COMMAND, vout_reg);
}

HAL_StatusTypeDef MP4201_StartDMAUpdate(MP4201_Handle_t* dev) {
    if (dev->state != MP4201_STATE_IDLE) {
        return HAL_BUSY;
    }

    dev->lastUpdateTime = HAL_GetTick();
    dev->state = MP4201_STATE_READ_VIN; // 先占位

    // 保存返回值进行检查
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->hi2c,
                                                    dev->devAddr,
                                                    REG_READ_VIN,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    dev->dmaRxBuffer,
                                                    2);

    // 如果启动失败 (同步失败)
    if (status != HAL_OK) {
        // 把状态还原为 IDLE，否则下次进不来了
        dev->state = MP4201_STATE_IDLE;
    }

    return status;
}

void MP4201_OnRxCplt(MP4201_Handle_t* dev) {
    uint16_t raw_val = (dev->dmaRxBuffer[1] << 8) | dev->dmaRxBuffer[0];
    const float offset_vin = 0.12f;
    const float offset_vout = 0.12f;
    const float offset_iin = 0.15f;
    const float offset_iout = 0.0f;
    const float i_out_filter_alpha = 0.5f;
    const float v_out_filter_alpha = 0.7f;
    const float power_out_filter_alpha = 0.05f;

    switch (dev->state) {
        case MP4201_STATE_READ_VIN: {
            float new_vin = raw_val * SCALE_READ_V_LSB;
            dev->vin_meas = new_vin + offset_vin;
            dev->state = MP4201_STATE_READ_VOUT;
            if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->devAddr, REG_READ_VOUT, I2C_MEMADD_SIZE_8BIT, dev->dmaRxBuffer, 2)
                != HAL_OK) { dev->errorCount++; }
            break;
        }
        case MP4201_STATE_READ_VOUT: {
            float new_vout = raw_val * SCALE_READ_V_LSB;

            // 低通滤波
            if (dev->vout_meas < 0.1f) {
                dev->vout_meas = new_vout;
            } else {
                dev->vout_meas = dev->vout_meas * (1.0f - v_out_filter_alpha) + new_vout * v_out_filter_alpha;
            }
            // offset
            dev->vout_meas += offset_vout;

            dev->state = MP4201_STATE_READ_IIN;
            if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->devAddr, REG_READ_IIN, I2C_MEMADD_SIZE_8BIT, dev->dmaRxBuffer, 2)
                != HAL_OK) { dev->errorCount++; }
            break;
        }
        case MP4201_STATE_READ_IIN: {
            float new_iin = raw_val * SCALE_READ_I_LSB;
            if (new_iin > 1.0f) { dev->iin_meas = new_iin + offset_iin; } else { dev->iin_meas = new_iin; }
            dev->state = MP4201_STATE_READ_IOUT;
            if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->devAddr, REG_READ_IOUT, I2C_MEMADD_SIZE_8BIT, dev->dmaRxBuffer, 2)
                != HAL_OK) { dev->errorCount++; }
            break;
        }
        case MP4201_STATE_READ_IOUT: {
            float new_iout = raw_val * SCALE_READ_I_LSB;

            // 低通滤波
            if (dev->iout_meas < 1e-5 && new_iout > 0.0f) {
                dev->iout_meas = new_iout;
            } else {
                dev->iout_meas = dev->iout_meas * (1.0f - i_out_filter_alpha) + new_iout * i_out_filter_alpha;
            }
            // offset
            if (dev->iout_meas > 1.0f) { dev->iout_meas += offset_iout; }

            float new_power_out = dev->vout_meas * dev->iout_meas;
            dev->power_meas = dev->power_meas * (1.0f - power_out_filter_alpha) + new_power_out *
                power_out_filter_alpha;
            dev->state = MP4201_STATE_PROCESSING;
            MP4201_CheckStatus(dev);
            MP4201_RunControlAlgorithm(dev);
            break;
        }
        default:
            dev->state = MP4201_STATE_IDLE;
            break;
    }

    // 异常
    switch (dev->controlMode) {
        case MP4201_MODE_CV:
            if (fabs(dev->targetVal - dev->vout_meas / dev->targetVal) > 0.05) {
                // WS2812_SetAll(255, 255, 0); // 电压值偏差太大，灯亮黄色
            } else {
                // WS2812_SetAll(0, 0, 0); // 否则取消
            }
            break;
        case MP4201_MODE_CC:
            if (fabs((dev->targetVal - dev->iout_meas) / dev->targetVal) > 0.05) {
                // WS2812_SetAll(0, 255, 0); // 电流值偏差太大，灯亮青色
            } else {
                // WS2812_SetAll(0, 0, 0); // 否则取消
            }
            break;
        case MP4201_MODE_CP:
            if (fabs((dev->targetVal - dev->power_meas) / dev->targetVal) > 0.05) {
                // WS2812_SetAll(0, 0, 255); // 功率值偏差太大，灯亮蓝色
            } else {
                // WS2812_SetAll(0, 0, 0); // 否则取消
            }
            break;
    }
}

void MP4201_OnTxCplt(MP4201_Handle_t* dev) {
    dev->state = MP4201_STATE_IDLE;
}

void MP4201_OnError(MP4201_Handle_t* dev) {
    // WS2812_SetAll(255, 0, 0); // 出现I2C错误，灯亮红色
    dev->errorCount++;
    dev->state = MP4201_STATE_IDLE;
}

/**
  * @brief  PI 控制算法
  */
static void MP4201_RunControlAlgorithm(MP4201_Handle_t* dev) {
    uint8_t need_write = 0; // 标记是否需要执行 I2C 写入
    uint8_t reg_addr = 0;   // 目标寄存器地址
    uint16_t reg_val = 0;   // 目标寄存器值

    // 0. 状态检查：如果设备未启用，强制进入空闲状态并退出
    if (!dev->isEnabled) {
        dev->state = MP4201_STATE_IDLE;
        return;
    }

    // 模式 A: 恒功率控制
    if (dev->controlMode == MP4201_MODE_CP) {
        // 使用 static 变量保存历史状态，用于下一轮计算
        static float next_iout = 0.0f;
        static float last_iout = 0.0f;
        // 施密特触发器状态：0=常规CP阶段, 1=接近满电CV阶段
        static uint8_t schmitt_trigger_CV_phase = 0;
        // --- 施密特触发器 (Schmitt Trigger) ---
        // 用于判断是否即将充满（进入 CV 阶段），避免在临界点反复跳变。
        // 逻辑：
        // - 当电压差 < 0.1V (非常接近满电) -> 进入 CV 锁定状态 (State = 1)
        // - 当电压差 > 2.0V (电压掉下来了) -> 回到 CP 调节状态 (State = 0)
        if (schmitt_trigger_CV_phase == 0 && fabs(dev->vout_meas - dev->maxVout) < 0.02f) {
            schmitt_trigger_CV_phase = 1; // 进入 CV 临界区
        } else if (schmitt_trigger_CV_phase == 1 && fabs(dev->vout_meas - dev->maxVout) > 2.0f) {
            schmitt_trigger_CV_phase = 0; // 退出 CV 临界区
        }

        // 误差计算 (目标功率 - 实际测量功率)
        float error_P = dev->targetVal - dev->power_meas;

        // 积分器更新 (带抗饱和与重置机制)
        // 积分抗饱和逻辑
        // 允许积分累加的条件：
        // 1. 当前输出电流未达到硬件最大值 (没有被限幅)
        // 2. 或者，虽然达到了最大值，但误差是负的 (说明功率超标了，需要通过积分减小电流)
        // 3. 补充条件：不在 "硬件电流已达上限" 的状态
        if ((dev->pi.integral_sum < dev->maxIout && !dev->isMaxIout) || (error_P < 0)) {
            dev->pi.integral_sum += error_P * dev->pi.Ki;
        }

        // 积分重置逻辑:
        // 1. 如果电流极小 (<0.05A)，说明可能空载，清除积分防止启动过冲。
        // 2. 如果输出电压非常接近最大设定电压 (<1.0V 差值)，说明即将进入恒压区，清除积分项，
        //    准备让出控制权给硬件电压环，防止 PI 强行推功率导致过压。
        if (dev->iout_meas < 0.05f || schmitt_trigger_CV_phase == 1) {
            dev->pi.integral_sum = 0;
        }

        // 积分器幅值钳位 (Clamping)
        if (dev->pi.integral_sum > dev->maxIout) { dev->pi.integral_sum = dev->maxIout; }
        if (dev->pi.integral_sum < 0.0f) { dev->pi.integral_sum = 0.0f; }

        // 前馈控制 (Feed-Forward) 与 模式切换判断
        // 基本前馈公式: I = P / V (功率守恒)
        float i_feed_forward = dev->targetVal / dev->vout_meas;

        // 前馈数值安全检查
        if (isinf(i_feed_forward)) {
            i_feed_forward = 0; // 防止除以零
        } else if (i_feed_forward > dev->maxIout) {
            i_feed_forward = dev->maxIout; // 限幅
        }

        // 根据触发器状态调整策略
        if (schmitt_trigger_CV_phase == 1) {
            // 进入 CV 阶段策略：
            // 关掉前馈，标记进入 CV 相位。此时我们不再希望软件强行推电流来维持功率，
            // 而是希望电流保持住，或者自然下降。
            i_feed_forward = 0;
            dev->isCVphase = true;
        } else {
            dev->isCVphase = false;
        }

        // 调试变量更新
        // update C++ wrapper debug value if available
        #ifdef __cplusplus
        MP4201::feedForwardDebug = i_feed_forward;
        #else
        (void)0;
        #endif

        // 计算最终输出电流 (PI + 前馈)
        last_iout = next_iout; // 保存上一轮的值

        // PI 输出公式: I_out = (Kp * Error) + Integral + FeedForward
        next_iout = (error_P * dev->pi.Kp) + dev->pi.integral_sum + i_feed_forward;

        // --- 输出锁定逻辑 (Hold Logic) ---
        // 如果已经进入 CV Phase (电压几乎满了)：
        // 这里的逻辑是：放弃 PI 计算的新值，直接维持上一次的电流限值。
        // 目的：让 MP4201 芯片内部的硬件电压环路起作用，软件不再增加电流限制去试图维持功率，
        // 从而实现从恒功率到恒压的平滑过渡。
        if (dev->isCVphase) {
            next_iout = last_iout;
        }

        // 最终安全限幅
        if (next_iout >= dev->maxIout) {
            next_iout = dev->maxIout;
            dev->isMaxIout = true; // 标记达到最大电流
        } else {
            dev->isMaxIout = false;
        }

        // 最小电流限制 (防止设为0导致关机或异常)
        if (next_iout < 0.1f) { next_iout = 0.1f; } // 建议加一个非零下限

        // 准备寄存器数据
        dev->calculated_iout_cmd = next_iout; // 记录用于调试

        // 写入 REG_IOUT_OC_FAULT_LIMIT (电流限制寄存器)
        // 通过不断调节电流上限，来实现恒功率控制
        reg_addr = REG_IOUT_OC_FAULT_LIMIT;
        reg_val = (uint16_t)(next_iout * SCALE_ILIM_SET_LSB_INV);
        need_write = 1;

        // 模式 B: 恒压控制 (CV Mode)
    } else if (dev->controlMode == MP4201_MODE_CV) {
        float next_vout = dev->targetVal;

        // 安全限幅
        if (next_vout > dev->maxVout)
            next_vout = dev->maxVout;
        if (next_vout < 0.0f)
            next_vout = 0.0f;

        dev->calculated_vout_cmd = next_vout;
        reg_addr = REG_VOUT_COMMAND;
        reg_val = (uint16_t)(next_vout * SCALE_VOUT_SET_LSB_INV);
        need_write = 1;
    }

    // 执行 I2C DMA 写入
    if (need_write) {
        dev->state = MP4201_STATE_WRITE_CTRL;

        // 填充 DMA 发送缓冲区 (小端序: 低字节在前)
        dev->dmaTxBuffer[0] = reg_val & 0xFF;
        dev->dmaTxBuffer[1] = (reg_val >> 8) & 0xFF;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(dev->hi2c,
                                                         dev->devAddr,
                                                         reg_addr,
                                                         I2C_MEMADD_SIZE_8BIT,
                                                         dev->dmaTxBuffer,
                                                         2);
        if (status != HAL_OK) {
            // 发送失败，回到 IDLE 等待下一次周期恢复
            dev->state = MP4201_STATE_IDLE;
        }
    } else {
        dev->state = MP4201_STATE_IDLE;
    }
}

static HAL_StatusTypeDef MP4201_WriteReg_Blocking(MP4201_Handle_t* dev, uint8_t reg, uint16_t data) {
    uint8_t buf[2];
    buf[0] = data & 0xFF;
    buf[1] = (data >> 8) & 0xFF;
    return HAL_I2C_Mem_Write(dev->hi2c, dev->devAddr, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

void MP4201_SetDirection(MP4201_Handle_t* dev, uint8_t direction) {
    bool was_enabled = dev->isEnabled;

    // 软件停机 (Soft Stop)
    MP4201_Enable(dev, 0);

    // 安全延时 (Safety Delay)
    // 手册建议等待 300ms 以上让输出电容放电，防止反向电流冲击
    HAL_Delay(300);

    // 修改方向 (Change Direction) 和 dead time
    // 1. 先读取 MFR_CTRL1 (0xD0) 的当前值，避免覆盖其他配置（如 UVLO, 死区时间等）
    uint8_t ctrl1_val = 0;
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->devAddr, REG_MFR_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 100) ==
        HAL_OK) {
        // 2. 修改 Bit 7 (DIR)
        if (direction == MP4201_DIR_CHARGE) {
            ctrl1_val |= (1 << 7); // 置 1：VIN -> OUT
        } else {
            ctrl1_val &= ~(1 << 7); // 清 0：OUT -> VIN
        }
        // dead time = 30ns
        ctrl1_val |= (1 << 1);
        ctrl1_val |= (1 << 0);
        // 3. 写回寄存器
        HAL_I2C_Mem_Write(dev->hi2c, dev->devAddr, REG_MFR_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 100);
    }

    // 软件启动 (Soft Start)
    MP4201_Enable(dev, was_enabled);
}

void MP4201_CheckStatus(MP4201_Handle_t* dev) {
    // 读取 0x79 寄存器 (2字节)
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->devAddr, REG_STATUS_WORD, I2C_MEMADD_SIZE_8BIT, dev->err_status, 2, 100) !=
        HAL_OK) {
        dev->err_status[0] = 0xFF;
        dev->err_status[1] = 0xFF;
    }
    if (dev->err_status[0] != 0x00) {
        dev->errorCount++;
    }
}