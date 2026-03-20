/**
  ******************************************************************************
  * @file    mp4201.h
  ******************************************************************************
  */

#ifndef __MP4201_H__
#define __MP4201_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <math.h>

// ============================================================================
// 用户配置区
// ============================================================================
#define TARGET_VOLTAGE_V        20.0f   // 目标满电电压 (V)
#define MAX_CURRENT_A           10.0f    // 最大允许充电电流 (A)
#define TARGET_POWER_W          30.0f   // 目标恒功率值 (W)
#define CV_ENTRY_THRESHOLD_V    0.5f    // 进入恒压区的阈值 (V) [当 Vout > (Target - Threshold) 时进入CV]
#define VOUT_READ_SCALE         1.0f    // 如果有分压电路，请修改此比例
#define IOUT_READ_SCALE         1.0f    // 电流读取比例

/* ==============================================================================
 * I2C 地址配置
 * ============================================================================== */
#define MP4201_ADDR_DEFAULT      (0x27 << 1)

/* ==============================================================================
 * 寄存器映射
 * ============================================================================== */
#define REG_OPERATION            0x01
#define REG_VOUT_COMMAND         0x21
#define REG_IOUT_OC_FAULT_LIMIT  0x46
#define REG_VIN_REG_THLD         0x59 // 新增：输入电压调节阈值
#define REG_READ_VIN             0x88
#define REG_READ_IIN             0x89
#define REG_READ_VOUT            0x8B
#define REG_READ_IOUT            0x8C
#define REG_READ_TEMP            0x8D
#define REG_MFR_OVP_CTRL         0xD6
#define REG_MFR_OCP_CTRL         0XD2
#define REG_MFR_CTRL1            0xD0
#define REG_STATUS_WORD          0x79
/* ==============================================================================
 * 转换系数
 * ============================================================================== */
#define SCALE_VOUT_SET_LSB_INV   50.0f   // 1 / 0.02V
#define SCALE_ILIM_SET_LSB_INV   20.0f   // 1 / 0.05A
#define SCALE_READ_V_LSB         0.0832f
#define SCALE_READ_I_LSB         0.043f
/* ==============================================================================
 * 方向
 * ============================================================================== */
#define MP4201_DIR_CHARGE    1      // VIN -> OUT (充电/正向)
#define MP4201_DIR_DISCHARGE 0      // OUT -> VIN (放电/反向)
/* ==============================================================================
 * 操作位
 * ============================================================================== */
#define OP_ON               0x80
#define OP_OFF              0x00
/* ==============================================================================
 * 异常状态
 * ============================================================================== */
#define STATUS_SCP_FAULT        (1 << 14)  // Short Circuit Protection (Latch)
#define STATUS_INPUT_OV         (1 << 13)  // Input Over Voltage (Latch)
#define STATUS_PG_FAULT         (1 << 11)  // Power Good Negated (1=Bad, 0=Good) (Live)
#define STATUS_CHARGE_COMPLETE  (1 << 9)   // Charge Complete (Live)
#define STATUS_IIN_OC           (1 << 8)   // Input Over Current (Latch)
#define STATUS_VOUT_OV          (1 << 5)   // Output Over Voltage (Latch)
#define STATUS_IOUT_OC          (1 << 4)   // Output Over Current (Latch) - 包含打嗝模式
#define STATUS_TEMP_FAULT       (1 << 2)   // Temperature Warning/Fault (Latch)
#define STATUS_CRC_ERROR        (1 << 0)   // CRC Error (Latch)
/* ==============================================================================
 * 驱动状态定义 (异步状态机)
 * ============================================================================== */
typedef enum {
    MP4201_STATE_IDLE = 0,   // 空闲
    MP4201_STATE_READ_VIN,   // 正在读取输入电压
    MP4201_STATE_READ_VOUT,  // 正在读取输出电压
    MP4201_STATE_READ_IIN,   // 正在读取输入电流
    MP4201_STATE_READ_IOUT,  // 正在读取输出电流
    MP4201_STATE_PROCESSING, // 数据读取完毕，正在计算 (PI算法)
    MP4201_STATE_WRITE_CTRL, // 正在写入控制字 (VOUT_COMMAND)
    MP4201_STATE_ERROR       // 发生错误
} MP4201_AsyncState_t;

typedef enum {
    MP4201_MODE_CV, // 恒压模式
    MP4201_MODE_CC, // 恒流模式
    MP4201_MODE_CP  // 恒功率模式 (PI控制电压)
} MP4201_ControlMode_t;

/* ==============================================================================
 * PI 控制器结构体
 * ============================================================================== */
typedef struct {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float integral_sum; // 积分累计值
    float out_max;      // 输出上限 (通常等于 maxVout)
    float out_min;      // 输出下限 (通常为 0 或当前电容电压)
} MP4201_PI_Config_t;

/* ==============================================================================
 * 主句柄结构体
 * ============================================================================== */
typedef struct {
    // --- 硬件接口 ---
    I2C_HandleTypeDef* hi2c;
    uint8_t devAddr;

    // --- 异步运行状态 ---
    volatile MP4201_AsyncState_t state;
    uint8_t dmaRxBuffer[2]; // DMA 接收缓冲区
    uint8_t dmaTxBuffer[3]; // DMA 发送缓冲区
    uint32_t lastUpdateTime;

    // --- 实时数据 ---
    float vin_meas;
    float vout_meas;
    float iin_meas;
    float iout_meas;
    float power_meas;

    // --- 控制目标 ---
    MP4201_ControlMode_t controlMode;
    float targetVal; // CV(V), CC(A), CP(W)

    // --- PI 控制器 (专用于 CP 模式) ---
    MP4201_PI_Config_t pi;

    // --- 保护限制 ---
    float maxVout;
    float maxIout;

    // --- 内部变量 ---
    uint8_t isEnabled;
    float calculated_vout_cmd; // 上一次写入的电压指令
    float calculated_iout_cmd;
    uint8_t isMaxIout;
    uint8_t isCVphase;

    // --- 错误计数 ---
    uint32_t errorCount;
    uint8_t err_status[2];

    // --- 无线充电 ---
    uint8_t charging_flag;

    // --- 初始化（非阻塞重试）控制 ---
    // 当调用 MP4201_Init_Start 后，如果首次初始化失败，会设置 init_pending = 1
    // 并在之后通过 MP4201_Init_Tick 周期性尝试初始化（非阻塞重试）。
    uint8_t init_pending;               // 是否还有初始化重试待处理
    uint32_t init_retry_interval_ms;    // 重试间隔（毫秒）
    uint32_t init_last_attempt;         // 上次尝试时间（HAL_GetTick）
    uint8_t init_retry_count;           // 已尝试次数
    uint8_t init_max_retries;           // 最大重试次数（0 表示无限重试）
} MP4201_Handle_t;

/* ==============================================================================
 * 函数原型
 * ============================================================================== */

// 初始化
HAL_StatusTypeDef MP4201_Init(MP4201_Handle_t* dev, I2C_HandleTypeDef* hi2c);
// 非阻塞初始化：尝试一次初始化，失败后按照 retry_interval_ms 在后台重试
HAL_StatusTypeDef MP4201_Init_Start(MP4201_Handle_t* dev, I2C_HandleTypeDef* hi2c, uint32_t retry_interval_ms, uint8_t max_retries);
// 在主循环/4kHz 处理（或任意定时任务）中周期性调用：会在到达重试间隔时发起新的初始化尝试
// 返回值: HAL_OK=已初始化成功, HAL_BUSY=仍在重试中, HAL_ERROR=达到最大重试并失败
HAL_StatusTypeDef MP4201_Init_Tick(MP4201_Handle_t* dev);
// 直接触发一次初始化尝试（立即调用），返回 MP4201_Init 的状态
HAL_StatusTypeDef MP4201_AttemptInit(MP4201_Handle_t* dev, I2C_HandleTypeDef* hi2c);

HAL_StatusTypeDef MP4201_ConfigureSafety(MP4201_Handle_t* dev, float maxVout, float maxIout);
HAL_StatusTypeDef MP4201_Enable(MP4201_Handle_t* dev, uint8_t enable);

/**
 * @brief 配置无线充电输入保护
 * @param minVin: 最低输入电压限制 (V)。当无线充输入低于此值时，MP4201会自动减小充电功率。
 */
HAL_StatusTypeDef MP4201_ConfigWirelessInput(MP4201_Handle_t* dev, float minVin);

// 模式设置
void MP4201_SetCV_Target(MP4201_Handle_t* dev, float voltage);
void MP4201_SetCC_Target(MP4201_Handle_t* dev, float current);
void MP4201_SetDirection(MP4201_Handle_t* dev, uint8_t direction);
// 设置恒功率目标及 PI 参数
// 建议参数: Kp=0.00, Ki=0.005 (需根据实际负载电容大小调试)
void MP4201_SetCP_Target(MP4201_Handle_t* dev, float power, float Kp, float Ki);

// 核心循环
HAL_StatusTypeDef MP4201_StartDMAUpdate(MP4201_Handle_t* dev);

// 回调接口
void MP4201_OnRxCplt(MP4201_Handle_t* dev);
void MP4201_OnTxCplt(MP4201_Handle_t* dev);
void MP4201_OnError(MP4201_Handle_t* dev);
void MP4201_CheckStatus(MP4201_Handle_t* dev);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstring>

// C++ wrapper class for MP4201 - thin wrapper around the existing C handle + functions
class MP4201 {
public:
    // Debug feed-forward value (shared)
    static float feedForwardDebug;

    // Construct with an I2C handle (does not call Init automatically)
    explicit MP4201(I2C_HandleTypeDef* hi2c = nullptr) {
        std::memset(&h_, 0, sizeof(h_));
        h_.hi2c = hi2c;
        h_.devAddr = MP4201_ADDR_DEFAULT;
        h_.state = MP4201_STATE_IDLE;
        h_.controlMode = MP4201_MODE_CV;
    }

    // Initialize device (calls C API) 但一般用不到，main.c不支持类
    HAL_StatusTypeDef init(I2C_HandleTypeDef* hi2c = nullptr) {
        if (hi2c) h_.hi2c = hi2c;
        return MP4201_Init(&h_, h_.hi2c);
    }

    HAL_StatusTypeDef configureSafety(float maxVout, float maxIout) {
        return MP4201_ConfigureSafety(&h_, maxVout, maxIout);
    }

    HAL_StatusTypeDef configWirelessInput(float minVin) {
        return MP4201_ConfigWirelessInput(&h_, minVin);
    }

    HAL_StatusTypeDef enable(bool en) {
        return MP4201_Enable(&h_, en ? 1 : 0);
    }

    HAL_StatusTypeDef startDMAUpdate() {
        return MP4201_StartDMAUpdate(&h_);
    }

    void setCV(float v) { MP4201_SetCV_Target(&h_, v); }
    void setCC(float i) { MP4201_SetCC_Target(&h_, i); }
    void setCP(float p, float Kp = 0.05f, float Ki = 0.005f) { MP4201_SetCP_Target(&h_, p, Kp, Ki); }
    void setDirection(uint8_t dir) { MP4201_SetDirection(&h_, dir); }

    // Callback handlers
    void onRxCplt() { MP4201_OnRxCplt(&h_); }
    void onTxCplt() { MP4201_OnTxCplt(&h_); }
    void onError()  { MP4201_OnError(&h_); }
    void checkStatus() { MP4201_CheckStatus(&h_); }

    // Read-only accessors
    float getVin() const { return h_.vin_meas; }
    float getVout() const { return h_.vout_meas; }
    float getIin() const { return h_.iin_meas; }
    float getIout() const { return h_.iout_meas; }
    float getPower() const { return h_.power_meas; }
    auto getChargingFlag() const {return h_.charging_flag;}
    uint32_t getErrorCount() const { return h_.errorCount; }

    // Expose the underlying C handle if raw access is required
    MP4201_Handle_t* get_handler() { return &h_; }
    const MP4201_Handle_t* get_handler() const { return &h_; }

private:
    MP4201_Handle_t h_;
};

#endif /* __cplusplus */

#endif /* __MP4201_H__ */
