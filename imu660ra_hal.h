/**
 * @file imu660ra_hal.h
 * @brief 逐飞科技 IMU660RA 模块驱动接口定义(STM32 HAL)
 * @author tanejo
 * @date 2025-11-22
 *
 * 说明：
 * - 本头文件定义了寄存器、配置枚举、数据结构与对外函数原型。
 * - 代码依赖 STM32 HAL 库，请根据实际芯片系列选择正确的 HAL 头文件。
 * - 如需移植到非 STM32 平台，重点替换 SPI 与 GPIO 控制接口。
 */

#ifndef __IMU660RA_HAL_H__
#define __IMU660RA_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h" // 根据你的芯片型号修改，如 stm32f1xx_hal.h, stm32h7xx_hal.h

/* --- 寄存器定义 --- */
// 芯片 ID 寄存器地址与期望值
#define IMU660RA_CHIP_ID_ADDR       0x00
#define IMU660RA_CHIP_ID_VAL        0x24

// 电源与初始化相关寄存器
#define IMU660RA_PWR_CONF           0x7C  // 省电/功耗配置
#define IMU660RA_PWR_CTRL           0x7D  // 传感器使能控制
#define IMU660RA_INIT_CTRL          0x59  // 初始化控制（加载配置开始/结束）
#define IMU660RA_INIT_DATA          0x5E  // 初始化配置数据写入口（Burst 写入）
#define IMU660RA_INT_STA            0x21  // 初始化完成状态标志

// 数据寄存器（低字节起始地址），按轴连续存放
#define IMU660RA_ACC_X_LSB          0x0C
#define IMU660RA_GYR_X_LSB          0x12

// 量程与采样率配置寄存器
#define IMU660RA_ACC_CONF           0x40
#define IMU660RA_ACC_RANGE          0x41
#define IMU660RA_GYR_CONF           0x42
#define IMU660RA_GYR_RANGE          0x43

/* --- 配置枚举 --- */

// 加速度量程（对应寄存器 IMU660RA_ACC_RANGE）
typedef enum {
    IMU660RA_ACC_RANGE_2G  = 0x00,
    IMU660RA_ACC_RANGE_4G  = 0x01,
    IMU660RA_ACC_RANGE_8G  = 0x02,
    IMU660RA_ACC_RANGE_16G = 0x03
} IMU660RA_AccRange_t;

// 陀螺仪量程（对应寄存器 IMU660RA_GYR_RANGE）
typedef enum {
    IMU660RA_GYR_RANGE_2000DPS = 0x00,
    IMU660RA_GYR_RANGE_1000DPS = 0x01,
    IMU660RA_GYR_RANGE_500DPS  = 0x02,
    IMU660RA_GYR_RANGE_250DPS  = 0x03,
    IMU660RA_GYR_RANGE_125DPS  = 0x04
} IMU660RA_GyrRange_t;

// ODR（输出数据率），仅列出常用值（与芯片文档的 ODR 编码一致）
typedef enum {
    IMU660RA_ODR_25HZ  = 0x06,
    IMU660RA_ODR_50HZ  = 0x07,
    IMU660RA_ODR_100HZ = 0x08,
    IMU660RA_ODR_200HZ = 0x09,
    IMU660RA_ODR_400HZ = 0x0A,
    IMU660RA_ODR_800HZ = 0x0B
} IMU660RA_ODR_t;

// 函数返回值状态枚举
typedef enum {
    IMU660RA_OK = 0,            // 操作成功
    IMU660RA_ERROR_NULL_PTR,    // 空指针错误
    IMU660RA_ERROR_CHIP_ID,     // 芯片 ID 不匹配
    IMU660RA_ERROR_INIT         // 内部初始化失败
} IMU660RA_Status_t;

/* --- 数据结构 --- */

// 原始数据（int16，单位为 LSB）
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp; // 如果需要温度，可扩展读取
} IMU660RA_RawData_t;

// 物理单位数据（float）
typedef struct {
    float ax_g, ay_g, az_g;       // 单位：g（重力加速度）
    float gx_dps, gy_dps, gz_dps; // 单位：dps（度/秒）
} IMU660RA_PhysData_t;

// 设备句柄（保存硬件接口、配置与数据缓存）
typedef struct {
    // 硬件接口
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CS_Port;
    uint16_t CS_Pin;

    // 配置记录（用于数据转换）
    IMU660RA_AccRange_t acc_range;
    IMU660RA_GyrRange_t gyr_range;
    float acc_scale; // LSB -> g 的转换系数
    float gyr_scale; // LSB -> dps 的转换系数

    // 数据存储
    IMU660RA_RawData_t raw;
    IMU660RA_PhysData_t data;
} IMU660RA_t;

/* --- 函数声明 --- */
/**
 * @brief 初始化 IMU660RA 设备
 * @param dev 设备句柄指针（调用前需提供有效内存）
 * @param hspi SPI 句柄指针
 * @param cs_port 片选 GPIO 端口
 * @param cs_pin 片选 GPIO 引脚号
 * @return 0 表示成功，非 0 表示失败（1 参数无效；2 芯片 ID 错误；3 初始化状态错误）
 */
IMU660RA_Status_t IMU660RA_Init(IMU660RA_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief 设置加速度计 ODR 与量程
 * @param dev 设备句柄
 * @param odr 输出数据率枚举
 * @param range 加速度量程枚举
 */
void IMU660RA_Set_Acc_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_AccRange_t range);

/**
 * @brief 设置陀螺仪 ODR 与量程
 * @param dev 设备句柄
 * @param odr 输出数据率枚举
 * @param range 陀螺仪量程枚举
 */
void IMU660RA_Set_Gyr_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_GyrRange_t range);

/**
 * @brief 一次性读取加速度与陀螺仪全部轴数据并完成单位转换
 * @param dev 设备句柄
 */
void IMU660RA_Read_All(IMU660RA_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __IMU660RA_HAL_H__ */
