/**
 * @file imu660ra_hal.c
 * @brief 逐飞科技 IMU660RA 模块驱动(STM32 HAL)
 * @author tanejo
 * @date 2025-11-22
 * 
 * 说明：
 * - 实现 SPI 片选控制、寄存器读写、设备初始化与数据读取流程。
 * - 单字节读需发送地址 | 0x80 并丢弃一个 Dummy 字节；多字节读同理。
 * - 初始化阶段向 INIT_DATA 连续写入配置文件，并通过状态位确认完成。
 */

#include "imu660ra_hal.h"

// 引用外部定义的固件数组（厂商提供的初始化配置，8192 字节）
extern const unsigned char imu660ra_config_file[8192];

/* --- 私有宏定义 --- */
// HAL SPI 传输超时时间（ms）
#define SPI_TIMEOUT     100
// 读取操作地址需置位最高位
#define SPI_READ_MASK   0x80
// 写入操作直接发送寄存器地址
#define SPI_WRITE_MASK  0x00

/* --- 私有辅助函数 --- */

/**
 * @brief 拉低片选
 */
static void IMU660RA_CS_Low(IMU660RA_t *dev) {
    HAL_GPIO_WritePin(dev->CS_Port, dev->CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高片选
 */
static void IMU660RA_CS_High(IMU660RA_t *dev) {
    HAL_GPIO_WritePin(dev->CS_Port, dev->CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 写单个寄存器
 * @param dev 设备句柄
 * @param reg 寄存器地址
 * @param data 要写入的字节
 * @return HAL 状态码（HAL_OK 等）
 */
static uint8_t IMU660RA_Write_Reg(IMU660RA_t *dev, uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg; // 大多数SPI写操作地址直接发 (Bosch写不需要特殊位，读需要最高位|0x80)
    tx_data[1] = data;

    IMU660RA_CS_Low(dev);
    uint8_t status = HAL_SPI_Transmit(dev->hspi, tx_data, 2, SPI_TIMEOUT);
    IMU660RA_CS_High(dev);
    return status;
}

/**
 * @brief 读单个寄存器
 * @param dev 设备句柄
 * @param reg 寄存器地址
 * @return 读取到的寄存器数据
 */
static uint8_t IMU660RA_Read_Reg(IMU660RA_t *dev, uint8_t reg) {
    uint8_t tx_data = reg | SPI_READ_MASK;
    uint8_t rx_data[2] = {0};

    IMU660RA_CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &tx_data, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(dev->hspi, rx_data, 2, SPI_TIMEOUT); // [0] Dummy, [1] Data
    IMU660RA_CS_High(dev);

    return rx_data[1];
}

/**
 * @brief 连续读取多个寄存器（支持 ACC/GYR 连续区）
 * @param dev 设备句柄
 * @param reg 起始寄存器地址
 * @param data 读取缓冲区
 * @param len 读取长度（字节）
 */
static void IMU660RA_Read_Burst(IMU660RA_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx_data = reg | SPI_READ_MASK;
    uint8_t dummy_byte;

    IMU660RA_CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &tx_data, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(dev->hspi, &dummy_byte, 1, SPI_TIMEOUT); // Dummy byte
    HAL_SPI_Receive(dev->hspi, data, len, SPI_TIMEOUT);
    IMU660RA_CS_High(dev);
}

/**
 * @brief 连续写多个字节（用于向 INIT_DATA 加载配置文件）
 * @param dev 设备句柄
 * @param reg 目标寄存器（通常为 IMU660RA_INIT_DATA）
 * @param data 待写入数据缓冲区
 * @param len 写入长度（字节）
 *
 * 说明：Bosch 传感器通常不支持普通寄存器的连续写入，但允许向特定入口（如 INIT_DATA）进行大块数据写入。
 */
static void IMU660RA_Write_Burst(IMU660RA_t *dev, uint8_t reg, const uint8_t *data, uint16_t len) {
    
    uint8_t addr = reg; 

    IMU660RA_CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &addr, 1, SPI_TIMEOUT);
    HAL_SPI_Transmit(dev->hspi, (uint8_t*)data, len, SPI_TIMEOUT);
    IMU660RA_CS_High(dev);
}

/* --- 公开接口函数 --- */

/**
 * @brief 初始化 IMU660RA
 * 流程：片选拉高→虚拟读 ID→校验 ID→功耗配置→加载配置→检查状态→开启传感器→默认量程与 ODR。
 * @return 0 成功；非 0 失败
 */
IMU660RA_Status_t IMU660RA_Init(IMU660RA_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (!dev || !hspi) return IMU660RA_ERROR_NULL_PTR;

    dev->hspi = hspi;
    dev->CS_Port = cs_port;
    dev->CS_Pin = cs_pin;

    IMU660RA_CS_High(dev);
    HAL_Delay(50); // 上电等待

    // 虚拟读取，切换 SPI 模式（首次交互稳定总线）
    IMU660RA_Read_Reg(dev, IMU660RA_CHIP_ID_ADDR);
    HAL_Delay(1);

    // 检查 Chip ID（硬件识别）
    uint8_t chip_id = IMU660RA_Read_Reg(dev, IMU660RA_CHIP_ID_ADDR);
    if (chip_id != IMU660RA_CHIP_ID_VAL) {
        return IMU660RA_ERROR_CHIP_ID; // ID 错误
    }

    // 关闭省电模式，准备配置
    IMU660RA_Write_Reg(dev, IMU660RA_PWR_CONF, 0x00);
    HAL_Delay(10);

    // 加载配置文件（向 INIT_DATA 连续写入配置数组）
    IMU660RA_Write_Reg(dev, IMU660RA_INIT_CTRL, 0x00); // Start Load
    IMU660RA_Write_Burst(dev, IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file)); // 引用外部数组
    IMU660RA_Write_Reg(dev, IMU660RA_INIT_CTRL, 0x01); // Finish Load
    
    HAL_Delay(150); // 等待 ASIC 初始化，这一步很重要

    // 检查初始化状态（要求状态位为 0x01 表示成功）
    uint8_t status = IMU660RA_Read_Reg(dev, IMU660RA_INT_STA);
    if (status != 0x01) {
        return IMU660RA_ERROR_INIT; // 初始化状态错误
    }

    // 开启电源（高性能模式）
    IMU660RA_Write_Reg(dev, IMU660RA_PWR_CTRL, 0x0E);

    // 设置默认配置（ODR 量程）
    IMU660RA_Set_Acc_Config(dev, IMU660RA_ODR_800HZ, IMU660RA_ACC_RANGE_8G);
    IMU660RA_Set_Gyr_Config(dev, IMU660RA_ODR_800HZ, IMU660RA_GYR_RANGE_2000DPS);

    return IMU660RA_OK; // 成功
}

/**
 * @brief 配置加速度计 ODR 与量程
 */
void IMU660RA_Set_Acc_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_AccRange_t range) {
    // 配置寄存器: ODR (bit 0-3), Bandwidth, Perf mode
    // 这里简化处理，假设 0xA0 是高性能模式基准，加上 ODR
    uint8_t conf_val = 0xA0 | (odr & 0x0F);
    IMU660RA_Write_Reg(dev, IMU660RA_ACC_CONF, conf_val);

    IMU660RA_Write_Reg(dev, IMU660RA_ACC_RANGE, range);
    
    dev->acc_range = range;
    
    // 更新转换系数 (LSB -> g)
    // 16bit signed int: -32768 to 32767
    switch (range) {
        case IMU660RA_ACC_RANGE_2G:  dev->acc_scale = 2.0f / 32768.0f; break;
        case IMU660RA_ACC_RANGE_4G:  dev->acc_scale = 4.0f / 32768.0f; break;
        case IMU660RA_ACC_RANGE_8G:  dev->acc_scale = 8.0f / 32768.0f; break;
        case IMU660RA_ACC_RANGE_16G: dev->acc_scale = 16.0f / 32768.0f; break;
        default: dev->acc_scale = 0; break;
    }
}

/**
 * @brief 配置陀螺仪 ODR 与量程
 */
void IMU660RA_Set_Gyr_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_GyrRange_t range) {
    uint8_t conf_val = 0xA0 | (odr & 0x0F);
    IMU660RA_Write_Reg(dev, IMU660RA_GYR_CONF, conf_val);
    
    IMU660RA_Write_Reg(dev, IMU660RA_GYR_RANGE, range);
    
    dev->gyr_range = range;

    // 更新转换系数 (LSB -> dps)
    switch (range) {
        case IMU660RA_GYR_RANGE_2000DPS: dev->gyr_scale = 2000.0f / 32768.0f; break;
        case IMU660RA_GYR_RANGE_1000DPS: dev->gyr_scale = 1000.0f / 32768.0f; break;
        case IMU660RA_GYR_RANGE_500DPS:  dev->gyr_scale = 500.0f  / 32768.0f; break;
        case IMU660RA_GYR_RANGE_250DPS:  dev->gyr_scale = 250.0f  / 32768.0f; break;
        case IMU660RA_GYR_RANGE_125DPS:  dev->gyr_scale = 125.0f  / 32768.0f; break;
        default: dev->gyr_scale = 0; break;
    }
}

/**
 * @brief 读取所有数据（推荐）
 * 说明：连续读取 ACC 与 GYR 寄存器效率更高，随后将原始值按当前量程转换为物理单位。
 */
void IMU660RA_Read_All(IMU660RA_t *dev) {
    uint8_t buf[12];
    // 寄存器地址 ACC_X (0x0C) 到 GYR_Z (0x17) 是连续的
    // 注意：根据手册，通常 ACC 在 0x0C-0x11, GYRO 在 0x12-0x17
    // 可以一次性读取 12 字节
    
    IMU660RA_Read_Burst(dev, IMU660RA_ACC_X_LSB, buf, 12);

    // 解析原始数据（小端序：低字节在前）
    dev->raw.ax = (int16_t)((buf[1] << 8) | buf[0]);
    dev->raw.ay = (int16_t)((buf[3] << 8) | buf[2]);
    dev->raw.az = (int16_t)((buf[5] << 8) | buf[4]);

    dev->raw.gx = (int16_t)((buf[7] << 8) | buf[6]);
    dev->raw.gy = (int16_t)((buf[9] << 8) | buf[8]);
    dev->raw.gz = (int16_t)((buf[11] << 8) | buf[10]);

    // 转换为物理单位（依据当前量程对应的缩放系数）
    dev->data.ax_g = dev->raw.ax * dev->acc_scale;
    dev->data.ay_g = dev->raw.ay * dev->acc_scale;
    dev->data.az_g = dev->raw.az * dev->acc_scale;

    dev->data.gx_dps = dev->raw.gx * dev->gyr_scale;
    dev->data.gy_dps = dev->raw.gy * dev->gyr_scale;
    dev->data.gz_dps = dev->raw.gz * dev->gyr_scale;
}
