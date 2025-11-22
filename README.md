# IMU660RA STM32 HAL 驱动

## 简介

这是一个为 [逐飞科技 IMU660RA](https://www.seekfree.com.cn/) 六轴姿态传感器编写的 STM32 HAL 库驱动程序。

该驱动旨在提供一个简单、易于移植的接口，方便在基于 STM32 的项目中使用 IMU660RA 传感器。

## 硬件要求

-   任何支持 SPI 通信的 STM32 微控制器。
-   逐飞科技 IMU660RA 传感器模块。

## 如何使用

1.  **添加文件到你的项目:**
    将以下三个文件添加到你的 STM32 项目中：
    -   `imu660ra_hal.h` (头文件)
    -   `imu660ra_hal.c` (驱动实现)
    -   `imu660ra_firmware.c` (初始化固件)

2.  **配置 SPI 和 GPIO:**
    在你的代码中，根据 `imu660ra_hal.h` 文件中的 `IMU660RA_t` 结构体定义，配置并传入一个 `SPI_HandleTypeDef` 句柄以及 CS (片选) 引脚的 GPIO 信息。

    ```c
    // 示例：在 main.c 中
    #include "imu660ra_hal.h"

    IMU660RA_t my_imu;

    void main(void) {
        // ... 系统初始化 ...
        // ... MX_SPIx_Init() ...
        // ... MX_GPIO_Init() ...

        // 初始化设备句柄
        IMU660RA_Init(&my_imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin);

        // ...
    }
    ```

3.  **初始化传感器:**
    调用 `IMU660RA_Init()` 函数来初始化传感器。该函数会执行内部自检并加载配置固件。建议检查其返回值以确保初始化成功。

    ```c
    IMU660RA_Status_t status = IMU660RA_Init(&my_imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin);

    if (status == IMU660RA_OK) {
        // 初始化成功
    } else {
        // 初始化失败，根据状态码进行调试
        switch (status) {
            case IMU660RA_ERROR_NULL_PTR:
                // 错误：传入了空指针
                break;
            case IMU660RA_ERROR_CHIP_ID:
                // 错误：读取到的芯片 ID 不正确，请检查硬件连接
                break;
            case IMU660RA_ERROR_INIT:
                // 错误：传感器内部初始化失败
                break;
        }
    }
    ```

4.  **读取数据:**
    调用 `IMU660RA_Read_All()` 函数来获取加速度、陀螺仪数据。

    ```c
    IMU660RA_Read_All(&my_imu);

    // 读取转换后的物理单位数据
    float accel_x = my_imu.data.ax_g;
    float accel_y = my_imu.data.ay_g;
    float accel_z = my_imu.data.az_g;

    float gyro_x = my_imu.data.gx_dps;
    float gyro_y = my_imu.data.gy_dps;
    float gyro_z = my_imu.data.gz_dps;
    ```

## API 参考

-   `IMU660RA_Status_t IMU660RA_Init(IMU660RA_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)`
    > 初始化传感器，执行自检并加载配置。返回一个 `IMU660RA_Status_t` 枚举值。

-   `void IMU660RA_Read_All(IMU660RA_t *dev)`
    > 从传感器读取最新的加速度和陀螺仪数据，并将其转换为标准单位（g, dps）存放在 `dev->data` 中。

-   `void IMU660RA_Set_Acc_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_AccRange_t range)`
    > 配置加速度计的输出数据率 (ODR) 和测量量程。

-   `void IMU660RA_Set_Gyr_Config(IMU660RA_t *dev, IMU660RA_ODR_t odr, IMU660RA_GyrRange_t range)`
    > 配置陀螺仪的输出数据率 (ODR) 和测量量程。

## 关于固件的说明

`imu660ra_firmware.c` 文件中包含一个由厂商提供的二进制数组，用于在初始化时配置传感器内部的 ASIC。
