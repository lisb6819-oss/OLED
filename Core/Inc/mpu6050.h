/**
 * @file mpu6050.h
 * @brief MPU6050 六轴基础驱动，使用 STM32 HAL 硬件 I2C，阻塞式轮询。
 * @note 初始化后量程固定为加速度 ±2g、角速度 ±250°/s。
 *       不包含零偏校准、姿态解算、FIFO 或 DMP。
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

/* 这里是 7 位地址；驱动内部会左移一位后传给 HAL。 */
#define MPU6050_ADDRESS_AD0_LOW  0x68U
#define MPU6050_ADDRESS_AD0_HIGH 0x69U

typedef struct {
  I2C_HandleTypeDef *i2c;  /**< 已经初始化的 I2C 句柄，可与 OLED 共用。 */
  uint8_t address;         /**< 7 位地址，0x68 或 0x69。 */
  uint8_t initialized;     /**< 仅在所有初始化步骤成功后置 1。 */
} MPU6050_HandleTypeDef;

typedef struct {
  int16_t accel_x; /**< 原始加速度：除以 16384.0f 可得到 g。 */
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;  /**< 原始角速度：除以 131.0f 可得到 °/s。 */
  int16_t gyro_y;
  int16_t gyro_z;
} MPU6050_RawData;

/**
 * @brief 复位、检查 WHO_AM_I，并配置六轴采样。
 * @param device 调用方持有的句柄，不可为 NULL。
 * @param i2c 已调用 MX_I2Cx_Init() 的 HAL I2C 句柄。
 * @param address 7 位地址；AD0 接低电平为 0x68，高电平为 0x69。
 * @return HAL_OK 成功；通信失败返回 HAL 状态；参数/芯片 ID 错误返回 HAL_ERROR。
 * @note 包含启动等待和复位等待，请在主循环上下文调用，不要在中断内调用。
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *device,
                              I2C_HandleTypeDef *i2c, uint8_t address);

/**
 * @brief 连续读取一组六轴有符号原始值，失败时不修改 data。
 * @note 从 0x3B 起读取 14 字节（包括中间的温度寄存器，温度不输出）。
 *       采用一次连续读取，避免逐轴读取时跨越采样更新。
 *       与 OLED 在主循环顺序调用；本驱动不负责多线程总线互斥。
 */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_HandleTypeDef *device,
                                 MPU6050_RawData *data);

#endif /* MPU6050_H */
