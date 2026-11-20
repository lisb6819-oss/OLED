/** @file mpu6050.c
 *  @brief 寄存器配置依据 InvenSense MPU-6000/MPU-6050 Register Map。
 */
#include "mpu6050.h"
#include <stddef.h>

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_PWR_MGMT_2   0x6CU
#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_I2C_TIMEOUT_MS   100U

/* HAL 的 DevAddress 要求左对齐地址，因此将 7 位地址左移一位。 */
static HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_HandleTypeDef *device,
                                              uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(device->i2c, (uint16_t)(device->address << 1),
                           reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
                           MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050_HandleTypeDef *device,
                                              uint8_t reg, uint8_t *data,
                                              uint16_t length)
{
  return HAL_I2C_Mem_Read(device->i2c, (uint16_t)(device->address << 1),
                          reg, I2C_MEMADD_SIZE_8BIT, data, length,
                          MPU6050_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *device,
                              I2C_HandleTypeDef *i2c, uint8_t address)
{
  uint8_t id;
  HAL_StatusTypeDef status;
  /* 依次唤醒、启用六轴、设置滤波/采样率及两个量程。 */
  static const uint8_t settings[][2] = {
    {MPU6050_REG_PWR_MGMT_1,   0x01}, /* 清除 SLEEP，选择 X 轴陀螺仪 PLL。 */
    {MPU6050_REG_PWR_MGMT_2,   0x00}, /* 六轴均不进入待机。 */
    {MPU6050_REG_CONFIG,       0x03}, /* DLPF：加速度约 44Hz，陀螺仪约 42Hz。 */
    {MPU6050_REG_SMPLRT_DIV,   0x09}, /* 开启 DLPF 后：1kHz / (1+9) = 100Hz。 */
    {MPU6050_REG_GYRO_CONFIG,  0x00}, /* 关闭自检，±250°/s，131 LSB/(°/s)。 */
    {MPU6050_REG_ACCEL_CONFIG, 0x00}  /* 关闭自检，±2g，16384 LSB/g。 */
  };

  if (device == NULL) return HAL_ERROR;
  device->initialized = 0;
  if (i2c == NULL || (address != MPU6050_ADDRESS_AD0_LOW &&
                      address != MPU6050_ADDRESS_AD0_HIGH)) return HAL_ERROR;
  device->i2c = i2c;
  device->address = address;

  HAL_Delay(100); /* 给传感器上电稳定时间，也适用于上电后立即调用。 */
  status = MPU6050_ReadRegisters(device, MPU6050_REG_WHO_AM_I, &id, 1);
  if (status != HAL_OK) return status;
  /* WHO_AM_I 固定为 0x68，不随 AD0 引脚变成 0x69。 */
  if (id != 0x68U) return HAL_ERROR;

  status = MPU6050_WriteRegister(device, MPU6050_REG_PWR_MGMT_1, 0x80);
  if (status != HAL_OK) return status;
  HAL_Delay(100); /* 软复位会恢复默认寄存器，需要等待后重新配置。 */

  for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i)
  {
    status = MPU6050_WriteRegister(device, settings[i][0], settings[i][1]);
    if (status != HAL_OK) return status;
  }
  HAL_Delay(100); /* 等待唤醒后的陀螺仪及滤波输出稳定。 */
  device->initialized = 1;
  return HAL_OK;
}

/* 传感器高字节在前；显式处理补码，保证负数正确还原。 */
static int16_t MPU6050_DecodeInt16(const uint8_t *bytes)
{
  uint16_t value = ((uint16_t)bytes[0] << 8) | bytes[1];
  int32_t signed_value = (value & 0x8000U) ? (int32_t)value - 65536 : value;
  return (int16_t)signed_value;
}

HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_HandleTypeDef *device,
                                 MPU6050_RawData *data)
{
  uint8_t bytes[14];
  HAL_StatusTypeDef status;
  if (device == NULL || data == NULL || !device->initialized ||
      device->i2c == NULL) return HAL_ERROR;

  status = MPU6050_ReadRegisters(device, MPU6050_REG_ACCEL_XOUT_H,
                                  bytes, sizeof(bytes));
  if (status != HAL_OK) return status; /* 不用失败传输的内容覆盖旧数据。 */

  data->accel_x = MPU6050_DecodeInt16(&bytes[0]);
  data->accel_y = MPU6050_DecodeInt16(&bytes[2]);
  data->accel_z = MPU6050_DecodeInt16(&bytes[4]);
  /* bytes[6..7] 是温度，跳过后才是三轴角速度。 */
  data->gyro_x = MPU6050_DecodeInt16(&bytes[8]);
  data->gyro_y = MPU6050_DecodeInt16(&bytes[10]);
  data->gyro_z = MPU6050_DecodeInt16(&bytes[12]);
  return HAL_OK;
}
