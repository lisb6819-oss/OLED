/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mpu6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* AD0 接低电平用 0x68；接高电平改为 MPU6050_ADDRESS_AD0_HIGH。 */
static MPU6050_HandleTypeDef mpu6050;
static MPU6050_RawData motion;
static HAL_StatusTypeDef mpu_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 用 6x8 字体显示一轴原始值，每行 8 像素。
 * @note %+6d 固定数值宽度且显示正负号，可容纳 -32768~+32767。
 *       使用整数格式化，无需启用 printf 浮点支持。
 */
static void DisplayAxis(uint8_t y, const char *name, int16_t value)
{
  char line[22];
  snprintf(line, sizeof(line), "%s:%+6d", name, (int)value);
  OLED_PrintASCIIString(0, y, line, &afont8x6, OLED_COLOR_NORMAL);
}

/** @brief 清空缓存，绘制六轴数据，最后统一刷新屏幕。 */
static void DisplayMotion(void)
{
  OLED_NewFrame();
  OLED_PrintASCIIString(0, 0, "MPU6050 RAW", &afont8x6, OLED_COLOR_NORMAL);
  DisplayAxis(8,  "AX", motion.accel_x);
  DisplayAxis(16, "AY", motion.accel_y);
  DisplayAxis(24, "AZ", motion.accel_z);
  DisplayAxis(32, "GX", motion.gyro_x);
  DisplayAxis(40, "GY", motion.gyro_y);
  DisplayAxis(48, "GZ", motion.gyro_z);
  OLED_PrintASCIIString(0, 56, "A:2g G:250dps", &afont8x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

/** @brief 失败时显示提示，避免将旧数据误认为实时数据。 */
static void DisplayMotionError(HAL_StatusTypeDef status, uint8_t init_failed)
{
  OLED_NewFrame();
  OLED_PrintASCIIString(0, 0, "MPU6050", &afont8x6, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 16, init_failed ? "INIT FAILED" : "READ FAILED",
                        &afont8x6, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 24, status == HAL_TIMEOUT ? "I2C TIMEOUT" :
                        status == HAL_BUSY ? "I2C BUSY" : "I2C/ID ERROR",
                        &afont8x6, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 40, "CHECK WIRE / AD0", &afont8x6, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 56, "RETRY IN 1s", &afont8x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20);
  OLED_Init();
  /* 两个器件共用 hi2c1，主循环顺序访问，不会同时占用总线。 */
  mpu_status = MPU6050_Init(&mpu6050, &hi2c1, MPU6050_ADDRESS_AD0_LOW);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (mpu_status != HAL_OK)
    {
      /* 初始化失败时每秒重试，不让主程序停在死循环中。 */
      DisplayMotionError(mpu_status, 1);
      HAL_Delay(1000);
      mpu_status = MPU6050_Init(&mpu6050, &hi2c1, MPU6050_ADDRESS_AD0_LOW);
      continue;
    }

    mpu_status = MPU6050_ReadRaw(&mpu6050, &motion);
    if (mpu_status == HAL_OK)
    {
      DisplayMotion();
      /* 传感器内部 100Hz 采样，屏幕每次刷新后延时 100ms，便于观察。 */
      HAL_Delay(100);
    }
    else
    {
      DisplayMotionError(mpu_status, 0);
      HAL_Delay(1000);
      mpu_status = MPU6050_Init(&mpu6050, &hi2c1, MPU6050_ADDRESS_AD0_LOW);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
