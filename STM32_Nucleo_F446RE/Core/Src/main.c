/* USER CODE BEGIN Header */

/******************************************************************************
 * @file    main.c
 * @brief   Main application entry point for the MPU6050 attitude estimator.
 *
 * @details Responsibilities:
 *          - Initialize hardware interfaces
 *          - Calibrate the IMU
 *          - Manage loop timing
 *          - Acquire sensor data
 *          - Execute attitude estimation algorithms
 *          - Output telemetry
 *
 * @author  Sumedh Camarushi
 * @date    June 25, 2026
 * ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************/

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

#include "mpu6050.h"
#include "calibration.h"
#include "attitude.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

// -----------------------------------------------------------------------------
// Timing and filter configuration
// -----------------------------------------------------------------------------

#define MILLISECONDS_PER_SECOND 1000.0f   ///< Number of milliseconds in one second (used for time conversion)
#define LOOP_PERIOD_MS          10U       ///< Loop period in milliseconds (10 ms = 100 Hz)
#define COMPLEMENTARY_ALPHA     0.98f     ///< Complementary filter weighting coefficient (0.98 = 98% gyro, 2% accel)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// -----------------------------------------------------------------------------
// Calibration data
// -----------------------------------------------------------------------------

AccelBias accelBias = {0};     ///< Accelerometer bias values
GyroBias gyroBias   = {0};     ///< Gyroscope bias values

// -----------------------------------------------------------------------------
// Attitude estimates
// -----------------------------------------------------------------------------

Attitude accelAttitude    = {0};     ///< Accelerometer-based attitude estimate
Attitude gyroOnlyAttitude = {0};     ///< Gyroscope-only attitude estimate (for plotting purposes)
Attitude gyroAttitude     = {0};     ///< Corrected attitude estimate used for gyro integration
Attitude filteredAttitude = {0};     ///< Complementary-filtered attitude estimate

// -----------------------------------------------------------------------------
// Loop timing state
// -----------------------------------------------------------------------------

uint32_t lastLoopTime_MS  = 0;       ///< Timestamp of the last loop iteration in milliseconds
uint8_t imuReady          = 0;       ///< IMU initialization state flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

/**
 * @brief          Prints one telemetry sample in CSV format.
 *
 * @details        Builds one comma-separated telemetry line and sends it
 *                 over UART.
 *
 *                 Output format:
 *                 time_MS, accel.roll, accel.pitch,
 *                 gyro.roll, gyro.pitch,
 *                 filtered.roll, filtered.pitch
 *
 *                 This format is useful for serial logging, plotting,
 *                 and host-side parsing.
 *
 * @param time_MS  Timestamp in milliseconds.
 * @param accel    Pointer to the accelerometer-only attitude estimate.
 * @param gyro     Pointer to the gyroscope-only attitude estimate.
 * @param filtered Pointer to the complementary-filtered attitude estimate.
 */
void printTelemetryCSV(
  uint32_t time_MS,
  const Attitude *accel,
  const Attitude *gyro,
  const Attitude *filtered);

/**
 * @brief   Performs one-time application initialization.
 *
 * @details Handles sensor startup, calibration, initial attitude estimation,
 *          and loop timing initialization.
 */
void appSetup(void);

/**
 * @brief   Executes one iteration of the main attitude estimation loop.
 *
 * @details Enforces loop timing, reads sensor data, converts measurements,
 *          updates the attitude estimates, and outputs telemetry.
 */
void appLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

void printTelemetryCSV(
  uint32_t time_MS,
  const Attitude *accel,
  const Attitude *gyro,
  const Attitude *filtered)
{
  // A buffer is temporary memory used to store text before transmission.
  // Here, the buffer holds one complete CSV telemetry line.

  char buffer[128];
  int length = 0;

  // Exit immediately if any input pointer is invalid.

  if ((accel == 0) || (gyro == 0) || (filtered == 0))
  {
    return;
  }

  // snprintf() writes formatted text into the buffer.
  // The format string creates one CSV line ending with \r\n.

  length = snprintf(
    buffer,
    sizeof(buffer),
    "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
    (unsigned long)time_MS,
    accel->roll,
    accel->pitch,
    gyro->roll,
    gyro->pitch,
    filtered->roll,
    filtered->pitch);

  // Transmit the formatted string only if formatting succeeded.

  if (length > 0)
  {
    uart_print(buffer);
  }
}

void appSetup(void)
{
  // -------------------------------------------------------------------------
  // Hardware initialization
  // -------------------------------------------------------------------------

  // Delay briefly so the sensor can power up before I2C communication begins.

  HAL_Delay(100);

  // -------------------------------------------------------------------------
  // Sensor initialization
  // -------------------------------------------------------------------------

  // Print status messages over UART during startup.

  uart_print("Initializing MPU6050...\r\n");

  // Give the MPU6050 driver access to the configured I2C peripheral.

  MPU6050_SetI2C(&hi2c1);

  // Initialize the sensor and apply its configuration.

  initializeMPU6050();

  // Calibrate the accelerometer while the IMU is stationary.

  uart_print("Calibrating accelerometer...\r\n");
  accelBias = calibrateAccel();

  // Calibrate the gyroscope while the IMU is stationary.

  uart_print("Calibrating gyroscope...\r\n");
  gyroBias = calibrateGyro();

  // -------------------------------------------------------------------------
  // Attitude initialization
  // -------------------------------------------------------------------------

  // Estimate the initial roll and pitch from the calibrated accelerometer.

  if (initializeAttitude(&accelBias, &accelAttitude) != HAL_OK)
  {
    uart_print("Attitude initialization failed.\r\n");
    Error_Handler();
  }

  // Initialize both gyro-based estimates to the same starting attitude.

  gyroOnlyAttitude = accelAttitude;
  gyroAttitude     = accelAttitude;

  // Compute the initial filtered estimate.

  if (complementaryFilter(
    &gyroAttitude,
    &accelAttitude,
    COMPLEMENTARY_ALPHA,
    &filteredAttitude) != HAL_OK)
  {
    uart_print("Complementary filter initialization failed.\r\n");
    Error_Handler();
  }

  uart_print("Calibration complete.\r\n");

  // Store the current tick count for loop timing.

  lastLoopTime_MS = HAL_GetTick();

  // Mark initialization as complete.

  imuReady = 1U;

  uart_print("MPU6050 IMU awake.\r\n\r\n");
}

void appLoop(void)
{
  // -------------------------------------------------------------------------
  // Timing control
  // -------------------------------------------------------------------------

  // Get the current system time in milliseconds.

  uint32_t currentTime_MS = HAL_GetTick();

  // Do nothing until initialization has completed.

  if (imuReady == 0U)
  {
    return;
  }

  // Skip this iteration until the desired loop period has elapsed.

  if (currentTime_MS - lastLoopTime_MS < LOOP_PERIOD_MS)
  {
    return;
  }

  // Convert elapsed time from milliseconds to seconds for gyro integration.

  float dt = (currentTime_MS - lastLoopTime_MS) / MILLISECONDS_PER_SECOND;

  // Save the current timestamp for the next loop iteration.

  lastLoopTime_MS = currentTime_MS;

  // -------------------------------------------------------------------------
  // Sensor acquisition & unit conversion
  // -------------------------------------------------------------------------

  // Store raw sensor readings from the MPU6050.

  AccelData accel = {0};
  GyroData gyro   = {0};

  // Read accelerometer data. Skip this iteration if the read fails.

  if (readRawAccel(&accel) != HAL_OK)
  {
    return;
  }

  // Read gyroscope data. Skip this iteration if the read fails.

  if (readRawGyro(&gyro) != HAL_OK)
  {
    return;
  }

  // Convert raw accelerometer values to g after bias correction.

  float axG = rawAccelToG(accel.x, accelBias.x);
  float ayG = rawAccelToG(accel.y, accelBias.y);
  float azG = rawAccelToG(accel.z, accelBias.z);

  // Convert raw gyroscope values to degrees per second after bias correction.

  float gxDPS = rawGyroToDPS(gyro.x, gyroBias.x);
  float gyDPS = rawGyroToDPS(gyro.y, gyroBias.y);
  float gzDPS = rawGyroToDPS(gyro.z, gyroBias.z);

  // -------------------------------------------------------------------------
  // Attitude estimation
  // -------------------------------------------------------------------------

  // Temporary variables hold the newly updated gyro estimates.

  Attitude updatedGyroOnly  = {0};
  Attitude updatedGyro      = {0};

  // Compute roll and pitch directly from the accelerometer.

  if (calculateAccelAttitude(axG, ayG, azG, &accelAttitude) != HAL_OK)
  {
    return;
  }

  // Update the gyro-only estimate used for comparison and plotting.

  if (updateGyroAttitude(
    &gyroOnlyAttitude,
    gxDPS,
    gyDPS,
    gzDPS,
    dt,
    &updatedGyroOnly) != HAL_OK)
  {
    return;
  }

  // Update the gyro estimate used by the complementary filter.

  if (updateGyroAttitude(
    &gyroAttitude,
    gxDPS,
    gyDPS,
    gzDPS,
    dt,
    &updatedGyro) != HAL_OK)
  {
    return;
  }

  // Save the updated gyro estimates.

  gyroOnlyAttitude = updatedGyroOnly;
  gyroAttitude = updatedGyro;

  // Fuse the gyro and accelerometer estimates.

  if (complementaryFilter(
    &gyroAttitude,
    &accelAttitude,
    COMPLEMENTARY_ALPHA,
    &filteredAttitude) != HAL_OK)
  {
    return;
  }

  // Feed the filtered result back into the gyro estimate to reduce drift.

  gyroAttitude = filteredAttitude;

  // -------------------------------------------------------------------------
  // Telemetry output
  // -------------------------------------------------------------------------

  // Legacy human-readable telemetry format kept here for reference.
  
  // Multiple telemetry formats are available.
  // Additional debug output can be enabled
  // by uncommenting the sections below.

  // char debugBuffer[96];
  //
  // snprintf(debugBuffer, sizeof(debugBuffer), "AX (g): %.3f AY (g): %.3f AZ (g): %.3f\r\n", axG, ayG, azG);
  // uart_print(debugBuffer);
  //
  // snprintf(debugBuffer, sizeof(debugBuffer), "GX (dps): %.3f GY (dps): %.3f GZ (dps): %.3f\r\n", gxDPS, gyDPS, gzDPS);
  // uart_print(debugBuffer);
  //
  // snprintf(debugBuffer, sizeof(debugBuffer), "Accel Roll: %.3f Gyro Roll: %.3f Filtered Roll: %.3f\r\n",
  //          accelAttitude.roll, gyroAttitude.roll, filteredAttitude.roll);
  // uart_print(debugBuffer);
  //
  // snprintf(debugBuffer, sizeof(debugBuffer), "Accel Pitch: %.3f Gyro Pitch: %.3f Filtered Pitch: %.3f\r\n",
  //          accelAttitude.pitch, gyroAttitude.pitch, filteredAttitude.pitch);
  // uart_print(debugBuffer);

  // Send one compact CSV telemetry line over UART.

  printTelemetryCSV(
    currentTime_MS,
    &accelAttitude,
    &gyroOnlyAttitude,
    &filteredAttitude);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  appSetup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    appLoop();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void uart_print(const char *msg)
{
  if (msg == 0)
  {
    return;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

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
