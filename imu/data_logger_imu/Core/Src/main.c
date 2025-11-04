/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <iks02a1_motion_sensors_ex.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "kalman_filter.h"
#include "magnetometer_calibration.h"
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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char buffer[150];
KalmanFilter_t kalman_pitch;
KalmanFilter_t kalman_roll;
KalmanFilter_t kalman_yaw;


MagnetometerAutoCalib magCalib = {
    .min = { FLT_MAX, FLT_MAX, FLT_MAX },
    .max = { -FLT_MAX, -FLT_MAX, -FLT_MAX },
    .sample_count = 0
};
//BT
//* --- Telemetría UART3 sin bloqueo ---*/ DONDE
volatile uint8_t uart3_busy = 0;     // 0 = libre, 1 = transmitiendo
char telem_buf[128];                 // buffer de salida
uint16_t telem_div = 20;             // envía cada 20 ciclos (~50 Hz si control=1 kHz)
uint16_t telem_cnt = 0;              // contador de ciclos de control


uint32_t calibration_start_time = 0;
const uint32_t calibration_duration_ms = 5000; // 5 segundos
bool calibration_done = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //inicializacion de sensores
  IKS02A1_MOTION_SENSOR_Init(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);
  IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2MDC_0, MOTION_MAGNETO);

  KalmanFilter_Init(&kalman_pitch);
  KalmanFilter_Init(&kalman_roll);
  KalmanFilter_Init(&kalman_yaw);


  calibration_start_time = HAL_GetTick();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // LED encendido: calibrando

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IKS02A1_MOTION_SENSOR_Axes_t accel_data, gyro_data, mag_data;
    //calibrate mag
    float raw_mag[3] = { (float)mag_data.x, (float)mag_data.y, (float)mag_data.z };
    if (!calibration_done) {
        if ((HAL_GetTick() - calibration_start_time) < calibration_duration_ms) {
            Magnetometer_UpdateCalibration(&magCalib, raw_mag);
        } else {
            Magnetometer_ComputeOffset(&magCalib);
            calibration_done = true;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LED apagado: calibración lista

            snprintf(buffer, sizeof(buffer),
                     "Offsets: X=%.2f Y=%.2f Z=%.2f\r\n",
                     magCalib.offset[0], magCalib.offset[1], magCalib.offset[2]);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
    }
    float calibrated_mag[3];
    for (int i = 0; i < 3; i++) {
        calibrated_mag[i] = raw_mag[i] - magCalib.offset[i];
    }
    /*if (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO, &accel_data) == BSP_ERROR_NONE) {
      snprintf(buffer, sizeof(buffer), "ACC: X=%ld Y=%ld Z=%ld\r\n", accel_data.x, accel_data.y, accel_data.z);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }

    if (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_GYRO, &gyro_data) == BSP_ERROR_NONE) {
      snprintf(buffer, sizeof(buffer), "GYRO: X=%ld Y=%ld Z=%ld\r\n", gyro_data.x, gyro_data.y, gyro_data.z);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }

    if (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2MDC_0, MOTION_MAGNETO, &mag_data) == BSP_ERROR_NONE) {
      snprintf(buffer, sizeof(buffer), "%ld,%ld,%ld\r\n", mag_data.x, mag_data.y, mag_data.z);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }*/


    if (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO, &accel_data) == BSP_ERROR_NONE &&
    IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_GYRO, &gyro_data) == BSP_ERROR_NONE &&
    IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2MDC_0, MOTION_MAGNETO, &mag_data) == BSP_ERROR_NONE)
	{
		// ---- Ángulos con acelerómetro ----
		float accel_pitch = atan2f((float)accel_data.x, sqrtf(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180.0f / 3.14159265f;
		float accel_roll  = atan2f((float)accel_data.y, (float)accel_data.z) * 180.0f / 3.14159265f;
		float yaw_raw = atan2f(calibrated_mag[1], calibrated_mag[0]) * 180.0f / 3.14159265f;

		float gyro_pitch_rate = (float)gyro_data.y / 1000.0f;
		float gyro_roll_rate  = (float)gyro_data.x / 1000.0f;
		float gyro_yaw_rate = (float)gyro_data.z / 1000.0f;


		float pitch = KalmanFilter_Update(&kalman_pitch, accel_pitch, gyro_pitch_rate);
		float roll  = KalmanFilter_Update(&kalman_roll,  accel_roll,  gyro_roll_rate);
		float yaw = KalmanFilter_Update(&kalman_yaw, yaw_raw, gyro_yaw_rate);

		if (++telem_cnt >= telem_div && !uart3_busy) {
			  int n = snprintf(buffer, sizeof(buffer),
						 "pitch: %.2f,roll: %.2f,yaw: %.2f\r\n",
						 pitch, roll, yaw);
			  if (n > 0) {
				  uart3_busy = 1; // marcar ocupada hasta que termine
				  HAL_UART_Transmit_IT(&huart3, (uint8_t*)telem_buf, (uint16_t)n);
			  }
			  telem_cnt = 0;
		  }



//		snprintf(buffer, sizeof(buffer),
//				 "Pitch: %.2f  Roll: %.2f  Yaw(abs): %.2f\r\n",
//				 pitch, roll, yaw);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	}




    HAL_Delay(1); // vel trasmi
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart3_busy = 0; // liberar para el siguiente envío
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
