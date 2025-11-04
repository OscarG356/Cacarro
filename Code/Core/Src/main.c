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
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "robot_kinematics.h"
#include "ekf_dd.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define ESPERA 15000

int Sampling = 0;
int16_t Acount = 0;
float Aangle = 0;
int16_t Bcount = 0;
float Bangle = 0;
float Apast_angle = 0;
float Aspeed = 0;
float Bpast_angle = 0;
float Bspeed = 0;
float Duty = 0;
float dt = 0;
float y_k = 0;
float By_k = 0;
float r = 0;
float Br = 0;
//float e_k = 0, e_k_1 = 0, e_k_2 = 0;
//float u_k = 0, u_k_1 = 0, u_k_2 = 0;
float u_k = 0, Bu_k = 0;
float coorx[4]= {2.0f,    2.0f,    0.0f,   0.0f};
float coory[4] = {0.0f,    2.0f,    2.0f,   0.0f};
float Xd = 0.0f;
float Yd = 0.0f;

//	EVASIÓN DE OBSTACULOS(4*6)

//float alphas[8][9] = {{0, 0, 0, 1.400864734998522, -0.816273551248890, 0.168912137812222, -0.012122886484352, 0.0, 0.0},
//		{4, 0, 0, -0.0578170886460264, 0.0433628164845197, -0.01084070412113, 0.0009033920100942, 0.0 , 0.0},
//		{4, 0, 0, -1.400864734998522, 0.816273551248890, -0.168912137812222, 0.012122886484352, 0.0, 0.0},
//		{0, 0, 0, 0.0578170886460264, -0.0433628164845197, 0.01084070412113, -0.0009033920100942, 0.0, 0.0},
//		{4, 0, 0, 0.333581735217150, -0.248149041130290, 0.055389808226058, -0.003948653881737, 0.0, 0.0},
//		{2, 0, 0, -0.098164604945153,  0.010898762967092, 0.003580247406582, -0.000494683160439, 0.0, 0.0},
//		{0, 0, 0,  -0.001789077758982,  0.001073446655389, -0.000214689331078, 0.000014312622072, 0.0, 0.0},
//		{0, 0, 0, 0.091066348578766, -0.054639809147260, 0.010927961829452, -0.000728530788630, 0.0, 0.0}};
//
//float betas[8][9] = {{0, 0, 0, 0.059645443779439, -0.044734082834579,  0.011183520708645, -0.000931960059054, 0.0, 0.0},
//		{0, 0, 0,0.057152225192306, 0.30869833110577, -0.129908957776442, 0.01375543398137, 0.0 , 0.0},
//		{6, 0, 0, -0.059645443779439, 0.044734082834579,  -0.011183520708645, 0.000931960059054,0.0, 0.0},
//		{6, 0, 0, -0.057152225192306, -0.30869833110577, 0.129908957776442, -0.01375543398137, 0.0, 0.0},
//		{6, 0, 0, -0.010782280545491, 0.006469368327295, -0.001293873665459, 0.000086258244364, 0.0, 0.0},
//		{6, 0, 0, -0.002813650178160, 0.001688190106896, -0.000337638021379, 0.000022509201425, 0.0, 0.0},
//		{6, 0, 0, -0.137896451670704,  0.010737871002422, 0.006492425799516, -0.000816828386634, 0.0, 0.0},
//		{3, 0, 0, 0.296007012321636,  -0.249604207392982, 0.058560841478597, -0.004288056098573, 0.0, 0.0}};


// TRAYECTORIA CUADRADA
float alphas[4][9] = {{0, 0, 0, 0.32, -0.096, 0.00768, 0.0, 0.0 , 0.0},
		{4, 0, 0, 0.0, 0.0, 0.0, 0, 0.0 , 0.0},
		{4, 0, 0, -0.32, 0.096, -0.00768, 0.0 , 0.0},
		{0, 0, 0, 0.0, 0.0, 0.0, 0, 0.0 , 0.0}};
float betas[4][9] = {{0, 0, 0, 0.0f, 0.0f, 0.0f, 0, 0.0 , 0.0},
		{0, 0, 0, 0.32, -0.096, 0.00768, 0.0, 0.0 , 0.0},
		{4, 0, 0, 0.0, 0.0, 0.0, 0, 0.0 , 0.0},
		{4, 0, 0, -0.32, 0.096, -0.00768, 0.0, 0.0 , 0.0}};

float AR[4][4] = {
    {0.5116f,    0.0070f,    0.0000f,   -0.0002f},
    {-5.6090f,    0.8795f,    0.0006f,   -0.0356f},
    {0.9522f,   -0.2830f,    0.9999f,    0.0064f},
    {0.0f,		 0.0f,		 0.0f, 		0.3679f}
};

float AL[4][4] = {
    {0.2982f,    0.0054f,    0.0000f,   -0.0003f},
    {-12.3759f,    0.7801f,    0.0009f,   -0.0576f},
    {2.2989f,   -0.2692f,    0.9999f,    0.0106f},
    {0.0f,		 0.0f,		 0.0f, 		0.3679f}
};

float BR[4][2] = {
    {0.0002f,    0.4884f},
    {0.0399f,    5.6090f},
    {0.2939f,   -0.9522f},
    {0.0f,		 0.0f}
};
float BL[4][2] = {
    {0.0002f,    0.7018f},
    {0.0652f,   12.3759f},
    {0.2898f,   -2.2989},
    {0.0f,		 0.0f}
};

float c11 = 0, c12 = -70, c13 = 1, c14 = -100;
float d11 = 70, d12 = 0;
float x1_k_1 = 0, x2_k_1 = 0, x3_k_1 = 0, x4_k_1 = 0;
float x1_k = 0, x2_k = 0, x3_k = 0, x4_k = 0;

float Bx1_k_1 = 0, Bx2_k_1 = 0, Bx3_k_1 = 0, Bx4_k_1 = 0;
float Bx1_k = 0, Bx2_k = 0, Bx3_k = 0, Bx4_k = 0;

volatile int32_t Acount_total = 0;
volatile int32_t Bcount_total = 0;

float phi_d = 0.0f;

float T_esp = 0;
float t = 0;
int count = 1;
int coor = 0;
int band = 1;
int band_giro = 0;

volatile uint8_t uart3_busy = 0;     // 0 = libre, 1 = transmitiendo
char telem_buf[128];                 // buffer de salida
uint16_t telem_div = 20;             // envía cada 20 ciclos (~50 Hz si control=1 kHz)
uint16_t telem_cnt = 0;              // contador de ciclos de controlstatic EKF_DD kf;
static const float DT = 0.01f;   // 100 Hz

//FILTRO DE KALMAN
static EKF_DD kf;
static void KF_Init(void)
{
    // Estado inicial
    float x0[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Covarianza inicial P0
    float P0[5][5] = {{0}};
    for (int i=0;i<5;i++) P0[i][i] = (i<3) ? 1.0f : 0.25f;

    // Ruido de proceso Q (discreto)
    float Qd[5][5] = {{0}};
    Qd[0][0]=1e-4f;   // x
    Qd[1][1]=1e-4f;   // y
    Qd[2][2]=5e-4f;   // theta
    Qd[3][3]=5e-3f;   // v
    Qd[4][4]=5e-3f;   // omega

    // Ruido de medición R (diag)
    float Rm[3][3] = {{0}};
    Rm[0][0] = 0.05f * 0.05f;                           // sigma_x ≈ 5 cm (si tienes x abs)
    Rm[1][1] = 0.05f * 0.05f;                           // sigma_y ≈ 5 cm (si tienes y abs)
    Rm[2][2] = (float)powf((float)M_PI/180.0f*2.0f, 2); // sigma_theta ≈ 2°

    ekf_dd_init(&kf, x0, P0, Qd, Rm);
}

RobotKinematics cacarro;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void control(float u, TIM_HandleTypeDef *htim);
static void Forward(float Duty, TIM_HandleTypeDef *htim);
static void Backward(float Duty, TIM_HandleTypeDef *htim);


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim14); // Start the sampling timer
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); // Start the encoder timer
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL); // Start the encoder timer
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	RobotKinematics_Init(&cacarro, 0.05,0.315);

//	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	t = 0;
	while (1)
	{

		while(Sampling)
		{
			Sampling = 0;

//			if(t<=20){
//				Xd = alphas[0][0] + alphas[0][1]*t + alphas[0][2]*powf(t,2) + alphas[0][3]*powf(t,3) + alphas[0][4]*powf(t,4) + alphas[0][5]*powf(t,5);
//				Yd = betas[0][0] + betas[0][1]*t + betas[0][2]*powf(t,2) + betas[0][3]*powf(t,3) + betas[0][4]*powf(t,4) + betas[0][5]*powf(t,5);
//				band = 1;
//			}
			// GIRO Y CUADRADO
			if(band_giro){
				if(t<=2){
					Xd = 1 * (1 - cos((2*3.1416/8) * t));
					Yd = 1 * sin((2*3.1416/8) * t);
					Xd = alphas[(coor+1)%4][0] + cos(t);
					Yd = betas[(coor+1)%4][0] + sin(t);
				}
			}
			else{
				if(t<=5){
					Xd = alphas[coor][0] + alphas[coor][1]*t + alphas[coor][2]*powf(t,2) + alphas[coor][3]*powf(t,3) + alphas[coor][4]*powf(t,4) + alphas[coor][5]*powf(t,5) + alphas[coor][6]*powf(t,6) + alphas[coor][7]*powf(t,7) + alphas[coor][8]*powf(t,8);
					Yd = betas[coor][0] + betas[coor][1]*t + betas[coor][2]*powf(t,2) + betas[coor][3]*powf(t,3) + betas[coor][4]*powf(t,4) + betas[coor][5]*powf(t,5) + betas[coor][6]*powf(t,6) + betas[coor][7]*powf(t,7) + betas[coor][8]*powf(t,8);
					band = 1;

				}
			}


//			if(t<=8){
//				Xd = alphas[coor][0] + alphas[coor][1]*t + alphas[coor][2]*powf(t,2) + alphas[coor][3]*powf(t,3) + alphas[coor][4]*powf(t,4) + alphas[coor][5]*powf(t,5) + alphas[coor][6]*powf(t,6) + alphas[coor][7]*powf(t,7);
//				Yd = betas[coor][0] + betas[coor][1]*t + betas[coor][2]*powf(t,2) + betas[coor][3]*powf(t,3) + betas[coor][4]*powf(t,4) + betas[coor][5]*powf(t,5) + betas[coor][6]*powf(t,6) + betas[coor][7]*powf(t,7);
//				band = 1;
//			}
//			else{
//				if(band){
//					alphas[count][0] = cacarro.x;
//					bethas[count][0] = cacarro.y;
//					count += 1;
//					band = 0;
//				}
//			}


			//IZQUIERDO
			Aspeed = (Aangle - Apast_angle)/0.001;
			Apast_angle = Aangle;
			y_k = Aangle;

			//DERECHO
			Bspeed = (Bangle - Bpast_angle)/0.001;
			Bpast_angle = Bangle;
			By_k = Bangle;

			RobotKinematics_Update(&cacarro, Aangle, Bangle, 0.001);

			phi_d = W_Control_Law(&cacarro, Xd, Yd);
			VL_Control_Law(&cacarro, Xd, Yd);
			Angular_Vel(&cacarro);

			r = cacarro.W_L_d;
			Br = cacarro.W_R_d;

			//MOTOR A : IZQUIERDO
			x1_k_1 = AR[0][0]*x1_k + AR[0][1]*x2_k + AR[0][2]*x3_k + AR[0][3]*x4_k + BR[0][0]*r + BR[0][1]*y_k;
			x2_k_1 = AR[1][0]*x1_k + AR[1][1]*x2_k + AR[1][2]*x3_k + AR[1][3]*x4_k + BR[1][0]*r + BR[1][1]*y_k;
			x3_k_1 = AR[2][0]*x1_k + AR[2][1]*x2_k + AR[2][2]*x3_k + AR[2][3]*x4_k + BR[2][0]*r + BR[2][1]*y_k;
			x4_k_1 = AR[3][0]*x1_k + AR[3][1]*x2_k + AR[3][2]*x3_k + AR[3][3]*x4_k + BR[3][0]*r + BR[3][1]*y_k;

			u_k = c11*x1_k + c12*x2_k +c13*x3_k + c14*x4_k + d11*r;
			control(u_k, &htim3);
			x1_k = x1_k_1;
			x2_k = x2_k_1;
			x3_k = x3_k_1;
			x4_k = x4_k_1;

			//MOTOR B : DERECHO

			Bx1_k_1 = AL[0][0]*Bx1_k + AL[0][1]*Bx2_k + AL[0][2]*Bx3_k + AL[0][3]*Bx4_k + BL[0][0]*Br + BL[0][1]*By_k;
			Bx2_k_1 = AL[1][0]*Bx1_k + AL[1][1]*Bx2_k + AL[1][2]*Bx3_k + AL[1][3]*Bx4_k + BL[1][0]*Br + BL[1][1]*By_k;
			Bx3_k_1 = AL[2][0]*Bx1_k + AL[2][1]*Bx2_k + AL[2][2]*Bx3_k + AL[2][3]*Bx4_k + BL[2][0]*Br + BL[2][1]*By_k;
			Bx4_k_1 = AL[3][0]*Bx1_k + AL[3][1]*Bx2_k + AL[3][2]*Bx3_k + AL[3][3]*Bx4_k + BL[3][0]*Br + BL[3][1]*By_k;

			Bu_k = c11*Bx1_k + c12*Bx2_k +c13*Bx3_k + c14*Bx4_k + d11*Br;
			control(Bu_k, &htim5);
			Bx1_k = Bx1_k_1;
			Bx2_k = Bx2_k_1;
			Bx3_k = Bx3_k_1;
			Bx4_k = Bx4_k_1;

			// ACTUALIZACIÓN DEL FILTRO
			// --- Paso de PREDICCIÓN (usas tu v y ω ya calculados) ---
			ekf_dd_predict_kin(&kf, v, omega, DT);

			// --- Paso de ACTUALIZACIÓN (mediciones disponibles este ciclo) ---z = (x,y,theta)
			float z[3] = {0.0f, 0.0f, 0.0f};
			unsigned mask = 0;

			// Yaw de IMU (recomendado siempre)
			if (have_yaw_imu()) {
				z[2] = get_yaw_imu_rad();     // Leer angulo y ponerlo en su posición
				mask |= EKF_MEAS_THETA;
			}

			// Posición absoluta opcional
			/*
			if (have_pos_abs()) {
				float x_abs, y_abs;
				get_pos_abs(&x_abs, &y_abs);
				z[0] = x_abs;  mask |= EKF_MEAS_X;
				z[1] = y_abs;  mask |= EKF_MEAS_Y;
			}
			*/

			// Si no hay mediciones, mask queda 0 y ekf_dd_update no hace nada
			ekf_dd_update(&kf, z, mask);

			// --- Usa el estado filtrado ---
			float x     = kf.x[0];
			float y     = kf.x[1];
			float theta = kf.x[2];
			float vhat  = kf.x[3];
			float omhat = kf.x[4];

			// (Opcional) Log
			// printf("%f,%f,%f,%f,%f\r\n", x, y, theta, vhat, omhat);

			 if (++telem_cnt >= telem_div && !uart3_busy) {
				  int n = snprintf(telem_buf, sizeof(telem_buf),
								   "%.3f,%.3f,%.3f\r\n",
									cacarro.x, cacarro.y, cacarro.theta);
				  if (n > 0) {
					  uart3_busy = 1; // marcar ocupada hasta que termine
					  HAL_UART_Transmit_IT(&huart3, (uint8_t*)telem_buf, (uint16_t)n);
				  }
				  telem_cnt = 0;
			  }

		}
		// send data
//		uart_buf_len = sprintf(uart_buf,"%0.2f \t", r);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//		uart_buf_len = sprintf(uart_buf,"%0.2f \t", y_k);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//		uart_buf_len = sprintf(uart_buf,"%0.2f \r\n", u_k);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 4-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 840-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	if(htim == &htim14)
	{
		Sampling = 1;
		if(T_esp <= ESPERA)
		{
			T_esp += 1;
		}
		if(t <= 20){//SEPARAR TIEMPOS DE ESPERA, VAR GLOBAL
			t += 0.001;
		}
		else{
			if(coor<3){
				coor += 1;
				t = 0;
			}
//			if(band_giro==1){//Raro
//				band_giro=0;
//				if(coor<3){
//					coor += 1;
//				}
//			}
//			else{
//				band_giro=1;
//			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart3_busy = 0; // liberar para el siguiente envío
    }
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim)
//{
//	if(htim == &htim2)
//	{
//		Bcount = (int16_t)__HAL_TIM_GET_COUNTER(htim);
//		Bangle = 2*3.1416*Bcount/383.6;
//	}
//	if(htim == &htim4){
//		Acount = (int16_t)__HAL_TIM_GET_COUNTER(htim);	//Encoder A
//		Aangle = (-2*3.1416f*Acount)/(383.6);
//	}
//}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim)
{
   if(htim == &htim2) // Encoder B
    {
        static uint16_t lastB = 0;
        uint16_t nowB = __HAL_TIM_GET_COUNTER(htim);
        int16_t deltaB = (int16_t)(nowB - lastB);
        Bcount_total += deltaB;
        lastB = nowB;

        Bangle = 2.0f * 3.1416f * Bcount_total / 383.6f;
    }

    if(htim == &htim4) // Encoder A
    {
        static uint16_t lastA = 0;
        uint16_t nowA = __HAL_TIM_GET_COUNTER(htim);
        int16_t deltaA = (int16_t)(nowA - lastA);
        Acount_total += deltaA;
        lastA = nowA;

        Aangle = (-2.0f * 3.1416f * Acount_total) / 383.6f;
    }
}

static void control(float u, TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		if(u >= 999){
			Forward(999, &htim3);
		}else if(u <= -999){
			Backward(999, &htim3);
		}else if(u>=0 && u<999){
			Forward(u, &htim3);
		}else if(u>-999 && u<0){
			Backward(-u, &htim3);
		}
	}
	if(htim == &htim5)
	{
		if(u >= 999){
			Forward(999, &htim5);
		}else if(u <= -999){
			Backward(999, &htim5);
		}else if(u>=0 && u<999){
			Forward(u, &htim5);
		}else if(u>-999 && u<0){
			Backward(-u, &htim5);
		}
	}
}

static void Forward(float Duty, TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		htim3.Instance->CCR2 = 0;
		htim3.Instance->CCR1 = Duty;		//Motor A
	}
	if(htim == &htim5)
	{
		htim5.Instance->CCR1 = 0;
		htim5.Instance->CCR2 = Duty;		//Motor B
	}
}

static void Backward(float Duty, TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		htim3.Instance->CCR1 = 0;
		htim3.Instance->CCR2 = Duty;
	}
	if(htim == &htim5)
	{
		htim5.Instance->CCR2 = 0;
		htim5.Instance->CCR1 = Duty;
	}
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
