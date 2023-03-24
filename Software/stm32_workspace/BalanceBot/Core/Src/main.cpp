/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "motor.h"
#include "math.h"
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_SIZE 8			// Bytes
#define STARTUP_TIME 1				// Seconds
#define CTRL_TIMEOUT 1 				// Seconds



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rec_flag = 0;
uint8_t UART1_rxBuffer[28] = {0};

uint32_t loop_count = 0;
uint32_t last_spi_msg = 0;
float dt = 0.001;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef   TxHeader;
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t				  RxData[8];
uint32_t count = 0;

Motor motor1;
Motor motor2;
IMU imu;

float current1;
float current2;
typedef struct
{
  int16_t vel_t_1;    	 	/*!< Specifies the target velocity for motor 1*/
  int16_t vel_t_2;    	 	/*!< Specifies the target velocity for motor 2*/

} VelocityTargetMsg;
VelocityTargetMsg vel_targ;

uint8_t spi_buf[SPI_BUFFER_SIZE];

float error, error_prev;
float P, I, D;
float I_accum, d_error;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 28);



  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  TxHeader.Identifier = 0x200;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

	current1 = 0.0;
	current2 = 0.0;
	float target_vel1 = 0.0;
	float target_vel2 = 0.0;
	int16_t current_word1 = 0.0;
	int16_t current_word2 = 0.0;

	float kp_gain = 0.0002;
	float ki_gain = 0.000;
	float kd_gain = 0.01;
	float max_current = 3.0;
	motor1.PID_Init(kp_gain, ki_gain, kd_gain, max_current);
	motor2.PID_Init(kp_gain, ki_gain, kd_gain, max_current);
	imu.init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  P = .03;
  I = 0.0006;
  D = 0.01;
  I_accum = 0.0;
  d_error = 0.0;
  while (1)
  {
	if(rec_flag)
	{
		imu.complimentary_update();
		if(abs(imu.get_angle()) < 45)
		{
			error_prev = error;
			error = imu.get_angle();
			I_accum += error;
			d_error = error-error_prev;
			float vel_targ = error*P + I_accum*I + d_error*D;
//			current1 = motor1.PID_Controller(vel_targ);
//			current2 = motor2.PID_Controller(-vel_targ);
			if(abs(vel_targ) > 1.5)
			{
				if(vel_targ > 0)
				{
					vel_targ = 1.5;
				}
				else
				{
					vel_targ = -1.5;
				}
			}
			if(abs(I_accum) > 0.5/I)
			{
				if(I_accum > 0)
				{
					I_accum = 0.5/I;
				}
				else
				{
					I_accum = -0.5/I;
				}
			}
			current1 = vel_targ;
			current2 = -vel_targ;
			rec_flag=0;
		}
		else
		{
			current1 = 0;
			current2 = 0;
		}

	}

	current_word1 = (int16_t)(current1/20.0*16384);
	current_word2 = (int16_t)(current2/20.0*16384);
	TxData[0] = current_word1 >> 8;
	TxData[1] = (int8_t)(current_word1 & 0x00ff);
	TxData[2] = current_word2 >> 8;
	TxData[3] = (int8_t)(current_word2 & 0x00ff);



	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)!= HAL_OK)
	{
//		Error_Handler();
	}
	HAL_Delay(1);
	loop_count++;



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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
//  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_0);  // toggle PA0
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	memcpy(&imu.sample.ms_counter, &UART1_rxBuffer, sizeof(imu.sample));
	rec_flag = 1;
//	HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 28);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    /* Reception Error */
//    Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
//      Error_Handler();
    }
  }
}
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

#ifdef  USE_FULL_ASSERT
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
