/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdint.h"
#include "AS5600.h"
#include "usbd_customhid.h"

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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t button_pressed = false;
  uint8_t previous_button_pressed = false;
  extern USBD_HandleTypeDef hUsbDeviceFS;

  STEPPER_Disable();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  AS5600_init(&hi2c1);
  AS5600_set_home();

  const int32_t precision = 4092;
  const int32_t ticks_per_degree = 1138; // (int)(4092f / 360f * 100)
  int32_t angle = 0;
  int32_t angle_max =  2 * ticks_per_degree / 100;
  int32_t angle_min = -2 * ticks_per_degree / 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  button_pressed = !HAL_GPIO_ReadPin(BUT_1_GPIO_Port, BUT_1_Pin);
	  if(previous_button_pressed != button_pressed)
	  {
		  uint8_t report[15] ={
			  1, // report ID
			  button_pressed,
			  0,0,0,0,0,
			  0,0,0,0,0,0,0,0
		  };
		  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, sizeof(report) / sizeof(report[0]));
		  HAL_GPIO_TogglePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin);
	  }

	  previous_button_pressed = button_pressed;

	  angle = AS5600_get_angle();

	  // apply force feedback
	  if(angle > angle_max)
	  {
		  STEPPER_CounterClockwise();
		  STEPPER_Enable();
	  }
	  else if (angle < angle_min)
	  {
		  STEPPER_Clockwise();
		  STEPPER_Enable();
	  }
	  else
	  {
		  // TODO: disable the steppers after a few milliseconds to prevent overshooting.
		  STEPPER_Disable();
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void STEPPER_Enable()
{
	// the A4988's Enable state is ACTIVE LOW
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_RESET);
}

void STEPPER_Disable()
{
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_SET);
}

void STEPPER_Clockwise()
{
	HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, GPIO_PIN_SET);
}

void STEPPER_CounterClockwise()
{
	HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, GPIO_PIN_RESET);
}

void STEPPER_Step()
{
	// not used. stepping is performed using the PWM timer.
}

float STEPPER_ReadAngle()
{
	const uint8_t AS5600_DEVICE_ADDRESS = 0x6C;
	const uint8_t AS5600_REGISTER_STATUS = 0x0B;
	const uint8_t AS5600_REGISTER_RAW_ANGLE_HI = 0x0C;
	const uint8_t AS5600_REGISTER_RAW_ANGLE_LO = 0x0D;
	const uint16_t precision = 4096; // the AS5600 encoder has a precision of 4096 counts

	uint8_t buffer[2];
	buffer[0] = AS5600_REGISTER_RAW_ANGLE_HI;

	HAL_I2C_Master_Transmit(&hi2c1, AS5600_DEVICE_ADDRESS, buffer, 1, 5);
    HAL_I2C_Master_Receive(&hi2c1, AS5600_DEVICE_ADDRESS, buffer, 2, 5);
    uint16_t adjusted_angle = buffer[0] << 8 | buffer[1];

    return 360.0f / (float)precision * (float)adjusted_angle;
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
