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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TIMEOUT 100

#define RED_REGISTER 0x02
#define GREEN_REGISTER 0x03
#define BLUE_REGISTER 0x04
#define WHITE_REGISTER 0x05

#define PCA9633_ADDRESS 0x20  //REQUIRES PROPER CONNECTION

#define WHITE_VALUE 128   // VALUE BETWEEN (0-255)
#define LIGHTNESS 50 //VALUE BETWEEN (0-100)

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

void PCA9633_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t reg, uint8_t value);
void PCA9633_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
void SetLedColor(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);

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
    uint8_t red=255;
	uint8_t green, blue,white;
    float lightness;
    uint8_t i;

    white = WHITE_VALUE;
    lightness = (((float)LIGHTNESS)/100.0);


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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  PCA9633_Init(&hi2c1,PCA9633_ADDRESS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  for (i = 0; i < 256; i++)
	  {
		 lightness = (float)i / 255.0;

		 // Calculate RGB values for rainbow color based on current position
		 if (i < 85) {
			 red = 255 - i * 3;
			 green = i * 3;
			 blue = 0;
		 } else if (i < 170) {
			 red = 0;
			 green = 255 - (i - 85) * 3;
			 blue = (i - 85) * 3;
		 } else {
			 red = (i - 170) * 3;
			 green = 0;
			 blue = 255 - (i - 170) * 3;
		 }

		 // Adjust the colors based on LIGHTNESS
		 red = (uint8_t)((float)red * lightness);
		 green = (uint8_t)((float)green * lightness);
		 blue = (uint8_t)((float)blue * lightness);
		 blue = (uint8_t)((float)white * lightness);


		 // Set the LED color
		 SetLedColor(&hi2c1,PCA9633_ADDRESS,red, green, blue,white);

		 HAL_Delay(50);  // blocking code...


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

void PCA9633_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    HAL_I2C_Master_Transmit(hi2c, DevAddress,buffer,2,I2C_TIMEOUT);
}

void PCA9633_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    PCA9633_WriteRegister(hi2c,DevAddress,0x00, 0x00);  	// Set MODE1 register for normal operation

    PCA9633_WriteRegister(hi2c,DevAddress,0x01, 0x00);		// Set MODE2 register for full output logic (no inverted outputs)
}

void SetLedColor(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
	PCA9633_WriteRegister(hi2c,DevAddress,RED_REGISTER,red);
	PCA9633_WriteRegister(hi2c,DevAddress,GREEN_REGISTER,green);
	PCA9633_WriteRegister(hi2c,DevAddress,BLUE_REGISTER,blue);
	PCA9633_WriteRegister(hi2c,DevAddress,WHITE_REGISTER,white);
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
