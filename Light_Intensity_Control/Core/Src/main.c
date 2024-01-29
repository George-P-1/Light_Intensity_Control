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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_config.h"
#include "encoder_config.h"
#include "bh1750_config.h"
#include "WS2812.h"

//#include "lcd_hd44780_i2c.h"
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
unsigned char character;
char text[LCD_LINE_BUF_LEN];
unsigned int i = 0;

// For UART
uint8_t tx_buffer[100];
uint8_t rx_buffer[100];
#define rxt_msg_len 7

// initial values for encoder, etc
uint32_t encoder_val = 0;
#define ENC_STEP 50
uint32_t set_point = 0;
uint32_t sensor_val = 0;
float Illuminance_lux = 0.0f;
unsigned int Illuminance_lux_Int = 0;

// control signal and flag
uint16_t brightness_level = 0;
uint16_t hal_delay_val = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to check if 'value' is within the range [lowerBound, upperBound]
int isWithinRange(int value, int lowerBound, int upperBound) {
    return (value >= lowerBound && value <= upperBound);
}

/**
  * @brief  Pulse Finished callback in DMA mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  datasentflag=1;
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim6)
  {
    encoder_val = ENC_GetCounter(&henc1);
    set_point = encoder_val * ENC_STEP;
    LCD_I2C_SetCursor(&hlcd3, 0, 6);
    LCD_I2C_printStr(&hlcd3, "       lux");
    LCD_I2C_SetCursor(&hlcd3, 0, 6);
    LCD_I2C_printDecInt(&hlcd3, set_point);

    Illuminance_lux = BH1750_ReadIlluminance_lux(&hbh1750);
    Illuminance_lux_Int = Illuminance_lux * 1000.0f;

//    int tx_msg_len = sprintf((char*)tx_buffer, "%05u.%03u\r", Illuminance_lux_Int / 1000, Illuminance_lux_Int % 1000);
    sensor_val = (uint32_t)Illuminance_lux;
    LCD_I2C_SetCursor(&hlcd3, 1, 6);
    LCD_I2C_printStr(&hlcd3, "       lux");
    LCD_I2C_SetCursor(&hlcd3, 1, 6);
    LCD_I2C_printDecInt(&hlcd3, sensor_val);

    int tx_msg_len = sprintf((char*)tx_buffer, "REF: %*lu lux  :  MSR: %*lu lux  :  CON: %*u [/255]\n\r", 4, set_point, 4, sensor_val, 3, brightness_level);
    HAL_UART_Transmit(&huart3, tx_buffer, tx_msg_len, 100);
  }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  sscanf((char*)rx_buffer, "ENC%lu", &set_point);
  encoder_val = set_point/ENC_STEP; // to maintain the 50 lumen step of reference signal
  ENC_SetCounter(&henc1, encoder_val);

//  memset(rx_buffer, 0, sizeof(rx_buffer));
  HAL_UART_Receive_IT(&huart3, rx_buffer, rxt_msg_len);
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
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize Encoder
  ENC_Init(&henc1);
  HAL_TIM_Base_Start_IT(&htim6); // timer to send rotary encoder updates to LCD and maybe not just encoder but also sensor

  // Initialize LCD
  LCD_I2C_Init(&hlcd3);
  LCD_I2C_printStr(&hlcd3, " REF:");
  LCD_I2C_SetCursor(&hlcd3, 1, 0);
  LCD_I2C_printStr(&hlcd3, " MSR:");
  LCD_I2C_SetCursor(&hlcd3, 0, 6);
  LCD_I2C_printStr(&hlcd3, ">");
  LCD_I2C_SetCursor(&hlcd3, 1, 6);
  LCD_I2C_printStr(&hlcd3, ">");

  // Initialize BH1750
  BH1750_Init(&hbh1750);

  // Initialize 2 LEDs with some color
  Set_LED(0, 255, 255, 255);
  Set_LED(1, 255, 255, 255);

  Set_Brightness(255);
  WS2812_Send();

//  HAL_UART_Receive_IT(&huart3, &character, 1);
  HAL_UART_Receive_IT(&huart3, tx_buffer, rxt_msg_len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// Check closeness of measurement and reference signals
    if(isWithinRange(sensor_val, set_point-50, set_point+50))
    {
      hal_delay_val = 1000;
    } else
    {
      hal_delay_val = 100;
    }

    // Control signal updated based on sign of error (ON-OFF Control, Two Position)
    if(sensor_val != set_point && sensor_val < set_point)
    {
      brightness_level = brightness_level + 1;
      if(brightness_level > 255) brightness_level = 255;
      else if(brightness_level < 0) brightness_level = 0;
      Set_Brightness(brightness_level);
      WS2812_Send();
      HAL_Delay (hal_delay_val);
    }
    else if (sensor_val != set_point && sensor_val > set_point)
    {
      brightness_level = brightness_level - 1;
      if(brightness_level > 255) brightness_level = 255;
      else if(brightness_level < 0) brightness_level = 0;
      Set_Brightness(brightness_level);
      WS2812_Send();
      HAL_Delay (hal_delay_val);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
