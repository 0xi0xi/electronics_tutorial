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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint32_t ticks;
uint8_t is_switched = 0;
uint8_t pre_is_switched = 0;
uint8_t mode;
uint32_t last_toggle = 0;
uint32_t blink_period = 50;
uint32_t key_last = 0;
uint32_t debounce = 20;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ticks = HAL_GetTick();
    //
    // HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_6_GPIO_Port,LED_6_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_7_GPIO_Port,LED_7_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_8_GPIO_Port,LED_8_Pin,GPIO_PIN_SET);
    // HAL_Delay(250);
    // HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_6_GPIO_Port,LED_6_Pin,GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED_7_GPIO_Port,LED_7_Pin,GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_8_GPIO_Port,LED_8_Pin,GPIO_PIN_RESET);
    // HAL_Delay(250);


    uint32_t now = HAL_GetTick();

    is_switched = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);

    if (is_switched == 1 && pre_is_switched == 0 && now - key_last >= debounce)
    {
      mode = !mode;
      key_last = now;
    }
    pre_is_switched = is_switched;

    if (now - last_toggle >= blink_period)
    {
      if (mode)
      {
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
      }
      else
      {
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
      }
      last_toggle = now;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
