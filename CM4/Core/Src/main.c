/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "app_subghz_phy.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <bme68x_necessary_functions.h>
#include <stdio.h>   // printf
#include <string.h>  // strcmp
#include "stm32_seq.h"      // Add this
#include "stm32_timer.h"    // Add this for timer functions

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CFG_SEQ_Task_Sensor  1    // Task ID = 1 (the ping-pong task is probably 0)
#define CFG_SEQ_Prio_0       0    // Priority level 0
#define UTIL_SEQ_RFU         0    // Reserved for future use flag
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

struct bme68x_data sensor_data;
UTIL_TIMER_Object_t SensorTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Sensor_Task(void);
void Sensor_Timer_Callback(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct bme68x_data sensor_data;

void Sensor_Task(void)
{
    if (bme68x_single_measure(&sensor_data) == 0) {
        // Measurement successful
        sensor_data.iaq_score = bme68x_iaq();
        char msgBuffer[120];
                for(uint16_t i = 0; i < 120; i++){
                    msgBuffer[i] = ' ';
                }
        // TODO: Send this data via LoRa
        // You can trigger the TX task here or store data for next transmission
                __HAL_RCC_GPIOB_CLK_ENABLE();

                // 2. Configure Pin 15 as Output (usually done in MX_GPIO_Init)
                GPIO_InitTypeDef GPIO_InitStruct = {0};
                GPIO_InitStruct.Pin = GPIO_PIN_15;
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

                // 3. Set the pin HIGH to turn ON the LED
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

                // Optional: Add a small delay if you need to see it on
                // HAL_Delay(1000);

                // 4. Set the pin LOW to turn OFF the LED
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

void Sensor_Timer_Callback(void *argument)
{
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Sensor), CFG_SEQ_Prio_0);
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
  MX_SubGHz_Phy_Init();
 // __HAL_RCC_I2C2_CLK_ENABLE();  // Force enable I2C2 clock
 // __HAL_RCC_GPIOA_CLK_ENABLE(); // Force enable GPIOA clock

  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //////////////////////////////BME688 INIT BEGIN/////////////////////////////
  //struct bme68x_data data;
  bme68x_start(&sensor_data, &hi2c2);
  //////////////////////////////BME688 INIT BEGIN/////////////////////////////
  // Register your sensor task with the sequencer
      UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Sensor), UTIL_SEQ_RFU, Sensor_Task);

      // Set the task to run periodically (optional, or trigger it from ping-pong)
      UTIL_TIMER_Create(&SensorTimer, 2000, UTIL_TIMER_PERIODIC, Sensor_Timer_Callback, NULL);
      UTIL_TIMER_Start(&SensorTimer);
      UTIL_SEQ_SetTask((1 << 1), CFG_SEQ_Prio_0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

      //Stop debugger from shutting off
      HAL_DBGMCU_EnableDBGSleepMode();
      HAL_DBGMCU_EnableDBGStopMode();
      HAL_DBGMCU_EnableDBGStandbyMode();


  while (1)
  {
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

    UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);

    /* USER CODE BEGIN 3 */
    //////////////////////////////BME START MEASURMENT//////////////////////////////

	//  printf("hello World!");
/*if (bme68x_single_measure(&data) == 0) {

			// Measurement is successful, so continue with IAQ
			data.iaq_score = bme68x_iaq(); // Calculate IAQ

			// Create a message buffer and clear it.
			char msgBuffer[120];
			for(uint16_t i = 0; i < 120; i++){
				msgBuffer[i] = ' ';
			}

			// Send the data through UART.
			//printf("BME680 MEASURMENT:\r\n");
		//	printf(
		//			"Temperature(deg C): %.2f, Pressure(Pa): %.2f, Humidity(%%): %.2f, IAQ: %.1f ,Gas resistance(ohm): %.2f\r\n",
		//			data.temperature, data.pressure, data.humidity,
		//			data.iaq_score, data.gas_resistance);

		  // HAL_UART_Transmit(&huart2, (uint8_t *) msgBuffer, sizeof(msgBuffer), 10);

		}*/

		//HAL_Delay(2000);
    //////////////////////////////BME END MEASURMENT//////////////////////////////

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
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
