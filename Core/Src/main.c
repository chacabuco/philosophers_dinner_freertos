/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "philosopher_interface.h"

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
 osThreadId defaultTaskHandle;
osThreadId Philosopher0Handle;
osThreadId Philosopher1Handle;
osThreadId Philosopher2Handle;
osThreadId Philosopher3Handle;
osThreadId Philosopher4Handle;
osSemaphoreId mutexHandle;
osSemaphoreId phil0semHandle;
osSemaphoreId phil1semHandle;
osSemaphoreId phil2semHandle;
osSemaphoreId phil3semHandle;
osSemaphoreId phil4semHandle;
osSemaphoreId phil_sem_arr[5];
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void philosopher(uint8_t argument);
void take_forks(uint8_t argument, uint8_t* state);
void test(uint8_t argument, uint8_t* state);

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of mutex */
  osSemaphoreDef(mutex);
  mutexHandle = osSemaphoreCreate(osSemaphore(mutex), 1);

  /* definition and creation of phil0sem */
  osSemaphoreDef(phil0sem);
  phil0semHandle = osSemaphoreCreate(osSemaphore(phil0sem), 1);
  phil_sem_arr[0] = phil0semHandle;
  vQueueAddToRegistry(phil0semHandle, "Phil0Semaphore");
  /* definition and creation of phil1sem */
  osSemaphoreDef(phil1sem);
  phil1semHandle = osSemaphoreCreate(osSemaphore(phil1sem), 1);
  phil_sem_arr[1] = phil1semHandle;
  vQueueAddToRegistry(phil1semHandle, "Phil1Semaphore");
  /* definition and creation of phil2sem */
  osSemaphoreDef(phil2sem);
  phil2semHandle = osSemaphoreCreate(osSemaphore(phil2sem), 1);
  phil_sem_arr[2] = phil2semHandle;
  vQueueAddToRegistry(phil2semHandle, "Phil2Semaphore");
  /* definition and creation of phil3sem */
  osSemaphoreDef(phil3sem);
  phil3semHandle = osSemaphoreCreate(osSemaphore(phil3sem), 1);
  phil_sem_arr[3] = phil3semHandle;
  vQueueAddToRegistry(phil3semHandle, "Phil3Semaphore");
  /* definition and creation of phil4sem */
  osSemaphoreDef(phil4sem);
  phil4semHandle = osSemaphoreCreate(osSemaphore(phil4sem), 1);
  phil_sem_arr[4] = phil4semHandle;
  vQueueAddToRegistry(phil4semHandle, "Phil4Semaphore");

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Philosopher0 */
  osThreadDef(Philosopher0, philosopher, osPriorityNormal, 0, 64);
  Philosopher0Handle = osThreadCreate(osThread(Philosopher0), 0);

  /* definition and creation of Philosopher1 */
  osThreadDef(Philosopher1, philosopher, osPriorityNormal, 0, 64);
  Philosopher1Handle = osThreadCreate(osThread(Philosopher1), 1);

  /* definition and creation of Philosopher2 */
  osThreadDef(Philosopher2, philosopher, osPriorityNormal, 0, 64);
  Philosopher2Handle = osThreadCreate(osThread(Philosopher2), 2);

  /* definition and creation of Philosopher3 */
  osThreadDef(Philosopher3, philosopher, osPriorityNormal, 0, 64);
  Philosopher3Handle = osThreadCreate(osThread(Philosopher3), 3);

  /* definition and creation of Philosopher4 */
  osThreadDef(Philosopher4, philosopher, osPriorityNormal, 0, 64);
  Philosopher4Handle = osThreadCreate(osThread(Philosopher4), 4);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	  for (int i=0; i<5; i++)
	  {
		  osSemaphoreWait(phil_sem_arr[i], 0);
	  }
  for(;;)
  {
    osDelay(0);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_philosopher */
/**
* @brief Function implementing the Philosoher0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_philosopher */
void philosopher(uint8_t argument)
{
  /* USER CODE BEGIN philosopher */
  /* Infinite loop */
  static uint8_t state[5] = {0,0,0,0,0};

  for(;;)
  {
	take_forks(argument, state);
	osDelay(3); // EATING
	put_forks(argument, state);

  }
  /* USER CODE END philosopher */
}

void take_forks(uint8_t argument, uint8_t* state)
{
	osSemaphoreWait(mutexHandle, 0);
	{
			state[argument] = HUNGRY;
			test(argument, state);
	}
	osSemaphoreRelease(mutexHandle);
	osSemaphoreWait(phil_sem_arr[argument], 5);

}

void put_forks(uint8_t argument, uint8_t* state)
{
	osSemaphoreWait(mutexHandle, 0);
		{
				state[argument] = THINKING;
				test(LEFT, state);
				test(RIGHT, state);

		}
		osSemaphoreRelease(mutexHandle);

}

void test(uint8_t argument, uint8_t* state)
{
	if (state[argument] == HUNGRY && state[LEFT] != EATING && state[RIGHT] != EATING)
	{
		state[argument] = EATING;
		osSemaphoreRelease(phil_sem_arr[argument]);
	}
}

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
