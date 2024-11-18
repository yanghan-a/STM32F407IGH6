/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "common_inc.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_red */
osThreadId_t led_redHandle;
const osThreadAttr_t led_red_attributes = {
  .name = "led_red",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sem_usb_rx */
osSemaphoreId_t sem_usb_rxHandle;
const osSemaphoreAttr_t sem_usb_rx_attributes = {
  .name = "sem_usb_rx"
};
/* Definitions for sem_usb_tx */
osSemaphoreId_t sem_usb_txHandle;
const osSemaphoreAttr_t sem_usb_tx_attributes = {
  .name = "sem_usb_tx"
};
/* Definitions for sem_uart6_rx_dma */
osSemaphoreId_t sem_uart6_rx_dmaHandle;
const osSemaphoreAttr_t sem_uart6_rx_dma_attributes = {
  .name = "sem_uart6_rx_dma"
};
/* Definitions for sem_uart6_tx_dma */
osSemaphoreId_t sem_uart6_tx_dmaHandle;
const osSemaphoreAttr_t sem_uart6_tx_dma_attributes = {
  .name = "sem_uart6_tx_dma"
};
/* Definitions for sem_can1_tx */
osSemaphoreId_t sem_can1_txHandle;
const osSemaphoreAttr_t sem_can1_tx_attributes = {
  .name = "sem_can1_tx"
};
/* Definitions for sem_can2_tx */
osSemaphoreId_t sem_can2_txHandle;
const osSemaphoreAttr_t sem_can2_tx_attributes = {
  .name = "sem_can2_tx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void led_red_func(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_usb_rx */
  sem_usb_rxHandle = osSemaphoreNew(1, 0, &sem_usb_rx_attributes);

  /* creation of sem_usb_tx */
  sem_usb_txHandle = osSemaphoreNew(1, 1, &sem_usb_tx_attributes);

  /* creation of sem_uart6_rx_dma */
  sem_uart6_rx_dmaHandle = osSemaphoreNew(1, 0, &sem_uart6_rx_dma_attributes);

  /* creation of sem_uart6_tx_dma */
  sem_uart6_tx_dmaHandle = osSemaphoreNew(1, 1, &sem_uart6_tx_dma_attributes);

  /* creation of sem_can1_tx */
  sem_can1_txHandle = osSemaphoreNew(1, 1, &sem_can1_tx_attributes);

  /* creation of sem_can2_tx */
  sem_can2_txHandle = osSemaphoreNew(1, 1, &sem_can2_tx_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_red */
  led_redHandle = osThreadNew(led_red_func, NULL, &led_red_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
__weak void DefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN DefaultTask */
  /* Infinite loop */
  Main();

  vTaskDelete(defaultTaskHandle);
  /* USER CODE END DefaultTask */
}

/* USER CODE BEGIN Header_led_red_func */
/**
* @brief Function implementing the led_red thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_red_func */
__weak void led_red_func(void *argument)
{
  /* USER CODE BEGIN led_red_func */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
    osDelay(500);
  }
  /* USER CODE END led_red_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

