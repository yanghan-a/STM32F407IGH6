#ifndef __FREERTOS_H
#define __FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os.h"
// List of semaphores
extern osSemaphoreId_t sem_usb_rxHandle;
extern osSemaphoreId_t sem_usb_txHandle;
extern osSemaphoreId_t sem_uart6_rx_dmaHandle;
extern osSemaphoreId_t sem_uart6_tx_dmaHandle;
extern osSemaphoreId_t sem_can1_txHandle;
extern osSemaphoreId_t sem_can2_txHandle;
// extern osSemaphoreId_t sem_can1_tx;
// extern osSemaphoreId_t sem_can2_tx;

extern osThreadId_t controlLoopFixUpdateHandle;
extern osThreadId_t ControlLoopUpdateHandle;
extern osThreadId_t uart6TxTaskHandle;
#ifdef __cplusplus
}
#endif

#endif