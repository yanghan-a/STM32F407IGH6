
/* Includes ------------------------------------------------------------------*/

#include "communication.hpp"
#include "common_inc.h"

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile bool endpointListValid = false;
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/
// @brief Sends a line on the specified output.

osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
    .name = "commTask",
    .stack_size = 45000,
    .priority = (osPriority_t) osPriorityNormal,
};

void InitCommunication(void)
{
    // Start command handling thread
    commTaskHandle = osThreadNew(CommunicationTask, nullptr, &commTask_attributes);

    // while (!endpointListValid)
    //     osDelay(1);
}

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void CommunicationTask(void* ctx)
{
    (void) ctx; // unused parameter

    CommitProtocol();

    // Allow main init to continue
    endpointListValid = true;

    StartUartServer();
    StartUsbServer();
    StartCanServer(CAN1);
    // StartCanServer(CAN2);

    for (;;)
    {
        osDelay(1000); // nothing to do
    }
}

// extern "C" {
// int _write(int file, const char* data, int len);
// }

// @brief This is what printf calls internally
// int _write(int file, const char* data, int len)
// {
//     usbStreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
//     uart4StreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
//
//     return len;
// }
