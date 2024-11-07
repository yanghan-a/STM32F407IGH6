#include "cmsis_os2.h"
# include "main.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "usbd_cdc_if.h"
char *msg = "UART1 test\n";


void uart1_task(void *argument)
{
    /* USER CODE BEGIN uart1_task */
    /* Infinite loop */
    for(;;)
    {
        uint8_t tx_buffer1[] = "Hello,world!\n";
        uint16_t size = sizeof(tx_buffer1);
        if(Recv_dlen)//判断是否接收到数据，接收置位处理在static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)函数
        {


        }

        osDelay(100);

    }
    /* USER CODE END uart1_task */
}

void Led_red(void *argument)
{
    /* USER CODE BEGIN Led_red */
    /* Infinite loop */
    for(;;)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        osDelay(500);
    }
    /* USER CODE END Led_red */
}