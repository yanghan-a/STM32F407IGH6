#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "common_inc.h"
#include "usart.h"
#include "usbd_cdc_if.h"

float myFloat = 3.14159f;
uint32_t num_uart;
uint8_t buffer_uart[128]; // float 是 4 字节
uint8_t buffer_usb[128]; // float 是 4 字节


void OnUartCmd(uint8_t* _data, uint16_t _len)
{

    // printf("%d\n",sizeof(_data));
    // tx_buffer[50] = sizeof(_data);
    // tx_buffer[51] = sizeof(tx_buffer);
    // HAL_UART_Transmit_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
    // HAL_UART_Transmit(&huart6, rx_buffer, _len, HAL_MAX_DELAY);

    uint8_t tx_buffer1[] = "Hello\n";
    uint16_t size = sizeof(tx_buffer1)-1;
    //
    // // 使用 DMA 发送数据
    // HAL_UART_Transmit(&huart6, tx_buffer1, size, HAL_MAX_DELAY);
    // printf("%d\n",size);
    num_uart = _len;
    memcpy(buffer_uart, _data, _len);  // 将 float 数据复制到字节缓冲区
    HAL_UART_Transmit_DMA(&huart6, buffer_uart,  num_uart);
    // HAL_UART_Transmit_DMA(&huart6, tx_buffer, txLen);


    //usb发送
    Recv_dlen = _len;
    memcpy(UserRxBuffer, _data, _len);
    CDC_Transmit_FS((uint8_t*)&UserRxBuffer, Recv_dlen);//把接收到的数据拷贝到发送
    Recv_dlen=0;//长度清零
}




