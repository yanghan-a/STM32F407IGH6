#ifndef LOOP_H
#define LOOP_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include "stdint-gcc.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "freertos_inc.h"
void Main();
void OnUartCmd(uint8_t* _data, uint16_t _len);
void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len);

#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#endif
#endif
