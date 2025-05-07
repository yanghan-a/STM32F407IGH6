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
#include "task.h"
#include "main.h"
#include "string.h"

void Main();
void OnTimer11Callback();
void OnTimer7Callback();
void OnTimer10Callback();


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include "communication.hpp"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "instances/dummy_robot.h"
#include "timer.hpp"
#endif
#endif
