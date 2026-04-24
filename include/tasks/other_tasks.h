#ifndef OTHER_TASKS_H
#define OTHER_TASKS_H
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "CAN/can.h"

void can_rx_task(void *pvParameters);   // 低优先级任务专门“吃掉”没用的can数据

#endif