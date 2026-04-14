#ifndef DATA_BRIDGE_TASKS_H
#define DATA_BRIDGE_TASKS_H
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "MPU6050.h"
#include "serialPackages.h"
#include "crc16.h"
#include "CAN/can.h"

extern SemaphoreHandle_t xSerialMutex; // 创建互斥锁句柄
extern QueueHandle_t xCommandQueue; // 全局队列句柄

void IMUTask(void *pvParameters);
void canRecTask(void *pvParameters);
void serialRecTask(void *pvParameters);

#endif