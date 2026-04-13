#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "tasks/ins_task.h"
// #include "bipedal_data.h"
#include "CAN/CAN_comm.h"
#include "CAN/can.h"
// #include "CAN/config.h"
// #include <WiFi.h>
// #include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ppm.h"
#include "Motor.h"
#include "robot.h"
#include "kinematics.h"
#include "pid.h"
#include "utils.h"
#include "crc16.h"
#include "serialPackages.h"

MPU6050 mpu6050 = MPU6050(Wire); // 实例化MPU6050

void IMUTask(void *pvParameters);
void canRecTask(void *pvParameters);
void serialRecTask(void *pvParameters);
void Open_thread_function(); // 启动线程

SemaphoreHandle_t xSerialMutex; // 创建互斥锁句柄

QueueHandle_t xCommandQueue; // 全局队列句柄

void setup()
{
  Wire.begin(1, 2, 400000UL); // 初始化IIC
  Serial.begin(921600);       // 初始化调试串口
  mpu6050.begin();            // 初始化MPU陀螺仪
  // mpu6050.setGyroOffsets(3.73, -1.59, -0.16);
  ppm_init(); // 遥控器读取中断初始化
  CANInit();
  delay(1000);
  enableJointMotors(); // 使能关节电机
  motorInit();         // 轮毂电机初始化

  // 创建互斥锁
  xSerialMutex = xSemaphoreCreateMutex();
  if (xSerialMutex == NULL)
  {
    Serial.println("Failed to create mutex");
    return; // 或采取其他错误处理措施
  }

  // 创建命令队列，长度可容纳 20 个包（避免消费不及时丢失）
  xCommandQueue = xQueueCreate(20, sizeof(SerialCommandPackage));

  Open_thread_function(); // 启动线程

  /* USER CALIBRATE IMU START */
  // 静止平放时解注释获取imu校准矩阵，正常运行时注释
  // delay(2);
  // mpu6050.calcGyroOffsets(true);
  // mpu6050.calibrateAccelerometer();
  /* USER CALIBRATE IMU END */
}

void loop()
{
  storeFilteredPPMData(); // 获取遥控器数据
  remoteSwitch();         // 获取遥控器模式
  mapPPMToRobotControl(); // 映射遥控器各通道数值为控制指令

  // 非阻塞处理接收到的命令
  SerialCommandPackage cmd;
  while (xQueueReceive(xCommandQueue, &cmd, 0) == pdTRUE)
  {
    // 根据 cmd.command 执行相应操作
    // handleSerialCommand(&cmd);
    sendTestPackage packet;
    packet.aaa = cmd.aaa;

    Append_CRC16_Check_Sum((uint8_t *)&packet, sizeof(sendTestPackage)); // 计算 CRC
    if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    {
      Serial.write((uint8_t *)&packet, sizeof(sendTestPackage)); // 发送数据
      xSemaphoreGive(xSerialMutex);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(1)); // 避免空转
}

// 启动线程
void Open_thread_function()
{
  // 陀螺仪读取任务进程
  xTaskCreatePinnedToCore(
      IMUTask,   // 任务函数
      "IMUTask", // 任务名称
      4096,      // 堆栈大小 4096 × 4 = 16384B
      NULL,      // 传递的参数
      5,         // 任务优先级
      NULL,      // 任务句柄
      1          // 运行在核心 1
  );
  xTaskCreatePinnedToCore(canRecTask, "canRecTask", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(
      serialRecTask,   // 任务函数
      "serialRecTask", // 任务名称
      4096,            // 堆栈大小
      NULL,            // 参数
      5,               // 优先级
      NULL,            // 句柄
      1                // 运行在核心 1
  );
}

// 如果不想用 ESP-IDF 专用 API，可以用一个简单的循环数组
static uint8_t rxRingBuffer[SERIAL_RING_BUFFER_SIZE];
static volatile size_t rxHead = 0;
static volatile size_t rxTail = 0;
static size_t rxCount = 0;

// 向环形缓冲区写入一个字节（生产者：串口接收）
void ringBufferWrite(uint8_t data)
{
  size_t nextHead = (rxHead + 1) % SERIAL_RING_BUFFER_SIZE;
  if (nextHead != rxTail)
  { // 未满
    rxRingBuffer[rxHead] = data;
    rxHead = nextHead;
  }
  // 如果满了，丢弃新数据（避免覆盖未处理数据）
}

// 从环形缓冲区读取一个字节（消费者：解析任务），返回读取成功与否
bool ringBufferRead(uint8_t *data)
{
  if (rxHead == rxTail)
    return false;
  *data = rxRingBuffer[rxTail];
  rxTail = (rxTail + 1) % SERIAL_RING_BUFFER_SIZE;
  return true;
}

// 接收任务
void serialRecTask(void *pvParameters) {
    uint8_t byte;
    const int MAX_PACKETS_PER_LOOP = 5;  // 单次循环最多处理 5 个完整包，防止占 CPU 过久

    while (true) {
        // 1. 将串口硬件 FIFO 中的数据快速转移到环形缓冲区
        while (Serial.available() > 0) {
            byte = Serial.read();
            ringBufferWrite(byte);
        }

        // 2. 解析环形缓冲区中的数据，但限制处理量
        static enum { WAIT_HEADER, READ_PACKET } state = WAIT_HEADER;
        static uint8_t packetBuffer[SERIAL_PACKET_SIZE];
        static size_t packetIndex = 0;

        while (ringBufferRead(&byte)) {
            switch (state) {
                case WAIT_HEADER:
                    if (byte == 0xA5) {
                        packetBuffer[0] = byte;
                        packetIndex = 1;
                        state = READ_PACKET;
                    }
                    break;
                case READ_PACKET:
                    packetBuffer[packetIndex++] = byte;
                    if (packetIndex == SERIAL_PACKET_SIZE) {
                        SerialCommandPackage *pkt = (SerialCommandPackage*)packetBuffer;
                        if (Verify_CRC16_Check_Sum((uint8_t*)pkt, SERIAL_PACKET_SIZE)) {
                            xQueueSend(xCommandQueue, pkt, 0);
                        }
                        state = WAIT_HEADER;
                    }
                    break;
            }
        }

        // 3. 主动让出 CPU，避免看门狗复位（改用 vTaskDelay(1) 而非 0）
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void canRecTask(void *pvParameters)
{
  while (true)
  {
    uint8_t motorID = recCANMessage();
    if (motorID != 0xFF)
    {
      uint8_t index = motorID - 1;
      motorStatePackage motorPacket;
      motorPacket.motorID = motorID;
      motorPacket.motorPos = devicesState[index].pos;
      motorPacket.motorVel = devicesState[index].vel;
      motorPacket.motorTor = devicesState[index].tor;

      Append_CRC16_Check_Sum((uint8_t *)&motorPacket, sizeof(motorStatePackage)); // 计算 CRC
      // 获取互斥锁
      if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
      {
        Serial.write((uint8_t *)&motorPacket, sizeof(motorStatePackage)); // 发送数据
        // 释放互斥锁
        xSemaphoreGive(xSerialMutex);
      }
    }
  }
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(1); // 将 1ms 转换为 Tick 数 (即 1 个 Tick)

  // 初始化“上次唤醒时间”为当前时间
  xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    mpu6050.update(false);

    imuStatePackage imuPacket;

    imuPacket.ax = mpu6050.getAccX();
    imuPacket.ay = mpu6050.getAccY();
    imuPacket.az = mpu6050.getAccZ();
    imuPacket.gx = mpu6050.getGyroX();
    imuPacket.gy = mpu6050.getGyroY();
    imuPacket.gz = mpu6050.getGyroZ();
    Append_CRC16_Check_Sum((uint8_t *)&imuPacket, sizeof(imuStatePackage)); // 计算 CRC

    // --- 串口写入保护 ---
    // 获取互斥锁 (如果另一个任务正在使用串口，此任务会在此处阻塞)
    if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    {
      // 只有拿到锁的任务才能执行 Serial.write
      Serial.write((uint8_t *)&imuPacket, sizeof(imuStatePackage)); // 发送数据
      // 发送完毕后，立刻释放互斥锁
      xSemaphoreGive(xSerialMutex);
    }
    // --- 串口写入保护结束 ---

    // static auto lastTime = micros();
    // auto currentTime = micros();
    // auto dt = (currentTime - lastTime) * 1.0e-6f;
    // lastTime = currentTime;
    // Serial.printf("currentTime:%d\tdt:%.3fms\tfreq:%.3f\n", currentTime, dt * 1000.f, 1.0f / dt);

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
