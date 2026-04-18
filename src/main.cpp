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
#include "tasks/data_bridge_tasks.h"
/**********************************************************************************************************/
MPU6050 mpu6050 = MPU6050(Wire); // 实例化MPU6050
SemaphoreHandle_t xSerialMutex;  // 串口发送互斥锁
SemaphoreHandle_t xCommandMutex;  // 命令更新互斥锁
SerialCommandPackage latestCommand; // 最新接收到的命令
unsigned long lastRecCmdTime = 0; // ms

// 控制loop函数周期
TickType_t xLastWakeTime_loop;
const TickType_t xPeriod_loop = pdMS_TO_TICKS(1);

float targetTorRightRear;
float targetTorLeftRear;
float targetTorRightFront;
float targetTorLeftFront;
float targetTorLeftWheel;
float targetTorRightWheel;

/**********************************************************************************************************/
void Open_thread_function(); // 启动线程
void sendWheelsState();      // 发送轮毂电机状态到串口
void handleRecCmd();         // 处理接收到的上位机指令
bool isCmdRec();             // 检测上位机是否在线
void emergencyStopCheck();   // 检查遥控器无力开关

/**********************************************************************************************************/
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
  xCommandMutex = xSemaphoreCreateMutex();

  Open_thread_function(); // 启动多线程任务

  xLastWakeTime_loop = xTaskGetTickCount();

  // delay(3000);
  /* USER CALIBRATE IMU START */
  // 静止平放时解注释获取imu校准矩阵，正常运行时注释
  // delay(2);
  // mpu6050.calcGyroOffsets(true);
  // mpu6050.calibrateAccelerometer();
  /* USER CALIBRATE IMU END */
}

/**********************************************************************************************************/
void loop()
{
  storeFilteredPPMData(); // 获取遥控器数据
  remoteSwitch();         // 获取遥控器模式
  // mapPPMToRobotControl(); // 映射遥控器各通道数值为控制指令

  sendWheelsState(); // 发送轮毂电机状态到串口

  handleRecCmd(); // 非阻塞处理接收到的命令
  isCmdRec();     // 检测上位机是否在线
  emergencyStopCheck();
  CAN_Control();                                             // 控制关节电机
  sendMotorTargets(targetTorRightWheel, targetTorLeftWheel); // 发送控制轮毂电机的目标值
  // Serial.printf("v1:%.3f\tv2:%.3f\n", motor1_vel, motor2_vel);
  vTaskDelayUntil(&xLastWakeTime_loop, xPeriod_loop); // 周期控制
}

/**********************************************************************************************************/
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

  xTaskCreatePinnedToCore(
      canRecTask,
      "canRecTask",
      4096,
      NULL,
      5,
      NULL,
      1);

  xTaskCreatePinnedToCore(
      serialRecTask,   // 任务函数
      "serialRecTask", // 任务名称
      4096,            // 堆栈大小
      NULL,            // 参数
      5,               // 优先级
      NULL,            // 句柄
      0                // 运行在核心 0
  );
}

void sendWheelsState()
{
  motorStatePackage packet;
  // 左轮
  packet.motorID = 5;
  packet.motorVel = -motor2_vel;
  Append_CRC16_Check_Sum((uint8_t *)&packet, sizeof(motorStatePackage));
  if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
  {
    Serial.write((uint8_t *)&packet, sizeof(motorStatePackage));
    xSemaphoreGive(xSerialMutex);
  }
  // 右轮
  packet.motorID = 6;
  packet.motorVel = -motor1_vel;
  Append_CRC16_Check_Sum((uint8_t *)&packet, sizeof(motorStatePackage));
  if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
  {
    Serial.write((uint8_t *)&packet, sizeof(motorStatePackage));
    xSemaphoreGive(xSerialMutex);
  }
}

void handleRecCmd()
{
  // if (lastRecCmdTime == 0)
  //   return;
  if (xSemaphoreTake(xCommandMutex, 0) == pdTRUE)
  {
    SerialCommandPackage cmd = latestCommand;
    xSemaphoreGive(xCommandMutex);

    switch (enableHubMotor)
    {
    case 0:
      targetTorRightRear = 0.0;
      targetTorLeftRear = 0.0;
      targetTorRightFront = 0.0;
      targetTorLeftFront = 0.0;
      targetTorLeftWheel = 0.0;
      targetTorRightWheel = 0.0;
      break;
    case 1:
      targetTorRightRear = cmd.motors_effort[0];
      targetTorLeftRear = cmd.motors_effort[1];
      targetTorRightFront = cmd.motors_effort[2];
      targetTorLeftFront = cmd.motors_effort[3];
      targetTorLeftWheel = cmd.motors_effort[4];
      targetTorRightWheel = cmd.motors_effort[5];
      break;
    default:
      break;
    }
    /* 以下是测试代码 */
    // sendTestPackage packet;
    // memcpy(packet.motors_effort, cmd.motors_effort, 6 * sizeof(float));

    // Append_CRC16_Check_Sum((uint8_t *)&packet, sizeof(sendTestPackage)); // 计算 CRC
    // if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    // {
    //   Serial.write((uint8_t *)&packet, sizeof(sendTestPackage)); // 发送数据
    //   xSemaphoreGive(xSerialMutex);
    // }
  }
}

bool isCmdRec()
{
  auto currentTime = millis();
  int dt = currentTime - lastRecCmdTime; // ms
  if (dt > 1000)                         // 超过1s没收到指令则认为上位机离线
  {
    targetTorRightRear = 0.0;
    targetTorLeftRear = 0.0;
    targetTorRightFront = 0.0;
    targetTorLeftFront = 0.0;
    targetTorLeftWheel = 0.0;
    targetTorRightWheel = 0.0;
    return false;
  }
  return true;
}

void emergencyStopCheck()
{
  if (!enableHubMotor)
  {
    targetTorRightRear = 0.0;
    targetTorLeftRear = 0.0;
    targetTorRightFront = 0.0;
    targetTorLeftFront = 0.0;
    targetTorLeftWheel = 0.0;
    targetTorRightWheel = 0.0;
  }
}