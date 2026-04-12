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
void Open_thread_function(); // 启动线程

SemaphoreHandle_t xSerialMutex; // 创建互斥锁句柄

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
  // interpolatePID();       // 根据腿高插值拟合pid参数

  // legEndCalculate();                                                                             // 计算足端目标位置
  // inverseKinematics(leftLegKinematics, rightLegKinematics, leftLegEndTarget, rightLegEndTarget); // 运动学逆解
  // mapJointMotorAngle();                                                                          // 将运动学逆解的结果映射到关节电机角度pos
  // CAN_Control();                                                                                 // 发送关节电机控制指令

  // wheelControlPID();                                                                           // pid计算轮毂电机扭矩
  // wheelControlLQR(); // lqr计算轮毂电机扭矩
  // sendMotorTargets(enableHubMotor * rightWheelTorTarget, enableHubMotor * leftWheelTorTarget); // 发送控制轮毂电机的目标值 右, 左
  // sendMotorTargets(enableHubMotor * remoteLinearVel, enableHubMotor * remoteLinearVel); // 发送控制轮毂电机的目标值 右, 左
  // Serial.printf("leftVel:%.2f\trightVel:%.2f\n", motor2_vel, motor1_vel);

  // static auto lastTime = micros();
  // auto currentTime = micros();
  // auto dt = (currentTime - lastTime) * 1.0e-6f;
  // lastTime = currentTime;
  // Serial.printf("currentTime:%d\tdt:%.6f\tfreq:%.3f\n", currentTime, dt, 1.0f / dt);
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
      1,         // 任务优先级
      NULL,      // 任务句柄
      1          // 运行在核心 1
  );
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

    imuPackage imuPacket;

    imuPacket.ax = mpu6050.getAccX();
    imuPacket.ay = mpu6050.getAccY();
    imuPacket.az = mpu6050.getAccZ();
    imuPacket.gx = mpu6050.getGyroX();
    imuPacket.gy = mpu6050.getGyroY();
    imuPacket.gz = mpu6050.getGyroZ();
    Append_CRC16_Check_Sum((uint8_t *)&imuPacket, sizeof(imuPackage)); // 计算 CRC

    // --- 串口写入保护 ---
    // 获取互斥锁 (如果另一个任务正在使用串口，此任务会在此处阻塞)
    if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    {
      // 只有拿到锁的任务才能执行 Serial.write
      Serial.write((uint8_t *)&imuPacket, sizeof(imuPackage)); // 发送数据
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
