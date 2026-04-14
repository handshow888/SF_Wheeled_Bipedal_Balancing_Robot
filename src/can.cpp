#include "CAN/can.h"

// 左腿关节电机MIT控制
MIT LeftFronMITCtrlParam;
MIT LeftRearMITCtrlParam;

// 右腿关节电机MIT控制
MIT RightFronMITCtrlParam;
MIT RightRearMITCtrlParam;
uint32_t prev_ts;
uint16_t printCount = 0;
unsigned long lastSendTime = 0; // 记录上次发送时间
float Am_kp = 0.0;
float motorRightRear;
float motorLeftRear;
float motorRightFront;
float motorLeftFront;

// 定义腿部标识和对应的控制参数
struct LegCommand
{
  uint8_t id;         // CAN ID
  MIT &control_param; // 对应的控制参数
};

void CAN_Control()
{
  unsigned long currentTime = millis(); // 获取当前时间

  // recCANMessage(); // CAN接收函数

  // 左腿关节电机4 控制参数
  LeftFronMITCtrlParam.pos = motorLeftFront;
  LeftFronMITCtrlParam.vel = 0;
  LeftFronMITCtrlParam.kp = Am_kp;
  LeftFronMITCtrlParam.kd = 0;
  LeftFronMITCtrlParam.tor = targetTorLeftFront;
  // 左腿关节电机2 控制参数
  LeftRearMITCtrlParam.pos = motorLeftRear;
  LeftRearMITCtrlParam.vel = 0;
  LeftRearMITCtrlParam.kp = Am_kp;
  LeftRearMITCtrlParam.kd = 0;
  LeftRearMITCtrlParam.tor = targetTorLeftRear;
  // 右腿关节电机3 控制参数
  RightFronMITCtrlParam.pos = motorRightFront;
  RightFronMITCtrlParam.vel = 0;
  RightFronMITCtrlParam.kp = Am_kp;
  RightFronMITCtrlParam.kd = 0;
  RightFronMITCtrlParam.tor = targetTorRightFront;
  // 右腿关节电机1 控制参数
  RightRearMITCtrlParam.pos = motorRightRear;
  RightRearMITCtrlParam.vel = 0;
  RightRearMITCtrlParam.kp = Am_kp;
  RightRearMITCtrlParam.kd = 0;
  RightRearMITCtrlParam.tor = targetTorRightRear;

  // 打印关节电机电角度 1 2 3 4
  // Serial.printf("%.2f,%.2f,%.2f,%.2f\n", devicesState[0].pos, devicesState[1].pos, devicesState[2].pos, devicesState[3].pos);

  if (currentTime - lastSendTime >= SEND_INTERVAL)
  {
    // 发送命令
    sendMITCommand(0x01, RightRearMITCtrlParam);
    sendMITCommand(0x02, LeftRearMITCtrlParam);
    sendMITCommand(0x03, RightFronMITCtrlParam);
    sendMITCommand(0x04, LeftFronMITCtrlParam);
    lastSendTime = currentTime; // 更新最后发送时间
  }
}

/**
 * @brief 将运动学逆解的结果映射到关节电机角度pos
 */
void mapJointMotorAngle()
{
  motorRightRear = (1.80 + 1.57 * 8) - (rightLegKinematics.alpha * 8); // 1
  motorRightFront = (1.66 + 1.57 * 8) - (rightLegKinematics.beta * 8); // 3

  motorLeftRear = (2.29 - 1.57 * 8) + (leftLegKinematics.alpha * 8); // 2
  motorLeftFront = (5.65 - 1.57 * 8) + (leftLegKinematics.beta * 8); // 4
  // Serial.printf("Target:%.2f,%.2f,%.2f,%.2f\talphaL:%.2f,betaL:%.2f,alphaR:%.2f,betaR:%.2f\n",
  //               motorRightRear,
  //               motorLeftRear,
  //               motorRightFront,
  //               motorLeftFront,
  //               leftLegKinematics.alpha,
  //               leftLegKinematics.beta,
  //               rightLegKinematics.alpha / PI * 180.0f,
  //               rightLegKinematics.beta / PI * 180.0f);
}

// 内部启动指定电机
void startMotor(int motorIndex)
{
  enableMotor((uint8_t)motorIndex);
  Serial.printf("Sent enable cmd to Motor %d\n", motorIndex);
}

void enableJointMotors()
{
  for (int i = 1; i <= 4; ++i)
  {
    Serial.println("...");
    startMotor(i); // 启动当前索引的电机
    delay(100);
  }
}