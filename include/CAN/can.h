#ifndef CAN_H
#define CAN_H

#include "Arduino.h"
#include "CAN_comm.h"
#include "robot.h"

#define SEND_INTERVAL 1 // 限制发送频率，单位为毫秒
extern float Am_kp;
extern float motorRightRear;
extern float motorLeftRear;
extern float motorRightFront;
extern float motorLeftFront;

extern float targetTorRightRear;
extern float targetTorLeftRear;
extern float targetTorRightFront;
extern float targetTorLeftFront;

void CAN_Control();
void startMotor(int motorIndex);
void enableJointMotors();
void mapJointMotorAngle();

#endif