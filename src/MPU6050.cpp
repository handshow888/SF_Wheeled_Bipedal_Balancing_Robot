#include "MPU6050.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w)
{
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC)
{
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin()
{
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // 将采样率分频器（SMPLRT_DIV）寄存器设为0x00，即采样率不分频，传感器输出的采样率为陀螺仪的输出速率（一般为8kHz）
  writeMPU6050(MPU6050_CONFIG, 0x00);       // 将配置（CONFIG）寄存器设为0x00，即关闭低通滤波器，选择内部8kHz时钟
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // 将陀螺仪配置（GYRO_CONFIG）寄存器设为0x08，即将陀螺仪量程设置为±500度/秒
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // 将加速度计配置（ACCEL_CONFIG）寄存器设为0x00，即将加速度计量程设置为±2g。
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // 将电源管理1（PWR_MGMT_1）寄存器设为0x01，即将传感器置于工作模式，并选择内部时钟源。
  this->update(false);
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data)
{
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg)
{
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data = wire->read();
  return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z)
{
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

/**
 * @brief 静止时获取陀螺仪偏置校准量
 */
void MPU6050::calcGyroOffsets(uint16_t delayBefore, uint16_t delayAfter)
{
  float x = 0, y = 0, z = 0;
  int16_t rx, ry, rz;

  delay(delayBefore);
  Serial.println();
  Serial.println("========================================");
  Serial.println("Calculating gyro offsets");
  Serial.print("DO NOT MOVE MPU6050");
  for (int i = 0; i < 3000; i++)
  {
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x43);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  Serial.println();
  Serial.println("Done!");
  Serial.print("X : ");
  Serial.println(gyroXoffset);
  Serial.print("Y : ");
  Serial.println(gyroYoffset);
  Serial.print("Z : ");
  Serial.println(gyroZoffset);
  Serial.println("Program will start after 3 seconds");
  Serial.print("========================================");
  delay(delayAfter);
}

/**
 * @brief 加速度计安装误差校准（小车需要水平静止放置）
 */
void MPU6050::calibrateAccelerometer(uint16_t delayBefore, uint16_t delayAfter)
{
  // 收集多个样本以减少噪声
  constexpr int num_samples = 1000;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  delay(delayBefore);
  Serial.println("开始加速度计校准，请确保小车水平静止...");

  for (int i = 0; i < num_samples; i++)
  {
    update(false);
    sum_ax += accX;
    sum_ay += accY;
    sum_az += accZ;
  }

  // 计算平均加速度
  float ax_mean = sum_ax / num_samples;
  float ay_mean = sum_ay / num_samples;
  float az_mean = sum_az / num_samples;

  Serial.print("原始平均加速度: ");
  Serial.print(ax_mean);
  Serial.print(", ");
  Serial.print(ay_mean);
  Serial.print(", ");
  Serial.println(az_mean);

  // 计算当前重力向量
  float gravity_norm = sqrt(ax_mean * ax_mean + ay_mean * ay_mean + az_mean * az_mean);

  // 归一化当前重力向量
  float gx_current = ax_mean / gravity_norm;
  float gy_current = ay_mean / gravity_norm;
  float gz_current = az_mean / gravity_norm;

  // 目标重力向量（垂直向下）
  float gx_target = 0.0;
  float gy_target = 0.0;
  float gz_target = 1.0;

  // 计算旋转轴（叉积）
  float cross_x = gy_current * gz_target - gz_current * gy_target;
  float cross_y = gz_current * gx_target - gx_current * gz_target;
  float cross_z = gx_current * gy_target - gy_current * gx_target;

  float sin_angle = sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
  float cos_angle = gx_current * gx_target + gy_current * gy_target + gz_current * gz_target;

  // 旋转角度
  float angle = atan2(sin_angle, cos_angle);

  // 构建旋转矩阵（Rodrigues旋转公式）
  if (sin_angle > 1e-6)
  {
    float ux = cross_x / sin_angle;
    float uy = cross_y / sin_angle;
    float uz = cross_z / sin_angle;

    float cos_a = cos(angle);
    float sin_a = sin(angle);
    float one_minus_cos = 1.0 - cos_a;

    // 旋转矩阵
    calibration_matrix[0][0] = cos_a + ux * ux * one_minus_cos;
    calibration_matrix[0][1] = ux * uy * one_minus_cos - uz * sin_a;
    calibration_matrix[0][2] = ux * uz * one_minus_cos + uy * sin_a;

    calibration_matrix[1][0] = uy * ux * one_minus_cos + uz * sin_a;
    calibration_matrix[1][1] = cos_a + uy * uy * one_minus_cos;
    calibration_matrix[1][2] = uy * uz * one_minus_cos - ux * sin_a;

    calibration_matrix[2][0] = uz * ux * one_minus_cos - uy * sin_a;
    calibration_matrix[2][1] = uz * uy * one_minus_cos + ux * sin_a;
    calibration_matrix[2][2] = cos_a + uz * uz * one_minus_cos;
  }

  Serial.println("加速度计校准完成！");
  Serial.println("校准矩阵:");
  for (int i = 0; i < 3; i++)
  {
    Serial.print("[");
    for (int j = 0; j < 3; j++)
    {
      Serial.print(calibration_matrix[i][j], 6);
      Serial.print(" ");
    }
    Serial.println("]");
  }

  delay(delayAfter);
}

void MPU6050::update(bool calibrated)
{
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read();
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  temp = (rawTemp + 12412.0) / 340.0;
  // 计算加速度 16384.0是因为加速度计的量程是±2g，对应的分辨率是16位。
  accX = -((float)rawAccX) / 16384.0;
  accY = -((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;
  gyroX = -((float)rawGyroX) / 65.5;
  gyroY = -((float)rawGyroY) / 65.5;
  gyroZ = ((float)rawGyroZ) / 65.5;

  if (!calibrated) // 如不需要校准，直接返回
    return;

  applyCalibration(); // 校准imu方向

  // 计算加速度角度 使用加速度计数据计算两个轴的倾角
  // angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
  // angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;
  // angleAccX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
  // angleAccY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

  // interval = (millis() - preInterval) * 0.001;

  // angleGyroX += gyroX * interval;
  // angleGyroY += gyroY * interval;
  // angleGyroZ += gyroZ * interval;
  // // 使用加权平均（融合算法）将陀螺仪和加速度计的角度值融合在一起，以获得更精确和稳定的角度值。其中gyroCoef和accCoef是两个权重系数，表示对陀螺仪和加速度计数据的信任程度。
  // angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  // angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  // angleZ = angleGyroZ;

  // preInterval = millis();
}

/**
 * @brief 校准imu方向
 */
void MPU6050::applyCalibration()
{
  // 读取原始数据
  // update();

  // 应用加速度计校准
  float acc_x_temp = accX;
  float acc_y_temp = accY;
  float acc_z_temp = accZ;

  accX = calibration_matrix[0][0] * acc_x_temp +
         calibration_matrix[0][1] * acc_y_temp +
         calibration_matrix[0][2] * acc_z_temp;

  accY = calibration_matrix[1][0] * acc_x_temp +
         calibration_matrix[1][1] * acc_y_temp +
         calibration_matrix[1][2] * acc_z_temp;

  accZ = calibration_matrix[2][0] * acc_x_temp +
         calibration_matrix[2][1] * acc_y_temp +
         calibration_matrix[2][2] * acc_z_temp;

  // 应用陀螺仪校准
  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  // 陀螺仪也需要应用相同的旋转矩阵
  float gyro_x_temp = gyroX;
  float gyro_y_temp = gyroY;
  float gyro_z_temp = gyroZ;

  gyroX = calibration_matrix[0][0] * gyro_x_temp +
          calibration_matrix[0][1] * gyro_y_temp +
          calibration_matrix[0][2] * gyro_z_temp;

  gyroY = calibration_matrix[1][0] * gyro_x_temp +
          calibration_matrix[1][1] * gyro_y_temp +
          calibration_matrix[1][2] * gyro_z_temp;

  gyroZ = calibration_matrix[2][0] * gyro_x_temp +
          calibration_matrix[2][1] * gyro_y_temp +
          calibration_matrix[2][2] * gyro_z_temp;
}
