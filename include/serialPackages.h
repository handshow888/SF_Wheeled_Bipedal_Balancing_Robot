#ifndef SERIAL_PACKAGES_H
#define SERIAL_PACKAGES_H
#include <cstdint>

/***************************************************** send *****************************************************/
typedef struct
{
    uint8_t header = 0x5A;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) imuStatePackage;

typedef struct
{
    uint8_t header = 0x5B;
    uint8_t motorID; // 左轮5 右轮6
    // float motorPos; // rad [-4pi, 4pi]
    float motorVel; // rad/s [-45.0, 45.0]
    // float motorTor; // Nm
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) motorStatePackage;

typedef struct
{
    uint8_t header = 0x5C;
    uint8_t jointMotorState = 0; // 0失能 1使能
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) motorSwitchPackage;

/***************************************************** recived *****************************************************/

typedef struct
{
    uint8_t header = 0xA5;
    float motors_effort[2] = {0.0}; // LW RW
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) SerialCommandPackage;

#define SERIAL_PACKET_SIZE sizeof(SerialCommandPackage)
#define SERIAL_RING_BUFFER_SIZE (SERIAL_PACKET_SIZE * 3) // 环形缓冲区大小，可容纳3个包

#endif