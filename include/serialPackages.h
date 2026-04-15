#ifndef SERIAL_PACKAGES_H
#define SERIAL_PACKAGES_H
#include <cstdint>

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
    uint8_t motorID;
    float motorPos; // rad [-4pi, 4pi]
    float motorVel; // rad/s [-45.0, 45.0]
    float motorTor; // Nm
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) motorStatePackage;

typedef struct
{
    uint8_t header = 0x5C;
    float motors_effort[6] = {0.0};
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) sendTestPackage;

typedef struct
{
    uint8_t header = 0xA5;
    float motors_effort[6] = {0.0}; // 按照电机ID:RR LR RF LF LW RW
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) SerialCommandPackage;

#define SERIAL_PACKET_SIZE sizeof(SerialCommandPackage)
#define SERIAL_RING_BUFFER_SIZE (SERIAL_PACKET_SIZE * 10) // 环形缓冲区大小，可容纳10个包

#endif