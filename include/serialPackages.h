#pragma once
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
    float motorTor;   // Nm
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed)) motorStatePackage;