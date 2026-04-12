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
} __attribute__((packed)) imuPackage;