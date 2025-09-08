#include "crc.h"

#define POLY   0x42F0E1EBA9EA3693ULL
#define INIT   0xFFFFFFFFFFFFFFFFULL
#define XOROUT 0xFFFFFFFFFFFFFFFFULL

// Reflect bits in a byte
static uint8_t reflect8(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// Reflect bits in 64-bit value
static uint64_t reflect64(uint64_t x)
{
    uint64_t r = 0;
    for (int i = 0; i < 64; i++) {
        if (x & (1ULL << i)) {
            r |= (1ULL << (63 - i));
        }
    }
    return r;
}

uint64_t calculate_crc(const uint8_t *data, size_t length)
{
    uint64_t crc = INIT;

    for (size_t i = 0; i < length; i++) {
        uint8_t byte = reflect8(data[i]);  // RefIn = True
        crc ^= (uint64_t)byte << 56;       // Align byte with top of CRC register

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000000000000000ULL) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    crc = reflect64(crc);  // RefOut = True
    return crc ^ XOROUT;   // Final XOR
}

