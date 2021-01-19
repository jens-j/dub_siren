#include <stdint.h>

#ifndef __FIXEDXPOINT_H
#define __FIXEDXPOINT_H

#define QS12_ONE        0x00000fffUL
#define QS15_ONE        0x00007fffUL
#define QS15_MINUS_ONE  0xffff8000UL
#define QU16_ONE        0x0000ffffUL
#define QU32_ONE        0xffffffffUL

typedef uint32_t qu8_t;  // unsigned [0 - 256) with 8 bit fractional precision
typedef int32_t qs12_t;  // signed fixed point [-8, -8)
typedef int32_t qs15_t;  // signed fixed point [-1 - 1)
typedef uint32_t qu16_t; // unsigned fixed point [0 - 1)
typedef uint32_t qu32_t; // unsigned fixed point [0 - 1)

inline qu16_t qu32_to_qu16 (qu32_t x) {return (qu16_t) (x >> 16);}
inline qs15_t qu32_to_qs15 (qu32_t x) {return (qs15_t) (x >> 17);}
inline qs15_t qu16_to_qs15 (qu16_t x) {return (qs15_t) (x >> 1);}
inline qu8_t qu16_to_qu8 (qu16_t x) {return (qu8_t) (x >> 8);}
inline qs12_t qs15_to_qs12 (qs15_t x) {return (qs12_t) (x >> 3);}
inline qu16_t uint16_to_qu16 (uint16_t x) {return (qu16_t) (x << 16);}
inline qs15_t uint16_to_qs15 (uint16_t x) {return (qs15_t) (x << 15);}
inline qu8_t uint16_to_qu8 (uint16_t x) {return (qu8_t) (x << 8);}
inline uint16_t qu16_to_uint16 (qu16_t x) {return (uint16_t) (x >> 16);}
inline uint32_t qu8_to_uint32 (qu8_t x) {return (uint32_t) (x >> 8);}
inline qu8_t float_to_qu8 (float x) {return (qu8_t) (x * 0x100);}
inline qs12_t float_to_qs12 (float x) {return (qs12_t) (x * 0x1000);}
inline qs15_t float_to_qs15 (float x) {return (qs15_t) (x * 0x8000);}
inline qu16_t float_to_qu16 (float x) {return (qu16_t) (x * 0x10000);}
inline qu32_t float_to_qu32 (float x) {return (qu32_t) (x * 0x100000000);}
inline qs15_t qs_invert (qs15_t x) {return (~x) + 1;}


// multiply two qu16_t values
inline qu16_t mul_qu16 (qu16_t x, qu16_t y) {

    return x * y >> 16;
}

// multiply two qu8_t values
inline qu8_t mul_qu8 (qu8_t x, qu8_t y) {

    return x * y >> 8;
}

// multiply two qs15_t values
inline qs15_t mul_qs15 (qs15_t x, qs15_t y) {

    return x * y >> 15;
}

// scale an int by a signed coefficient
inline int16_t mul_qs12_int16 (qs12_t x, int16_t y) {

    return (int16_t) (x * (int32_t) y >> 12);
}

// scale an int by a signed coefficient
inline int16_t mul_qs15_int16 (qs15_t x, int16_t y) {

    return (int16_t) (x * (int32_t) y >> 15);
}

// scale an integer by a unsigned coefficient
inline uint16_t mul_qu16_uint16 (qu16_t x, uint16_t y) {

    return (uint16_t) (x * (uint32_t) y >> 16);
}

// scale an integer by a unsigned coefficient
inline uint32_t mul_qu8_uint32 (qu8_t x, uint32_t y) {

    return (x * y) >> 8;
}

// scale an integer by a qu32 fixed point value
inline uint16_t mul_qu32_uint16 (qu32_t x, uint16_t y) {

    return mul_qu16_uint16(qu32_to_qu16(x), y);
}

// divide two signed values. The divident can be shifted before division to preserve precision
inline qs12_t div_qs12 (qs12_t x, qs12_t y) {

    return (x << 12) / y;
}

#endif
