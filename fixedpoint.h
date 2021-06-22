#include <stdint.h>

#ifndef __FIXEDXPOINT_H
#define __FIXEDXPOINT_H

#define QS12_ONE            0x00000fffUL
#define QS15_ONE            0x00007fffUL
#define QS15_MINUS_ONE      0xffff8000UL
#define QS15_MINUS_HALF     0xffffc000UL
#define QS15_MINUS_QUARTER  0xffffe000UL
#define QU16_ONE            0x0000ffffUL
#define QU32_ONE            0xffffffffUL

typedef uint32_t qu8_t;  // unsigned [0 - 256) with 8 bit fractional precision
typedef int32_t qs12_t;  // signed fixed point [-8, -8)
typedef int32_t qs15_t;  // signed fixed point [-1 - 1)
typedef uint32_t qu16_t; // unsigned fixed point [0 - 1)
typedef uint32_t qu32_t; // unsigned fixed point [0 - 1)

inline qu16_t qu32_to_qu16 (qu32_t x) {return (qu16_t) (x >> 16);}
inline qs15_t qu32_to_qs15 (qu32_t x) {return (qs15_t) (x >> 17);}

inline qs15_t qu16_to_qs15 (qu16_t x) {return (qs15_t) (x >> 1);}
inline qu8_t qu16_to_qu8 (qu16_t x) {return (qu8_t) (x >> 8);}
inline uint16_t qu16_to_uint16 (qu16_t x) {return (uint16_t) (x >> 16);}

inline qu16_t qs15_to_qu16 (qs15_t x) {return (qu16_t) (x << 1);}
inline qs12_t qs15_to_qs12 (qs15_t x) {return (qs12_t) (x >> 3);}

inline qs12_t qu8_to_qs12 (qu8_t x) {return (qs12_t) (x << 4);}
inline uint32_t qu8_to_uint32 (qu8_t x) {return (uint32_t) (x >> 8);}
inline uint16_t qu8_to_uint16 (qu8_t x) {return (uint16_t) (x >> 8);}

inline qu16_t uint16_to_qu16 (uint16_t x) {return (qu16_t) (x << 16);}
inline qs15_t uint16_to_qs15 (uint16_t x) {return (qs15_t) (x << 15);}
inline qu8_t uint16_to_qu8 (uint16_t x) {return (qu8_t) (x << 8);}

inline float qu8_to_float (qu8_t x) {return ((float) x) / 256.0;}
inline float qs12_to_float (qs12_t x) {return ((float) x) / 4096.0;}
inline float qs15_to_float (qs15_t x) {return ((float) x) / 32768.0;}
inline float qu16_to_float (qu16_t x) {return ((float) x) / 65536.0;}
inline float qu32_to_float (qu16_t x) {return ((float) x) / 4294967296.0;}

inline qu8_t float_to_qu8 (float x) {return (qu8_t) (x * 0x100);}
inline qs12_t float_to_qs12 (float x) {return (qs12_t) (x * 0x1000);}
inline qs15_t float_to_qs15 (float x) {return (qs15_t) (x * 0x8000);}
inline qu16_t float_to_qu16 (float x) {return (qu16_t) (x * 0x10000);}
inline qu32_t float_to_qu32 (float x) {return (qu32_t) (x * 0x100000000);}

inline qs15_t qs_invert (qs15_t x) {return (~x) + 1;}


inline uint16_t add_uint16_clip(uint16_t x, uint16_t y) {

    uint16_t z = x + y;

    if (x & y & ~z & 0x8000) {
        return 0x8000;
    } else if (~x & ~y & z & 0x8000) {
        return 0x7FFF;
    } else {
        return z;
    }
}

// multiply two qu16_t values
inline qu16_t mul_qu16 (qu16_t x, qu16_t y) {

    // qu16_t z = x * y >> 16;
    //
    // if (z & 0x8000) {
    //     z |= 0xFFFF0000;
    // }
    //
    // return z;

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

inline qs15_t mul_qs12_qs15 (qs12_t x, qs15_t y) {

    return x * y >> 12;
}

inline qs12_t mul_qs15_qs12 (qs15_t x, qs12_t y) {

    return x * y >> 15;
}


// scale an int by a signed coefficient
inline int16_t mul_qs12_int16 (qs12_t x, int16_t y) {

    return (int16_t) (x * (int32_t) y >> 12);
}

// scale an int by a signed coefficient
inline int32_t mul_qs12_int32 (qs12_t x, int32_t y) {

    return x * y >> 12;
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

inline qs15_t rshift1_qs15 (qs15_t x) {

    if (x & 0x8000) {
        return 0x8000 | (x >> 1);
    } else {
        return x >> 1;
    }
}

inline qs15_t rshift2_qs15 (qs15_t x) {

    if (x & 0x8000) {
        return 0xC000 | (x >> 2);
    } else {
        return x >> 2;
    }
}

inline int16_t clip_uint32_uint16 (int32_t x) {

    if (x & 0xFFFF0000) { // detect overflow
        if (x & 0x80000000) {
            return -32768;}
        else {
            return 32767;}
    } else {
        return (int16_t) x;
    }
}

#endif
