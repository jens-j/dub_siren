#include "dubsiren.h"
#include "fixedpoint.h"

#ifndef __SINE_H
#define __SINE_H

#define SINE_SAMPLES 1024
#define CAPACITOR_SAMPLES 2048

const qs15_t SINE_TABLE[SINE_SAMPLES] = {
    0x0000,0x0032,0x0064,0x0096,0x00c9,0x00fb,0x012d,0x015f,0x0192,0x01c4,
    0x01f6,0x0228,0x025b,0x028d,0x02bf,0x02f1,0x0324,0x0356,0x0388,0x03ba,
    0x03ed,0x041f,0x0451,0x0483,0x04b6,0x04e8,0x051a,0x054c,0x057e,0x05b1,
    0x05e3,0x0615,0x0647,0x067a,0x06ac,0x06de,0x0710,0x0742,0x0774,0x07a7,
    0x07d9,0x080b,0x083d,0x086f,0x08a1,0x08d4,0x0906,0x0938,0x096a,0x099c,
    0x09ce,0x0a00,0x0a32,0x0a65,0x0a97,0x0ac9,0x0afb,0x0b2d,0x0b5f,0x0b91,
    0x0bc3,0x0bf5,0x0c27,0x0c59,0x0c8b,0x0cbd,0x0cef,0x0d21,0x0d53,0x0d85,
    0x0db7,0x0de9,0x0e1b,0x0e4d,0x0e7f,0x0eb1,0x0ee3,0x0f15,0x0f47,0x0f79,
    0x0fab,0x0fdc,0x100e,0x1040,0x1072,0x10a4,0x10d6,0x1107,0x1139,0x116b,
    0x119d,0x11cf,0x1200,0x1232,0x1264,0x1296,0x12c7,0x12f9,0x132b,0x135d,
    0x138e,0x13c0,0x13f2,0x1423,0x1455,0x1486,0x14b8,0x14ea,0x151b,0x154d,
    0x157e,0x15b0,0x15e1,0x1613,0x1644,0x1676,0x16a7,0x16d9,0x170a,0x173c,
    0x176d,0x179f,0x17d0,0x1801,0x1833,0x1864,0x1895,0x18c7,0x18f8,0x1929,
    0x195b,0x198c,0x19bd,0x19ee,0x1a20,0x1a51,0x1a82,0x1ab3,0x1ae4,0x1b15,
    0x1b46,0x1b78,0x1ba9,0x1bda,0x1c0b,0x1c3c,0x1c6d,0x1c9e,0x1ccf,0x1d00,
    0x1d31,0x1d62,0x1d93,0x1dc3,0x1df4,0x1e25,0x1e56,0x1e87,0x1eb8,0x1ee8,
    0x1f19,0x1f4a,0x1f7b,0x1fab,0x1fdc,0x200d,0x203d,0x206e,0x209f,0x20cf,
    0x2100,0x2130,0x2161,0x2191,0x21c2,0x21f2,0x2223,0x2253,0x2284,0x22b4,
    0x22e4,0x2315,0x2345,0x2375,0x23a6,0x23d6,0x2406,0x2436,0x2467,0x2497,
    0x24c7,0x24f7,0x2527,0x2557,0x2587,0x25b7,0x25e7,0x2617,0x2647,0x2677,
    0x26a7,0x26d7,0x2707,0x2737,0x2767,0x2797,0x27c6,0x27f6,0x2826,0x2856,
    0x2885,0x28b5,0x28e5,0x2914,0x2944,0x2973,0x29a3,0x29d2,0x2a02,0x2a31,
    0x2a61,0x2a90,0x2ac0,0x2aef,0x2b1e,0x2b4e,0x2b7d,0x2bac,0x2bdb,0x2c0b,
    0x2c3a,0x2c69,0x2c98,0x2cc7,0x2cf6,0x2d25,0x2d54,0x2d83,0x2db2,0x2de1,
    0x2e10,0x2e3f,0x2e6e,0x2e9d,0x2ecc,0x2efa,0x2f29,0x2f58,0x2f86,0x2fb5,
    0x2fe4,0x3012,0x3041,0x306f,0x309e,0x30cc,0x30fb,0x3129,0x3158,0x3186,
    0x31b4,0x31e3,0x3211,0x323f,0x326d,0x329c,0x32ca,0x32f8,0x3326,0x3354,
    0x3382,0x33b0,0x33de,0x340c,0x343a,0x3468,0x3496,0x34c3,0x34f1,0x351f,
    0x354d,0x357a,0x35a8,0x35d6,0x3603,0x3631,0x365e,0x368c,0x36b9,0x36e7,
    0x3714,0x3741,0x376f,0x379c,0x37c9,0x37f6,0x3824,0x3851,0x387e,0x38ab,
    0x38d8,0x3905,0x3932,0x395f,0x398c,0x39b9,0x39e6,0x3a12,0x3a3f,0x3a6c,
    0x3a99,0x3ac5,0x3af2,0x3b1f,0x3b4b,0x3b78,0x3ba4,0x3bd1,0x3bfd,0x3c29,
    0x3c56,0x3c82,0x3cae,0x3cdb,0x3d07,0x3d33,0x3d5f,0x3d8b,0x3db7,0x3de3,
    0x3e0f,0x3e3b,0x3e67,0x3e93,0x3ebf,0x3eeb,0x3f16,0x3f42,0x3f6e,0x3f99,
    0x3fc5,0x3ff1,0x401c,0x4048,0x4073,0x409e,0x40ca,0x40f5,0x4120,0x414c,
    0x4177,0x41a2,0x41cd,0x41f8,0x4223,0x424e,0x4279,0x42a4,0x42cf,0x42fa,
    0x4325,0x4350,0x437a,0x43a5,0x43d0,0x43fa,0x4425,0x444f,0x447a,0x44a4,
    0x44cf,0x44f9,0x4523,0x454e,0x4578,0x45a2,0x45cc,0x45f6,0x4620,0x464a,
    0x4674,0x469e,0x46c8,0x46f2,0x471c,0x4746,0x476f,0x4799,0x47c3,0x47ec,
    0x4816,0x483f,0x4869,0x4892,0x48bc,0x48e5,0x490e,0x4938,0x4961,0x498a,
    0x49b3,0x49dc,0x4a05,0x4a2e,0x4a57,0x4a80,0x4aa9,0x4ad2,0x4afa,0x4b23,
    0x4b4c,0x4b74,0x4b9d,0x4bc5,0x4bee,0x4c16,0x4c3f,0x4c67,0x4c8f,0x4cb8,
    0x4ce0,0x4d08,0x4d30,0x4d58,0x4d80,0x4da8,0x4dd0,0x4df8,0x4e20,0x4e48,
    0x4e6f,0x4e97,0x4ebf,0x4ee6,0x4f0e,0x4f35,0x4f5d,0x4f84,0x4fac,0x4fd3,
    0x4ffa,0x5022,0x5049,0x5070,0x5097,0x50be,0x50e5,0x510c,0x5133,0x515a,
    0x5180,0x51a7,0x51ce,0x51f4,0x521b,0x5241,0x5268,0x528e,0x52b5,0x52db,
    0x5301,0x5328,0x534e,0x5374,0x539a,0x53c0,0x53e6,0x540c,0x5432,0x5458,
    0x547d,0x54a3,0x54c9,0x54ef,0x5514,0x553a,0x555f,0x5585,0x55aa,0x55cf,
    0x55f4,0x561a,0x563f,0x5664,0x5689,0x56ae,0x56d3,0x56f8,0x571d,0x5742,
    0x5766,0x578b,0x57b0,0x57d4,0x57f9,0x581d,0x5842,0x5866,0x588a,0x58af,
    0x58d3,0x58f7,0x591b,0x593f,0x5963,0x5987,0x59ab,0x59cf,0x59f3,0x5a16,
    0x5a3a,0x5a5e,0x5a81,0x5aa5,0x5ac8,0x5aec,0x5b0f,0x5b32,0x5b56,0x5b79,
    0x5b9c,0x5bbf,0x5be2,0x5c05,0x5c28,0x5c4b,0x5c6e,0x5c90,0x5cb3,0x5cd6,
    0x5cf8,0x5d1b,0x5d3d,0x5d60,0x5d82,0x5da4,0x5dc6,0x5de9,0x5e0b,0x5e2d,
    0x5e4f,0x5e71,0x5e93,0x5eb4,0x5ed6,0x5ef8,0x5f1a,0x5f3b,0x5f5d,0x5f7e,
    0x5fa0,0x5fc1,0x5fe2,0x6004,0x6025,0x6046,0x6067,0x6088,0x60a9,0x60ca,
    0x60eb,0x610c,0x612d,0x614d,0x616e,0x618e,0x61af,0x61cf,0x61f0,0x6210,
    0x6230,0x6251,0x6271,0x6291,0x62b1,0x62d1,0x62f1,0x6311,0x6330,0x6350,
    0x6370,0x638f,0x63af,0x63cf,0x63ee,0x640d,0x642d,0x644c,0x646b,0x648a,
    0x64a9,0x64c8,0x64e7,0x6506,0x6525,0x6544,0x6562,0x6581,0x65a0,0x65be,
    0x65dd,0x65fb,0x6619,0x6638,0x6656,0x6674,0x6692,0x66b0,0x66ce,0x66ec,
    0x670a,0x6728,0x6746,0x6763,0x6781,0x679e,0x67bc,0x67d9,0x67f7,0x6814,
    0x6831,0x684e,0x686b,0x6888,0x68a5,0x68c2,0x68df,0x68fc,0x6919,0x6935,
    0x6952,0x696e,0x698b,0x69a7,0x69c4,0x69e0,0x69fc,0x6a18,0x6a34,0x6a50,
    0x6a6c,0x6a88,0x6aa4,0x6ac0,0x6adb,0x6af7,0x6b13,0x6b2e,0x6b4a,0x6b65,
    0x6b80,0x6b9c,0x6bb7,0x6bd2,0x6bed,0x6c08,0x6c23,0x6c3e,0x6c59,0x6c73,
    0x6c8e,0x6ca8,0x6cc3,0x6cdd,0x6cf8,0x6d12,0x6d2c,0x6d47,0x6d61,0x6d7b,
    0x6d95,0x6daf,0x6dc9,0x6de3,0x6dfc,0x6e16,0x6e30,0x6e49,0x6e63,0x6e7c,
    0x6e95,0x6eaf,0x6ec8,0x6ee1,0x6efa,0x6f13,0x6f2c,0x6f45,0x6f5e,0x6f76,
    0x6f8f,0x6fa8,0x6fc0,0x6fd9,0x6ff1,0x7009,0x7022,0x703a,0x7052,0x706a,
    0x7082,0x709a,0x70b2,0x70ca,0x70e1,0x70f9,0x7111,0x7128,0x7140,0x7157,
    0x716e,0x7186,0x719d,0x71b4,0x71cb,0x71e2,0x71f9,0x7210,0x7226,0x723d,
    0x7254,0x726a,0x7281,0x7297,0x72ae,0x72c4,0x72da,0x72f0,0x7306,0x731c,
    0x7332,0x7348,0x735e,0x7374,0x7389,0x739f,0x73b5,0x73ca,0x73df,0x73f5,
    0x740a,0x741f,0x7434,0x7449,0x745e,0x7473,0x7488,0x749d,0x74b1,0x74c6,
    0x74db,0x74ef,0x7503,0x7518,0x752c,0x7540,0x7554,0x7568,0x757c,0x7590,
    0x75a4,0x75b8,0x75cc,0x75df,0x75f3,0x7606,0x761a,0x762d,0x7640,0x7653,
    0x7667,0x767a,0x768d,0x76a0,0x76b2,0x76c5,0x76d8,0x76eb,0x76fd,0x7710,
    0x7722,0x7734,0x7747,0x7759,0x776b,0x777d,0x778f,0x77a1,0x77b3,0x77c4,
    0x77d6,0x77e8,0x77f9,0x780b,0x781c,0x782e,0x783f,0x7850,0x7861,0x7872,
    0x7883,0x7894,0x78a5,0x78b6,0x78c6,0x78d7,0x78e7,0x78f8,0x7908,0x7919,
    0x7929,0x7939,0x7949,0x7959,0x7969,0x7979,0x7989,0x7998,0x79a8,0x79b8,
    0x79c7,0x79d7,0x79e6,0x79f5,0x7a05,0x7a14,0x7a23,0x7a32,0x7a41,0x7a4f,
    0x7a5e,0x7a6d,0x7a7c,0x7a8a,0x7a99,0x7aa7,0x7ab5,0x7ac4,0x7ad2,0x7ae0,
    0x7aee,0x7afc,0x7b0a,0x7b18,0x7b25,0x7b33,0x7b41,0x7b4e,0x7b5c,0x7b69,
    0x7b76,0x7b83,0x7b91,0x7b9e,0x7bab,0x7bb8,0x7bc4,0x7bd1,0x7bde,0x7beb,
    0x7bf7,0x7c04,0x7c10,0x7c1c,0x7c29,0x7c35,0x7c41,0x7c4d,0x7c59,0x7c65,
    0x7c70,0x7c7c,0x7c88,0x7c93,0x7c9f,0x7caa,0x7cb6,0x7cc1,0x7ccc,0x7cd7,
    0x7ce2,0x7ced,0x7cf8,0x7d03,0x7d0e,0x7d18,0x7d23,0x7d2e,0x7d38,0x7d42,
    0x7d4d,0x7d57,0x7d61,0x7d6b,0x7d75,0x7d7f,0x7d89,0x7d93,0x7d9c,0x7da6,
    0x7db0,0x7db9,0x7dc2,0x7dcc,0x7dd5,0x7dde,0x7de7,0x7df0,0x7df9,0x7e02,
    0x7e0b,0x7e13,0x7e1c,0x7e25,0x7e2d,0x7e36,0x7e3e,0x7e46,0x7e4e,0x7e56,
    0x7e5e,0x7e66,0x7e6e,0x7e76,0x7e7e,0x7e85,0x7e8d,0x7e94,0x7e9c,0x7ea3,
    0x7eaa,0x7eb2,0x7eb9,0x7ec0,0x7ec7,0x7ece,0x7ed4,0x7edb,0x7ee2,0x7ee8,
    0x7eef,0x7ef5,0x7efc,0x7f02,0x7f08,0x7f0e,0x7f14,0x7f1a,0x7f20,0x7f26,
    0x7f2c,0x7f31,0x7f37,0x7f3c,0x7f42,0x7f47,0x7f4c,0x7f52,0x7f57,0x7f5c,
    0x7f61,0x7f66,0x7f6a,0x7f6f,0x7f74,0x7f78,0x7f7d,0x7f81,0x7f86,0x7f8a,
    0x7f8e,0x7f92,0x7f96,0x7f9a,0x7f9e,0x7fa2,0x7fa6,0x7fa9,0x7fad,0x7fb1,
    0x7fb4,0x7fb7,0x7fbb,0x7fbe,0x7fc1,0x7fc4,0x7fc7,0x7fca,0x7fcd,0x7fcf,
    0x7fd2,0x7fd5,0x7fd7,0x7fd9,0x7fdc,0x7fde,0x7fe0,0x7fe2,0x7fe4,0x7fe6,
    0x7fe8,0x7fea,0x7fec,0x7fee,0x7fef,0x7ff1,0x7ff2,0x7ff3,0x7ff5,0x7ff6,
    0x7ff7,0x7ff8,0x7ff9,0x7ffa,0x7ffb,0x7ffb,0x7ffc,0x7ffd,0x7ffd,0x7ffe,
    0x7ffe,0x7ffe,0x7ffe,0x7fff
};

const qs15_t SINE_H3_TABLE[SINE_SAMPLES] = {
    0x0000,0x0074,0x00e9,0x015e,0x01d3,0x0247,0x02bc,0x0331,0x03a6,0x041b,
    0x048f,0x0504,0x0579,0x05ed,0x0662,0x06d7,0x074b,0x07c0,0x0834,0x08a9,
    0x091d,0x0992,0x0a06,0x0a7b,0x0aef,0x0b63,0x0bd8,0x0c4c,0x0cc0,0x0d34,
    0x0da8,0x0e1c,0x0e90,0x0f04,0x0f78,0x0fec,0x1060,0x10d4,0x1147,0x11bb,
    0x122f,0x12a2,0x1316,0x1389,0x13fc,0x146f,0x14e3,0x1556,0x15c9,0x163c,
    0x16af,0x1721,0x1794,0x1807,0x1879,0x18ec,0x195e,0x19d0,0x1a42,0x1ab4,
    0x1b26,0x1b98,0x1c0a,0x1c7c,0x1cee,0x1d5f,0x1dd0,0x1e42,0x1eb3,0x1f24,
    0x1f95,0x2006,0x2077,0x20e7,0x2158,0x21c8,0x2239,0x22a9,0x2319,0x2389,
    0x23f9,0x2468,0x24d8,0x2547,0x25b7,0x2626,0x2695,0x2704,0x2772,0x27e1,
    0x2850,0x28be,0x292c,0x299a,0x2a08,0x2a76,0x2ae4,0x2b51,0x2bbf,0x2c2c,
    0x2c99,0x2d06,0x2d72,0x2ddf,0x2e4b,0x2eb8,0x2f24,0x2f90,0x2ffb,0x3067,
    0x30d2,0x313e,0x31a9,0x3214,0x327e,0x32e9,0x3353,0x33be,0x3428,0x3492,
    0x34fb,0x3565,0x35ce,0x3637,0x36a0,0x3709,0x3772,0x37da,0x3842,0x38aa,
    0x3912,0x397a,0x39e1,0x3a48,0x3aaf,0x3b16,0x3b7d,0x3be3,0x3c49,0x3caf,
    0x3d15,0x3d7b,0x3de0,0x3e45,0x3eaa,0x3f0f,0x3f73,0x3fd8,0x403c,0x409f,
    0x4103,0x4166,0x41ca,0x422d,0x428f,0x42f2,0x4354,0x43b6,0x4418,0x447a,
    0x44db,0x453c,0x459d,0x45fe,0x465e,0x46be,0x471e,0x477e,0x47dd,0x483d,
    0x489c,0x48fa,0x4959,0x49b7,0x4a15,0x4a73,0x4ad0,0x4b2d,0x4b8a,0x4be7,
    0x4c44,0x4ca0,0x4cfc,0x4d57,0x4db3,0x4e0e,0x4e69,0x4ec4,0x4f1e,0x4f78,
    0x4fd2,0x502c,0x5085,0x50de,0x5137,0x518f,0x51e7,0x523f,0x5297,0x52ee,
    0x5345,0x539c,0x53f3,0x5449,0x549f,0x54f5,0x554a,0x559f,0x55f4,0x5649,
    0x569d,0x56f1,0x5745,0x5798,0x57eb,0x583e,0x5891,0x58e3,0x5935,0x5987,
    0x59d8,0x5a29,0x5a7a,0x5aca,0x5b1a,0x5b6a,0x5bba,0x5c09,0x5c58,0x5ca7,
    0x5cf5,0x5d43,0x5d91,0x5dde,0x5e2b,0x5e78,0x5ec5,0x5f11,0x5f5d,0x5fa8,
    0x5ff4,0x603f,0x6089,0x60d4,0x611e,0x6167,0x61b1,0x61fa,0x6242,0x628b,
    0x62d3,0x631b,0x6362,0x63a9,0x63f0,0x6436,0x647d,0x64c2,0x6508,0x654d,
    0x6592,0x65d6,0x661b,0x665e,0x66a2,0x66e5,0x6728,0x676b,0x67ad,0x67ef,
    0x6830,0x6871,0x68b2,0x68f3,0x6933,0x6973,0x69b2,0x69f2,0x6a30,0x6a6f,
    0x6aad,0x6aeb,0x6b28,0x6b65,0x6ba2,0x6bdf,0x6c1b,0x6c57,0x6c92,0x6ccd,
    0x6d08,0x6d42,0x6d7c,0x6db6,0x6def,0x6e29,0x6e61,0x6e9a,0x6ed1,0x6f09,
    0x6f40,0x6f77,0x6fae,0x6fe4,0x701a,0x7050,0x7085,0x70ba,0x70ee,0x7122,
    0x7156,0x7189,0x71bd,0x71ef,0x7222,0x7254,0x7285,0x72b7,0x72e8,0x7318,
    0x7348,0x7378,0x73a8,0x73d7,0x7406,0x7434,0x7463,0x7490,0x74be,0x74eb,
    0x7517,0x7544,0x7570,0x759b,0x75c7,0x75f2,0x761c,0x7646,0x7670,0x769a,
    0x76c3,0x76eb,0x7714,0x773c,0x7764,0x778b,0x77b2,0x77d9,0x77ff,0x7825,
    0x784a,0x786f,0x7894,0x78b9,0x78dd,0x7900,0x7924,0x7947,0x796a,0x798c,
    0x79ae,0x79cf,0x79f1,0x7a11,0x7a32,0x7a52,0x7a72,0x7a91,0x7ab0,0x7acf,
    0x7aed,0x7b0b,0x7b29,0x7b46,0x7b63,0x7b80,0x7b9c,0x7bb8,0x7bd3,0x7bef,
    0x7c09,0x7c24,0x7c3e,0x7c58,0x7c71,0x7c8a,0x7ca3,0x7cbb,0x7cd3,0x7cea,
    0x7d02,0x7d18,0x7d2f,0x7d45,0x7d5b,0x7d70,0x7d85,0x7d9a,0x7daf,0x7dc3,
    0x7dd6,0x7dea,0x7dfd,0x7e0f,0x7e21,0x7e33,0x7e45,0x7e56,0x7e67,0x7e78,
    0x7e88,0x7e98,0x7ea7,0x7eb6,0x7ec5,0x7ed3,0x7ee2,0x7eef,0x7efd,0x7f0a,
    0x7f16,0x7f23,0x7f2f,0x7f3a,0x7f46,0x7f51,0x7f5b,0x7f66,0x7f70,0x7f79,
    0x7f83,0x7f8c,0x7f94,0x7f9c,0x7fa4,0x7fac,0x7fb3,0x7fba,0x7fc1,0x7fc7,
    0x7fcd,0x7fd2,0x7fd8,0x7fdd,0x7fe1,0x7fe5,0x7fe9,0x7fed,0x7ff0,0x7ff3,
    0x7ff6,0x7ff8,0x7ffa,0x7ffb,0x7ffd,0x7ffe,0x7ffe,0x7fff,0x7ffe,0x7ffe,
    0x7ffd,0x7ffc,0x7ffb,0x7ffa,0x7ff8,0x7ff5,0x7ff3,0x7ff0,0x7fed,0x7fe9,
    0x7fe5,0x7fe1,0x7fdc,0x7fd8,0x7fd3,0x7fcd,0x7fc7,0x7fc1,0x7fbb,0x7fb4,
    0x7fad,0x7fa6,0x7f9e,0x7f97,0x7f8e,0x7f86,0x7f7d,0x7f74,0x7f6b,0x7f61,
    0x7f57,0x7f4d,0x7f42,0x7f37,0x7f2c,0x7f20,0x7f15,0x7f08,0x7efc,0x7eef,
    0x7ee2,0x7ed5,0x7ec8,0x7eba,0x7eac,0x7e9d,0x7e8f,0x7e80,0x7e70,0x7e61,
    0x7e51,0x7e41,0x7e31,0x7e20,0x7e0f,0x7dfe,0x7dec,0x7dda,0x7dc8,0x7db6,
    0x7da4,0x7d91,0x7d7e,0x7d6a,0x7d57,0x7d43,0x7d2e,0x7d1a,0x7d05,0x7cf0,
    0x7cdb,0x7cc6,0x7cb0,0x7c9a,0x7c83,0x7c6d,0x7c56,0x7c3f,0x7c28,0x7c10,
    0x7bf8,0x7be0,0x7bc8,0x7bb0,0x7b97,0x7b7e,0x7b64,0x7b4b,0x7b31,0x7b17,
    0x7afd,0x7ae2,0x7ac8,0x7aad,0x7a92,0x7a76,0x7a5a,0x7a3f,0x7a22,0x7a06,
    0x79ea,0x79cd,0x79b0,0x7992,0x7975,0x7957,0x7939,0x791b,0x78fd,0x78de,
    0x78bf,0x78a0,0x7881,0x7862,0x7842,0x7822,0x7802,0x77e2,0x77c1,0x77a1,
    0x7780,0x775f,0x773e,0x771c,0x76fa,0x76d8,0x76b6,0x7694,0x7672,0x764f,
    0x762c,0x7609,0x75e6,0x75c2,0x759f,0x757b,0x7557,0x7533,0x750f,0x74ea,
    0x74c5,0x74a1,0x747b,0x7456,0x7431,0x740b,0x73e6,0x73c0,0x739a,0x7373,
    0x734d,0x7326,0x7300,0x72d9,0x72b2,0x728b,0x7263,0x723c,0x7214,0x71ec,
    0x71c4,0x719c,0x7174,0x714b,0x7123,0x70fa,0x70d1,0x70a8,0x707f,0x7056,
    0x702c,0x7003,0x6fd9,0x6faf,0x6f85,0x6f5b,0x6f31,0x6f07,0x6edc,0x6eb2,
    0x6e87,0x6e5c,0x6e31,0x6e06,0x6ddb,0x6db0,0x6d84,0x6d59,0x6d2d,0x6d01,
    0x6cd5,0x6ca9,0x6c7d,0x6c51,0x6c25,0x6bf8,0x6bcc,0x6b9f,0x6b72,0x6b46,
    0x6b19,0x6aec,0x6abf,0x6a91,0x6a64,0x6a37,0x6a09,0x69dc,0x69ae,0x6981,
    0x6953,0x6925,0x68f7,0x68c9,0x689b,0x686d,0x683e,0x6810,0x67e2,0x67b3,
    0x6785,0x6756,0x6728,0x66f9,0x66ca,0x669b,0x666c,0x663d,0x660e,0x65df,
    0x65b0,0x6581,0x6552,0x6523,0x64f3,0x64c4,0x6495,0x6465,0x6436,0x6406,
    0x63d7,0x63a7,0x6377,0x6348,0x6318,0x62e8,0x62b8,0x6289,0x6259,0x6229,
    0x61f9,0x61c9,0x6199,0x6169,0x6139,0x6109,0x60d9,0x60a9,0x6079,0x6049,
    0x6019,0x5fe9,0x5fb9,0x5f89,0x5f59,0x5f29,0x5ef9,0x5ec9,0x5e99,0x5e69,
    0x5e39,0x5e09,0x5dd9,0x5da9,0x5d78,0x5d48,0x5d18,0x5ce8,0x5cb8,0x5c89,
    0x5c59,0x5c29,0x5bf9,0x5bc9,0x5b99,0x5b69,0x5b39,0x5b0a,0x5ada,0x5aaa,
    0x5a7a,0x5a4b,0x5a1b,0x59eb,0x59bc,0x598c,0x595d,0x592d,0x58fe,0x58cf,
    0x589f,0x5870,0x5841,0x5812,0x57e3,0x57b4,0x5785,0x5756,0x5727,0x56f8,
    0x56c9,0x569a,0x566c,0x563d,0x560f,0x55e0,0x55b2,0x5583,0x5555,0x5527,
    0x54f9,0x54cb,0x549d,0x546f,0x5441,0x5413,0x53e6,0x53b8,0x538b,0x535d,
    0x5330,0x5303,0x52d5,0x52a8,0x527b,0x524e,0x5222,0x51f5,0x51c8,0x519c,
    0x516f,0x5143,0x5117,0x50eb,0x50bf,0x5093,0x5067,0x503b,0x5010,0x4fe4,
    0x4fb9,0x4f8e,0x4f62,0x4f37,0x4f0c,0x4ee2,0x4eb7,0x4e8c,0x4e62,0x4e38,
    0x4e0d,0x4de3,0x4db9,0x4d8f,0x4d66,0x4d3c,0x4d13,0x4ce9,0x4cc0,0x4c97,
    0x4c6e,0x4c45,0x4c1c,0x4bf4,0x4bcc,0x4ba3,0x4b7b,0x4b53,0x4b2b,0x4b04,
    0x4adc,0x4ab5,0x4a8d,0x4a66,0x4a3f,0x4a18,0x49f2,0x49cb,0x49a5,0x497f,
    0x4959,0x4933,0x490d,0x48e7,0x48c2,0x489d,0x4877,0x4853,0x482e,0x4809,
    0x47e5,0x47c0,0x479c,0x4778,0x4754,0x4731,0x470d,0x46ea,0x46c7,0x46a4,
    0x4681,0x465f,0x463c,0x461a,0x45f8,0x45d6,0x45b4,0x4593,0x4572,0x4550,
    0x452f,0x450f,0x44ee,0x44ce,0x44ad,0x448d,0x446d,0x444e,0x442e,0x440f,
    0x43f0,0x43d1,0x43b2,0x4394,0x4376,0x4357,0x433a,0x431c,0x42fe,0x42e1,
    0x42c4,0x42a7,0x428a,0x426e,0x4251,0x4235,0x4219,0x41fe,0x41e2,0x41c7,
    0x41ac,0x4191,0x4176,0x415c,0x4142,0x4128,0x410e,0x40f4,0x40db,0x40c2,
    0x40a9,0x4090,0x4077,0x405f,0x4047,0x402f,0x4017,0x4000,0x3fe9,0x3fd2,
    0x3fbb,0x3fa4,0x3f8e,0x3f78,0x3f62,0x3f4c,0x3f37,0x3f22,0x3f0d,0x3ef8,
    0x3ee3,0x3ecf,0x3ebb,0x3ea7,0x3e94,0x3e80,0x3e6d,0x3e5a,0x3e47,0x3e35,
    0x3e23,0x3e11,0x3dff,0x3ded,0x3ddc,0x3dcb,0x3dba,0x3daa,0x3d99,0x3d89,
    0x3d79,0x3d6a,0x3d5a,0x3d4b,0x3d3c,0x3d2d,0x3d1f,0x3d11,0x3d03,0x3cf5,
    0x3ce8,0x3cda,0x3ccd,0x3cc1,0x3cb4,0x3ca8,0x3c9c,0x3c90,0x3c84,0x3c79,
    0x3c6e,0x3c63,0x3c59,0x3c4e,0x3c44,0x3c3a,0x3c31,0x3c27,0x3c1e,0x3c15,
    0x3c0d,0x3c04,0x3bfc,0x3bf5,0x3bed,0x3be6,0x3bde,0x3bd8,0x3bd1,0x3bcb,
    0x3bc4,0x3bbf,0x3bb9,0x3bb3,0x3bae,0x3ba9,0x3ba5,0x3ba0,0x3b9c,0x3b98,
    0x3b95,0x3b91,0x3b8e,0x3b8b,0x3b89,0x3b86,0x3b84,0x3b82,0x3b81,0x3b7f,
    0x3b7e,0x3b7d,0x3b7d,0x3b7c
};

const qs15_t CAPACITOR_TABLE[CAPACITOR_SAMPLES] = {
    0x8001,0x8066,0x80cb,0x8130,0x8194,0x81f9,0x825d,0x82c1,0x8325,0x8389,
    0x83ec,0x8450,0x84b3,0x8516,0x8579,0x85dc,0x863f,0x86a2,0x8704,0x8767,
    0x87c9,0x882b,0x888d,0x88ee,0x8950,0x89b1,0x8a13,0x8a74,0x8ad5,0x8b36,
    0x8b97,0x8bf7,0x8c58,0x8cb8,0x8d18,0x8d78,0x8dd8,0x8e38,0x8e98,0x8ef7,
    0x8f56,0x8fb6,0x9015,0x9074,0x90d2,0x9131,0x9190,0x91ee,0x924c,0x92aa,
    0x9308,0x9366,0x93c4,0x9421,0x947f,0x94dc,0x9539,0x9596,0x95f3,0x9650,
    0x96ac,0x9709,0x9765,0x97c1,0x981d,0x9879,0x98d5,0x9931,0x998c,0x99e8,
    0x9a43,0x9a9e,0x9af9,0x9b54,0x9baf,0x9c09,0x9c64,0x9cbe,0x9d18,0x9d72,
    0x9dcc,0x9e26,0x9e80,0x9ed9,0x9f32,0x9f8c,0x9fe5,0xa03e,0xa097,0xa0ef,
    0xa148,0xa1a1,0xa1f9,0xa251,0xa2a9,0xa301,0xa359,0xa3b1,0xa408,0xa460,
    0xa4b7,0xa50e,0xa565,0xa5bc,0xa613,0xa66a,0xa6c0,0xa717,0xa76d,0xa7c3,
    0xa819,0xa86f,0xa8c5,0xa91b,0xa970,0xa9c6,0xaa1b,0xaa70,0xaac5,0xab1a,
    0xab6f,0xabc4,0xac18,0xac6d,0xacc1,0xad15,0xad69,0xadbd,0xae11,0xae65,
    0xaeb8,0xaf0c,0xaf5f,0xafb2,0xb005,0xb058,0xb0ab,0xb0fe,0xb150,0xb1a3,
    0xb1f5,0xb247,0xb29a,0xb2ec,0xb33d,0xb38f,0xb3e1,0xb432,0xb484,0xb4d5,
    0xb526,0xb577,0xb5c8,0xb619,0xb66a,0xb6ba,0xb70b,0xb75b,0xb7ab,0xb7fb,
    0xb84b,0xb89b,0xb8eb,0xb93a,0xb98a,0xb9d9,0xba29,0xba78,0xbac7,0xbb16,
    0xbb65,0xbbb3,0xbc02,0xbc50,0xbc9f,0xbced,0xbd3b,0xbd89,0xbdd7,0xbe25,
    0xbe72,0xbec0,0xbf0d,0xbf5b,0xbfa8,0xbff5,0xc042,0xc08f,0xc0dc,0xc128,
    0xc175,0xc1c1,0xc20e,0xc25a,0xc2a6,0xc2f2,0xc33e,0xc38a,0xc3d5,0xc421,
    0xc46c,0xc4b7,0xc503,0xc54e,0xc599,0xc5e4,0xc62e,0xc679,0xc6c4,0xc70e,
    0xc758,0xc7a3,0xc7ed,0xc837,0xc881,0xc8ca,0xc914,0xc95e,0xc9a7,0xc9f0,
    0xca3a,0xca83,0xcacc,0xcb15,0xcb5e,0xcba6,0xcbef,0xcc37,0xcc80,0xccc8,
    0xcd10,0xcd58,0xcda0,0xcde8,0xce30,0xce78,0xcebf,0xcf07,0xcf4e,0xcf95,
    0xcfdc,0xd023,0xd06a,0xd0b1,0xd0f8,0xd13e,0xd185,0xd1cb,0xd212,0xd258,
    0xd29e,0xd2e4,0xd32a,0xd370,0xd3b5,0xd3fb,0xd440,0xd486,0xd4cb,0xd510,
    0xd555,0xd59a,0xd5df,0xd624,0xd669,0xd6ad,0xd6f2,0xd736,0xd77a,0xd7bf,
    0xd803,0xd847,0xd88a,0xd8ce,0xd912,0xd956,0xd999,0xd9dc,0xda20,0xda63,
    0xdaa6,0xdae9,0xdb2c,0xdb6f,0xdbb1,0xdbf4,0xdc36,0xdc79,0xdcbb,0xdcfd,
    0xdd3f,0xdd81,0xddc3,0xde05,0xde47,0xde88,0xdeca,0xdf0b,0xdf4d,0xdf8e,
    0xdfcf,0xe010,0xe051,0xe092,0xe0d3,0xe114,0xe154,0xe195,0xe1d5,0xe215,
    0xe255,0xe296,0xe2d6,0xe315,0xe355,0xe395,0xe3d5,0xe414,0xe454,0xe493,
    0xe4d2,0xe511,0xe551,0xe590,0xe5ce,0xe60d,0xe64c,0xe68b,0xe6c9,0xe708,
    0xe746,0xe784,0xe7c2,0xe800,0xe83e,0xe87c,0xe8ba,0xe8f8,0xe935,0xe973,
    0xe9b0,0xe9ee,0xea2b,0xea68,0xeaa5,0xeae2,0xeb1f,0xeb5c,0xeb99,0xebd5,
    0xec12,0xec4e,0xec8b,0xecc7,0xed03,0xed3f,0xed7b,0xedb7,0xedf3,0xee2f,
    0xee6a,0xeea6,0xeee2,0xef1d,0xef58,0xef94,0xefcf,0xf00a,0xf045,0xf080,
    0xf0ba,0xf0f5,0xf130,0xf16a,0xf1a5,0xf1df,0xf219,0xf254,0xf28e,0xf2c8,
    0xf302,0xf33c,0xf375,0xf3af,0xf3e9,0xf422,0xf45c,0xf495,0xf4ce,0xf507,
    0xf540,0xf579,0xf5b2,0xf5eb,0xf624,0xf65d,0xf695,0xf6ce,0xf706,0xf73f,
    0xf777,0xf7af,0xf7e7,0xf81f,0xf857,0xf88f,0xf8c7,0xf8fe,0xf936,0xf96e,
    0xf9a5,0xf9dc,0xfa14,0xfa4b,0xfa82,0xfab9,0xfaf0,0xfb27,0xfb5e,0xfb95,
    0xfbcb,0xfc02,0xfc38,0xfc6f,0xfca5,0xfcdb,0xfd11,0xfd47,0xfd7d,0xfdb3,
    0xfde9,0xfe1f,0xfe55,0xfe8a,0xfec0,0xfef5,0xff2b,0xff60,0xff95,0xffca,
    0xffff,0x0033,0x0068,0x009d,0x00d2,0x0107,0x013b,0x0170,0x01a4,0x01d9,
    0x020d,0x0241,0x0275,0x02a9,0x02dd,0x0311,0x0345,0x0379,0x03ad,0x03e0,
    0x0414,0x0447,0x047b,0x04ae,0x04e1,0x0514,0x0547,0x057a,0x05ad,0x05e0,
    0x0613,0x0646,0x0678,0x06ab,0x06dd,0x0710,0x0742,0x0774,0x07a7,0x07d9,
    0x080b,0x083d,0x086f,0x08a0,0x08d2,0x0904,0x0935,0x0967,0x0998,0x09ca,
    0x09fb,0x0a2c,0x0a5e,0x0a8f,0x0ac0,0x0af1,0x0b22,0x0b52,0x0b83,0x0bb4,
    0x0be4,0x0c15,0x0c45,0x0c76,0x0ca6,0x0cd6,0x0d07,0x0d37,0x0d67,0x0d97,
    0x0dc7,0x0df6,0x0e26,0x0e56,0x0e85,0x0eb5,0x0ee4,0x0f14,0x0f43,0x0f73,
    0x0fa2,0x0fd1,0x1000,0x102f,0x105e,0x108d,0x10bb,0x10ea,0x1119,0x1147,
    0x1176,0x11a4,0x11d3,0x1201,0x122f,0x125d,0x128c,0x12ba,0x12e8,0x1316,
    0x1343,0x1371,0x139f,0x13cc,0x13fa,0x1428,0x1455,0x1482,0x14b0,0x14dd,
    0x150a,0x1537,0x1564,0x1591,0x15be,0x15eb,0x1618,0x1644,0x1671,0x169e,
    0x16ca,0x16f7,0x1723,0x174f,0x177c,0x17a8,0x17d4,0x1800,0x182c,0x1858,
    0x1884,0x18b0,0x18db,0x1907,0x1933,0x195e,0x198a,0x19b5,0x19e0,0x1a0c,
    0x1a37,0x1a62,0x1a8d,0x1ab8,0x1ae3,0x1b0e,0x1b39,0x1b64,0x1b8e,0x1bb9,
    0x1be4,0x1c0e,0x1c39,0x1c63,0x1c8d,0x1cb8,0x1ce2,0x1d0c,0x1d36,0x1d60,
    0x1d8a,0x1db4,0x1dde,0x1e08,0x1e32,0x1e5b,0x1e85,0x1eae,0x1ed8,0x1f01,
    0x1f2b,0x1f54,0x1f7d,0x1fa6,0x1fd0,0x1ff9,0x2022,0x204b,0x2074,0x209c,
    0x20c5,0x20ee,0x2116,0x213f,0x2168,0x2190,0x21b8,0x21e1,0x2209,0x2231,
    0x225a,0x2282,0x22aa,0x22d2,0x22fa,0x2322,0x2349,0x2371,0x2399,0x23c0,
    0x23e8,0x2410,0x2437,0x245e,0x2486,0x24ad,0x24d4,0x24fc,0x2523,0x254a,
    0x2571,0x2598,0x25bf,0x25e6,0x260c,0x2633,0x265a,0x2680,0x26a7,0x26cd,
    0x26f4,0x271a,0x2741,0x2767,0x278d,0x27b3,0x27d9,0x27ff,0x2825,0x284b,
    0x2871,0x2897,0x28bd,0x28e3,0x2908,0x292e,0x2953,0x2979,0x299e,0x29c4,
    0x29e9,0x2a0e,0x2a34,0x2a59,0x2a7e,0x2aa3,0x2ac8,0x2aed,0x2b12,0x2b37,
    0x2b5b,0x2b80,0x2ba5,0x2bca,0x2bee,0x2c13,0x2c37,0x2c5c,0x2c80,0x2ca4,
    0x2cc8,0x2ced,0x2d11,0x2d35,0x2d59,0x2d7d,0x2da1,0x2dc5,0x2de9,0x2e0c,
    0x2e30,0x2e54,0x2e78,0x2e9b,0x2ebf,0x2ee2,0x2f05,0x2f29,0x2f4c,0x2f6f,
    0x2f93,0x2fb6,0x2fd9,0x2ffc,0x301f,0x3042,0x3065,0x3088,0x30ab,0x30cd,
    0x30f0,0x3113,0x3135,0x3158,0x317a,0x319d,0x31bf,0x31e1,0x3204,0x3226,
    0x3248,0x326a,0x328c,0x32af,0x32d0,0x32f2,0x3314,0x3336,0x3358,0x337a,
    0x339b,0x33bd,0x33df,0x3400,0x3422,0x3443,0x3465,0x3486,0x34a7,0x34c9,
    0x34ea,0x350b,0x352c,0x354d,0x356e,0x358f,0x35b0,0x35d1,0x35f2,0x3612,
    0x3633,0x3654,0x3674,0x3695,0x36b6,0x36d6,0x36f7,0x3717,0x3737,0x3758,
    0x3778,0x3798,0x37b8,0x37d8,0x37f8,0x3818,0x3838,0x3858,0x3878,0x3898,
    0x38b8,0x38d7,0x38f7,0x3917,0x3936,0x3956,0x3975,0x3995,0x39b4,0x39d4,
    0x39f3,0x3a12,0x3a32,0x3a51,0x3a70,0x3a8f,0x3aae,0x3acd,0x3aec,0x3b0b,
    0x3b2a,0x3b49,0x3b67,0x3b86,0x3ba5,0x3bc3,0x3be2,0x3c00,0x3c1f,0x3c3d,
    0x3c5c,0x3c7a,0x3c99,0x3cb7,0x3cd5,0x3cf3,0x3d11,0x3d30,0x3d4e,0x3d6c,
    0x3d8a,0x3da8,0x3dc5,0x3de3,0x3e01,0x3e1f,0x3e3c,0x3e5a,0x3e78,0x3e95,
    0x3eb3,0x3ed0,0x3eee,0x3f0b,0x3f29,0x3f46,0x3f63,0x3f80,0x3f9e,0x3fbb,
    0x3fd8,0x3ff5,0x4012,0x402f,0x404c,0x4069,0x4086,0x40a2,0x40bf,0x40dc,
    0x40f9,0x4115,0x4132,0x414e,0x416b,0x4187,0x41a4,0x41c0,0x41dd,0x41f9,
    0x4215,0x4231,0x424e,0x426a,0x4286,0x42a2,0x42be,0x42da,0x42f6,0x4312,
    0x432e,0x4349,0x4365,0x4381,0x439d,0x43b8,0x43d4,0x43ef,0x440b,0x4426,
    0x4442,0x445d,0x4479,0x4494,0x44af,0x44cb,0x44e6,0x4501,0x451c,0x4537,
    0x4552,0x456d,0x4588,0x45a3,0x45be,0x45d9,0x45f4,0x460e,0x4629,0x4644,
    0x465f,0x4679,0x4694,0x46ae,0x46c9,0x46e3,0x46fe,0x4718,0x4732,0x474d,
    0x4767,0x4781,0x479b,0x47b6,0x47d0,0x47ea,0x4804,0x481e,0x4838,0x4852,
    0x486c,0x4885,0x489f,0x48b9,0x48d3,0x48ed,0x4906,0x4920,0x4939,0x4953,
    0x496c,0x4986,0x499f,0x49b9,0x49d2,0x49eb,0x4a05,0x4a1e,0x4a37,0x4a50,
    0x4a6a,0x4a83,0x4a9c,0x4ab5,0x4ace,0x4ae7,0x4b00,0x4b19,0x4b31,0x4b4a,
    0x4b63,0x4b7c,0x4b94,0x4bad,0x4bc6,0x4bde,0x4bf7,0x4c0f,0x4c28,0x4c40,
    0x4c59,0x4c71,0x4c89,0x4ca2,0x4cba,0x4cd2,0x4cea,0x4d03,0x4d1b,0x4d33,
    0x4d4b,0x4d63,0x4d7b,0x4d93,0x4dab,0x4dc3,0x4ddb,0x4df2,0x4e0a,0x4e22,
    0x4e3a,0x4e51,0x4e69,0x4e81,0x4e98,0x4eb0,0x4ec7,0x4edf,0x4ef6,0x4f0d,
    0x4f25,0x4f3c,0x4f53,0x4f6b,0x4f82,0x4f99,0x4fb0,0x4fc7,0x4fde,0x4ff6,
    0x500d,0x5024,0x503b,0x5051,0x5068,0x507f,0x5096,0x50ad,0x50c4,0x50da,
    0x50f1,0x5108,0x511e,0x5135,0x514b,0x5162,0x5178,0x518f,0x51a5,0x51bc,
    0x51d2,0x51e8,0x51ff,0x5215,0x522b,0x5241,0x5258,0x526e,0x5284,0x529a,
    0x52b0,0x52c6,0x52dc,0x52f2,0x5308,0x531e,0x5333,0x5349,0x535f,0x5375,
    0x538b,0x53a0,0x53b6,0x53cb,0x53e1,0x53f7,0x540c,0x5422,0x5437,0x544d,
    0x5462,0x5477,0x548d,0x54a2,0x54b7,0x54cc,0x54e2,0x54f7,0x550c,0x5521,
    0x5536,0x554b,0x5560,0x5575,0x558a,0x559f,0x55b4,0x55c9,0x55de,0x55f3,
    0x5607,0x561c,0x5631,0x5646,0x565a,0x566f,0x5683,0x5698,0x56ad,0x56c1,
    0x56d6,0x56ea,0x56fe,0x5713,0x5727,0x573c,0x5750,0x5764,0x5778,0x578d,
    0x57a1,0x57b5,0x57c9,0x57dd,0x57f1,0x5805,0x5819,0x582d,0x5841,0x5855,
    0x5869,0x587d,0x5891,0x58a4,0x58b8,0x58cc,0x58e0,0x58f3,0x5907,0x591b,
    0x592e,0x5942,0x5955,0x5969,0x597c,0x5990,0x59a3,0x59b7,0x59ca,0x59dd,
    0x59f1,0x5a04,0x5a17,0x5a2a,0x5a3e,0x5a51,0x5a64,0x5a77,0x5a8a,0x5a9d,
    0x5ab0,0x5ac3,0x5ad6,0x5ae9,0x5afc,0x5b0f,0x5b22,0x5b35,0x5b47,0x5b5a,
    0x5b6d,0x5b80,0x5b92,0x5ba5,0x5bb8,0x5bca,0x5bdd,0x5bf0,0x5c02,0x5c15,
    0x5c27,0x5c3a,0x5c4c,0x5c5e,0x5c71,0x5c83,0x5c95,0x5ca8,0x5cba,0x5ccc,
    0x5cde,0x5cf1,0x5d03,0x5d15,0x5d27,0x5d39,0x5d4b,0x5d5d,0x5d6f,0x5d81,
    0x5d93,0x5da5,0x5db7,0x5dc9,0x5ddb,0x5dec,0x5dfe,0x5e10,0x5e22,0x5e34,
    0x5e45,0x5e57,0x5e69,0x5e7a,0x5e8c,0x5e9d,0x5eaf,0x5ec0,0x5ed2,0x5ee3,
    0x5ef5,0x5f06,0x5f17,0x5f29,0x5f3a,0x5f4b,0x5f5d,0x5f6e,0x5f7f,0x5f90,
    0x5fa2,0x5fb3,0x5fc4,0x5fd5,0x5fe6,0x5ff7,0x6008,0x6019,0x602a,0x603b,
    0x604c,0x605d,0x606e,0x607f,0x608f,0x60a0,0x60b1,0x60c2,0x60d3,0x60e3,
    0x60f4,0x6105,0x6115,0x6126,0x6136,0x6147,0x6158,0x6168,0x6179,0x6189,
    0x6199,0x61aa,0x61ba,0x61cb,0x61db,0x61eb,0x61fc,0x620c,0x621c,0x622c,
    0x623d,0x624d,0x625d,0x626d,0x627d,0x628d,0x629d,0x62ad,0x62bd,0x62cd,
    0x62dd,0x62ed,0x62fd,0x630d,0x631d,0x632d,0x633d,0x634c,0x635c,0x636c,
    0x637c,0x638b,0x639b,0x63ab,0x63ba,0x63ca,0x63da,0x63e9,0x63f9,0x6408,
    0x6418,0x6427,0x6437,0x6446,0x6455,0x6465,0x6474,0x6484,0x6493,0x64a2,
    0x64b1,0x64c1,0x64d0,0x64df,0x64ee,0x64fe,0x650d,0x651c,0x652b,0x653a,
    0x6549,0x6558,0x6567,0x6576,0x6585,0x6594,0x65a3,0x65b2,0x65c1,0x65d0,
    0x65de,0x65ed,0x65fc,0x660b,0x661a,0x6628,0x6637,0x6646,0x6654,0x6663,
    0x6672,0x6680,0x668f,0x669d,0x66ac,0x66ba,0x66c9,0x66d7,0x66e6,0x66f4,
    0x6703,0x6711,0x671f,0x672e,0x673c,0x674a,0x6759,0x6767,0x6775,0x6783,
    0x6792,0x67a0,0x67ae,0x67bc,0x67ca,0x67d8,0x67e6,0x67f4,0x6802,0x6810,
    0x681e,0x682c,0x683a,0x6848,0x6856,0x6864,0x6872,0x6880,0x688e,0x689b,
    0x68a9,0x68b7,0x68c5,0x68d2,0x68e0,0x68ee,0x68fc,0x6909,0x6917,0x6924,
    0x6932,0x6940,0x694d,0x695b,0x6968,0x6976,0x6983,0x6991,0x699e,0x69ab,
    0x69b9,0x69c6,0x69d4,0x69e1,0x69ee,0x69fb,0x6a09,0x6a16,0x6a23,0x6a30,
    0x6a3e,0x6a4b,0x6a58,0x6a65,0x6a72,0x6a7f,0x6a8c,0x6a9a,0x6aa7,0x6ab4,
    0x6ac1,0x6ace,0x6adb,0x6ae7,0x6af4,0x6b01,0x6b0e,0x6b1b,0x6b28,0x6b35,
    0x6b42,0x6b4e,0x6b5b,0x6b68,0x6b75,0x6b81,0x6b8e,0x6b9b,0x6ba7,0x6bb4,
    0x6bc1,0x6bcd,0x6bda,0x6be7,0x6bf3,0x6c00,0x6c0c,0x6c19,0x6c25,0x6c32,
    0x6c3e,0x6c4a,0x6c57,0x6c63,0x6c70,0x6c7c,0x6c88,0x6c95,0x6ca1,0x6cad,
    0x6cb9,0x6cc6,0x6cd2,0x6cde,0x6cea,0x6cf7,0x6d03,0x6d0f,0x6d1b,0x6d27,
    0x6d33,0x6d3f,0x6d4b,0x6d57,0x6d63,0x6d6f,0x6d7b,0x6d87,0x6d93,0x6d9f,
    0x6dab,0x6db7,0x6dc3,0x6dcf,0x6ddb,0x6de6,0x6df2,0x6dfe,0x6e0a,0x6e15,
    0x6e21,0x6e2d,0x6e39,0x6e44,0x6e50,0x6e5c,0x6e67,0x6e73,0x6e7e,0x6e8a,
    0x6e96,0x6ea1,0x6ead,0x6eb8,0x6ec4,0x6ecf,0x6edb,0x6ee6,0x6ef2,0x6efd,
    0x6f08,0x6f14,0x6f1f,0x6f2a,0x6f36,0x6f41,0x6f4c,0x6f58,0x6f63,0x6f6e,
    0x6f79,0x6f85,0x6f90,0x6f9b,0x6fa6,0x6fb1,0x6fbc,0x6fc8,0x6fd3,0x6fde,
    0x6fe9,0x6ff4,0x6fff,0x700a,0x7015,0x7020,0x702b,0x7036,0x7041,0x704c,
    0x7057,0x7062,0x706c,0x7077,0x7082,0x708d,0x7098,0x70a3,0x70ad,0x70b8,
    0x70c3,0x70ce,0x70d8,0x70e3,0x70ee,0x70f8,0x7103,0x710e,0x7118,0x7123,
    0x712e,0x7138,0x7143,0x714d,0x7158,0x7162,0x716d,0x7177,0x7182,0x718c,
    0x7197,0x71a1,0x71ab,0x71b6,0x71c0,0x71cb,0x71d5,0x71df,0x71ea,0x71f4,
    0x71fe,0x7208,0x7213,0x721d,0x7227,0x7231,0x723c,0x7246,0x7250,0x725a,
    0x7264,0x726e,0x7278,0x7283,0x728d,0x7297,0x72a1,0x72ab,0x72b5,0x72bf,
    0x72c9,0x72d3,0x72dd,0x72e7,0x72f1,0x72fb,0x7304,0x730e,0x7318,0x7322,
    0x732c,0x7336,0x7340,0x7349,0x7353,0x735d,0x7367,0x7370,0x737a,0x7384,
    0x738e,0x7397,0x73a1,0x73ab,0x73b4,0x73be,0x73c7,0x73d1,0x73db,0x73e4,
    0x73ee,0x73f7,0x7401,0x740a,0x7414,0x741d,0x7427,0x7430,0x743a,0x7443,
    0x744d,0x7456,0x745f,0x7469,0x7472,0x747c,0x7485,0x748e,0x7498,0x74a1,
    0x74aa,0x74b3,0x74bd,0x74c6,0x74cf,0x74d8,0x74e2,0x74eb,0x74f4,0x74fd,
    0x7506,0x750f,0x7518,0x7522,0x752b,0x7534,0x753d,0x7546,0x754f,0x7558,
    0x7561,0x756a,0x7573,0x757c,0x7585,0x758e,0x7597,0x75a0,0x75a9,0x75b2,
    0x75ba,0x75c3,0x75cc,0x75d5,0x75de,0x75e7,0x75ef,0x75f8,0x7601,0x760a,
    0x7613,0x761b,0x7624,0x762d,0x7636,0x763e,0x7647,0x7650,0x7658,0x7661,
    0x7669,0x7672,0x767b,0x7683,0x768c,0x7694,0x769d,0x76a6,0x76ae,0x76b7,
    0x76bf,0x76c8,0x76d0,0x76d9,0x76e1,0x76e9,0x76f2,0x76fa,0x7703,0x770b,
    0x7713,0x771c,0x7724,0x772c,0x7735,0x773d,0x7745,0x774e,0x7756,0x775e,
    0x7767,0x776f,0x7777,0x777f,0x7787,0x7790,0x7798,0x77a0,0x77a8,0x77b0,
    0x77b8,0x77c1,0x77c9,0x77d1,0x77d9,0x77e1,0x77e9,0x77f1,0x77f9,0x7801,
    0x7809,0x7811,0x7819,0x7821,0x7829,0x7831,0x7839,0x7841,0x7849,0x7851,
    0x7859,0x7861,0x7869,0x7870,0x7878,0x7880,0x7888,0x7890,0x7898,0x789f,
    0x78a7,0x78af,0x78b7,0x78be,0x78c6,0x78ce,0x78d6,0x78dd,0x78e5,0x78ed,
    0x78f4,0x78fc,0x7904,0x790b,0x7913,0x791b,0x7922,0x792a,0x7931,0x7939,
    0x7941,0x7948,0x7950,0x7957,0x795f,0x7966,0x796e,0x7975,0x797d,0x7984,
    0x798c,0x7993,0x799a,0x79a2,0x79a9,0x79b1,0x79b8,0x79bf,0x79c7,0x79ce,
    0x79d6,0x79dd,0x79e4,0x79ec,0x79f3,0x79fa,0x7a01,0x7a09,0x7a10,0x7a17,
    0x7a1e,0x7a26,0x7a2d,0x7a34,0x7a3b,0x7a42,0x7a4a,0x7a51,0x7a58,0x7a5f,
    0x7a66,0x7a6d,0x7a74,0x7a7c,0x7a83,0x7a8a,0x7a91,0x7a98,0x7a9f,0x7aa6,
    0x7aad,0x7ab4,0x7abb,0x7ac2,0x7ac9,0x7ad0,0x7ad7,0x7ade,0x7ae5,0x7aec,
    0x7af3,0x7afa,0x7b01,0x7b07,0x7b0e,0x7b15,0x7b1c,0x7b23,0x7b2a,0x7b31,
    0x7b37,0x7b3e,0x7b45,0x7b4c,0x7b53,0x7b59,0x7b60,0x7b67,0x7b6e,0x7b74,
    0x7b7b,0x7b82,0x7b89,0x7b8f,0x7b96,0x7b9d,0x7ba3,0x7baa,0x7bb1,0x7bb7,
    0x7bbe,0x7bc4,0x7bcb,0x7bd2,0x7bd8,0x7bdf,0x7be5,0x7bec,0x7bf3,0x7bf9,
    0x7c00,0x7c06,0x7c0d,0x7c13,0x7c1a,0x7c20,0x7c27,0x7c2d,0x7c34,0x7c3a,
    0x7c40,0x7c47,0x7c4d,0x7c54,0x7c5a,0x7c60,0x7c67,0x7c6d,0x7c74,0x7c7a,
    0x7c80,0x7c87,0x7c8d,0x7c93,0x7c9a,0x7ca0,0x7ca6,0x7cac,0x7cb3,0x7cb9,
    0x7cbf,0x7cc5,0x7ccc,0x7cd2,0x7cd8,0x7cde,0x7ce5,0x7ceb,0x7cf1,0x7cf7,
    0x7cfd,0x7d03,0x7d0a,0x7d10,0x7d16,0x7d1c,0x7d22,0x7d28,0x7d2e,0x7d34,
    0x7d3a,0x7d40,0x7d46,0x7d4d,0x7d53,0x7d59,0x7d5f,0x7d65,0x7d6b,0x7d71,
    0x7d77,0x7d7d,0x7d83,0x7d88,0x7d8e,0x7d94,0x7d9a,0x7da0,0x7da6,0x7dac,
    0x7db2,0x7db8,0x7dbe,0x7dc4,0x7dc9,0x7dcf,0x7dd5,0x7ddb,0x7de1,0x7de7,
    0x7dec,0x7df2,0x7df8,0x7dfe,0x7e04,0x7e09,0x7e0f,0x7e15,0x7e1b,0x7e20,
    0x7e26,0x7e2c,0x7e32,0x7e37,0x7e3d,0x7e43,0x7e48,0x7e4e,0x7e54,0x7e59,
    0x7e5f,0x7e65,0x7e6a,0x7e70,0x7e75,0x7e7b,0x7e81,0x7e86,0x7e8c,0x7e91,
    0x7e97,0x7e9c,0x7ea2,0x7ea7,0x7ead,0x7eb3,0x7eb8,0x7ebe,0x7ec3,0x7ec9,
    0x7ece,0x7ed3,0x7ed9,0x7ede,0x7ee4,0x7ee9,0x7eef,0x7ef4,0x7efa,0x7eff,
    0x7f04,0x7f0a,0x7f0f,0x7f15,0x7f1a,0x7f1f,0x7f25,0x7f2a,0x7f2f,0x7f35,
    0x7f3a,0x7f3f,0x7f45,0x7f4a,0x7f4f,0x7f54,0x7f5a,0x7f5f,0x7f64,0x7f69,
    0x7f6f,0x7f74,0x7f79,0x7f7e,0x7f84,0x7f89,0x7f8e,0x7f93,0x7f98,0x7f9e,
    0x7fa3,0x7fa8,0x7fad,0x7fb2,0x7fb7,0x7fbc,0x7fc2,0x7fc7,0x7fcc,0x7fd1,
    0x7fd6,0x7fdb,0x7fe0,0x7fe5,0x7fea,0x7fef,0x7ff4,0x7ff9
};

#endif
