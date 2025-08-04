#ifndef TOOLS_H
#define TOOLS_H

#include <sys/types.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define REPEAT_1(FN) FN(1)
#define REPEAT_2(FN) REPEAT_1(FN) , FN(2)
#define REPEAT_3(FN) REPEAT_2(FN) , FN(3)
#define REPEAT_4(FN) REPEAT_3(FN) , FN(4)
#define REPEAT__(FN, N) REPEAT_##N(FN)
#define REPEAT(FN, N) REPEAT__(FN, N)

inline int32_t int32_constrain(int32_t x, int32_t min, int32_t max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

inline int32_t int16_constrain(int16_t x, int16_t min, int16_t max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

inline int32_t int32_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    int32_t value = (((x - in_min) * (out_max - out_min) * 2 / (in_max - in_min)) + (out_min * 2) + 1) / 2;
    return int32_constrain(value, out_min, out_max);
}

inline int32_t int32_max(int32_t a, int32_t b) {
    return((a) > (b) ? (a) : (b));
}

inline int32_t int32_min(int32_t a, int32_t b) {
    return((a) < (b) ? (a) : (b));
}

inline uint32_t uint32_max(uint32_t a, uint32_t b) {
    return((a) > (b) ? (a) : (b));
}

inline uint32_t uint32_min(uint32_t a, uint32_t b) {
    return((a) < (b) ? (a) : (b));
}

#endif
