#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#define UTIL_ARRAY_LENGTH(A) (sizeof(A) / sizeof(A[0]))

#define UTIL_CLAMP(A, MIN, MAX) ((A < MIN) ? MIN : ((A > MAX) ? MAX : A))

void util_reboot_with_delay(uint32_t delay_ms);

#endif
