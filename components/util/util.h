#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#define ARRAY_LENGTH(A) (sizeof(A) / sizeof(A[0]))

void util_reboot_with_delay(uint32_t delay_ms);

#endif
