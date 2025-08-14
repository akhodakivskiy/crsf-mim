#ifndef LIBCRSF_CRC8_H
#define LIBCRSF_CRC8_H

#include <stdint.h>
#include <stdbool.h>

#include "libcrsf_def.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t crsf_calc_crc8(crsf_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
