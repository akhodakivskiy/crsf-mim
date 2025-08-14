#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void async_logging_init(uint32_t core_id);
void async_logging_deinit();

#ifdef __cplusplus
}
#endif

#endif
