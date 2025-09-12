#ifndef LOGGING_H
#define LOGGING_H

#include <freertos/FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

void async_logging_init(BaseType_t priority, BaseType_t core_id);
void async_logging_deinit();

#ifdef __cplusplus
}
#endif

#endif
