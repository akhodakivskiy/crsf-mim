#ifndef LOGGING_H
#define LOGGING_H

#include <freertos/FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

void async_logging_init(BaseType_t core_id, UBaseType_t priority);
void async_logging_flush();
void async_logging_deinit();

#ifdef __cplusplus
}
#endif

#endif
