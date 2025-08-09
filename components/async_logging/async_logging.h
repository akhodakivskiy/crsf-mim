#ifndef LOGGING_H
#define LOGGING_H

#include <sdkconfig.h>

#include <stdbool.h>
#include <stdint.h>
#include <esp_log.h>

#ifndef CONFIG_ASYNC_LOGGING_BUFFER_LEN
#define CONFIG_ASYNC_LOGGING_BUFFER_LEN 4096
#endif

void async_logging_init(uint32_t core_id);
void async_logging_deinit();
bool async_logging_is_enabled();
void async_logging_set_enabled(bool enabled);

#endif
