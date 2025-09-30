#include "async_logging.h"
#include "portmacro.h"
#include <sdkconfig.h>

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char *TAG = "ASYNC_LOGGING";

#ifndef CONFIG_ASYNC_LOGGING_BUFFER_LEN
#define CONFIG_ASYNC_LOGGING_BUFFER_LEN 4096
#endif

#if CONFIG_ASYNC_LOGGING_ENABLED == 1

vprintf_like_t _defalut_vprintf = NULL;
RingbufHandle_t _buffer;
TaskHandle_t _logging_task_handle;
bool _logging_init = false;

void IRAM_ATTR _logging_loop(void *arg) {
    char *data;
    size_t size;
    while (_logging_init) {
        data = reinterpret_cast<char *>(xRingbufferReceive(_buffer, &size, portMAX_DELAY));
        for (int i = 0; i < size; i++) {
            putchar(data[i]);
        }
        vRingbufferReturnItem(_buffer, data);
        taskYIELD();
    }
}

int IRAM_ATTR _logging_vprintf(const char *format, va_list arg) {
    // measure line length
    va_list argCopy;
    va_copy(argCopy, arg);
    int len = vsnprintf(NULL, 0, format, argCopy);
    va_end(argCopy);

    // write to buffer
    char *lineBuf = (char *)malloc(len + 1);

    len = vsnprintf(lineBuf, len + 1, format, arg);
    xRingbufferSend(_buffer, lineBuf, len, 0);

    free(lineBuf);

    va_end(arg);

    return len;
}
#endif

void async_logging_init(BaseType_t priority, BaseType_t core_id) {
#if CONFIG_ASYNC_LOGGING_ENABLED == 1
    assert(!_logging_init);
    _logging_init = true;
    _buffer = xRingbufferCreate(CONFIG_ASYNC_LOGGING_BUFFER_LEN, RINGBUF_TYPE_BYTEBUF);
    xTaskCreatePinnedToCore(_logging_loop, "task_logging", 4096, NULL, priority, &_logging_task_handle, core_id);
    _defalut_vprintf = esp_log_set_vprintf(_logging_vprintf);
    ESP_LOGI(TAG, "async logging init");
#endif
}

void async_logging_flush() {
#if CONFIG_ASYNC_LOGGING_ENABLED == 1
    assert(_logging_init);
    while (xRingbufferGetCurFreeSize(_buffer) < CONFIG_ASYNC_LOGGING_BUFFER_LEN) {
        vTaskDelay(1);
    }
#endif
}

void async_logging_deinit() {
#if CONFIG_ASYNC_LOGGING_ENABLED == 1
    assert(_logging_init);
    _logging_init = false;
    vTaskDelete(_logging_task_handle);
    esp_log_set_vprintf(_defalut_vprintf);
    ESP_LOGI(TAG, "async logging deinit");
#endif
}
