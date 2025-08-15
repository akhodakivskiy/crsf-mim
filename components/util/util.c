#include "util.h"

#include <freertos/FreeRTOS.h>



static void _restart_task(void *arg)
{
    uint32_t delay_ms = (uint32_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    esp_restart();
}

void util_reboot_with_delay(uint32_t delay_ms) {
    xTaskCreate(_restart_task, "restart_task", 2048, (void*)delay_ms, 1, NULL);
}
