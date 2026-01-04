#ifndef MIM_PANEL_H
#define MIM_PANEL_H

#include "sdkconfig.h"

#ifdef CONFIG_MIM_PANEL_ENABLED

#include "mim_conn.h"
#include "mim_nav.h"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mim_panel_ctx_s *mim_panel_handle_t;

typedef struct {
    mim_nav_handle_t nav;
    mim_conn_handle_t conn;
    uint16_t http_port;
    uint8_t update_rate_hz;
    uint8_t max_clients;
    BaseType_t core;
    UBaseType_t priority;
} mim_panel_config_t;

#define MIM_PANEL_CONFIG_DEFAULT()                                                                                     \
    {                                                                                                                  \
        .nav = NULL,                                                                                                   \
        .conn = NULL,                                                                                                  \
        .http_port = CONFIG_MIM_PANEL_HTTP_PORT,                                                                       \
        .update_rate_hz = CONFIG_MIM_PANEL_WS_UPDATE_RATE_HZ,                                                          \
        .max_clients = CONFIG_MIM_PANEL_MAX_CLIENTS,                                                                   \
        .core = PRO_CPU_NUM,                                                                                           \
        .priority = 5,                                                                                                 \
    }

esp_err_t mim_panel_init(const mim_panel_config_t *config, mim_panel_handle_t *handle);
esp_err_t mim_panel_deinit(mim_panel_handle_t handle);
bool mim_panel_is_running(mim_panel_handle_t handle);
int mim_panel_get_client_count(mim_panel_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif

#endif
