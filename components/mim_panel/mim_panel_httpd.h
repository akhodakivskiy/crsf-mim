#ifndef MIM_PANEL_HTTPD_H
#define MIM_PANEL_HTTPD_H

#include <esp_err.h>
#include <esp_http_server.h>

#include "sdkconfig.h"

#ifdef CONFIG_MIM_PANEL_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

#define MIM_PANEL_HTTPD_MAX_CLIENTS CONFIG_MIM_PANEL_MAX_CLIENTS

typedef struct mim_panel_httpd_ctx_s *mim_panel_httpd_handle_t;

typedef void (*mim_panel_httpd_cmd_callback_t)(const char *json, size_t len, void *user_ctx);

typedef struct {
    uint16_t port;
    uint8_t max_clients;
    mim_panel_httpd_cmd_callback_t on_command;
    void *user_ctx;
} mim_panel_httpd_config_t;

esp_err_t mim_panel_httpd_init(const mim_panel_httpd_config_t *config, mim_panel_httpd_handle_t *handle);
esp_err_t mim_panel_httpd_deinit(mim_panel_httpd_handle_t handle);
esp_err_t mim_panel_httpd_broadcast(mim_panel_httpd_handle_t handle, const char *json, size_t len);
int mim_panel_httpd_client_count(mim_panel_httpd_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif

#endif
