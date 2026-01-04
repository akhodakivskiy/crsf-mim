#include "mim_panel.h"

#ifdef CONFIG_MIM_PANEL_ENABLED

#include "mim_conn.h"
#include "mim_nav.h"
#include "mim_panel_httpd.h"
#include "mim_settings.h"
#include "nav.h"

#include <cJSON.h>
#include <esp_log.h>
#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "MIM_PANEL";

#define PANEL_TASK_STACK_SIZE 4096
#define JSON_BUFFER_SIZE 1024

struct mim_panel_ctx_s {
    mim_panel_config_t config;
    mim_panel_httpd_handle_t httpd;
    TaskHandle_t task;
    bool running;
};

static void _panel_task(void *arg);
static void _on_command(const char *json, size_t len, void *user_ctx);
static char *_build_state_json(mim_panel_handle_t h);
static void _handle_cmd_engage(mim_panel_handle_t h, cJSON *root);
static void _handle_cmd_settings(mim_panel_handle_t h, cJSON *root);
static void _handle_cmd_save(mim_panel_handle_t h);

esp_err_t mim_panel_init(const mim_panel_config_t *config, mim_panel_handle_t *handle) {
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->nav == NULL || config->conn == NULL) {
        ESP_LOGE(TAG, "nav and conn handles are required");
        return ESP_ERR_INVALID_ARG;
    }

    mim_panel_handle_t ctx = (mim_panel_handle_t)heap_caps_calloc(
        1, sizeof(struct mim_panel_ctx_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(&ctx->config, config, sizeof(mim_panel_config_t));

    mim_panel_httpd_config_t httpd_config
        = {.port = config->http_port, .max_clients = config->max_clients, .on_command = _on_command, .user_ctx = ctx};

    esp_err_t ret = mim_panel_httpd_init(&httpd_config, &ctx->httpd);
    if (ret != ESP_OK) {
        free(ctx);
        return ret;
    }

    ctx->running = true;
    BaseType_t xret = xTaskCreatePinnedToCore(
        _panel_task, "panel", PANEL_TASK_STACK_SIZE, ctx, config->priority, &ctx->task, config->core);

    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create panel task");
        mim_panel_httpd_deinit(ctx->httpd);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Panel initialized, update_rate=%dHz", config->update_rate_hz);

    *handle = ctx;
    return ESP_OK;
}

esp_err_t mim_panel_deinit(mim_panel_handle_t handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->running = false;
    if (handle->task) {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(handle->task);
    }

    mim_panel_httpd_deinit(handle->httpd);

    free(handle);

    ESP_LOGI(TAG, "Panel deinitialized");
    return ESP_OK;
}

bool mim_panel_is_running(mim_panel_handle_t handle) { return handle != NULL && handle->running; }

int mim_panel_get_client_count(mim_panel_handle_t handle) {
    if (handle == NULL)
        return 0;
    return mim_panel_httpd_client_count(handle->httpd);
}

static void _panel_task(void *arg) {
    mim_panel_handle_t h = (mim_panel_handle_t)arg;

    TickType_t delay = pdMS_TO_TICKS(1000 / h->config.update_rate_hz);
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Panel task started, delay=%lu ticks", (unsigned long)delay);

    while (h->running) {
        if (mim_panel_httpd_client_count(h->httpd) > 0) {
            char *json = _build_state_json(h);
            if (json != NULL) {
                mim_panel_httpd_broadcast(h->httpd, json, strlen(json));
                free(json);
            }
        }

        xTaskDelayUntil(&last_wake, delay);
    }

    ESP_LOGI(TAG, "Panel task exiting");
    vTaskDelete(NULL);
}

static char *_build_state_json(mim_panel_handle_t h) {
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
        return NULL;

    int64_t now_us = esp_timer_get_time();

    cJSON_AddStringToObject(root, "type", "state");
    cJSON_AddNumberToObject(root, "ts", (double)(now_us / 1000.0));

    cJSON *status = cJSON_AddObjectToObject(root, "status");
    if (status) {
        bool connected = mim_conn_is_connected(h->config.conn);
        bool engaging = mim_nav_is_engaging(h->config.nav);
        mim_nav_estimate_status_t tgt_status = mim_nav_target_status(h->config.nav);
        mim_nav_estimate_status_t int_status = mim_nav_interceptor_status(h->config.nav);

        cJSON_AddBoolToObject(status, "connected", connected);
        cJSON_AddBoolToObject(status, "engaging", engaging);
        cJSON_AddBoolToObject(status, "targetReady", tgt_status != MIM_NAV_ESTIMATE_STATUS_NONE);
        cJSON_AddBoolToObject(status, "interceptorReady", int_status != MIM_NAV_ESTIMATE_STATUS_NONE);

        const char *int_source = "none";
        if (int_status == MIM_NAV_ESTIMATE_STATUS_SKYMAP)
            int_source = "skymap";
        else if (int_status == MIM_NAV_ESTIMATE_STATUS_CRSF)
            int_source = "crsf";
        cJSON_AddStringToObject(status, "interceptorSource", int_source);
    }

    const nav_state_t *target = mim_nav_get_target(h->config.nav);
    if (target && target->timestamp_us > 0) {
        cJSON *tgt = cJSON_AddObjectToObject(root, "target");
        if (tgt) {
            cJSON_AddNumberToObject(tgt, "lat", target->lat);
            cJSON_AddNumberToObject(tgt, "lon", target->lon);
            cJSON_AddNumberToObject(tgt, "alt", target->alt);
            cJSON_AddNumberToObject(tgt, "velN", target->vel_north);
            cJSON_AddNumberToObject(tgt, "velE", target->vel_east);
            cJSON_AddNumberToObject(tgt, "velU", target->vel_up);
            cJSON_AddNumberToObject(tgt, "ageMs", (now_us - target->timestamp_us) / 1000.0);
        }
    }

    const nav_state_t *interceptor = mim_nav_get_interceptor(h->config.nav);
    if (interceptor && interceptor->timestamp_us > 0) {
        cJSON *intr = cJSON_AddObjectToObject(root, "interceptor");
        if (intr) {
            cJSON_AddNumberToObject(intr, "lat", interceptor->lat);
            cJSON_AddNumberToObject(intr, "lon", interceptor->lon);
            cJSON_AddNumberToObject(intr, "alt", interceptor->alt);
            cJSON_AddNumberToObject(intr, "velN", interceptor->vel_north);
            cJSON_AddNumberToObject(intr, "velE", interceptor->vel_east);
            cJSON_AddNumberToObject(intr, "velU", interceptor->vel_up);
            cJSON_AddNumberToObject(intr, "ageMs", (now_us - interceptor->timestamp_us) / 1000.0);
        }
    }

    const nav_command_t *cmd = mim_nav_get_last_command(h->config.nav);
    if (cmd) {
        cJSON *guidance = cJSON_AddObjectToObject(root, "guidance");
        if (guidance) {
            const char *nav_type = "none";
            if (cmd->type == NAV_PURSUIT)
                nav_type = "pursuit";
            else if (cmd->type == NAV_PRONAV)
                nav_type = "pronav";

            cJSON_AddStringToObject(guidance, "type", nav_type);
            cJSON_AddNumberToObject(guidance, "range", cmd->range);
            cJSON_AddNumberToObject(guidance, "rangeHor", cmd->range_hor);
            cJSON_AddNumberToObject(guidance, "rangeVer", cmd->range_ver);
            cJSON_AddNumberToObject(guidance, "accelLat", cmd->accel_lat);
            cJSON_AddNumberToObject(guidance, "accelVer", cmd->accel_ver);
            cJSON_AddNumberToObject(guidance, "ttg", cmd->time_to_go_s);
            cJSON_AddNumberToObject(guidance, "zem", cmd->zero_effort_miss_m);
        }
    }

    cJSON *network = cJSON_AddObjectToObject(root, "network");
    if (network) {
        ip4_addr_t mim_addr = mim_conn_get_mim_addr(h->config.conn);
        ip4_addr_t skymap_addr = mim_conn_get_skymap_addr(h->config.conn);

        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&mim_addr));
        cJSON_AddStringToObject(network, "mimIp", ip_str);

        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&skymap_addr));
        cJSON_AddStringToObject(network, "skymapIp", ip_str);
    }

    const mim_settings_t *settings = mim_settings_get();
    if (settings) {
        cJSON *cfg = cJSON_AddObjectToObject(root, "settings");
        if (cfg) {
            cJSON *nav = cJSON_AddObjectToObject(cfg, "nav");
            if (nav) {
                cJSON_AddNumberToObject(nav, "N", settings->nav.N);
                cJSON_AddNumberToObject(nav, "maxRollDeg", settings->nav.max_roll_deg);
                cJSON_AddNumberToObject(nav, "attackAngleDeg", settings->nav.attack_angle_deg);
                cJSON_AddNumberToObject(nav, "attackFactor", settings->nav.attack_factor);
            }

            cJSON *pitcher = cJSON_AddObjectToObject(cfg, "pitcher");
            if (pitcher) {
                cJSON_AddNumberToObject(pitcher, "kp", settings->pitcher.kp);
                cJSON_AddNumberToObject(pitcher, "ki", settings->pitcher.ki);
                cJSON_AddNumberToObject(pitcher, "kd", settings->pitcher.kd);
                cJSON_AddNumberToObject(pitcher, "maxRate", settings->pitcher.max_rate);
                cJSON_AddBoolToObject(pitcher, "inverted", settings->pitcher.inverted);
            }
        }
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json_str;
}

static void _on_command(const char *json, size_t len, void *user_ctx) {
    mim_panel_handle_t h = (mim_panel_handle_t)user_ctx;

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        ESP_LOGW(TAG, "Failed to parse command JSON");
        return;
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type)) {
        ESP_LOGW(TAG, "Command missing 'type' field");
        cJSON_Delete(root);
        return;
    }

    const char *type_str = type->valuestring;

    if (strcmp(type_str, "cmd") == 0) {
        cJSON *action = cJSON_GetObjectItem(root, "action");
        if (cJSON_IsString(action)) {
            if (strcmp(action->valuestring, "engage") == 0) {
                _handle_cmd_engage(h, root);
            } else if (strcmp(action->valuestring, "save_settings") == 0) {
                _handle_cmd_save(h);
            }
        }
    } else if (strcmp(type_str, "settings") == 0) {
        _handle_cmd_settings(h, root);
    }

    cJSON_Delete(root);
}

static void _handle_cmd_engage(mim_panel_handle_t h, cJSON *root) {
    cJSON *value = cJSON_GetObjectItem(root, "value");
    if (!cJSON_IsBool(value)) {
        ESP_LOGW(TAG, "engage command missing 'value'");
        return;
    }

    bool engage = cJSON_IsTrue(value);
    mim_nav_set_engaging(h->config.nav, engage);

    ESP_LOGI(TAG, "Engage command: %s", engage ? "ON" : "OFF");
}

static void _handle_cmd_settings(mim_panel_handle_t h, cJSON *root) {
    cJSON *nav = cJSON_GetObjectItem(root, "nav");
    if (cJSON_IsObject(nav)) {
        cJSON *item;

        item = cJSON_GetObjectItem(nav, "N");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_N((float)item->valuedouble);
        }

        item = cJSON_GetObjectItem(nav, "maxRollDeg");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_max_roll_deg((uint8_t)item->valuedouble);
        }

        item = cJSON_GetObjectItem(nav, "attackAngleDeg");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_attack_angle_deg((uint8_t)item->valuedouble);
        }

        item = cJSON_GetObjectItem(nav, "attackFactor");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_attack_factor((float)item->valuedouble);
        }
    }

    cJSON *pitcher = cJSON_GetObjectItem(root, "pitcher");
    if (cJSON_IsObject(pitcher)) {
        cJSON *item;

        item = cJSON_GetObjectItem(pitcher, "kp");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_pitcher_p_gain((float)item->valuedouble);
        }

        item = cJSON_GetObjectItem(pitcher, "ki");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_pitcher_i_gain((float)item->valuedouble);
        }

        item = cJSON_GetObjectItem(pitcher, "kd");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_pitcher_d_gain((float)item->valuedouble);
        }

        item = cJSON_GetObjectItem(pitcher, "maxRate");
        if (cJSON_IsNumber(item)) {
            mim_settings_set_nav_pitcher_max_rate((float)item->valuedouble);
        }

        item = cJSON_GetObjectItem(pitcher, "inverted");
        if (cJSON_IsBool(item)) {
            mim_settings_set_nav_pitcher_inverted(cJSON_IsTrue(item));
        }
    }

    ESP_LOGI(TAG, "Settings updated from panel");
}

static void _handle_cmd_save(mim_panel_handle_t h) {
    esp_err_t ret = mim_settings_save();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Settings saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save settings: %s", esp_err_to_name(ret));
    }
}

#endif
