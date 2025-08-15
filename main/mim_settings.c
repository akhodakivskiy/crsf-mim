#include "mim_settings.h"
#include "sdkconfig.h"

#define SETTINGS_NAMESPACE "mim_settings"
#define SETTINGS_KEY "mim_config"

#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <string.h>

static const char* TAG = "mim_settings";

static mim_settings_t mim_settings_current;
static bool mim_settings_initialized = false;

static const mim_settings_t mim_settings_default = {
    .mode = MIM_SETTINGS_MODE_ETHERNET,
    .wifi_ssid = CONFIG_CRSF_MIM_WIFI_SSID,
    .wifi_password = CONFIG_CRSF_MIM_WIFI_PASSWORD,
    .skymap_udp_port = 8888,
};

esp_err_t mim_settings_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize with defaults
    memcpy(&mim_settings_current, &mim_settings_default, sizeof(mim_settings_t));
    mim_settings_initialized = true;

    ESP_LOGI(TAG, "Settings system initialized");
    return ESP_OK;
}

esp_err_t mim_settings_load(void) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No saved settings found, using defaults");
        return ESP_OK;
    }

    size_t required_size = sizeof(mim_settings_t);
    ret = nvs_get_blob(nvs_handle, SETTINGS_KEY, &mim_settings_current, &required_size);

    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Settings loaded from flash");
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved settings found, using defaults");
        ret = ESP_OK;
    } else {
        ESP_LOGE(TAG, "Error loading settings: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mim_settings_save(void) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(nvs_handle, SETTINGS_KEY, &mim_settings_current, sizeof(mim_settings_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving settings: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Settings saved to flash");
    } else {
        ESP_LOGE(TAG, "Error committing settings: %s", esp_err_to_name(ret));
    }

    return ret;
}

const mim_settings_t* mim_settings_get(void) {
    if (!mim_settings_initialized) {
        return NULL;
    }
    return &mim_settings_current;
}

esp_err_t mim_settings_set_mode(mim_settings_mode_t mode) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    mim_settings_current.mode = mode;
    return ESP_OK;
}

esp_err_t mim_settings_set_wifi(const char* ssid, const char* password) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(mim_settings_current.wifi_ssid, ssid, sizeof(mim_settings_current.wifi_ssid) - 1);
    mim_settings_current.wifi_ssid[sizeof(mim_settings_current.wifi_ssid) - 1] = '\0';

    strncpy(mim_settings_current.wifi_password, password, sizeof(mim_settings_current.wifi_password) - 1);
    mim_settings_current.wifi_password[sizeof(mim_settings_current.wifi_password) - 1] = '\0';

    return ESP_OK;
}

esp_err_t mim_settings_set_skymap_udp_port(uint16_t port) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    mim_settings_current.skymap_udp_port = port;
    return ESP_OK;
}

esp_err_t mim_settings_reset_to_defaults(void) {
    if (!mim_settings_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&mim_settings_current, &mim_settings_default, sizeof(mim_settings_t));
    ESP_LOGI(TAG, "Settings reset to defaults");
    return ESP_OK;
}
