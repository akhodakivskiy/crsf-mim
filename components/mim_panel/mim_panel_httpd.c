#include "mim_panel_httpd.h"

#ifdef CONFIG_MIM_PANEL_ENABLED

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_spiffs.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "MIM_PANEL_HTTPD";

#define SCRATCH_BUFSIZE 4096
#define WS_MAX_FRAME_SIZE 1024
#define FILE_PATH_MAX 128
#define BASE_PATH "/www"

typedef struct {
    const char *ext;
    const char *mime;
} mime_map_t;

static const mime_map_t mime_types[] = {{".html", "text/html"},
                                        {".js", "application/javascript"},
                                        {".css", "text/css"},
                                        {".json", "application/json"},
                                        {".png", "image/png"},
                                        {".ico", "image/x-icon"},
                                        {".svg", "image/svg+xml"},
                                        {NULL, "application/octet-stream"}};

typedef struct {
    int fd;
    bool active;
} ws_client_t;

struct mim_panel_httpd_ctx_s {
    httpd_handle_t server;
    mim_panel_httpd_config_t config;
    ws_client_t clients[MIM_PANEL_HTTPD_MAX_CLIENTS];
    char scratch[SCRATCH_BUFSIZE];
};

static esp_err_t _init_spiffs(void);
static esp_err_t _file_get_handler(httpd_req_t *req);
static esp_err_t _ws_handler(httpd_req_t *req);
static const char *_get_mime_type(const char *path);
static void _ws_async_send(void *arg);

static esp_err_t _init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf
        = {.base_path = BASE_PATH, .partition_label = "www", .max_files = 5, .format_if_mount_failed = false};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition not found");
        } else {
            ESP_LOGE(TAG, "SPIFFS init failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("www", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: total=%zu, used=%zu", total, used);
    }

    return ESP_OK;
}

static const char *_get_mime_type(const char *path) {
    const char *ext = strrchr(path, '.');
    if (ext == NULL) {
        return "application/octet-stream";
    }

    for (int i = 0; mime_types[i].ext != NULL; i++) {
        if (strcasecmp(ext, mime_types[i].ext) == 0) {
            return mime_types[i].mime;
        }
    }
    return "application/octet-stream";
}

static esp_err_t _file_get_handler(httpd_req_t *req) {
    mim_panel_httpd_handle_t ctx = (mim_panel_httpd_handle_t)req->user_ctx;

    char filepath[FILE_PATH_MAX];
    const char *uri = req->uri;

    if (strcmp(uri, "/") == 0) {
        uri = "/index.html";
    }

    snprintf(filepath, sizeof(filepath), "%s%s", BASE_PATH, uri);

    if (strstr(filepath, "..") != NULL) {
        ESP_LOGW(TAG, "Rejecting path with '..': %s", filepath);
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Forbidden");
        return ESP_FAIL;
    }

    int fd = open(filepath, O_RDONLY);
    if (fd < 0) {
        snprintf(filepath, sizeof(filepath), "%s/index.html", BASE_PATH);
        fd = open(filepath, O_RDONLY);
        if (fd < 0) {
            ESP_LOGW(TAG, "File not found: %s", req->uri);
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
            return ESP_FAIL;
        }
    }

    const char *mime = _get_mime_type(filepath);
    httpd_resp_set_type(req, mime);

    if (strstr(filepath, ".js") || strstr(filepath, ".css")) {
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    }

    ssize_t read_bytes;
    while ((read_bytes = read(fd, ctx->scratch, SCRATCH_BUFSIZE)) > 0) {
        if (httpd_resp_send_chunk(req, ctx->scratch, read_bytes) != ESP_OK) {
            close(fd);
            ESP_LOGE(TAG, "File send failed");
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }
    }

    close(fd);
    httpd_resp_send_chunk(req, NULL, 0);

    ESP_LOGD(TAG, "Served: %s (%s)", req->uri, mime);
    return ESP_OK;
}

typedef struct {
    httpd_handle_t hd;
    int fd;
    char *data;
    size_t len;
} ws_send_arg_t;

static void _ws_async_send(void *arg) {
    ws_send_arg_t *a = (ws_send_arg_t *)arg;

    httpd_ws_frame_t ws_pkt = {.payload = (uint8_t *)a->data, .len = a->len, .type = HTTPD_WS_TYPE_TEXT};

    httpd_ws_send_frame_async(a->hd, a->fd, &ws_pkt);

    free(a->data);
    free(a);
}

esp_err_t mim_panel_httpd_broadcast(mim_panel_httpd_handle_t handle, const char *json, size_t len) {
    if (handle == NULL || handle->server == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < MIM_PANEL_HTTPD_MAX_CLIENTS; i++) {
        if (handle->clients[i].active) {
            ws_send_arg_t *arg = malloc(sizeof(ws_send_arg_t));
            if (arg == NULL)
                continue;

            arg->data = malloc(len);
            if (arg->data == NULL) {
                free(arg);
                continue;
            }

            memcpy(arg->data, json, len);
            arg->len = len;
            arg->hd = handle->server;
            arg->fd = handle->clients[i].fd;

            if (httpd_queue_work(handle->server, _ws_async_send, arg) != ESP_OK) {
                free(arg->data);
                free(arg);
            }
        }
    }

    return ESP_OK;
}

int mim_panel_httpd_client_count(mim_panel_httpd_handle_t handle) {
    if (handle == NULL)
        return 0;

    int count = 0;
    for (int i = 0; i < MIM_PANEL_HTTPD_MAX_CLIENTS; i++) {
        if (handle->clients[i].active)
            count++;
    }
    return count;
}

static esp_err_t _ws_handler(httpd_req_t *req) {
    mim_panel_httpd_handle_t ctx = (mim_panel_httpd_handle_t)req->user_ctx;

    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake, fd=%d", fd);

        for (int i = 0; i < MIM_PANEL_HTTPD_MAX_CLIENTS; i++) {
            if (!ctx->clients[i].active) {
                ctx->clients[i].fd = fd;
                ctx->clients[i].active = true;
                ESP_LOGI(TAG, "WebSocket client added, slot=%d, fd=%d", i, fd);
                break;
            }
        }

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame len: %s", esp_err_to_name(ret));
        return ret;
    }

    if (ws_pkt.len == 0) {
        return ESP_OK;
    }

    if (ws_pkt.len > WS_MAX_FRAME_SIZE) {
        ESP_LOGW(TAG, "Frame too large: %zu", ws_pkt.len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = malloc(ws_pkt.len + 1);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate WS buffer");
        return ESP_ERR_NO_MEM;
    }

    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive frame: %s", esp_err_to_name(ret));
    } else {
        buf[ws_pkt.len] = '\0';

        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
            int fd = httpd_req_to_sockfd(req);
            ESP_LOGI(TAG, "WebSocket close, fd=%d", fd);

            for (int i = 0; i < MIM_PANEL_HTTPD_MAX_CLIENTS; i++) {
                if (ctx->clients[i].active && ctx->clients[i].fd == fd) {
                    ctx->clients[i].active = false;
                    ESP_LOGI(TAG, "WebSocket client removed, slot=%d", i);
                    break;
                }
            }
        }

        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            ESP_LOGD(TAG, "WS received: %s", (char *)buf);

            if (ctx->config.on_command) {
                ctx->config.on_command((const char *)buf, ws_pkt.len, ctx->config.user_ctx);
            }
        }
    }

    free(buf);
    return ret;
}

esp_err_t mim_panel_httpd_init(const mim_panel_httpd_config_t *config, mim_panel_httpd_handle_t *handle) {
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = _init_spiffs();
    if (ret != ESP_OK) {
        return ret;
    }

    mim_panel_httpd_handle_t ctx = calloc(1, sizeof(struct mim_panel_httpd_ctx_s));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(&ctx->config, config, sizeof(mim_panel_httpd_config_t));

    for (int i = 0; i < MIM_PANEL_HTTPD_MAX_CLIENTS; i++) {
        ctx->clients[i].active = false;
    }

    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    httpd_config.server_port = config->port;
    httpd_config.uri_match_fn = httpd_uri_match_wildcard;
    httpd_config.max_open_sockets = config->max_clients + 2;

    ret = httpd_start(&ctx->server, &httpd_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        free(ctx);
        return ret;
    }

    httpd_uri_t ws_uri = {.uri = "/ws",
                          .method = HTTP_GET,
                          .handler = _ws_handler,
                          .user_ctx = ctx,
                          .is_websocket = true,
                          .handle_ws_control_frames = true};
    httpd_register_uri_handler(ctx->server, &ws_uri);

    httpd_uri_t file_uri = {.uri = "/*", .method = HTTP_GET, .handler = _file_get_handler, .user_ctx = ctx};
    httpd_register_uri_handler(ctx->server, &file_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config->port);

    *handle = ctx;
    return ESP_OK;
}

esp_err_t mim_panel_httpd_deinit(mim_panel_httpd_handle_t handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->server) {
        httpd_stop(handle->server);
    }

    esp_vfs_spiffs_unregister("www");

    free(handle);

    ESP_LOGI(TAG, "HTTP server stopped");
    return ESP_OK;
}

#endif
