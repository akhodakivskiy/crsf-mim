#include "libnet.h"
#include "esp_err.h"
#include "esp_eth_netif_glue.h"
#include "esp_wifi_default.h"
#include "freertos/projdefs.h"

#include <esp_wifi.h>
#include <esp_eth.h>
#include <esp_eth_driver.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <lwip/ip4_addr.h>
#include <lwip/ip_addr.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <lwip/sockets.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <sdkconfig.h>

static const char* TAG = "LIBNET";

struct libnet_ctx_s {
    libnet_config_t config;

    esp_netif_t* netif;
    union {
        struct {
        } wifi;
        struct {
            bool isr_service_installed;
            spi_device_handle_t spi_handle;
            esp_eth_mac_t *mac;
            esp_eth_phy_t *phy;
            esp_eth_handle_t eth_handle;
            esp_eth_netif_glue_handle_t eth_glue_handle;
        } eth;
    } net;
    int udp_server_socket;
    TaskHandle_t udp_server_task;

    bool initialized;
    bool connected;
    bool udp_server_running;
    ip4_addr_t ip_address;
};

// Forward declarations
static void _wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void _eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void _ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void _udp_server_task(void* pvParameters);
static esp_err_t _init_wifi(libnet_handle_t h);
static esp_err_t _init_ethernet(libnet_handle_t h);

esp_err_t libnet_init(const libnet_config_t* config, libnet_handle_t *handle) {
    if (!config) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    libnet_handle_t h = (libnet_handle_t) heap_caps_calloc(1, sizeof(struct libnet_ctx_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    memset(h, 0, sizeof(struct libnet_ctx_s));

    memcpy(&h->config, config, sizeof(libnet_config_t));
    h->udp_server_socket = -1;
    h->connected = false;
    ip4_addr_set_u32(&h->ip_address, 0);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    switch (config->interface) {
        case LIBNET_INTERFACE_WIFI:
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &_ip_event_handler, h));
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &_ip_event_handler, h));
            ESP_ERROR_CHECK(_init_wifi(h));
            ESP_LOGI(TAG, "Initialized with WiFi interface, ssid=%s", h->config.net.wifi.ssid);
            break;
        case LIBNET_INTERFACE_ETHERNET:
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &_ip_event_handler, h));
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &_ip_event_handler, h));
            ESP_ERROR_CHECK(_init_ethernet(h));
            ESP_LOGI(TAG, "Initialized with Ethernet interface");
            break;
    }

    h->initialized = true;

    *handle = h;

    return ESP_OK;
}

esp_err_t libnet_deinit(libnet_handle_t h) {
    if (!h->initialized) {
        ESP_LOGE(TAG, "not initialized");
        return ESP_ERR_INVALID_ARG;
    }


    switch (h->config.interface) {
        case LIBNET_INTERFACE_WIFI:
            ESP_ERROR_CHECK(esp_wifi_stop());

            esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &_ip_event_handler);
            esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_LOST_IP, &_ip_event_handler);
            esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler);

            ESP_ERROR_CHECK(esp_wifi_deinit());

            esp_netif_destroy_default_wifi(h->netif);
            break;
        case LIBNET_INTERFACE_ETHERNET:
            ESP_ERROR_CHECK(esp_eth_stop(h->net.eth.eth_handle));
            vTaskDelay(pdMS_TO_TICKS(100));

            esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &_ip_event_handler);
            esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_LOST_IP, &_ip_event_handler);
            esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &_eth_event_handler);

            ESP_ERROR_CHECK(esp_eth_del_netif_glue(h->net.eth.eth_glue_handle));
            esp_netif_destroy(h->netif);
            ESP_ERROR_CHECK(esp_eth_driver_uninstall(h->net.eth.eth_handle));
            h->net.eth.phy->del(h->net.eth.phy);
            h->net.eth.mac->del(h->net.eth.mac);
            ESP_ERROR_CHECK(spi_bus_remove_device(h->net.eth.spi_handle));
            spi_bus_free(SPI2_HOST);
            if (h->net.eth.isr_service_installed) {
                gpio_uninstall_isr_service();
            }
            break;
    }

    esp_netif_deinit();
    ESP_ERROR_CHECK(esp_event_loop_delete_default());

    h->initialized = false;

    heap_caps_free(h);

    return ESP_OK;
}

esp_err_t libnet_udp_server_start(libnet_handle_t h, uint16_t port) {
    if (h->udp_server_running) {
        ESP_LOGW(TAG, "UDP server already running");
        return ESP_OK;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return ESP_FAIL;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket to port %d", port);
        close(sock);
        return ESP_FAIL;
    }

    h->udp_server_socket = sock;
    h->udp_server_running = true;

    xTaskCreatePinnedToCore(_udp_server_task, "task_udp_server", 4096, h, h->config.priority, &h->udp_server_task, h->config.core_id);
    ESP_LOGI(TAG, "UDP server started on port %d", port);
    return ESP_OK;
}

esp_err_t libnet_udp_server_stop(libnet_handle_t h) {
    if (!h->udp_server_running) {
        ESP_LOGE(TAG, "Server not running");
        return ESP_ERR_INVALID_STATE;
    }

    h->udp_server_running = false;

    if (h->udp_server_socket >= 0) {
        close(h->udp_server_socket);
        h->udp_server_socket = -1;
    }

    if (h->udp_server_task) {
        vTaskDelete(h->udp_server_task);
        h->udp_server_task = NULL;
    }

    ESP_LOGI(TAG, "UDP server stopped");
    return ESP_OK;
}

esp_err_t IRAM_ATTR libnet_udp_send(libnet_handle_t h, const ip4_addr_t *dest_ip, uint16_t dest_port, const uint8_t* data, size_t len) {
    if (!h->connected) {
        ESP_LOGE(TAG, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = dest_ip->addr;
    dest_addr.sin_port = htons(dest_port);

    int sent = sendto(h->udp_server_socket, data, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));

    if (sent < 0) {
        ESP_LOGE(TAG, "Failed to send UDP packet");
        return ESP_FAIL;
    }


    ESP_LOGD(TAG, "Sent %d bytes to " IPSTR ":%d", sent, IP2STR(dest_ip), dest_port);
    return ESP_OK;
}

esp_err_t libnet_udp_send_broadcast(libnet_handle_t h, uint16_t dest_port, const uint8_t* data, size_t len) {
    return libnet_udp_send(h, IP4_ADDR_BROADCAST, dest_port, data, len);
}

bool libnet_is_connected(libnet_handle_t h) {
    return h->connected;
}

void libnet_get_ip_address(libnet_handle_t h, ip4_addr_t *addr) {
    memcpy(addr, &h->ip_address, sizeof(ip4_addr_t));
}

/* internal functions implementation */

static esp_err_t _init_wifi(libnet_handle_t h) {
    h->netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler, h));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strcpy((char*)wifi_config.sta.ssid, h->config.net.wifi.ssid);
    strcpy((char*)wifi_config.sta.password, h->config.net.wifi.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

static esp_err_t _init_ethernet(libnet_handle_t h) {
    h->net.eth.isr_service_installed = 
        (gpio_install_isr_service(0) == ESP_OK);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    h->netif = esp_netif_new(&netif_cfg);

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_LIBNET_W5500_SPI_MISO,
        .mosi_io_num = CONFIG_LIBNET_W5500_SPI_MOSI,
        .sclk_io_num = CONFIG_LIBNET_W5500_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configure W5500
    spi_device_interface_config_t devcfg = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = 12 * 1000 * 1000,
        .spics_io_num = CONFIG_LIBNET_W5500_SPI_CS,
        .queue_size = 20
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &h->net.eth.spi_handle));

    // Configure W5500 MAC and PHY
    eth_w5500_config_t w5500_config = {
        .int_gpio_num = CONFIG_LIBNET_W5500_SPI_INT,
        .poll_period_ms = 0,
        .spi_host_id = SPI2_HOST,
        .spi_devcfg = &devcfg,
        .custom_spi_driver = ETH_DEFAULT_SPI,
    };
    w5500_config.int_gpio_num = CONFIG_LIBNET_W5500_SPI_INT;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    h->net.eth.mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    h->net.eth.phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(h->net.eth.mac, h->net.eth.phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &h->net.eth.eth_handle));
    ESP_LOGI(TAG, "ETH driver installed");

    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(base_mac_addr));
    uint8_t local_mac[ETH_ADDR_LEN];
    esp_derive_local_mac(local_mac, base_mac_addr);

    ESP_ERROR_CHECK(esp_eth_ioctl(h->net.eth.eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));

    ESP_LOGI(TAG, "ETH MAC address: " MACSTR, MAC2STR(local_mac));

    h->net.eth.eth_glue_handle = esp_eth_new_netif_glue(h->net.eth.eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(h->netif, h->net.eth.eth_glue_handle));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &_eth_event_handler, h));

    ESP_ERROR_CHECK(esp_eth_start(h->net.eth.eth_handle));

    ESP_LOGI(TAG, "ETH interface started");

    return ESP_OK;
}


// Event handlers
static void _wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    libnet_handle_t h = (libnet_handle_t)arg;

    static uint16_t retry_num = 0;

    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        retry_num++;
        ESP_LOGI(TAG, "Connecting to AP, retry #%u", retry_num);

        if (h->connected) {
            if (h->config.callbacks.disconnected) {
                h->config.callbacks.disconnected(h->config.user_ctx);
            }
            h->connected = false;
        }
        ip4_addr_set_u32(&h->ip_address, 0);
    }
}

static void _eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    libnet_handle_t h = (libnet_handle_t)arg;

    if (event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Up");
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
        h->connected = false;
        ip4_addr_set_u32(&h->ip_address, 0);
        if (h->config.callbacks.disconnected) {
            h->config.callbacks.disconnected(h->config.user_ctx);
        }
    }
}

static void _ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    libnet_handle_t h = (libnet_handle_t)arg;

    switch (event_id) {
        case IP_EVENT_STA_GOT_IP:
        case IP_EVENT_ETH_GOT_IP: {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            h->connected = true;
            ip4_addr_set_u32(&h->ip_address, event->ip_info.ip.addr);
            if (h->config.callbacks.connected) {
                h->config.callbacks.connected(h->config.user_ctx, &h->ip_address);
            }
            break;
        }
        case IP_EVENT_STA_LOST_IP:
        case IP_EVENT_ETH_LOST_IP: {
            ESP_LOGI(TAG, "Lost IP");
            h->connected = false;
            ip4_addr_set_u32(&h->ip_address, 0);
            if (h->config.callbacks.disconnected) {
                h->config.callbacks.disconnected(h->config.user_ctx);
            }
            break;
        }
    }
}

static void _udp_server_task(void* arg) {
    libnet_handle_t h = (libnet_handle_t)arg;

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    uint8_t packet_data[LIBNET_MAX_PACKET_SIZE];

    while (h->udp_server_running) {
        int packet_len = recvfrom(h->udp_server_socket, 
                                  packet_data, 
                                  LIBNET_MAX_PACKET_SIZE, 
                                  0,
                                  (struct sockaddr*)&client_addr, 
                                  &client_addr_len);

        if (packet_len > 0) {
            //packet.len = received;
            //packet.src_ip = client_addr.sin_addr.s_addr;
            //packet.src_port = ntohs(client_addr.sin_port);

            ip4_addr_t ip;
            ip4_addr_set_u32(&ip, client_addr.sin_addr.s_addr);
            uint16_t port = ntohs(client_addr.sin_port);

            //ESP_LOGI(TAG, "Received %d bytes from " IPSTR ": %u", packet_len, IP2STR(&ip), port);

            if (h->config.callbacks.packet) {
                h->config.callbacks.packet(h->config.user_ctx, packet_data, packet_len, &ip, port);
            }
        } else if (packet_len < 0 && errno != EAGAIN) {
            ESP_LOGE(TAG, "UDP receive error: %d", errno);
            break;
        }
    }

    ESP_LOGI(TAG, "UDP server task ended");
    vTaskDelete(NULL);
}
