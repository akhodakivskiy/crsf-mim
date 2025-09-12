#include "libnet.h"

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

typedef struct {
    libnet_config_t config;
    esp_netif_t* netif;
    esp_eth_handle_t eth_handle;
    esp_eth_netif_glue_handle_t eth_glue_handle;
    int udp_server_socket;
    TaskHandle_t udp_server_task;
    bool initialized;
    bool connected;
    bool udp_server_running;
    ip4_addr_t ip_address;
} libnet_context_t;

static libnet_context_t s_libnet_ctx = {0};

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void udp_server_task(void* pvParameters);
static esp_err_t init_wifi(void);
static esp_err_t init_ethernet(void);

esp_err_t libnet_init(const libnet_config_t* config) {
    if (s_libnet_ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_libnet_ctx.config, config, sizeof(libnet_config_t));
    s_libnet_ctx.udp_server_socket = -1;
    s_libnet_ctx.connected = false;
    ip4_addr_set_u32(&s_libnet_ctx.ip_address, 0);

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
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &ip_event_handler, NULL));
            ESP_ERROR_CHECK(init_wifi());
            ESP_LOGI(TAG, "Initialized with WiFi interface, ssid=%s", s_libnet_ctx.config.net.wifi.ssid);
            break;
        case LIBNET_INTERFACE_ETHERNET:
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, NULL));
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &ip_event_handler, NULL));
            ESP_ERROR_CHECK(init_ethernet());
            ESP_LOGI(TAG, "Initialized with Ethernet interface");
            break;
    }

    s_libnet_ctx.initialized = true;

    return ESP_OK;
}

static esp_err_t init_wifi(void) {
    s_libnet_ctx.netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strcpy((char*)wifi_config.sta.ssid, s_libnet_ctx.config.net.wifi.ssid);
    strcpy((char*)wifi_config.sta.password, s_libnet_ctx.config.net.wifi.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

static esp_err_t init_ethernet(void) {
    gpio_install_isr_service(0);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_libnet_ctx.netif = esp_netif_new(&netif_cfg);

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

    spi_device_handle_t spi_handle = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

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
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_libnet_ctx.eth_handle));
    ESP_LOGI(TAG, "ETH driver installed");

    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(base_mac_addr));
    uint8_t local_mac[ETH_ADDR_LEN];
    esp_derive_local_mac(local_mac, base_mac_addr);

    ESP_ERROR_CHECK(esp_eth_ioctl(s_libnet_ctx.eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));

    ESP_LOGI(TAG, "ETH MAC address: " MACSTR, MAC2STR(local_mac));

    s_libnet_ctx.eth_glue_handle = esp_eth_new_netif_glue(s_libnet_ctx.eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_libnet_ctx.netif, s_libnet_ctx.eth_glue_handle));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(s_libnet_ctx.eth_handle));

    ESP_LOGI(TAG, "ETH interface started");

    return ESP_OK;
}

esp_err_t libnet_deinit(void) {
    if (!s_libnet_ctx.initialized) {
        return ESP_OK;
    }

    switch (s_libnet_ctx.config.interface) {
        case LIBNET_INTERFACE_WIFI:
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
            ESP_ERROR_CHECK(esp_wifi_deinit());
            break;
        case LIBNET_INTERFACE_ETHERNET:
            ESP_ERROR_CHECK(esp_eth_stop(s_libnet_ctx.eth_handle));
            ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler));
            ESP_ERROR_CHECK(esp_eth_driver_uninstall(s_libnet_ctx.eth_handle));
            break;
    }

    esp_netif_destroy(s_libnet_ctx.netif);
    s_libnet_ctx.initialized = false;
    return ESP_OK;
}

esp_err_t libnet_udp_server_start(uint16_t port) {
    if (s_libnet_ctx.udp_server_running) {
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

    s_libnet_ctx.udp_server_socket = sock;
    s_libnet_ctx.udp_server_running = true;

    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, &port, s_libnet_ctx.config.priority, &s_libnet_ctx.udp_server_task, s_libnet_ctx.config.core_id);
    ESP_LOGI(TAG, "UDP server started on port %d", port);
    return ESP_OK;
}

esp_err_t libnet_udp_server_stop(void) {
    if (!s_libnet_ctx.udp_server_running) {
        return ESP_OK;
    }

    s_libnet_ctx.udp_server_running = false;

    if (s_libnet_ctx.udp_server_socket >= 0) {
        close(s_libnet_ctx.udp_server_socket);
        s_libnet_ctx.udp_server_socket = -1;
    }

    if (s_libnet_ctx.udp_server_task) {
        vTaskDelete(s_libnet_ctx.udp_server_task);
        s_libnet_ctx.udp_server_task = NULL;
    }

    ESP_LOGI(TAG, "UDP server stopped");
    return ESP_OK;
}

esp_err_t IRAM_ATTR libnet_udp_send(const ip4_addr_t *dest_ip, uint16_t dest_port, const uint8_t* data, size_t len) {
    if (!s_libnet_ctx.connected) {
        ESP_LOGE(TAG, "Network not connected");
        return ESP_ERR_INVALID_STATE;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket for sending");
        return ESP_FAIL;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = dest_ip->addr;
    dest_addr.sin_port = htons(dest_port);

    int sent = sendto(sock, data, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    close(sock);

    if (sent < 0) {
        ESP_LOGE(TAG, "Failed to send UDP packet");
        return ESP_FAIL;
    }


    ESP_LOGD(TAG, "Sent %d bytes to " IPSTR ":%d", sent, IP2STR(dest_ip), dest_port);
    return ESP_OK;
}

esp_err_t libnet_udp_send_broadcast(uint16_t dest_port, const uint8_t* data, size_t len) {
    return libnet_udp_send(IP4_ADDR_BROADCAST, dest_port, data, len);
}

bool libnet_is_connected(void) {
    return s_libnet_ctx.connected;
}

void libnet_get_ip_address(ip4_addr_t *addr) {
    memcpy(addr, &s_libnet_ctx.ip_address, sizeof(ip4_addr_t));
}

// Event handlers
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static uint16_t retry_num = 0;
    
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        retry_num++;
        ESP_LOGI(TAG, "Connecting to AP, retry #%u", retry_num);

        if (s_libnet_ctx.connected) {
            if (s_libnet_ctx.config.callbacks.disconnected) {
                s_libnet_ctx.config.callbacks.disconnected(s_libnet_ctx.config.user_ctx);
            }
            s_libnet_ctx.connected = false;
        }
        ip4_addr_set_u32(&s_libnet_ctx.ip_address, 0);
    }
}

static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Up");
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
        s_libnet_ctx.connected = false;
        ip4_addr_set_u32(&s_libnet_ctx.ip_address, 0);
        if (s_libnet_ctx.config.callbacks.disconnected) {
            s_libnet_ctx.config.callbacks.disconnected(s_libnet_ctx.config.user_ctx);
        }
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case IP_EVENT_STA_GOT_IP:
        case IP_EVENT_ETH_GOT_IP: {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            s_libnet_ctx.connected = true;
            ip4_addr_set_u32(&s_libnet_ctx.ip_address, event->ip_info.ip.addr);
            if (s_libnet_ctx.config.callbacks.connected) {
                s_libnet_ctx.config.callbacks.connected(s_libnet_ctx.config.user_ctx, &s_libnet_ctx.ip_address);
            }
            break;
        }
        case IP_EVENT_STA_LOST_IP:
        case IP_EVENT_ETH_LOST_IP: {
            ESP_LOGI(TAG, "Lost IP");
            s_libnet_ctx.connected = false;
            ip4_addr_set_u32(&s_libnet_ctx.ip_address, 0);
            if (s_libnet_ctx.config.callbacks.disconnected) {
                s_libnet_ctx.config.callbacks.disconnected(s_libnet_ctx.config.user_ctx);
            }
            break;
        }
    }
}

static void udp_server_task(void* arg) {
    uint16_t port = *(uint16_t *)arg;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    ESP_LOGI(TAG, "UDP server task started on port %d", port);

    uint8_t packet_data[LIBNET_MAX_PACKET_SIZE];

    while (s_libnet_ctx.udp_server_running) {
        int packet_len = recvfrom(s_libnet_ctx.udp_server_socket, 
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

            if (s_libnet_ctx.config.callbacks.packet) {
                s_libnet_ctx.config.callbacks.packet(s_libnet_ctx.config.user_ctx, packet_data, packet_len, &ip, port);
            }
        } else if (packet_len < 0 && errno != EAGAIN) {
            ESP_LOGE(TAG, "UDP receive error: %d", errno);
            break;
        }
    }

    ESP_LOGI(TAG, "UDP server task ended");
    vTaskDelete(NULL);
}
