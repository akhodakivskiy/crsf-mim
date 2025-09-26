#ifndef LIBCRSF_H
#define LIBCRSF_H

#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <stddef.h>
#include <lwip/ip4_addr.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIBNET_MAX_PACKET_SIZE 1024
#define LIBNET_MAX_CLIENTS 5

typedef enum {
    LIBNET_INTERFACE_WIFI = 0,
    LIBNET_INTERFACE_ETHERNET = 1
} libnet_interface_t;

typedef enum {
    LIBNET_EVENT_CONNECTED,
    LIBNET_EVENT_DISCONNECTED,
    LIBNET_EVENT_UDP_RECEIVED,
    LIBNET_EVENT_ERROR
} libnet_event_t;

typedef struct {
    char ssid[32];
    char password[64];
} libnet_wifi_config_t;

typedef struct {
    uint8_t dummy;
} libnet_ethernet_config_t;

typedef void (*libnet_callback_connected_t)(void *user_ctx, ip4_addr_t *addr);
typedef void (*libnet_callback_disconnected_t)(void *user_ctx);
typedef void (*libnet_callback_packet_t)(void *user_ctx, uint8_t* data, uint16_t len, ip4_addr_t *addr_from, uint16_t port);

typedef struct {
    BaseType_t priority;
    BaseType_t core_id;
    struct {
        libnet_callback_connected_t connected;
        libnet_callback_disconnected_t disconnected;
        libnet_callback_packet_t packet;
    } callbacks;
    libnet_interface_t interface;
    union {
        libnet_wifi_config_t wifi;
        libnet_ethernet_config_t eth;
    } net;
    void *user_ctx;
} libnet_config_t;

typedef struct libnet_ctx_s *libnet_handle_t;

// Core functions
esp_err_t libnet_init(const libnet_config_t* config, libnet_handle_t *handle);
esp_err_t libnet_deinit(libnet_handle_t handle);

// UDP functions
esp_err_t libnet_udp_server_start(libnet_handle_t handle, uint16_t port);
esp_err_t libnet_udp_server_stop(libnet_handle_t handle);
esp_err_t libnet_udp_send(libnet_handle_t handle, const ip4_addr_t *dest_ip, uint16_t dest_port, const uint8_t* data, size_t len);
esp_err_t libnet_udp_send_broadcast(libnet_handle_t handle, uint16_t dest_port, const uint8_t* data, size_t len);

// Utility functions
bool libnet_is_connected(libnet_handle_t handle);
void libnet_get_ip_address(libnet_handle_t handle, ip4_addr_t *addr);

#ifdef __cplusplus
}
#endif

#endif // LIBNET_H
