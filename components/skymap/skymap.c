#include "skymap.h"

#include "lwip/ip4_addr.h"
#include "skymap.pb.h"

#include <esp_log.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

static const char *TAG = "SKYMAP";

#define SKYMAP_TATEP_HEADER_LEN 4

static const uint8_t _skymap_header[4] = { 0x54, 0x54, 0x50, 0x01 };

void skymap_reset(skymap_t *sm) {
    sm->is_connected = false;
    sm->is_ready_to_engage = false;
    sm->is_client_message_ready = false;

    sm->last_time_client_message = 0;
    sm->last_time_server_message = 0;
    sm->last_time_ping = 0;
    sm->last_time_interceptor = 0;
    sm->last_time_target = 0;
}

void skymap_update(skymap_t *sm, int64_t time) {
    sm->is_connected = SKYMAP_CONNECTION_TIMEOUT_US > (time - sm->last_time_server_message);
    sm->is_ready_to_engage = 
        (SKYMAP_CONNECTION_TIMEOUT_US > (time - sm->last_time_interceptor)) &&
        (SKYMAP_CONNECTION_TIMEOUT_US > (time - sm->last_time_target));
    sm->is_client_message_ready = SKYMAP_STATUS_TIMEOUT_US <= (time - sm->last_time_client_message);
}

skymap_err_t skymap_read_server_message(skymap_t *sm, int64_t time, uint8_t *data, uint16_t len) {
    assert(data != NULL && len > 0);

    skymap_err_t err = SKYMAP_OK;

    pb_istream_t stream = pb_istream_from_buffer(data, len);

    uint8_t header[4] = {0};
    ai_skyfortress_guidance_ServerMessage m = ai_skyfortress_guidance_ServerMessage_init_default;

    if (!pb_read(&stream, header, SKYMAP_TATEP_HEADER_LEN)) {
        ESP_LOGE(TAG, "Message too short");
        err = SKYMAP_ERR_HEADER_INVALID;
    } else if (memcmp(_skymap_header, header, SKYMAP_TATEP_HEADER_LEN) != 0) {
        ESP_LOGE(TAG, "Unexpected TATEP header: %2x%2x%2x%2x", header[0], header[1], header[2], header[3]);
        err = SKYMAP_ERR_HEADER_INVALID;
    } else if (!pb_decode_ex(&stream, ai_skyfortress_guidance_ServerMessage_fields, &m, PB_DECODE_DELIMITED)) {
        ESP_LOGE(TAG, "Failed to parse TATEP message");
        err = SKYMAP_ERR_MESSAGE_INVALID;
    } else {
        sm->last_time_server_message = time;

        switch (m.which_message) {
            case ai_skyfortress_guidance_ServerMessage_ping_tag:
                //ESP_LOGI(TAG, "ping");
                sm->last_time_ping = time;
                break;
            case ai_skyfortress_guidance_ServerMessage_target_estimate_tag:
                //ESP_LOGI(TAG, "target estimate");
                sm->last_time_target = time;
                break;
            case ai_skyfortress_guidance_ServerMessage_interceptor_estimate_tag:
                //ESP_LOGI(TAG, "interceptor estimate");
                sm->last_time_interceptor = time;
                break;
            case ai_skyfortress_guidance_ServerMessage_target_raw_tag:
                //ESP_LOGI(TAG, "target raw");
                sm->last_time_target = time;
                break;
            case ai_skyfortress_guidance_ServerMessage_interceptor_raw_tag:
                //ESP_LOGI(TAG, "interceptor raw");
                sm->last_time_interceptor = time;
                break;
        }
    }

    skymap_update(sm, time);

    return err;
}

skymap_err_t skymap_write_client_message(skymap_t *sm, int64_t time, uint8_t *data, uint16_t len, uint16_t *len_written) {
    skymap_err_t err = SKYMAP_OK;

    ai_skyfortress_guidance_ClientMessage m = ai_skyfortress_guidance_ClientMessage_init_default;
    m.which_message = ai_skyfortress_guidance_ClientMessage_status_tag;
    m.message.status.has_position = false;
    m.message.status.state = ai_skyfortress_guidance_ClientState_Ready;

    pb_ostream_t stream = pb_ostream_from_buffer(data, len);

    if (!pb_write(&stream, _skymap_header, SKYMAP_TATEP_HEADER_LEN)) {
        ESP_LOGE(TAG, "Failed to write message header");
        err = SKYMAP_ERR_HEADER_INVALID;
    } else if (!pb_encode_ex(&stream, ai_skyfortress_guidance_ClientMessage_fields, &m, PB_ENCODE_DELIMITED)) {
        ESP_LOGE(TAG, "Failed to encode client status message");
        err = SKYMAP_ERR_MESSAGE_INVALID;
    } else {
        sm->last_time_client_message = time;
        *len_written = stream.bytes_written;
    }

    skymap_update(sm, time);

    return err;
}
