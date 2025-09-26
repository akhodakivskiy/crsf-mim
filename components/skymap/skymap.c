#include "skymap.h"

#include "lwip/ip4_addr.h"
#include "skymap.pb.h"

#include <esp_log.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

static const char *TAG = "SKYMAP";

#define _SKYMAP_HEADER_LEN 4

static const uint8_t _SKYMAP_HEADER[4] = { 0x54, 0x54, 0x50, 0x01 };

skymap_err_t IRAM_ATTR skymap_read_server_message(
    ai_skyfortress_guidance_ServerMessage *sm, 
    const uint8_t *data, uint16_t len) {

    assert(data != NULL && len > 0);

    skymap_err_t err = SKYMAP_OK;

    pb_istream_t stream = pb_istream_from_buffer(data, len);

    uint8_t header[4] = {0};

    if (!pb_read(&stream, header, _SKYMAP_HEADER_LEN)) {
        ESP_LOGE(TAG, "Message too short");
        err = SKYMAP_ERR_HEADER_MISSING;
    } else if (memcmp(_SKYMAP_HEADER, header, _SKYMAP_HEADER_LEN) != 0) {
        ESP_LOGE(TAG, "Unexpected TATEP header: %2x%2x%2x%2x", header[0], header[1], header[2], header[3]);
        err = SKYMAP_ERR_HEADER_INVALID;
    } else if (!pb_decode_ex(&stream, ai_skyfortress_guidance_ServerMessage_fields, sm, PB_DECODE_DELIMITED)) {
        ESP_LOGE(TAG, "Failed to parse Skymap message");
        err = SKYMAP_ERR_MESSAGE_INVALID;
    }

    return err;
}

skymap_err_t skymap_write_client_message(
    const ai_skyfortress_guidance_ClientMessage *sm, 
    uint8_t *data, uint16_t len, uint16_t *len_written) {

    skymap_err_t err = SKYMAP_OK;

    pb_ostream_t stream = pb_ostream_from_buffer(data, len);

    if (!pb_write(&stream, _SKYMAP_HEADER, _SKYMAP_HEADER_LEN)) {
        ESP_LOGE(TAG, "Failed to write message header");
        err = SKYMAP_ERR_HEADER_INVALID;
    } else if (!pb_encode_ex(&stream, ai_skyfortress_guidance_ClientMessage_fields, sm, PB_ENCODE_DELIMITED)) {
        ESP_LOGE(TAG, "Failed to encode client status message");
        err = SKYMAP_ERR_MESSAGE_INVALID;
    } else if (len_written != NULL) {
        *len_written = stream.bytes_written;
    }

    return err;
}
