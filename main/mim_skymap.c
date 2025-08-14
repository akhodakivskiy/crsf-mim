#include "mim_skymap.h"

#include <esp_log.h>

#include "lwip/ip_addr.h"
#include "pb_decode.h"
#include "skymap.pb.h"
#include "pb.h"

static const char *TAG = "MIM_SKYMAP";

void mim_skymap_init(mim_skymap_t *skymap) {
    skymap->skymap_port = 0;
}

void mim_skymap_handle_packet(mim_skymap_t *skymap, uint8_t *data, uint16_t len) {
    pb_istream_t is = pb_istream_from_buffer(data, len);
    bool status = pb_decode(&is, ai_skyfortress_guidance_ServerMessage_fields, &skymap->skymap_msg);

    if (!status) {
        ESP_LOGI(TAG, "failed to decode message, len=%u", len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    } else {
        /*
        switch(skymap->skymap_msg.which_message) {
            case 
        }
        */
    }
}
