#include "libcrsf.h"

#include <esp_attr.h>
#include <stdbool.h>
#include <stdint.h>

#define CRSF_CRC_POLY 0xD5
#define CRSF_CRC_TAB_SIZE 256

static uint8_t _crsf_tab[CRSF_CRC_TAB_SIZE];
static bool _crsf_tab_is_init = false;

static void _crsf_crc_tab_init(uint8_t poly) {
    uint8_t crc = 0;

    if (!_crsf_tab_is_init) {
        for (uint16_t i = 0; i < CRSF_CRC_TAB_SIZE; i++) {
            crc = i;
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
            }
            _crsf_tab[i] = crc & 0xFF;
        }

        _crsf_tab_is_init = true;
    }
}

uint8_t IRAM_ATTR _crsf_calc_crc8_one(uint8_t value, uint8_t crc) {
    if (!_crsf_tab_is_init) {
        _crsf_crc_tab_init(CRSF_CRC_POLY);
    }

    return _crsf_tab[crc ^ value];
}

uint8_t IRAM_ATTR _crsf_calc_crc8_all(const uint8_t *data, uint16_t len, uint8_t crc) {
    if (!_crsf_tab_is_init) {
        _crsf_crc_tab_init(CRSF_CRC_POLY);
    }

    while (len--) {
        crc = _crsf_tab[crc ^ *data++];
    }

    return crc;
}

uint8_t IRAM_ATTR crsf_calc_crc8(crsf_frame_t *frame) {
    uint8_t crc = _crsf_calc_crc8_one(frame->type, 0);
    return _crsf_calc_crc8_all(frame->data, frame->length - 2, crc); // - type - crc
}
