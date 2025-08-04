#include "esp_attr.h"
#include "crc8.h"

Crc8::Crc8(uint8_t poly): _poly(poly) {
    uint8_t crc;

    for (uint16_t i = 0; i < CRC_TAB_SIZE; i++) {
        crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        _tab[i] = crc & 0xFF;
    }
}

uint8_t IRAM_ATTR Crc8::calc(const uint8_t data) {
    return _tab[data];
}

uint8_t IRAM_ATTR Crc8::calc(const uint8_t *data, uint16_t len, uint8_t crc) {
    while (len--) {
        crc = _tab[crc ^ *data++];
    }
    return crc;
}