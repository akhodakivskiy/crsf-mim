#ifndef CRC_8_H
#define CRC_8_H

#include <sys/types.h>
#include <esp_attr.h>

#define CRC_TAB_SIZE 256

class Crc8
{
private:
    uint8_t _tab[CRC_TAB_SIZE];
    uint8_t _poly;

public:
    Crc8(uint8_t poly);
    uint8_t calc(const uint8_t data);
    uint8_t calc(const uint8_t *data, uint16_t len, uint8_t crc = 0);
};

#endif