#include "libcrsf_parser.h"

#include "libcrsf_crc8.h"

#include <esp_attr.h>
#include <esp_log.h>
#include <assert.h>

static const char *TAG = "CRSF_PARSER";

void IRAM_ATTR crsf_parser_init(crsf_parser_t *parser, crsf_frame_t *frame) {
    assert(parser != NULL);
    assert(frame != NULL);

    frame->sync = 0x0;
    frame->length = 0;
    frame->type = CRSF_FRAME_TYPE_NONE;
    frame->is_extended = false;
    frame->dest = CRSF_ADDRESS_BROADCAST;
    frame->source = CRSF_ADDRESS_BROADCAST;

    parser->state = CRSF_PARSE_WAIT_SYNC;
    parser->frame = frame;
    parser->bytes_received = 0;
    parser->expected_length = 0;
}

crsf_parse_result_t crsf_parser_parse_byte(crsf_parser_t *parser, uint8_t byte) {
    switch (parser->state) {
        case CRSF_PARSE_WAIT_SYNC:
            if (byte == CRSF_SYNC_BYTE || byte == CRSF_SYNC_BYTE_EDGETX || byte == CRSF_ADDRESS_RADIO_TRANSMITTER) {
                parser->frame->sync = byte;
                parser->state = CRSF_PARSE_WAIT_LENGTH;
                parser->bytes_received = 1;
            }
            // Stay in WAIT_SYNC state if not sync byte
            break;

        case CRSF_PARSE_WAIT_LENGTH:
            if (byte < CRSF_MIN_FRAME_LEN || byte > CRSF_MAX_FRAME_LEN) {
                parser->state = CRSF_PARSE_WAIT_SYNC;
                return CRSF_PARSE_RESULT_ERROR_LENGTH;
            }
            parser->frame->length = byte;
            parser->expected_length = byte;
            parser->state = CRSF_PARSE_WAIT_TYPE;
            parser->bytes_received = 2;
            break;

        case CRSF_PARSE_WAIT_TYPE:
            parser->frame->type = byte;
            parser->bytes_received = 3;
            if (parser->frame->type < CRSF_FRAME_TYPE_DEVICE_PING) {
                parser->state = CRSF_PARSE_WAIT_DATA;
            } else {
                parser->frame->is_extended = true;
                parser->state = CRSF_PARSE_WAIT_EXTENDED_DEST;
            }
            break;
        case CRSF_PARSE_WAIT_EXTENDED_DEST:
            parser->frame->data[parser->bytes_received - 3] = byte;
            parser->frame->dest = byte;
            parser->bytes_received++;
            parser->state = CRSF_PARSE_WAIT_EXTENDED_SOURCE;
            break;
        case CRSF_PARSE_WAIT_EXTENDED_SOURCE:
            parser->frame->data[parser->bytes_received - 3] = byte;
            parser->frame->source = byte;
            parser->bytes_received++;
            parser->state = CRSF_PARSE_WAIT_DATA;
            break;
        case CRSF_PARSE_WAIT_DATA:
            parser->frame->data[parser->bytes_received - 3] = byte;
            parser->bytes_received++;

            // Check if we've received all expected bytes (length + sync + length bytes)
            if (parser->bytes_received >= (parser->expected_length + 2)) {
                parser->state = CRSF_PARSE_FRAME_COMPLETE;
                // Validate CRC
                uint8_t payload_len = parser->expected_length - 2; // exclude type and crc
                uint8_t calculated_crc = crsf_calc_crc8(parser->frame);
                uint8_t received_crc = parser->frame->data[payload_len];
                if (calculated_crc != received_crc) {
                    parser->state = CRSF_PARSE_WAIT_SYNC;
                    return CRSF_PARSE_RESULT_ERROR_CRC;
                }
                return CRSF_PARSE_RESULT_FRAME_COMPLETE;
            }
            break;

        case CRSF_PARSE_FRAME_COMPLETE:
            // Reset parser for next frame
            parser->state = CRSF_PARSE_WAIT_SYNC;
            // Process this byte as start of new frame
            return crsf_parser_parse_byte(parser, byte);
    }

    return CRSF_PARSE_RESULT_NEED_MORE_DATA;
}

bool crsf_parser_frame_ready(const crsf_parser_t *parser) {
    return parser->state == CRSF_PARSE_FRAME_COMPLETE;
}

void crsf_parser_reset(crsf_parser_t *parser) {
    parser->state = CRSF_PARSE_WAIT_SYNC;
    parser->bytes_received = 0;
    parser->expected_length = 0;
}
