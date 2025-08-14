#ifndef LIBCRSF_PARSER_H
#define LIBCRSF_PARSER_H

#include "libcrsf_def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CRSF_PARSE_RESULT_NEED_MORE_DATA,
    CRSF_PARSE_RESULT_FRAME_COMPLETE,
    CRSF_PARSE_RESULT_ERROR_SYNC,
    CRSF_PARSE_RESULT_ERROR_LENGTH,
    CRSF_PARSE_RESULT_ERROR_CRC
} crsf_parse_result_t;

typedef enum {
    CRSF_PARSE_WAIT_SYNC,
    CRSF_PARSE_WAIT_LENGTH,
    CRSF_PARSE_WAIT_TYPE,
    CRSF_PARSE_WAIT_EXTENDED_DEST,
    CRSF_PARSE_WAIT_EXTENDED_SOURCE,
    CRSF_PARSE_WAIT_DATA,
    CRSF_PARSE_FRAME_COMPLETE
} crsf_parser_state_t;

typedef struct {
    crsf_parser_state_t state;
    crsf_frame_t *frame;
    uint8_t bytes_received;
    uint8_t expected_length;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *parser, crsf_frame_t *frame);

crsf_parse_result_t crsf_parser_parse_byte(crsf_parser_t *parser, uint8_t byte);

bool crsf_parser_frame_ready(const crsf_parser_t *parser);

void crsf_parser_reset(crsf_parser_t *parser);

#ifdef __cplusplus
}
#endif

#endif
