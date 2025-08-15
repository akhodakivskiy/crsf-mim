#ifndef UTIL_LQCALC_H
#define UTIL_LQCALC_H

#include <stdint.h>
#include <esp_attr.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t period_max; // max number of periods to track

    uint8_t received;
    uint8_t period_count; // periods prepared capped by `period_max`
    uint8_t current_index; // current position in `lq_array`
    uint32_t current_bit_mask;
    uint32_t *lq_array;
} util_lqcalc_t;

void util_lqcalc_init(util_lqcalc_t *lqc, uint8_t period_max);

void util_lqcalc_deinit(util_lqcalc_t *lqc);

void util_lqcalc_reset(util_lqcalc_t *lqc);

void util_lqcalc_reset_100(util_lqcalc_t *lqc);

void util_lqcalc_prepare(util_lqcalc_t *lqc);

void util_lqcalc_receive(util_lqcalc_t *lqc);

uint8_t util_lqcalc_get_received(util_lqcalc_t *lqc);

uint8_t util_lqcalc_get_lq(util_lqcalc_t *lqc);

#ifdef __cplusplus
}
#endif

#endif
