#ifndef LQCALC_H
#define LQCALC_H

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
} lqcalc_t;

void lqcalc_init(lqcalc_t *lqc, uint8_t period_max);

void lqcalc_deinit(lqcalc_t *lqc);

void lqcalc_reset(lqcalc_t *lqc);

void lqcalc_reset_100(lqcalc_t *lqc);

void lqcalc_prepare(lqcalc_t *lqc);

void lqcalc_receive(lqcalc_t *lqc);

uint8_t lqcalc_get_received(lqcalc_t *lqc);

uint8_t lqcalc_get_lq(lqcalc_t *lqc);

#ifdef __cplusplus
}
#endif

#endif
