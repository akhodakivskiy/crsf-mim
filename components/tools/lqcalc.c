#include "lqcalc.h"

#include <stdlib.h>
#include <string.h>

#define LQCALC_PERIOD_SIZE sizeof(uint64_t)
#define LQCALC_ARRAY_SIZE(PERIOD_MAX) \
((PERIOD_MAX + LQCALC_PERIOD_SIZE - 1) / LQCALC_PERIOD_SIZE)

void lqcalc_init(lqcalc_t *lqc, uint8_t period_max) {
    lqc->lq_array = (uint32_t *)calloc(LQCALC_ARRAY_SIZE(period_max), LQCALC_PERIOD_SIZE);

    lqc->period_max = period_max;
    lqcalc_reset(lqc);
}

void lqcalc_deinit(lqcalc_t *lqc) {
    cfree(lqc->lq_array);
    memset(lqc, 0, sizeof(lqcalc_t));
}

void lqcalc_reset(lqcalc_t *lqc) {
    lqc->received = 0;
    lqc->period_count = 0;
    lqc->current_index = 0;
    lqc->current_bit_mask = 0;
}

void lqcalc_reset_100(lqcalc_t *lqc) {
    lqc->received = lqc->period_max;
    lqc->period_count = lqc->period_count;
    lqc->current_index = 0;
    lqc->current_bit_mask = 0;

    uint16_t len = LQCALC_ARRAY_SIZE(lqc->period_max);
    for (int i = 0; i < len; i++) {
        lqc->lq_array[i] = UINT32_MAX;
    }
}

void lqcalc_prepare(lqcalc_t *lqc) {
    // advance current index and mask by 1
    lqc->current_bit_mask = lqc->current_bit_mask << 1;
    if (lqc->current_bit_mask == 0) {
        lqc->current_bit_mask = (1 << 0);
        lqc->current_index += 1;
    }

    // if period_max is exceeded then wrap around
    if ((lqc->current_index == (lqc->period_max / LQCALC_PERIOD_SIZE)) && 
        (lqc->current_bit_mask & (1 << (lqc->period_max % LQCALC_PERIOD_SIZE)))) {
        lqc->current_index = 0;
        lqc->current_bit_mask = (1 << 0);
    }

    // clear current bit and decrease lq if necessary
    if ((lqc->lq_array[lqc->current_index] & lqc->current_bit_mask) != 0) {
        lqc->lq_array[lqc->current_index] &= ~lqc->current_bit_mask;
        lqc->received -= 1;
    }

    // increment period count
    if (lqc->period_count < lqc->period_max) {
        lqc->period_count += 1;
    }
}

void IRAM_ATTR lqcalc_receive(lqcalc_t *lqc) {
    if ((lqc->lq_array[lqc->current_index] & lqc->current_bit_mask) == 0) {
        lqc->lq_array[lqc->current_index] |= lqc->current_bit_mask;
        lqc->received += 1;
    }
}

uint8_t IRAM_ATTR lqcalc_get_received(lqcalc_t *lqc) {
    return lqc->received;
}

uint8_t IRAM_ATTR lqcalc_get_lq(lqcalc_t *lqc) {
    return (uint32_t)lqc->received * 100U / lqc->period_count;
}
