#ifndef LQCALC_H
#define LQCALC_H

#include <stdint.h>
#include <esp_attr.h>
#include <esp_bit_defs.h>

#include "tools.h"

template <uint8_t N>
class LqCalc
{
public:
    LqCalc(uint8_t period_max) {
        reset100(period_max);
    }
    LqCalc(void): LqCalc(N) { }

    /* Set the bit for the current period to true and update the running LQ */
    void IRAM_ATTR add() {
        if (currentIsSet()) {
            return;
        }
        _lq_array[_index] |= _lq_mask;
        _lq += 1;
    }

    /*  Return true if the current period was add()ed */
    bool IRAM_ATTR currentIsSet() const {
        return _lq_array[_index] & _lq_mask;
    }

    /* Start a new period */
    void IRAM_ATTR prepare()
    {
        // Increment the counter by shifting one bit higher
        // If we've shifted out all the bits, move to next idx
        _lq_mask = _lq_mask << 1;
        if (_lq_mask == 0) {
            _lq_mask = (1 << 0);
            _index += 1;
        }

        // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
        if ((_index == (_period_max / 32)) && (_lq_mask & (1 << (_period_max % 32)))) {
            _index = 0;
            _lq_mask = (1 << 0);
        }

        if ((_lq_array[_index] & _lq_mask) != 0) {
            _lq_array[_index] &= ~_lq_mask;
            _lq -= 1;
        }

        if (_period_count < _period_max) {
          ++_period_count;
        }
    }

    /* Return the current running total of bits set, in percent */
    uint8_t IRAM_ATTR getLq() const {
        return (uint32_t)_lq * 100U / _period_count;
    }

    /* Return the current running total of bits set, up to N */
    uint8_t getLQRaw() const {
        return _lq;
    }

    /* Return the number of periods recorded so far, up to N */
    uint8_t getCount() const {
        return _period_count;
    }

    /* the size of the LQ history */
    uint8_t getSize() const {
        return _period_max;
    }

    bool isFull() const {
        return _period_count == _period_max;
    }

    /* Initialize and zero the history */
    void reset(uint8_t period_max = N) {
        // period_count is intentonally not zeroed here to start LQ counting up from 0
        // after a failsafe, instead of down from 100. Use reset100() to start from 100
        _period_max = period_max;
        _lq = 0;
        _index = 0;
        _lq_mask = (1 << 0);
        for (uint8_t i = 0; i < ARRAY_SIZE(_lq_array); i++) {
            _lq_array[i] = 0;
        }
    }

    /* Reset and start at 100% */
    void reset100(uint8_t period_max = N) {
        reset(period_max);
        _period_count = 1;
    }

private:
    uint8_t _lq;
    uint8_t _index; // current position in LQArray
    uint8_t _period_count;
    uint8_t _period_max;
    uint32_t _lq_mask;
    uint32_t _lq_array[(N + 31)/32];
};

#endif
