#include <unity.h>

#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_payload.h"

TEST_CASE("test rc channels", "libcrsf")  {
    crsf_payload_rc_channels_t payload_in = {
        .ch1 = 1,
        .ch2 = 2,
        .ch3 = 3,
        .ch4 = 4,
        .ch5 = 5,
        .ch6 = 6,
        .ch7 = 7,
        .ch8 = 8,
        .ch9 = 9,
        .ch10 = 10,
        .ch11 = 11,
        .ch12 = 12,
        .ch13 = 13,
        .ch14 = 14,
        .ch15 = 15,
        .ch16 = 16,
    };

    crsf_frame_t frame;

    crsf_payload__rc_channels_pack(&frame, &payload_in);

    crsf_payload_rc_channels_t payload_out = {0};

    TEST_ASSERT(crsf_payload__rc_channels_unpack(&frame, &payload_out));

    for (int i = 1; i <= 16; i++) {
        TEST_ASSERT_EQUAL(crsf_payload__rc_channels_get(&payload_out, i), i);
    }
}
