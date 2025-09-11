#include <unity.h>

#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_payload.h"

TEST_CASE("test rc channels", "libcrsf")  {
    crsf_payload_rc_channels_t payload_in = {
        .channels = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }
    };

    crsf_frame_t frame;

    crsf_payload_pack__rc_channels(&frame, &payload_in);

    crsf_payload_rc_channels_t payload_out = {0};

    TEST_ASSERT(crsf_payload_unpack__rc_channels(&frame, &payload_out));

    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL(payload_in.channels[i], payload_out.channels[i]);
    }
}
