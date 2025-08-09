#include <unity.h>

#include "libcrsf.h"

TEST_CASE("test crc", "[libcrsf]")  {
    crsf_calc_crc8("1234", 4);
}
