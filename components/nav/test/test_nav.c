#include <unity.h>

#include <esp_log.h>

#include "la.h"
#include "nav.h"

static const char *TAG = "TEST_NAV";

/* pursuit tests */

TEST_CASE("state to ned", "nav") {
    nav_state_t i = { 50, 30, 1000, 0, 0, 0 };
    nav_state_t t = { 50.1, 30.1, 1100, 0, 0, 0 };

    la_float range[3] = {0};
    nav_state_to_ned(&i, &t, range);

    TEST_ASSERT_FLOAT_WITHIN(1.0, 11121.0, range[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 7141.0, range[1]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, -100.0, range[2]);

    nav_state_to_ned(&t, &i, range);

    TEST_ASSERT_FLOAT_WITHIN(1.0, -11121.0, range[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, -7141.0, range[1]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 100.0, range[2]);

    nav_state_to_ned(&i, &i, range);

    TEST_ASSERT_EQUAL_FLOAT(0, range[0]);
    TEST_ASSERT_EQUAL_FLOAT(0, range[1]);
    TEST_ASSERT_EQUAL_FLOAT(0, range[2]);

    nav_state_t ie = { -0.1, 0, 1000, 0, 0, 0 };
    nav_state_t te = { 0.1, 0, 1100, 0, 0, 0 };

    nav_state_to_ned(&te, &ie, range);

    TEST_ASSERT_FLOAT_WITHIN(1.0, -22242.6, range[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, range[1]);
    TEST_ASSERT_EQUAL_FLOAT(100.0, range[2]);
}

TEST_CASE("state to ctx", "nav") {
    nav_state_t i = { 30, 50, 1000, 10, 20, 5 };
    nav_state_t t = { 30.1, 50.1, 1100, -2, 10, -4 };

    nav_ctx_t ctx;
    nav_state_to_ctx(&i, &t, &ctx);

    TEST_ASSERT_FLOAT_WITHIN(0.1, 14709.3, ctx.range_norm);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 22.9, ctx.vel_i_norm);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 11, ctx.vel_t_norm);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 18, ctx.vel_rel_norm);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 15.7, ctx.vel_closing);
}

TEST_CASE("guidance type", "nav") {
    nav_state_t i = { 50, 30, 1000, -10, -20, -5 };
    nav_state_t t1 = { 50 + .1, 30 + .1, 1100, 2, 10, 4 };
    nav_state_t t2 = { 50 - .1, 30 - .1, 1100, 2, 10, 4 };

    nav_config_t cfg = { 
        .N = 3, 
        .tail_cone_deg = 0,
        .tail_factor = 0
    };
    nav_command_t cmd;

    nav_compute_command(&cfg, &i, &t1, &cmd);
    TEST_ASSERT_EQUAL(cmd.type, NAV_PURSUIT);

    nav_compute_command(&cfg, &i, &t2, &cmd);
    TEST_ASSERT_EQUAL(cmd.type, NAV_PRONAV);
}

TEST_CASE("tail bias", "nav") {
    nav_state_t i = { 50, 30, 1000, 10, 0, 0 };
    nav_state_t t = { 50.1, 30, 1000, 5, 0, 0 };
    nav_ctx_t ctx;
    nav_state_to_ctx(&i, &t, &ctx);

    la_float a_bias[3];
    nav_compute_tail_bias(NAV_DEG_TO_RAD(20), 0.1, &ctx, a_bias);

    TEST_ASSERT_EQUAL_FLOAT(a_bias[0], 0.0);
    TEST_ASSERT_EQUAL_FLOAT(a_bias[1], 0.0);
    TEST_ASSERT_EQUAL_FLOAT(a_bias[2], 0.0);

    t.vel_north = 0;
    t.vel_east = 5;
    nav_state_to_ctx(&i, &t, &ctx);
    nav_compute_tail_bias(NAV_DEG_TO_RAD(20), 0.1, &ctx, a_bias);

    TEST_ASSERT_GREATER_THAN_FLOAT(0.0, a_bias[0]);
    TEST_ASSERT_LESS_THAN_FLOAT(0.0, a_bias[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, a_bias[2]);
}

TEST_CASE("compute pursuit", "nav") {
    nav_state_t i = { 50, 30, 1000, -10, 5, 0 };
    nav_state_t t = { 50.1, 30, 1000, 10, 0, 0 };
    nav_ctx_t ctx;
    nav_state_to_ctx(&i, &t, &ctx);

    la_float a[3];
    nav_pursuit(3, &ctx, a);

    TEST_ASSERT_GREATER_THAN_FLOAT(0.0, a[0]);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0, a[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, a[2]);

    nav_config_t cfg = {
        .N = 3,
        .tail_cone_deg = 0,
        .tail_factor = 0
    };

    nav_type_t type = nav_compute_accel(&cfg, &ctx, a);

    TEST_ASSERT_EQUAL(type, NAV_PURSUIT);
}

TEST_CASE("compute pronav", "nav") {
    nav_state_t i = { 50, 30, 1000, 10, 0, 0 };
    nav_state_t t = { 50.1, 30, 1000, 0, 5, 0 };
    nav_ctx_t ctx;
    nav_state_to_ctx(&i, &t, &ctx);

    la_float a[3];
    nav_pronav(3, &ctx, a);

    TEST_ASSERT_EQUAL_FLOAT(0.0, a[0]);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0, a[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, a[2]);

    nav_config_t cfg = {
        .N = 3,
        .tail_cone_deg = 0,
        .tail_factor = 0
    };

    nav_type_t type = nav_compute_accel(&cfg, &ctx, a);

    TEST_ASSERT_EQUAL(type, NAV_PRONAV);
}

TEST_CASE("compute tgo+zem", "nav") {
    nav_state_t i = { 50, 30, 1000, 5, 5, 0 };
    nav_state_t t = { 50.1, 30, 1000, 0, 5, 0 };
    nav_ctx_t ctx;
    nav_state_to_ctx(&i, &t, &ctx);

    la_float tgo, zem;
    bool result = nav_compute_tgo_zem(&ctx, &tgo, &zem);

    TEST_ASSERT_EQUAL(true, result);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 2224.2, tgo);
    TEST_ASSERT_EQUAL_FLOAT(0.0, zem);

    nav_state_t i2 = { 50, 30, 1000, -5, 0, 0 };
    nav_state_t t2 = { 50.1, 30, 1000, 5, 0, 0 };

    nav_state_to_ctx(&i2, &t2, &ctx);
    result = nav_compute_tgo_zem(&ctx, &tgo, &zem);

    ESP_LOGI(TAG, "v_c=%f", ctx.vel_closing);

    TEST_ASSERT_EQUAL(false, result);
}
