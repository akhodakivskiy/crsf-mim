#include <unity.h>

#include <esp_log.h>

#include "la.h"
#include "nav_guidance.h"

static const char *TAG = "TEST_NAV";

/* pursuit tests */

TEST_CASE("pursuit aligned", "nav") {
    la_float range[3] = { 100, 0, 0 };
    la_float vel_i[3] = { 10, 0, 0 };

    la_float accel[3] = {0, 0, 0};

    bool result = nav_guidance_pursuit(3, range, vel_i, accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT(accel[1] == 0);
    TEST_ASSERT(accel[2] == 0);
}

TEST_CASE("pursuit right", "nav") {
    la_float range[3] = {100,  10,  0 };
    la_float vel_i[3] = {10,  0,  0 };

    la_float accel[3] = {0, 0, 0};

    bool result = nav_guidance_pursuit(3, range, vel_i, accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.99, accel[1]);
    TEST_ASSERT(accel[2] == 0);
}

TEST_CASE("pursuit left", "nav") {
    la_float range[3] = { 100, -10, 0 };
    la_float vel_i[3] = { 10, 0, 0 };

    la_float accel[3] = { 0, 0, 0};

    bool result = nav_guidance_pursuit(3, range, vel_i, accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -2.99, accel[1]);
    TEST_ASSERT(accel[2] == 0);
}


/* proportional navigation tests */

TEST_CASE("pronav aligned", "nav") {
    la_float range[3] = { 100, 0, 0 };
    la_float vel_i[3] = { 10, 0, 0 };
    la_float vel_t[3] = { 5, 0, 0 };

    la_float accel[3] = { 0, 0, 0 };

    bool result = nav_guidance_pronav_true(3.0, range, vel_i, vel_t, accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT(accel[1] == 0);
    TEST_ASSERT(accel[2] == 0);
}

TEST_CASE("pronav right", "nav") {
    la_float range[3] = { 100, 10, 0 };
    la_float vel_i[3] = { 10, 0, 0 };
    la_float vel_t[3] = { 5, 0, 0 };

    la_float accel[3] = { 0, 0, 0 };

    bool result = nav_guidance_pronav_true(3.0, range, vel_i, vel_t, accel);

    //ESP_LOGI(TAG, "ax: %lf, ay: %lf, az: %lf", accel.x, accel.y, accel.z);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.074257, accel[1]);
    TEST_ASSERT(accel[2] == 0);
}

TEST_CASE("pronav left", "nav") {
    la_float range[3] = { 100, -10, 0 };
    la_float vel_i[3] = { 10, 0, 0 };
    la_float vel_t[3] = { 5, 0, 0 };

    la_float accel[3] = { 0, 0, 0 };

    bool result = nav_guidance_pronav_true(3.0, range, vel_i, vel_t, accel);

    //ESP_LOGI(TAG, "ax: %lf, ay: %lf, az: %lf", accel.x, accel.y, accel.z);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel[0] == 0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.074257, accel[1]);
    TEST_ASSERT(accel[2] == 0);
}

/* guidance tests */

TEST_CASE("guidance pronav", "nav") {
    nav_guidance_state_t ic = {
        .lat= 30.0,
        .lon= 50.0,
        .alt= 1000.0,
        .vel_north= 100.0,
        .vel_east= 0.0,
        .vel_up= 0.0
    };

    nav_guidance_state_t tg = {
        .lat= 30.1,
        .lon= 50.1,
        .alt= 1500.0,
        .vel_north= 10.0,
        .vel_east= 0.0,
        .vel_up= 0.0
    };

    nav_guidance_t g = {
        .N = 3.0,
        .max_roll_deg = 35,
        .max_pitch_deg = 25
    };

    nav_guidance_command_t c = {0};

    nav_guidance_compute_command(&g, &ic, &tg, &c);

    TEST_ASSERT(c.type == NAV_GUIDANCE_PRONAV);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.18, c.roll_cmd);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.013, c.pitch_cmd);
}

TEST_CASE("guidance pursuit", "nav") {
    nav_guidance_state_t ic = {
        .lat= 30.0,
        .lon= 50.0,
        .alt= 1000.0,
        .vel_north= 5.0,
        .vel_east= 0.0,
        .vel_up= 0.0
    };

    nav_guidance_state_t tg = {
        .lat= 30.1,
        .lon= 50.1,
        .alt= 1500.0,
        .vel_north= 10.0,
        .vel_east= 0.0,
        .vel_up= 0.0
    };

    nav_guidance_t g = {
        .N = 3.0,
        .max_roll_deg = 35,
        .max_pitch_deg = 25
    };

    nav_guidance_command_t c = {0};

    nav_guidance_compute_command(&g, &ic, &tg, &c);

    TEST_ASSERT(c.type == NAV_GUIDANCE_PURSUIT);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0, c.roll_cmd);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.129, c.pitch_cmd);
}
