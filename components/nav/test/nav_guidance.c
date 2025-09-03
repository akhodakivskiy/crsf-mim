#include <unity.h>

#include <esp_log.h>

#include "nav_guidance.h"
#include "nav_vector.h"

static const char *TAG = "TEST_NAV";

/* pursuit tests */

TEST_CASE("pursuit aligned", "[pursuit]") {
    nav_vector_t range = { .x = 100, .y = 0, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };

    nav_vector_t accel = {0};

    bool result = nav_guidance_pursuit(3.0, &range, &vel_i, &accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT(accel.y == 0);
    TEST_ASSERT(accel.z == 0);
}

TEST_CASE("pursuit right", "[pursuit]") {
    nav_vector_t range = { .x = 100, .y = 10, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };

    nav_vector_t accel = {0};

    bool result = nav_guidance_pursuit(3.0, &range, &vel_i, &accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 2.99, accel.y);
    TEST_ASSERT(accel.z == 0);
}

TEST_CASE("pursuit left", "[pursuit]") {
    nav_vector_t range = { .x = 100, .y = -10, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };

    nav_vector_t accel = {0};

    bool result = nav_guidance_pursuit(3.0, &range, &vel_i, &accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -2.99, accel.y);
    TEST_ASSERT(accel.z == 0);
}


/* proportional navigation tests */

TEST_CASE("pronav aligned", "[pronav]") {
    nav_vector_t range = { .x = 100, .y = 0, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };
    nav_vector_t vel_t = { .x = 5, .y = 0, .z = 0 };

    nav_vector_t accel = { .x = 0, .y = 0, .z = 0 };

    bool result = nav_guidance_pronav_true(3.0, &range, &vel_i, &vel_t, &accel);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT(accel.y == 0);
    TEST_ASSERT(accel.z == 0);
}

TEST_CASE("pronav right", "[pronav]") {
    nav_vector_t range = { .x = 100, .y = 10, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };
    nav_vector_t vel_t = { .x = 5, .y = 0, .z = 0 };

    nav_vector_t accel = { .x = 0, .y = 0, .z = 0 };

    bool result = nav_guidance_pronav_true(3.0, &range, &vel_i, &vel_t, &accel);

    //ESP_LOGI(TAG, "ax: %lf, ay: %lf, az: %lf", accel.x, accel.y, accel.z);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT_DOUBLE_WITHIN(0.0001, 0.074257, accel.y);
    TEST_ASSERT(accel.z == 0);
}

TEST_CASE("pronav left", "[pronav]") {
    nav_vector_t range = { .x = 100, .y = -10, .z = 0 };
    nav_vector_t vel_i = { .x = 10, .y = 0, .z = 0 };
    nav_vector_t vel_t = { .x = 5, .y = 0, .z = 0 };

    nav_vector_t accel = { .x = 0, .y = 0, .z = 0 };

    bool result = nav_guidance_pronav_true(3.0, &range, &vel_i, &vel_t, &accel);

    //ESP_LOGI(TAG, "ax: %lf, ay: %lf, az: %lf", accel.x, accel.y, accel.z);

    TEST_ASSERT(result == true);
    TEST_ASSERT(accel.x == 0);
    TEST_ASSERT_DOUBLE_WITHIN(0.0001, -0.074257, accel.y);
    TEST_ASSERT(accel.z == 0);
}

/* guidance tests */

TEST_CASE("guidance pronav", "[nav]") {
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
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.18, c.roll_cmd);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.013, c.pitch_cmd);
}

TEST_CASE("guidance pursuit", "[nav]") {
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
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, c.roll_cmd);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.129, c.pitch_cmd);
}
