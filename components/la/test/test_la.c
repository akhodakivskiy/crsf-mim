#include <unity.h>
#include "la.h"

#define FLOAT_TOLERANCE 1e-5

TEST_CASE("test matrix addition", "la") {
    la_float a[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    la_float b[9] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
    la_float c[9];
    
    la_add(a, b, c, 9);
    
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 10.0, c[i]);
    }
}

TEST_CASE("test matrix subtraction", "la") {
    la_float a[9] = {10, 20, 30, 40, 50, 60, 70, 80, 90};
    la_float b[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    la_float c[9];
    
    la_sub(a, b, c, 9);
    
    la_float expected[9] = {9, 18, 27, 36, 45, 54, 63, 72, 81};
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, expected[i], c[i]);
    }
}

TEST_CASE("test matrix operations with different sizes", "la") {
    // Test 2x2 matrices
    la_float a2[4] = {1, 2, 3, 4};
    la_float b2[4] = {5, 6, 7, 8};
    la_float c2[4];
    
    la_add(a2, b2, c2, 4);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 6.0, c2[0]);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 12.0, c2[3]);
    
    // Test 5x5 matrices
    la_float a5[25], b5[25], c5[25];
    for (int i = 0; i < 25; i++) {
        a5[i] = i;
        b5[i] = 25 - i;
    }
    
    la_add(a5, b5, c5, 25);
    for (int i = 0; i < 25; i++) {
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 25.0, c5[i]);
    }
}
