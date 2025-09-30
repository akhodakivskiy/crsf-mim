#include "la.h"

#include <string.h>
#include <math.h>

la_float la_clamp(la_float x, la_float min, la_float max) {
    return x > max ? max : ((x < min) ? min : x);
}

la_float la_clamp2(la_float x, la_float max) {
    return la_clamp(x, -max, max);
}

void la_copy(const la_float *a, la_float *b, int size) {
    memcpy(b, a, size * sizeof(la_float));
}

void la_sub(const la_float *a, const la_float *b, la_float *c, int size) {
    for (int i = 0; i < size; i++) {
        c[i] = a[i] - b[i];
    }
}

void la_sub_inplace(la_float *a, const la_float *b, int size) {
    for (int i = 0; i < size; i++) {
        a[i] -= b[i];
    }
}

void la_add(const la_float *a, const la_float *b, la_float *c, int size) {
    for (int i = 0; i < size; i++) {
        c[i] = a[i] + b[i];
    }
}

void la_add_inplace(la_float *a, const la_float *b, int size) {
    for (int i = 0; i < size; i++) {
        a[i] += b[i];
    }
}

void la_scale(const la_float *a, la_float factor, la_float *b, int size) {
    for (int i = 0; i < size; i++) {
        b[i] = a[i] * factor;
    }
}

void la_mult(const la_float *a, const la_float *b, la_float *c, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            la_float sum = 0.0;
            for (int k = 0; k < n; k++) {
                sum += a[i * n + k] * b[k * p + j];
            }
            c[i * p + j] = sum;
        }
    }
}

void la_eye(la_float *a, int n) {
    memset(a, 0, n * n * sizeof(la_float));
    for (int i = 0; i < n; i++) {
        a[i * n + i] = 1.0;
    }
}

void la_zero(la_float *a, int n) {
    memset(a, 0, n * sizeof(la_float));
}

void la_transpose(const la_float *a, la_float *b, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            b[j * m + i] = a[i * n + j];
        }
    }
}

/* vector operations */

void la_vec_cross_3(const la_float *a, const la_float *b, la_float *c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

la_float la_vec_dot(const la_float *a, const la_float *b, int size) {
    la_float res = 0.0;
    for (int i = 0; i < size; i++) {
        res += a[i] * b[i];
    }
    return res;
}

la_float la_vec_norm(const la_float *a, int size) {
    return la_sqrt(la_vec_dot(a, a, size));
}

la_float la_vec_unit(const la_float *a, la_float *b, int size) {
    la_float norm = la_vec_norm(a, size);
    if (norm > LA_EPSILON) {
        la_scale(a, 1.0 / norm, b, size);
        return norm;
    } else {
        return 0.0;
    }
}

void la_vec_rotate_rodrigues(
    const la_float *v, 
    const la_float *k, 
    la_float theta_rad, 
    la_float *out) {

    la_zero(out, 3);
    la_float theta_cos = la_cos(theta_rad);

    // v * cos(theta)
    la_float t[3] = {0};
    la_scale(v, theta_cos, t, 3);
    la_add(out, t, out, 3);

    // (k x v) * sin(theta)
    la_vec_cross_3(k, v, t);
    la_scale(t, la_sin(theta_rad), t, 3);
    la_add(out, t, out, 3);

    // k * (k . v) * (1 - cos(theta))
    la_scale(k, la_vec_dot(k, v, 3) * (1 - theta_cos), t, 3);
    la_add(out, t, out, 3);
}
