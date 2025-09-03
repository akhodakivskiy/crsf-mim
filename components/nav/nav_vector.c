#include "nav_vector.h"

#include <float.h>

nav_vector_t nav_vector_subtract(const nav_vector_t *a, const nav_vector_t *b) {
    return (nav_vector_t) {
        .x = a->x - b->x,
        .y = a->y - b->y,
        .z = a->z - b->z,
    };
}

nav_vector_t nav_vector_add(const nav_vector_t *a, const nav_vector_t *b) {
    return (nav_vector_t) {
        .x = b->x + a->x,
        .y = b->y + a->y,
        .z = b->z + a->z,
    };
}

nav_vector_t nav_vector_scale(const nav_vector_t *a, double factor) {
    return (nav_vector_t) {
        .x = a->x * factor,
        .y = a->y * factor,
        .z = a->z * factor,
    };
}

void nav_vector_scale_self(nav_vector_t *a, double factor) {
    a->x = a->x * factor;
    a->y = a->y * factor;
    a->z = a->z * factor;
}

double nav_vector_dot(const nav_vector_t *a, const nav_vector_t *b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

nav_vector_t nav_vector_cross(const nav_vector_t *a, const nav_vector_t *b) {
    return (nav_vector_t) {
        .x = a->y * b->z - a->z * b->y,
        .y = a->z * b->x - a->x * b->z,
        .z = a->x * b->y - a->y * b->x,
    };
}

nav_vector_t nav_vector_unit(const nav_vector_t *a, double *norm) {
    double n = nav_vector_norm(a);
    if (norm != NULL) {
        *norm = n;
    }
    if (n < DBL_EPSILON) {
        return (nav_vector_t) {0, 0, 0};
    } else {
        return nav_vector_scale(a, 1.0 / n);
    }
}

void nav_vector_set_zero(nav_vector_t *a) {
    *a = (nav_vector_t) {0, 0, 0};
}
