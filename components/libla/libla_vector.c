#include "libla_vector.h"

#include <float.h>

la_vector_t la_vector_subtract(const la_vector_t *a, const la_vector_t *b) {
    return (la_vector_t) {
        .x = a->x - b->x,
        .y = a->y - b->y,
        .z = a->z - b->z,
    };
}

la_vector_t la_vector_add(const la_vector_t *a, const la_vector_t *b) {
    return (la_vector_t) {
        .x = b->x + a->x,
        .y = b->y + a->y,
        .z = b->z + a->z,
    };
}

la_vector_t la_vector_scale(const la_vector_t *a, double factor) {
    return (la_vector_t) {
        .x = a->x * factor,
        .y = a->y * factor,
        .z = a->z * factor,
    };
}

void la_vector_scale_self(la_vector_t *a, double factor) {
    a->x = a->x * factor;
    a->y = a->y * factor;
    a->z = a->z * factor;
}

double la_vector_dot(const la_vector_t *a, const la_vector_t *b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

la_vector_t la_vector_cross(const la_vector_t *a, const la_vector_t *b) {
    return (la_vector_t) {
        .x = a->y * b->z - a->z * b->y,
        .y = a->z * b->x - a->x * b->z,
        .z = a->x * b->y - a->y * b->x,
    };
}

la_vector_t la_vector_unit(const la_vector_t *a, double *norm) {
    double n = la_vector_norm(a);
    if (norm != NULL) {
        *norm = n;
    }
    if (n < DBL_EPSILON) {
        return (la_vector_t) {0, 0, 0};
    } else {
        return la_vector_scale(a, 1.0 / n);
    }
}

void la_vector_set_zero(la_vector_t *a) {
    *a = (la_vector_t) {0, 0, 0};
}
