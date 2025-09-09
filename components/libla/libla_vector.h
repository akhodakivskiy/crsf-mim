#ifndef LIBLA_VECTOR_H
#define LIBLA_VECTOR_H

#include <math.h>

typedef struct {
    double x;
    double y;
    double z;
} la_vector_t;

la_vector_t la_vector_subtract(const la_vector_t *a, const la_vector_t *b);

la_vector_t la_vector_add(const la_vector_t *a, const la_vector_t *b);

la_vector_t la_vector_scale(const la_vector_t *a, double factor);

void la_vector_scale_self(la_vector_t *a, double factor);

double la_vector_dot(const la_vector_t *a, const la_vector_t *b);

#define la_vector_norm(a) sqrt(la_vector_dot(a, a));

la_vector_t la_vector_cross(const la_vector_t *a, const la_vector_t *b);

la_vector_t la_vector_unit(const la_vector_t *a, double *norm);

void la_vector_set_zero(la_vector_t *a);

#endif
