#ifndef NAV_VECTOR_H
#define NAV_VECTOR_H

#include <math.h>

typedef struct {
    double x;
    double y;
    double z;
} nav_vector_t;

nav_vector_t nav_vector_subtract(const nav_vector_t *a, const nav_vector_t *b);

nav_vector_t nav_vector_add(const nav_vector_t *a, const nav_vector_t *b);

nav_vector_t nav_vector_scale(const nav_vector_t *a, double factor);

void nav_vector_scale_self(nav_vector_t *a, double factor);

double nav_vector_dot(const nav_vector_t *a, const nav_vector_t *b);

#define nav_vector_norm(a) sqrt(nav_vector_dot(a, a));

nav_vector_t nav_vector_cross(const nav_vector_t *a, const nav_vector_t *b);

nav_vector_t nav_vector_unit(const nav_vector_t *a, double *norm);

void nav_vector_set_zero(nav_vector_t *a);

#endif
