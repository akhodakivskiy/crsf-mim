#ifndef LA_H
#define LA_H

#include <float.h>
#include <math.h>
#include <stdbool.h>

/* matrix operations */

#ifdef LA_FLOAT_DOUBLE
typedef double la_float;
#define LA_EPSILON DBL_EPSILON
#define la_abs fabs
#define la_sqrt sqrt
#define la_atan2 atan2
#define la_acos acos
#define la_cos cos
#define la_tan tan
#define la_sin sin
#define la_asin asin
#define la_pow pow
#else
typedef float la_float;
#define LA_EPSILON FLT_EPSILON
#define la_abs fabsf
#define la_sqrt sqrtf
#define la_atan2 atan2f
#define la_acos acosf
#define la_cos cosf
#define la_tan tanf
#define la_sin sinf
#define la_asin asinf
#define la_pow powf
#endif
//typedef double la_float;
//#define LA_EPSILON DBL_EPSILON
//#define la_fabs fabs

la_float la_clamp(la_float x, la_float min, la_float max);
la_float la_clamp2(la_float x, la_float max);

void la_copy(const la_float *a, la_float *b, int size);

void la_sub(const la_float *a, const la_float *b, la_float *c, int size);
void la_sub_inplace(la_float *a, const la_float *b, int size);

void la_add(const la_float *a, const la_float *b, la_float *c, int size);
void la_add_inplace(la_float *a, const la_float *b, int size);

void la_scale(const la_float *a, la_float factor, la_float *b, int size);

// multiply a[m x n] and b[m x p] = c[m x p]
void la_mult(const la_float *a, const la_float *b, la_float *c, int m, int n, int p);

// identity matrix n*n
void la_eye(la_float *a, int n);

void la_zero(la_float *a, int n);

/* vector operations */

void la_vec_cross_3(const la_float *a, const la_float *b, la_float *c);

la_float la_vec_dot(const la_float *a, const la_float *b, int size);

la_float la_vec_norm(const la_float *a, int size);

la_float la_vec_unit(const la_float *a, la_float *b, int size);

void la_vec_rotate_rodrigues(
    const la_float *v, 
    const la_float *k, 
    la_float theta_rad, 
    la_float *out);

#endif
