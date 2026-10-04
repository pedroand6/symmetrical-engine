#ifndef VECTOR_H
#define VECTOR_H

#include <stddef.h>

float* vector_add(const float* a, const float* b, size_t size);

float* vector_subtract(const float* a, const float* b, size_t size);

float* vector_scale(const float* a, float scalar, size_t size);

float vector_dot(const float* a, const float* b, size_t size);

float* vector_cross(const float* a, const float* b);

float* vector_normalize(const float* a, size_t size);

float vector_length(const float* a, size_t size);

#endif