#ifndef VECTOR_H
#define VECTOR_H

#include <stddef.h>

void vector_add(const float* a, const float* b, size_t size, float* result);

void vector_subtract(const float* a, const float* b, size_t size, float* result);

void vector_scale(const float* a, float scalar, size_t size, float* result);

float vector_dot(const float* a, const float* b, size_t size);

void vector_cross(const float* a, const float* b, float* result);

void vector_normalize(const float* a, size_t size, float* result);

float vector_length(const float* a, size_t size);

#endif