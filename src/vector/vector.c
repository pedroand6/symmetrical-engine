#include "vector.h"
#include <math.h>
#include <stdlib.h>

void vector_add(const float* a, const float* b, size_t size, float* result) {
    for (size_t i = 0; i < size; ++i) {
        result[i] = a[i] + b[i];
    }
}

void vector_subtract(const float* a, const float* b, size_t size, float* result) {
    for (size_t i = 0; i < size; ++i) {
        result[i] = a[i] - b[i];
    }
}

void vector_scale(const float* a, float scalar, size_t size, float* result) {
    for (size_t i = 0; i < size; ++i) {
        result[i] = a[i] * scalar;
    }
}

float vector_dot(const float* a, const float* b, size_t size) {
    float result = 0;
    for (size_t i = 0; i < size; ++i) {
        result += a[i] * b[i];
    }
    return result;
}

void vector_cross(const float* a, const float* b, float* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_length(const float* a, size_t size) {
    float length = 0;

    for (size_t i = 0; i < size; ++i) {
        length += a[i] * a[i];
    }

    return sqrt(length);
}

void vector_normalize(const float* a, size_t size, float* result) {
    float length = vector_length(a, size);

    for (size_t i = 0; i < size; ++i) {
        result[i] = a[i] / length;
    }
}