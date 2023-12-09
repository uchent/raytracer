#pragma once
#include "vec3.h"
#include <random>

inline float random_float(float min = 0.0, float max = 1.0) {
    static std::uniform_real_distribution<float> distribution(min, max);
    static std::mt19937 generator;
    return distribution(generator);
}