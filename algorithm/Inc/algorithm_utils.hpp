#ifndef __ALGORITHM_UTILS_HPP__
#define __ALGORITHM_UTILS_HPP__

#include <stdint.h>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>

constexpr float kPI_ = 3.14159265358979323846f;
constexpr float kDivSqrt3_ = 0.57735026919f;
constexpr float kDivTwoSqrt3_ = 1.15470053838f;
constexpr float kSqrt3Div2_ = 0.86602540378f;
constexpr float kRad2Deg_ = 57.2958f;
constexpr float kDeg2Rad_ =  (kPI_ / (180.0f));

// ----------------
// Inline functions

template<typename T>
constexpr T SQ(const T& x){
    return x * x;
}

/**
 * @brief Small helper to make array with known size
 * in contrast to initializer lists the number of arguments
 * has to match exactly. Whereas initializer lists allow
 * less arguments.
 */
template <class T, class... Tail>
std::array<T, 1 + sizeof...(Tail)> MakeArray(T head, Tail... tail) {
    return std::array<T, 1 + sizeof...(Tail)>({head, tail...});
}

// To allow use of -ffast-math we need to have a special check for nan
// that bypasses the "ignore nan" flag
__attribute__((optimize("-fno-finite-math-only")))
inline bool IsNan(float x) {
    return __builtin_isnan(x);
}

// Round to integer
// Default rounding mode: round to nearest
inline int RoundInt(float x) {
#ifdef __arm__
    int res;
    asm("vcvtr.s32.f32   %[res], %[x]"
        : [res] "=X" (res)
        : [x] "w" (x) );
    return res;
#else
    return (int)nearbyint(x);
#endif
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float WrapPm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}

// Same as fmodf but result is positive and y must be positive
inline float FmodfPos(float x, float y) {
    float res = WrapPm(x, y);
    if (res < 0) res += y;
    return res;
}

inline float WrapPmPi(float x) {
    return WrapPm(x, 2 * kPI_);
}

// Evaluate polynomials in an efficient way
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
inline float HornerPolyEval(float x, const float *coeffs, size_t count) {
    float result = 0.0f;
    for (size_t idx = 0; idx < count; ++idx)
        result = (result * x) + coeffs[idx];
    return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int Mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}


#endif // ! __ALGORITHM_UTILS_HPP__
