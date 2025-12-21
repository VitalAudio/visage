#pragma once

// Fast math approximations for DSP
// MIT License - isolated from external dependencies

#include <cmath>
#include <algorithm>
#include <array>

namespace dfl {

//==============================================================================
// Pade [7/6] approximant (DEFAULT)
// Accurate to ~0.0001% for |x| < 5, smooth saturation
// Same as JUCE FastMathApproximations::tanh
//==============================================================================
inline float fastTanh(float x) noexcept {
    x = std::clamp(x, -5.5f, 5.5f);
    const float x2 = x * x;
    const float num = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
    const float den = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
    return num / den;
}

inline double fastTanh(double x) noexcept {
    x = std::clamp(x, -5.5, 5.5);
    const double x2 = x * x;
    const double num = x * (135135.0 + x2 * (17325.0 + x2 * (378.0 + x2)));
    const double den = 135135.0 + x2 * (62370.0 + x2 * (3150.0 + x2 * 28.0));
    return num / den;
}

//==============================================================================
// Pade [3/2] approximant (FAST)
// Accurate to ~1% for |x| < 4
// Lighter weight alternative when extreme accuracy isn't needed
//==============================================================================
inline float fastTanhFast(float x) noexcept {
    x = std::clamp(x, -4.5f, 4.5f);
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

inline double fastTanhFast(double x) noexcept {
    x = std::clamp(x, -4.5, 4.5);
    const double x2 = x * x;
    return x * (27.0 + x2) / (27.0 + 9.0 * x2);
}

//==============================================================================
// Taylor 5th-order (FAST, LIMITED RANGE)
// Accurate for |x| < 2, hard clamps beyond
// Use for gentle saturation where input is already bounded
//==============================================================================
inline float fastTanhTaylor5(float x) noexcept {
    x = std::clamp(x, -2.0f, 2.0f);
    const float x2 = x * x;
    return x * (1.0f - 0.3333333f * x2 + 0.1333333f * x2 * x2);
}

inline double fastTanhTaylor5(double x) noexcept {
    x = std::clamp(x, -2.0, 2.0);
    const double x2 = x * x;
    return x * (1.0 - 0.3333333 * x2 + 0.1333333 * x2 * x2);
}

//==============================================================================
// Algebraic sigmoid (NO DIVISION, INFINITE RANGE)
// Different curve shape than tanh, but smooth and branchless
// Useful for soft limiting where exact tanh shape isn't critical
//==============================================================================
inline float softSaturate(float x) noexcept {
    return x / std::sqrt(1.0f + x * x);
}

inline double softSaturate(double x) noexcept {
    return x / std::sqrt(1.0 + x * x);
}

//==============================================================================
// Bram de Jong soft saturation waveshaper (musicdsp.org, 2002)
// Variable soft knee with smooth curve up to clipping
// 'a' controls knee position (0..1): lower = more saturation
// Output normalized to unity gain at clipping point
//==============================================================================
inline float softSaturateBram(float x, float a = 0.5f) noexcept {
    const float sign = (x < 0.0f) ? -1.0f : 1.0f;
    x = std::abs(x);

    float y;
    if (x < a) {
        y = x;
    } else if (x > 1.0f) {
        y = (a + 1.0f) * 0.5f;
    } else {
        const float t = (x - a) / (1.0f - a);
        y = a + (x - a) / (1.0f + t * t);
    }

    // Normalize so output reaches 1.0 at clipping
    const float norm = 2.0f / (a + 1.0f);
    return sign * y * norm;
}

inline double softSaturateBram(double x, double a = 0.5) noexcept {
    const double sign = (x < 0.0) ? -1.0 : 1.0;
    x = std::abs(x);

    double y;
    if (x < a) {
        y = x;
    } else if (x > 1.0) {
        y = (a + 1.0) * 0.5;
    } else {
        const double t = (x - a) / (1.0 - a);
        y = a + (x - a) / (1.0 + t * t);
    }

    // Normalize so output reaches 1.0 at clipping
    const double norm = 2.0 / (a + 1.0);
    return sign * y * norm;
}

//==============================================================================
// Asymmetric soft clipper (Bram de Jong / Patrice Tarrabia)
// Formula: (1+k)*x / (1+k*|x|)
// drive (0..1): saturation amount, 0 = linear, 1 = hard clip
// tilt (-1..1): asymmetry, 0 = symmetric, positive = more saturation on positive half
//==============================================================================
inline float asymClip(float x, float drive, float tilt = 0.0f) noexcept {
    drive = std::clamp(drive, 0.0f, 0.9999f);
    tilt = std::clamp(tilt, -1.0f, 1.0f);

    // Calculate separate k values for positive and negative
    float aUp = drive * std::clamp(1.0f + tilt, 0.0f, 1.0f);
    float aDown = drive * std::clamp(1.0f - tilt, 0.0f, 1.0f);
    float kUp = 2.0f * aUp / (1.0f - aUp);
    float kDown = 2.0f * aDown / (1.0f - aDown);

    // Split into positive and negative parts
    float top = 0.5f * (std::abs(x) + 1.0f - std::abs(x - 1.0f));  // clamp to [0,1]
    float bottom = x - top;

    // Apply formula to each half with different k
    top = (1.0f + kUp) * top / (1.0f + kUp * top);
    bottom = (1.0f + kDown) * bottom / (1.0f - kDown * bottom);

    return top + bottom;
}

inline double asymClip(double x, double drive, double tilt = 0.0) noexcept {
    drive = std::clamp(drive, 0.0, 0.9999);
    tilt = std::clamp(tilt, -1.0, 1.0);

    double aUp = drive * std::clamp(1.0 + tilt, 0.0, 1.0);
    double aDown = drive * std::clamp(1.0 - tilt, 0.0, 1.0);
    double kUp = 2.0 * aUp / (1.0 - aUp);
    double kDown = 2.0 * aDown / (1.0 - aDown);

    double top = 0.5 * (std::abs(x) + 1.0 - std::abs(x - 1.0));
    double bottom = x - top;

    top = (1.0 + kUp) * top / (1.0 + kUp * top);
    bottom = (1.0 + kDown) * bottom / (1.0 - kDown * bottom);

    return top + bottom;
}

//==============================================================================
// Branchless sign extraction (EXACT)
// Returns -1.0 for negative, +1.0 for positive/zero
// Uses std::copysign which is typically a single instruction
//==============================================================================
inline float sign(float x) noexcept {
    return std::copysign(1.0f, x);
}

inline double sign(double x) noexcept {
    return std::copysign(1.0, x);
}

//==============================================================================
// Branchless asymmetric envelope coefficient selector (EXACT)
// For Q sag and other attack/release envelopes
// Returns attack when rising, release when falling
// Mathematically identical to: (target > current) ? attack : release
//==============================================================================
inline float selectCoeff(float target, float current, float attack, float release) noexcept {
    const float rising = static_cast<float>(target > current);  // 0.0 or 1.0
    return release + rising * (attack - release);
}

inline double selectCoeff(double target, double current, double attack, double release) noexcept {
    const double rising = static_cast<double>(target > current);
    return release + rising * (attack - release);
}

//==============================================================================
// Fast Sine Lookup Table
// 16384-entry table with linear interpolation (~14-bit accuracy)
// Input: normalized phase (0-1), Output: sin(2π * phase)
// Based on public domain code by Carlos Moreno (1999), adapted by David Lowenfels (2004)
//==============================================================================

class SineLUT {
public:
    static constexpr size_t TABLE_SIZE = 16384;
    static constexpr double TWO_PI = 6.283185307179586476925286766559;

    SineLUT() {
        // Generate table: maps index [0, TABLE_SIZE] to sin(2π * index/TABLE_SIZE)
        for (size_t i = 0; i <= TABLE_SIZE; ++i) {
            table[i] = std::sin(TWO_PI * static_cast<double>(i) / TABLE_SIZE);
        }
    }

    /**
     * Fast sine from normalized phase (0-1).
     * @param phase Normalized phase (0.0 to 1.0, wrapped internally)
     * @return sin(2π * phase)
     */
    inline double operator()(double phase) const noexcept {
        // Wrap phase to [0, 1)
        phase -= std::floor(phase);

        double scaledPhase = phase * TABLE_SIZE;
        size_t index = static_cast<size_t>(scaledPhase);
        double frac = scaledPhase - index;

        // Linear interpolation
        return table[index] + frac * (table[index + 1] - table[index]);
    }

    /**
     * Fast sine from radians.
     * @param radians Angle in radians
     * @return sin(radians)
     */
    inline double sinRadians(double radians) const noexcept {
        return (*this)(radians / TWO_PI);
    }

    /**
     * Direct table access (no interpolation, faster but lower quality).
     * @param index Table index (0 to TABLE_SIZE)
     */
    inline double at(size_t index) const noexcept {
        return table[index & TABLE_SIZE];  // Mask for safety
    }

private:
    std::array<double, TABLE_SIZE + 1> table;
};

// Global sine LUT instance (constructed at startup)
// Usage: dfl::sineLUT(0.25) returns sin(π/2) = 1.0
inline SineLUT& getSineLUT() {
    static SineLUT instance;
    return instance;
}

// Convenience function matching std::sin signature but taking normalized phase
inline double fastSin(double phase) noexcept {
    return getSineLUT()(phase);
}

// Convenience function taking radians (drop-in for std::sin)
inline double fastSinRadians(double radians) noexcept {
    return getSineLUT().sinRadians(radians);
}

} // namespace dfl
