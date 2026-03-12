#pragma once

#include <cmath>
#include <algorithm>
#include "../../include/beads/types.h"

namespace beads {

// One-pole low-pass filter macro
// state += coefficient * (input - state)
#define ONE_POLE(state, input, coefficient) \
    state += (coefficient) * ((input) - (state))

inline float Clamp(float value, float min_val, float max_val) {
    return std::min(std::max(value, min_val), max_val);
}

inline float Crossfade(float a, float b, float mix) {
    return a + (b - a) * mix;
}

// Equal-power crossfade
inline void EqualPowerCrossfade(float dry, float wet, float mix,
                                 float& out) {
    float dry_gain = std::cos(mix * 0.5f * 3.14159265f);
    float wet_gain = std::sin(mix * 0.5f * 3.14159265f);
    out = dry * dry_gain + wet * wet_gain;
}

inline StereoFrame EqualPowerCrossfade(StereoFrame dry, StereoFrame wet, float mix) {
    float dry_gain = std::cos(mix * 0.5f * 3.14159265f);
    float wet_gain = std::sin(mix * 0.5f * 3.14159265f);
    return {
        dry.l * dry_gain + wet.l * wet_gain,
        dry.r * dry_gain + wet.r * wet_gain
    };
}

inline float SemitonesToRatio(float semitones) {
    return std::exp2(semitones / 12.0f);
}

// Soft clipping using tanh approximation
inline float SoftClip(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f) return 1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// Hard clip
inline float HardClip(float x, float limit = 1.0f) {
    return Clamp(x, -limit, limit);
}

// dB to linear gain
inline float DbToGain(float db) {
    return std::pow(10.0f, db / 20.0f);
}

// Linear gain to dB
inline float GainToDb(float gain) {
    return 20.0f * std::log10(std::max(gain, 1e-10f));
}

// Fast approximation of tanh
inline float FastTanh(float x) {
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Mu-law compression (for tape quality mode)
inline float MuLawCompress(float x, float mu = 255.0f) {
    float sign = x >= 0.0f ? 1.0f : -1.0f;
    return sign * std::log(1.0f + mu * std::abs(x)) / std::log(1.0f + mu);
}

inline float MuLawExpand(float x, float mu = 255.0f) {
    float sign = x >= 0.0f ? 1.0f : -1.0f;
    return sign * (std::pow(1.0f + mu, std::abs(x)) - 1.0f) / mu;
}

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kTwoPi = 2.0f * kPi;

} // namespace beads
