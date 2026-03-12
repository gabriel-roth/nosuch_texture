#pragma once

#include "fx_engine.h"
#include "../util/dsp_utils.h"

namespace beads {

// Dattorro plate reverb, ported from Clouds' topology but retuned for Beads.
//
// Topology:
//   4 input allpass diffusers (pre-delay)
//   2-path feedback loop, each path has: 2 allpass + modulated delay
//   LP in feedback loop for frequency-dependent decay
//   LFO modulation on delay taps (slow, subtle)
//   Output tapped from multiple points in the feedback loop
//
// Retuning for Beads (vs Clouds):
//   Delay lengths ~15% shorter (smaller room / "Thoreau's cabin" character)
//   Warmer tone: lower LP cutoff in feedback loop
//   Quality mode shifts reverb LP coefficient
class Reverb {
public:
    void Init(float* buffer, size_t buffer_size, float sample_rate);
    void SetAmount(float amount);       // 0-1, overall wet level
    void SetDecay(float decay);         // 0-1, mapped to feedback gain
    void SetDiffusion(float diff);      // 0-1
    void SetLpCutoff(float cutoff);     // Normalized frequency for feedback LP
    void Process(float left_in, float right_in,
                 float* left_out, float* right_out);

private:
    FxEngine engine_;
    float sample_rate_ = 48000.0f;
    float amount_ = 0.0f;
    float decay_ = 0.5f;
    float diffusion_ = 0.7f;
    float lp_ = 0.7f;

    // LFO for modulation
    float lfo_phase_ = 0.0f;
    float lfo_increment_ = 0.0f;

    // Diffuser state (4 input allpass filters)
    float ap_state_[4] = {};

    // Feedback loop state
    float lp_state_l_ = 0.0f;
    float lp_state_r_ = 0.0f;
    float feedback_l_ = 0.0f;
    float feedback_r_ = 0.0f;

    // -- Pre-tuned delay offsets for Beads (~15% shorter than Clouds) --

    // Input diffuser allpass delays
    static constexpr size_t kApIn1 = 113;
    static constexpr size_t kApIn2 = 162;
    static constexpr size_t kApIn3 = 241;
    static constexpr size_t kApIn4 = 399;

    // Left feedback path delays
    static constexpr size_t kDelayL1 = 1559;
    static constexpr size_t kDelayL2 = 2656;

    // Right feedback path delays
    static constexpr size_t kDelayR1 = 1493;
    static constexpr size_t kDelayR2 = 2423;

    // Left feedback allpass delays
    static constexpr size_t kApL1 = 569;
    static constexpr size_t kApL2 = 829;

    // Right feedback allpass delays
    static constexpr size_t kApR1 = 601;
    static constexpr size_t kApR2 = 887;

    // Modulated delay variation: +/- 16 samples at ~0.5 Hz LFO
    static constexpr float kModDepth = 16.0f;
    static constexpr float kLfoHz = 0.5f;
};

} // namespace beads
