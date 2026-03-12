#include "reverb.h"
#include <cmath>

namespace beads {

void Reverb::Init(float* buffer, size_t buffer_size, float sample_rate) {
    sample_rate_ = sample_rate;
    engine_.Init(buffer, buffer_size);

    amount_ = 0.0f;
    decay_ = 0.5f;
    diffusion_ = 0.7f;
    lp_ = 0.7f;

    lfo_phase_ = 0.0f;
    lfo_increment_ = kLfoHz / sample_rate_;

    for (int i = 0; i < 4; ++i) ap_state_[i] = 0.0f;
    lp_state_l_ = 0.0f;
    lp_state_r_ = 0.0f;
    feedback_l_ = 0.0f;
    feedback_r_ = 0.0f;
}

void Reverb::SetAmount(float amount) {
    amount_ = Clamp(amount, 0.0f, 1.0f);
}

void Reverb::SetDecay(float decay) {
    decay_ = Clamp(decay, 0.0f, 1.0f);
}

void Reverb::SetDiffusion(float diff) {
    diffusion_ = Clamp(diff, 0.0f, 1.0f);
}

void Reverb::SetLpCutoff(float cutoff) {
    lp_ = Clamp(cutoff, 0.0f, 1.0f);
}

void Reverb::Process(float left_in, float right_in,
                     float* left_out, float* right_out) {
    auto c = engine_.GetContext();

    // Map decay knob (0-1) to feedback gain.
    // Non-linear mapping: slow ramp then steep near 1.0 for long tails.
    float fb = 0.2f + decay_ * 0.75f;
    if (decay_ > 0.9f) {
        fb += (decay_ - 0.9f) * 0.5f;  // extra push near max
    }
    fb = Clamp(fb, 0.0f, 0.9995f);

    // LP coefficient for feedback loop (one-pole).
    // Higher lp_ = brighter. Warmer default than Clouds.
    float lp_coeff = lp_;

    // Diffusion coefficient for allpass stages
    float ap_coeff = diffusion_ * 0.75f;

    // ------------------------------------------------------------------
    // LFO for modulated delay taps
    // ------------------------------------------------------------------
    lfo_phase_ += lfo_increment_;
    if (lfo_phase_ >= 1.0f) lfo_phase_ -= 1.0f;

    float lfo_sin = std::sin(lfo_phase_ * kTwoPi);
    float lfo_cos = std::cos(lfo_phase_ * kTwoPi);
    float mod_l = kModDepth * lfo_sin;
    float mod_r = kModDepth * lfo_cos;

    // ------------------------------------------------------------------
    // Input diffusion (4 series allpass filters)
    // ------------------------------------------------------------------
    float input = (left_in + right_in) * 0.5f;

    // AP1
    float ap_read = c.Read(kApIn1);
    float ap_out = ap_read - ap_coeff * input;
    ap_state_[0] = input + ap_coeff * ap_out;
    // We don't need to write these into the delay line explicitly;
    // the engine write pointer is for the main feedback loop.
    // Instead, chain through local state.
    float diffused = ap_out;

    // For the remaining diffusers, treat the output of each as the
    // input to the next.  State is kept in ap_state_ for continuity.
    //
    // AP2
    ap_read = c.Read(kApIn2);
    ap_out = ap_read - ap_coeff * diffused;
    ap_state_[1] = diffused + ap_coeff * ap_out;
    diffused = ap_out;

    // AP3
    ap_read = c.Read(kApIn3);
    ap_out = ap_read - ap_coeff * diffused;
    ap_state_[2] = diffused + ap_coeff * ap_out;
    diffused = ap_out;

    // AP4
    ap_read = c.Read(kApIn4);
    ap_out = ap_read - ap_coeff * diffused;
    ap_state_[3] = diffused + ap_coeff * ap_out;
    diffused = ap_out;

    // ------------------------------------------------------------------
    // Feedback network (two interleaved paths, Dattorro topology)
    // ------------------------------------------------------------------

    // LEFT PATH --------------------------------------------------------
    // Read from modulated delay taps
    float delay_l1 = c.ReadInterpolated(static_cast<float>(kDelayL1) + mod_l);
    float delay_l2 = c.Read(kDelayL2);

    // Allpass in left path
    float ap_l1_read = c.Read(kApL1);
    float ap_l1 = ap_l1_read + ap_coeff * delay_l1;
    float left_tank = delay_l1 - ap_coeff * ap_l1;

    float ap_l2_read = c.Read(kApL2);
    float ap_l2 = ap_l2_read + ap_coeff * left_tank;
    left_tank = left_tank - ap_coeff * ap_l2;

    // One-pole LP in feedback
    ONE_POLE(lp_state_l_, left_tank + delay_l2, lp_coeff);
    feedback_l_ = lp_state_l_ * fb + diffused;

    // RIGHT PATH -------------------------------------------------------
    float delay_r1 = c.ReadInterpolated(static_cast<float>(kDelayR1) + mod_r);
    float delay_r2 = c.Read(kDelayR2);

    float ap_r1_read = c.Read(kApR1);
    float ap_r1 = ap_r1_read + ap_coeff * delay_r1;
    float right_tank = delay_r1 - ap_coeff * ap_r1;

    float ap_r2_read = c.Read(kApR2);
    float ap_r2 = ap_r2_read + ap_coeff * right_tank;
    right_tank = right_tank - ap_coeff * ap_r2;

    ONE_POLE(lp_state_r_, right_tank + delay_r2, lp_coeff);
    feedback_r_ = lp_state_r_ * fb + diffused;

    // ------------------------------------------------------------------
    // Write feedback back into the delay memory
    // ------------------------------------------------------------------
    c.Write(feedback_l_ * 0.5f + feedback_r_ * 0.5f);

    // ------------------------------------------------------------------
    // Output: tap from multiple points for decorrelated stereo
    // ------------------------------------------------------------------
    float wet_l = delay_l1 * 0.6f + delay_r2 * 0.4f;
    float wet_r = delay_r1 * 0.6f + delay_l2 * 0.4f;

    *left_out  = left_in  + amount_ * (wet_l - left_in);
    *right_out = right_in + amount_ * (wet_r - right_in);

    engine_.Advance();
}

} // namespace beads
