#include "auto_gain.h"
#include "../util/dsp_utils.h"

#include <cmath>

namespace beads {

void AutoGain::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    envelope_ = 0.0f;
    gain_ = 1.0f;
    target_gain_ = 1.0f;

    // Fast attack: ~1ms time constant
    // coefficient = 1 - exp(-1 / (time_constant * sample_rate))
    attack_coeff_ = 1.0f - std::exp(-1.0f / (0.001f * sample_rate_));

    // Slow release: 5 seconds
    release_coeff_ = 1.0f - std::exp(-1.0f / (5.0f * sample_rate_));
}

StereoFrame AutoGain::Process(StereoFrame input, float manual_gain_db) {
    // Peak detection: take the max of left and right absolute values.
    // Guard against NaN input so the envelope follower doesn't latch to NaN.
    float abs_l = std::abs(input.l);
    float abs_r = std::abs(input.r);
    if (std::isnan(abs_l)) abs_l = 0.0f;
    if (std::isnan(abs_r)) abs_r = 0.0f;
    float peak = std::max(abs_l, abs_r);

    // Envelope follower: fast attack, slow release
    if (peak > envelope_) {
        ONE_POLE(envelope_, peak, attack_coeff_);
    } else {
        ONE_POLE(envelope_, peak, release_coeff_);
    }

    // Determine the gain to apply
    float gain_db;
    if (!std::isnan(manual_gain_db)) {
        // Manual gain mode: use the provided value directly
        gain_db = Clamp(manual_gain_db, kMinGainDb, kMaxGainDb);
    } else {
        // Auto-gain mode: lower input level -> higher gain, up to +32dB
        // Make up the difference between current level and 0dBFS.
        float input_db = GainToDb(envelope_);
        gain_db = Clamp(-input_db, kMinGainDb, kMaxGainDb);
    }

    target_gain_ = DbToGain(gain_db);

    // Smooth gain changes with ONE_POLE (coefficient ~0.0001 for very smooth)
    ONE_POLE(gain_, target_gain_, 0.0001f);

    return input * gain_;
}

float AutoGain::InputLevel() const {
    return envelope_;
}

} // namespace beads
