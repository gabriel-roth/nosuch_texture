#include "grain.h"
#include "../buffer/recording_buffer.h"
#include "../util/dsp_utils.h"

#include <cmath>
#include <algorithm>

namespace beads {

void Grain::Init() {
    active_ = false;
    read_position_ = 0.0f;
    phase_increment_ = 0.0f;
    envelope_phase_ = 0.0f;
    envelope_increment_ = 0.0f;
    shape_ = 0.5f;
    pan_l_ = 1.0f;
    pan_r_ = 1.0f;
    pre_delay_ = 0;
}

void Grain::Start(const GrainParameters& params) {
    active_ = true;

    read_position_ = params.position;
    phase_increment_ = params.pitch_ratio;

    // Envelope increments once per sample over the grain's duration.
    // Phase goes from 0 to 1 over params.size samples.
    if (params.size > 0.0f) {
        envelope_increment_ = 1.0f / params.size;
    } else {
        envelope_increment_ = 1.0f;
    }
    envelope_phase_ = 0.0f;
    shape_ = Clamp(params.shape, 0.0f, 1.0f);

    // Equal-power panning.
    // pan: -1 (full left) to +1 (full right), 0 = center.
    float p = Clamp(params.pan * 0.5f + 0.5f, 0.0f, 1.0f);  // map to 0..1
    pan_l_ = std::cos(p * kPi * 0.5f);
    pan_r_ = std::sin(p * kPi * 0.5f);

    pre_delay_ = std::max(params.pre_delay, 0);
}

bool Grain::Process(const RecordingBuffer& buffer, float* out_l, float* out_r) {
    if (!active_) {
        *out_l = 0.0f;
        *out_r = 0.0f;
        return false;
    }

    // Pre-delay: output silence until the grain's sub-block offset is reached.
    if (pre_delay_ > 0) {
        --pre_delay_;
        *out_l = 0.0f;
        *out_r = 0.0f;
        return true;
    }

    // Envelope
    float env = ComputeEnvelope(envelope_phase_, shape_);
    envelope_phase_ += envelope_increment_;

    if (envelope_phase_ >= 1.0f) {
        active_ = false;
        *out_l = 0.0f;
        *out_r = 0.0f;
        return false;
    }

    // Read from recording buffer with Hermite interpolation.
    float sample_l = buffer.ReadHermite(0, read_position_);
    float sample_r = buffer.ReadHermite(1, read_position_);

    // Advance read position, wrapping around buffer size.
    read_position_ += phase_increment_;
    float buf_size = static_cast<float>(buffer.size());
    while (read_position_ >= buf_size) {
        read_position_ -= buf_size;
    }
    while (read_position_ < 0.0f) {
        read_position_ += buf_size;
    }

    // Apply envelope and panning.
    *out_l = sample_l * env * pan_l_;
    *out_r = sample_r * env * pan_r_;

    return true;
}

float Grain::ComputeEnvelope(float phase, float shape) {
    // Derive smoothness and slope from shape parameter.
    //
    // shape < 0.5:
    //   smoothness ramps from 0 (rectangular/trapezoidal) to 1 (Hann window)
    //   slope stays at 0.5 (symmetric attack/decay)
    //
    // shape >= 0.5:
    //   smoothness is 1.0 (always smooth)
    //   slope moves from 0.5 (symmetric) toward 0.0 (reversed envelope)
    float smoothness;
    float slope;
    if (shape < 0.5f) {
        smoothness = shape * 2.0f;
        slope = 0.5f;
    } else {
        smoothness = 1.0f;
        slope = 1.0f - (shape - 0.5f) * 2.0f;
    }

    // Trapezoidal envelope.
    // slope controls the attack/decay balance:
    //   slope near 0 → long attack, short decay
    //   slope = 0.5 → symmetric
    //   slope near 1 → short attack, long decay
    float trap;
    if (slope < 0.001f) {
        // Degenerate: fully reversed, ramp down over entire duration.
        trap = 1.0f - phase;
    } else if (slope > 0.999f) {
        // Degenerate: ramp up over entire duration.
        trap = phase;
    } else {
        if (phase < slope) {
            trap = phase / slope;
        } else if (phase < 1.0f - slope) {
            trap = 1.0f;
        } else {
            trap = (1.0f - phase) / slope;
        }
    }
    trap = Clamp(trap, 0.0f, 1.0f);

    // Hann window: 0.5 - 0.5 * cos(2*pi*phase)
    float hann = 0.5f - 0.5f * std::cos(kTwoPi * phase);

    // Blend between trapezoidal and Hann based on smoothness.
    return Crossfade(trap, hann, smoothness);
}

} // namespace beads
