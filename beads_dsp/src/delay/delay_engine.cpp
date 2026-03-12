#include "delay_engine.h"
#include "../buffer/recording_buffer.h"
#include "../util/dsp_utils.h"
#include "../util/interpolation.h"

#include <cmath>

namespace beads {

void DelayEngine::Init(float sample_rate, RecordingBuffer* buffer) {
    sample_rate_ = sample_rate;
    buffer_ = buffer;

    read_position_ = 0.0f;
    delay_time_samples_ = 0.0f;
    delay_time_target_ = 0.0f;
    smoothed_delay_time_ = 0.0f;

    pitch_shift_phase_[0] = 0.0f;
    pitch_shift_phase_[1] = 0.5f;
    pitch_shift_increment_ = 0.0f;

    envelope_phase_ = 0.0f;

    frozen_ = false;
    loop_start_ = 0.0f;
    loop_length_ = 0.0f;
}

float DelayEngine::SlerpDelay(float target, float current, float coefficient) {
    return current + coefficient * (target - current);
}

void DelayEngine::Process(const BeadsParameters& params,
                          StereoFrame* output,
                          size_t num_frames) {
    if (!buffer_) return;

    const float buffer_size = static_cast<float>(buffer_->size());

    // ---------------------------------------------------------------
    // DENSITY -> delay time (exponential mapping)
    // density=0 -> full buffer length, density=1.0 -> ~1ms (audio rate)
    // ---------------------------------------------------------------
    const float min_delay_samples = sample_rate_ * 0.001f;  // 1ms
    const float max_delay_samples = buffer_size;

    // Exponential mapping: delay = max * exp(-k * density)
    // At density=0: delay = max
    // At density=1: delay ~= min
    // k = ln(max / min)
    float k = std::log(max_delay_samples / min_delay_samples);
    delay_time_target_ = max_delay_samples * std::exp(-k * params.density);

    // ---------------------------------------------------------------
    // Secondary tap: golden ratio of primary, fades in when density > 0.5
    // ---------------------------------------------------------------
    float secondary_mix = 0.0f;
    if (params.density > 0.5f) {
        secondary_mix = (params.density - 0.5f) * 2.0f;  // 0 at 0.5, 1 at 1.0
    }

    // ---------------------------------------------------------------
    // PITCH -> pitch shifter ratio
    // ---------------------------------------------------------------
    float pitch_ratio = SemitonesToRatio(params.pitch);
    pitch_shift_increment_ = pitch_ratio;

    // ---------------------------------------------------------------
    // SHAPE -> tremolo / slicer envelope
    // 0.0 = no modulation, 0.5 = sine tremolo, 1.0 = hard slicer
    // Rate derived from delay time for tempo-synced feel
    // ---------------------------------------------------------------
    float envelope_rate = 0.0f;
    if (smoothed_delay_time_ > 0.0f) {
        // One full tremolo cycle per delay time period
        envelope_rate = 1.0f / smoothed_delay_time_;
    }

    // ---------------------------------------------------------------
    // FREEZE handling
    // ---------------------------------------------------------------
    if (params.freeze && !frozen_) {
        // Entering freeze: capture loop slice
        frozen_ = true;
        loop_start_ = static_cast<float>(buffer_->write_head());
        // SIZE selects loop duration (scaled from minimum to buffer length)
        float min_loop = sample_rate_ * 0.01f;  // 10ms minimum loop
        loop_length_ = min_loop + params.size * (buffer_size - min_loop);
    } else if (!params.freeze && frozen_) {
        // Exiting freeze
        frozen_ = false;
    }

    // If frozen, TIME selects which portion of the buffer to loop
    if (frozen_) {
        loop_start_ = params.time * (buffer_size - loop_length_);
        float min_loop = sample_rate_ * 0.01f;
        loop_length_ = min_loop + params.size * (buffer_size - min_loop);
    }

    // ---------------------------------------------------------------
    // Per-sample processing
    // ---------------------------------------------------------------
    for (size_t i = 0; i < num_frames; ++i) {
        // Smooth delay time changes (ONE_POLE, coefficient ~0.001)
        ONE_POLE(smoothed_delay_time_, delay_time_target_, 0.001f);

        float current_delay = smoothed_delay_time_;

        // ---------------------------------------------------------------
        // Compute primary read position
        // ---------------------------------------------------------------
        float primary_pos;
        if (frozen_) {
            // In freeze mode, read from the captured loop
            read_position_ += pitch_ratio;
            // Wrap within loop
            while (read_position_ >= loop_length_) {
                read_position_ -= loop_length_;
            }
            while (read_position_ < 0.0f) {
                read_position_ += loop_length_;
            }
            primary_pos = loop_start_ + read_position_;
            // Wrap within buffer
            while (primary_pos >= buffer_size) {
                primary_pos -= buffer_size;
            }
            while (primary_pos < 0.0f) {
                primary_pos += buffer_size;
            }
        } else {
            // Normal delay mode: read position relative to write head
            float write_pos = static_cast<float>(buffer_->write_head());
            primary_pos = write_pos - current_delay;
            while (primary_pos < 0.0f) {
                primary_pos += buffer_size;
            }
            while (primary_pos >= buffer_size) {
                primary_pos -= buffer_size;
            }
        }

        // ---------------------------------------------------------------
        // Pitch-shifted read using rotary-head technique
        // Two overlapping read-heads, 180 degrees apart in phase
        // Triangular crossfade: one fading in while the other fades out
        // ---------------------------------------------------------------
        StereoFrame pitched_sample = {0.0f, 0.0f};
        float window_size = current_delay * 0.5f;
        if (window_size < 64.0f) window_size = 64.0f;

        for (int head = 0; head < 2; ++head) {
            // Advance phase for this head
            float phase = pitch_shift_phase_[head];

            // Triangular crossfade envelope based on phase (0-1)
            // Phase 0..0.5 -> ramp up, 0.5..1 -> ramp down
            float tri_env;
            if (phase < 0.5f) {
                tri_env = phase * 2.0f;
            } else {
                tri_env = 2.0f - phase * 2.0f;
            }

            // Offset from primary position based on phase
            float head_offset = phase * window_size;
            float head_pos = primary_pos + head_offset;
            while (head_pos >= buffer_size) {
                head_pos -= buffer_size;
            }
            while (head_pos < 0.0f) {
                head_pos += buffer_size;
            }

            // Read from buffer with Hermite interpolation
            float left = buffer_->ReadHermite(0, head_pos);
            float right = buffer_->ReadHermite(1, head_pos);

            pitched_sample.l += left * tri_env;
            pitched_sample.r += right * tri_env;

            // Advance the phase: increment determines pitch shift
            // pitch_ratio > 1 = faster read = higher pitch
            // The phase accumulator wraps at 1.0 (one window traversal)
            pitch_shift_phase_[head] += (pitch_shift_increment_ - 1.0f) / window_size;
            if (pitch_shift_phase_[head] >= 1.0f) {
                pitch_shift_phase_[head] -= 1.0f;
            }
            if (pitch_shift_phase_[head] < 0.0f) {
                pitch_shift_phase_[head] += 1.0f;
            }
        }

        // ---------------------------------------------------------------
        // Secondary tap (golden ratio of primary delay time)
        // ---------------------------------------------------------------
        StereoFrame mixed_sample = pitched_sample;

        if (secondary_mix > 0.0f) {
            float secondary_delay = current_delay * secondary_tap_ratio_;
            if (secondary_delay > buffer_size) {
                secondary_delay = buffer_size;
            }

            float secondary_pos;
            if (frozen_) {
                // In freeze mode, secondary tap also reads from the loop
                float sec_offset = secondary_delay;
                while (sec_offset >= loop_length_) {
                    sec_offset -= loop_length_;
                }
                secondary_pos = loop_start_ + sec_offset;
                while (secondary_pos >= buffer_size) {
                    secondary_pos -= buffer_size;
                }
                while (secondary_pos < 0.0f) {
                    secondary_pos += buffer_size;
                }
            } else {
                float write_pos = static_cast<float>(buffer_->write_head());
                secondary_pos = write_pos - secondary_delay;
                while (secondary_pos < 0.0f) {
                    secondary_pos += buffer_size;
                }
                while (secondary_pos >= buffer_size) {
                    secondary_pos -= buffer_size;
                }
            }

            float sec_l = buffer_->ReadHermite(0, secondary_pos);
            float sec_r = buffer_->ReadHermite(1, secondary_pos);

            // Mix both taps with equal gain
            float primary_gain = 1.0f - secondary_mix * 0.5f;
            float secondary_gain = secondary_mix * 0.5f;

            mixed_sample.l = pitched_sample.l * primary_gain + sec_l * secondary_gain;
            mixed_sample.r = pitched_sample.r * primary_gain + sec_r * secondary_gain;
        }

        // ---------------------------------------------------------------
        // SHAPE -> tremolo / slicer amplitude envelope
        // ---------------------------------------------------------------
        float env_gain = 1.0f;
        if (params.shape > 0.001f) {
            // Advance envelope phase
            envelope_phase_ += envelope_rate;
            if (envelope_phase_ >= 1.0f) {
                envelope_phase_ -= 1.0f;
            }

            // Generate envelope waveform based on shape
            float sine_env = 0.5f + 0.5f * std::sin(envelope_phase_ * kTwoPi);

            if (params.shape <= 0.5f) {
                // 0.0 -> 0.5: blend from steady (1.0) to sine tremolo
                float tremolo_depth = params.shape * 2.0f;  // 0 at shape=0, 1 at shape=0.5
                env_gain = 1.0f - tremolo_depth * (1.0f - sine_env);
            } else {
                // 0.5 -> 1.0: morph from sine tremolo to hard slicer (square gating)
                float slicer_blend = (params.shape - 0.5f) * 2.0f;  // 0 at 0.5, 1 at 1.0
                float square_env = (envelope_phase_ < 0.5f) ? 1.0f : 0.0f;
                env_gain = Crossfade(sine_env, square_env, slicer_blend);
            }
        }

        mixed_sample.l *= env_gain;
        mixed_sample.r *= env_gain;

        output[i] = mixed_sample;
    }
}

} // namespace beads
