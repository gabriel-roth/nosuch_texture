#include "grain_scheduler.h"
#include "../util/dsp_utils.h"

#include <cmath>
#include <algorithm>

namespace beads {

void GrainScheduler::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    latched_phase_ = 0.0f;
    prev_gate_ = false;
    gate_phase_ = 0.0f;
    prev_clock_ = false;
    clock_period_ = 0.0f;
    samples_since_clock_ = 0;
    random_.Init(0xBEAD5EED);
}

float GrainScheduler::DensityToRate(float density) {
    // density 0.5 → 0 Hz (silence)
    // density 0.0 → ~100 Hz (fast, regular)
    // density 1.0 → ~100 Hz average (fast, random)
    //
    // Both halves map the distance from center to a rate.
    // The mapping is exponential for a musically useful range.
    float distance = std::abs(density - 0.5f) * 2.0f;  // 0..1
    if (distance < 0.001f) return 0.0f;

    // Exponential mapping: 0 → ~0.25 Hz, 1 → ~100 Hz
    return 0.25f * std::exp2(distance * 8.64f);  // 2^8.64 ≈ 400, 0.25*400=100
}

int GrainScheduler::Process(const BeadsParameters& params, size_t block_size,
                            int* trigger_samples, int max_triggers) {
    int trigger_count = 0;

    switch (params.trigger_mode) {
    case TriggerMode::kLatched: {
        // Internal phasor mode.
        // Draw initial modulated density; re-draw after each trigger.
        auto mod_density = [&]() {
            return Clamp(params.density + params.density_cv, 0.0f, 1.0f);
        };

        float eff_density = mod_density();
        float rate = DensityToRate(eff_density);
        if (rate <= 0.0f) break;

        float phase_inc = rate / sample_rate_;
        bool use_random = (eff_density > 0.5f);

        for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
            latched_phase_ += phase_inc;

            if (latched_phase_ >= 1.0f) {
                latched_phase_ -= 1.0f;
                trigger_samples[trigger_count++] = static_cast<int>(i);

                // Draw new modulated density for next grain interval.
                eff_density = mod_density();
                rate = DensityToRate(eff_density);
                phase_inc = rate / sample_rate_;
                use_random = (eff_density > 0.5f);

                if (use_random) {
                    float exp_rand = random_.NextExponential();
                    latched_phase_ = Clamp(1.0f - exp_rand, -2.0f, 0.99f);
                }
            }
        }
        break;
    }

    case TriggerMode::kGated: {
        // Gate high: generate grains at density-controlled rate (same phasor
        // as kLatched). Gate low: pause grain generation.
        bool gate = params.gate;

        if (gate) {
            auto mod_density = [&]() {
                return Clamp(params.density + params.density_cv, 0.0f, 1.0f);
            };

            float eff_density = mod_density();
            float rate = DensityToRate(eff_density);
            if (rate <= 0.0f) break;

            float phase_inc = rate / sample_rate_;
            bool use_random = (eff_density > 0.5f);

            for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
                latched_phase_ += phase_inc;

                if (latched_phase_ >= 1.0f) {
                    latched_phase_ -= 1.0f;
                    trigger_samples[trigger_count++] = static_cast<int>(i);

                    eff_density = mod_density();
                    rate = DensityToRate(eff_density);
                    phase_inc = rate / sample_rate_;
                    use_random = (eff_density > 0.5f);

                    if (use_random) {
                        float exp_rand = random_.NextExponential();
                        latched_phase_ = Clamp(1.0f - exp_rand, -2.0f, 0.99f);
                    }
                }
            }
        } else {
            // Gate low: reset phase so next gate-high starts firing immediately.
            latched_phase_ = 0.0f;
        }

        prev_gate_ = gate;
        break;
    }

    case TriggerMode::kClocked: {
        bool clock = params.gate;
        bool rising_edge = clock && !prev_clock_;
        // Cap the counter to prevent uint32_t overflow after very long
        // periods without a clock edge (~24h at 48kHz).  10 seconds of
        // samples is already far beyond any musical clock period.
        uint32_t max_samples = static_cast<uint32_t>(sample_rate_ * 10.0f);
        if (samples_since_clock_ < max_samples) {
            samples_since_clock_ += static_cast<uint32_t>(block_size);
        }

        if (rising_edge) {
            // Measure period between clock edges.
            if (clock_period_ > 0.0f) {
                // Smooth the period estimate.
                OnePole(clock_period_,
                         static_cast<float>(samples_since_clock_), 0.5f);
            } else {
                clock_period_ = static_cast<float>(samples_since_clock_);
            }
            samples_since_clock_ = 0;

            float eff_density = Clamp(params.density + params.density_cv, 0.0f, 1.0f);
            if (eff_density < 0.5f) {
                // CCW from noon: clock division.
                // Map 0.0 → /16, 0.5 → /1
                float div_amount = (0.5f - eff_density) * 2.0f;  // 0..1
                // Exponential mapping to division ratios: 1, 2, 4, 8, 16
                int division = 1 << static_cast<int>(div_amount * 4.0f);
                division = std::min(division, 16);

                // Use a simple counter to divide.
                // We repurpose gate_phase_ as a clock-division counter.
                // Increment first, then check: the first clock starts at 1
                // and triggers when counter reaches the division value.
                gate_phase_ += 1.0f;
                if (static_cast<int>(gate_phase_) >= division) {
                    gate_phase_ = 0.0f;
                    if (trigger_count < max_triggers) {
                        trigger_samples[trigger_count++] = 0;
                    }
                }
                // Note: for division=1, every clock triggers (1 >= 1).
                // For division=2, every other clock triggers (1 < 2, 2 >= 2).
                // This means the first clock after init (gate_phase_ starts
                // at 0) triggers for division=1 but is delayed by one for
                // division>=2.  This matches typical clock divider behavior
                // where the first output aligns with the Nth input clock.
            } else if (eff_density > 0.5f) {
                // CW from noon: probability trigger.
                // Map 0.5 → 0%, 1.0 → 100%
                float probability = (eff_density - 0.5f) * 2.0f;
                if (random_.NextFloat() < probability) {
                    if (trigger_count < max_triggers) {
                        trigger_samples[trigger_count++] = 0;
                    }
                }
            } else {
                // Exactly 0.5: trigger on every clock.
                if (trigger_count < max_triggers) {
                    trigger_samples[trigger_count++] = 0;
                }
            }
        }

        prev_clock_ = clock;
        break;
    }
    case TriggerMode::kMidi:
        // Falls through to identical gated logic — MIDI host sets gate/pitch/velocity.
        goto midi_gated;
    } // switch

    goto scheduler_done;

    midi_gated: {
        bool gate = params.gate;
        bool rising_edge = gate && !prev_gate_;

        if (rising_edge) {
            if (trigger_count < max_triggers) {
                trigger_samples[trigger_count++] = 0;
            }
            gate_phase_ = 0.0f;
        }

        if (gate && params.density != 0.5f) {
            float eff_density = Clamp(params.density + params.density_cv, 0.0f, 1.0f);
            if (eff_density > 0.5f) {
                float repeat_rate = DensityToRate(eff_density);
                if (repeat_rate > 0.0f) {
                    float phase_inc = repeat_rate / sample_rate_;
                    bool skip_first = rising_edge;
                    for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
                        gate_phase_ += phase_inc;
                        if (gate_phase_ >= 1.0f) {
                            gate_phase_ -= 1.0f;
                            if (skip_first) {
                                skip_first = false;
                            } else {
                                trigger_samples[trigger_count++] = static_cast<int>(i);
                            }
                        }
                    }
                }
            } else if (rising_edge && eff_density < 0.5f) {
                float burst_amount = (0.5f - eff_density) * 2.0f;
                int burst_count = static_cast<int>(burst_amount * 15.0f);
                for (int b = 0; b < burst_count && trigger_count < max_triggers; ++b) {
                    int offset = static_cast<int>(
                        (static_cast<float>(b + 1) / static_cast<float>(burst_count + 1))
                        * static_cast<float>(block_size));
                    offset = std::min(offset, static_cast<int>(block_size) - 1);
                    trigger_samples[trigger_count++] = offset;
                }
            }
        }

        prev_gate_ = gate;
    }

    scheduler_done:

    grain_triggered_ = (trigger_count > 0);
    return trigger_count;
}

} // namespace beads
