#include "quality_processor.h"
#include <cmath>

namespace beads {

void QualityProcessor::Init(float sample_rate) {
    sample_rate_ = sample_rate;

    input_lp_l_.Init();
    input_lp_r_.Init();
    output_lp_l_.Init();
    output_lp_r_.Init();

    // Set default cutoff frequencies — they'll be overridden per-mode in
    // Process*, but a sane default avoids uninitialized filter state.
    input_lp_l_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
    input_lp_r_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
    output_lp_l_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);
    output_lp_r_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);

    // Gentle resonance (Butterworth-ish)
    input_lp_l_.SetQ(0.707f);
    input_lp_r_.SetQ(0.707f);
    output_lp_l_.SetQ(0.707f);
    output_lp_r_.SetQ(0.707f);

    // LFO state
    wow_phase_ = 0.0f;
    flutter_phase_ = 0.0f;
    wow_increment_ = kWowHz / sample_rate_;
    flutter_increment_ = kFlutterHz / sample_rate_;

    noise_gen_.Init(0xBEAD5EED);
}

// ---------------------------------------------------------------------------
// ProcessInput: quality-mode coloring applied *before* recording into the
// grain buffer.
// ---------------------------------------------------------------------------
StereoFrame QualityProcessor::ProcessInput(StereoFrame input, QualityMode mode) {
    switch (mode) {
        case QualityMode::kHiFi:
            // No degradation
            return input;

        case QualityMode::kClouds: {
            // LP at ~14 kHz
            input_lp_l_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
            input_lp_r_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
            return {
                input_lp_l_.ProcessLP(input.l),
                input_lp_r_.ProcessLP(input.r)
            };
        }

        case QualityMode::kCleanLoFi:
            // Input is clean — LP only applied on the wet output path
            return input;

        case QualityMode::kTape: {
            // LP at ~8 kHz, then mono sum + tape hiss
            input_lp_l_.SetFrequencyHz(kTapeLpHz, sample_rate_);
            input_lp_r_.SetFrequencyHz(kTapeLpHz, sample_rate_);
            float filtered_l = input_lp_l_.ProcessLP(input.l);
            float filtered_r = input_lp_r_.ProcessLP(input.r);

            // Mono sum
            float mono = (filtered_l + filtered_r) * 0.5f;

            // Add subtle tape hiss
            float hiss = noise_gen_.NextBipolar() * kTapeHissLevel;
            mono += hiss;

            // Mu-law compression
            mono = MuLawCompress(mono, 255.0f);

            return { mono, mono };
        }
    }
    return input;
}

// ---------------------------------------------------------------------------
// ProcessOutput: quality-mode coloring applied *after* grain / delay readout.
// ---------------------------------------------------------------------------
StereoFrame QualityProcessor::ProcessOutput(StereoFrame input, QualityMode mode) {
    switch (mode) {
        case QualityMode::kHiFi:
            return input;

        case QualityMode::kClouds: {
            // 12-bit quantization: multiply by 2048, round, divide by 2048
            float l = std::round(input.l * kQuantScale) / kQuantScale;
            float r = std::round(input.r * kQuantScale) / kQuantScale;
            return { l, r };
        }

        case QualityMode::kCleanLoFi: {
            // LP at ~10 kHz on the wet path
            output_lp_l_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);
            output_lp_r_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);
            return {
                output_lp_l_.ProcessLP(input.l),
                output_lp_r_.ProcessLP(input.r)
            };
        }

        case QualityMode::kTape: {
            // Mu-law expansion (undo compression for playback fidelity),
            // then LP at 8 kHz again to soften any aliasing from
            // pitch-shifted readback.
            float l = MuLawExpand(input.l, 255.0f);
            float r = MuLawExpand(input.r, 255.0f);
            output_lp_l_.SetFrequencyHz(kTapeLpHz, sample_rate_);
            output_lp_r_.SetFrequencyHz(kTapeLpHz, sample_rate_);
            return {
                output_lp_l_.ProcessLP(l),
                output_lp_r_.ProcessLP(r)
            };
        }
    }
    return input;
}

// ---------------------------------------------------------------------------
// GetPitchModulation: returns a pitch *ratio* multiplier for the current
// sample.  Only tape mode produces modulation; all others return 1.0.
//
// Wow  = slow (~0.5 Hz), +/- 0.3 semitones
// Flutter = fast (~6 Hz), +/- 0.05 semitones
// ---------------------------------------------------------------------------
float QualityProcessor::GetPitchModulation(QualityMode mode) {
    if (mode != QualityMode::kTape) {
        return 1.0f;
    }

    // Advance LFO phases
    wow_phase_ += wow_increment_;
    if (wow_phase_ >= 1.0f) wow_phase_ -= 1.0f;

    flutter_phase_ += flutter_increment_;
    if (flutter_phase_ >= 1.0f) flutter_phase_ -= 1.0f;

    // Combined pitch deviation in semitones
    float wow_st     = kWowSemitones     * std::sin(wow_phase_     * kTwoPi);
    float flutter_st = kFlutterSemitones * std::sin(flutter_phase_ * kTwoPi);

    // Convert semitones offset to ratio
    return SemitonesToRatio(wow_st + flutter_st);
}

} // namespace beads
