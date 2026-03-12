#include "wavetable_oscillator.h"
#include "../util/dsp_utils.h"
#include "../util/interpolation.h"

#include <cmath>

namespace beads {

void WavetableOscillator::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    provider_ = nullptr;
    phase_ = 0.0f;
    phase_increment_ = 0.0f;
    silence_counter_ = 0.0f;
    active_ = false;
}

void WavetableOscillator::SetProvider(WavetableProvider* provider) {
    provider_ = provider;
}

void WavetableOscillator::Process(float pitch_semitones, float bank_select,
                                   StereoFrame* output, size_t num_frames) {
    if (!provider_ || provider_->NumBanksAvailable() == 0) {
        // No wavetable data available: output silence
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }

    // ---------------------------------------------------------------
    // PITCH -> frequency via V/Oct
    // base_freq = 261.63 Hz (middle C, C4)
    // frequency = base_freq * SemitonesToRatio(pitch_semitones)
    // ---------------------------------------------------------------
    constexpr float kBaseFreq = 261.63f;  // Middle C
    // Clamp pitch to a sane range to prevent extreme frequencies that
    // could cause the phase wrapping loops below to run for billions of
    // iterations (or produce inf/NaN).
    float clamped_pitch = Clamp(pitch_semitones, -120.0f, 120.0f);
    float frequency = kBaseFreq * SemitonesToRatio(clamped_pitch);
    // phase_increment is how much of the wavetable we advance per sample
    // One full cycle = kWavetableSize entries
    phase_increment_ = frequency / sample_rate_ * static_cast<float>(kWavetableSize);

    // ---------------------------------------------------------------
    // bank_select (FEEDBACK parameter, 0-1) -> bank & waveform index
    // Integer part of (bank_select * total_waveforms) -> waveform index
    // Fractional part -> crossfade between adjacent waveforms
    // ---------------------------------------------------------------
    int num_banks = provider_->NumBanksAvailable();
    int waveforms_per_bank = provider_->WaveformsPerBank();
    int total_waveforms = num_banks * waveforms_per_bank;

    if (total_waveforms == 0) {
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }

    // Map bank_select to a continuous position across all waveforms
    float waveform_pos = bank_select * static_cast<float>(total_waveforms - 1);
    int waveform_idx_a = static_cast<int>(waveform_pos);
    float crossfade = waveform_pos - static_cast<float>(waveform_idx_a);

    // Clamp indices
    if (waveform_idx_a < 0) waveform_idx_a = 0;
    if (waveform_idx_a >= total_waveforms) waveform_idx_a = total_waveforms - 1;
    int waveform_idx_b = waveform_idx_a + 1;
    if (waveform_idx_b >= total_waveforms) waveform_idx_b = waveform_idx_a;

    // Resolve bank and index within bank for both waveforms
    int bank_a = waveform_idx_a / waveforms_per_bank;
    int index_a = waveform_idx_a % waveforms_per_bank;
    int bank_b = waveform_idx_b / waveforms_per_bank;
    int index_b = waveform_idx_b % waveforms_per_bank;

    const float* wave_a = provider_->GetWaveform(bank_a, index_a);
    const float* wave_b = provider_->GetWaveform(bank_b, index_b);

    if (!wave_a || !wave_b) {
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }

    // ---------------------------------------------------------------
    // Per-sample processing
    // ---------------------------------------------------------------
    for (size_t i = 0; i < num_frames; ++i) {
        // Integer and fractional parts of phase for table lookup
        int phase_int = static_cast<int>(phase_);
        float phase_frac = phase_ - static_cast<float>(phase_int);

        // Wrap phase_int into table range
        phase_int = phase_int & (kWavetableSize - 1);
        int next_idx = (phase_int + 1) & (kWavetableSize - 1);

        // Linear interpolation within each waveform
        float sample_a = InterpolateLinear(wave_a[phase_int], wave_a[next_idx], phase_frac);
        float sample_b = InterpolateLinear(wave_b[phase_int], wave_b[next_idx], phase_frac);

        // Crossfade between adjacent waveforms
        float sample = Crossfade(sample_a, sample_b, crossfade);

        // Output mono signal to both channels
        output[i] = {sample, sample};

        // Advance phase
        phase_ += phase_increment_;
        // Wrap phase to avoid float precision loss over time.
        // Use fmod instead of while loops to avoid unbounded iteration
        // if phase_increment_ is very large.
        float table_size_f = static_cast<float>(kWavetableSize);
        phase_ = std::fmod(phase_, table_size_f);
        if (phase_ < 0.0f) phase_ += table_size_f;
    }
}

bool WavetableOscillator::ShouldActivate(const StereoFrame* input, size_t num_frames) {
    float threshold_samples = kSilenceThresholdSeconds * sample_rate_;

    for (size_t i = 0; i < num_frames; ++i) {
        float peak = std::max(std::abs(input[i].l), std::abs(input[i].r));

        if (peak > kSilenceThreshold) {
            // Signal detected: reset silence counter and deactivate.
            silence_counter_ = 0.0f;
            active_ = false;
        } else {
            // Below threshold: accumulate silence duration
            silence_counter_ += 1.0f;
            if (silence_counter_ >= threshold_samples && !active_) {
                active_ = true;
            }
        }
    }

    return active_;
}

bool WavetableOscillator::IsActive() const {
    return active_;
}

void WavetableOscillator::Deactivate() {
    active_ = false;
    silence_counter_ = 0.0f;
}

} // namespace beads
