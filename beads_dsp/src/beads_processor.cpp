#include "beads_processor.h"
#include "util/dsp_utils.h"
#include <cstring>
#include <cmath>
#include <new>
#include <algorithm>

namespace beads {

static constexpr size_t kImplAlignment = 16;

static size_t AlignUp(size_t size, size_t alignment = kImplAlignment) {
    return (size + alignment - 1) & ~(alignment - 1);
}

BeadsProcessor::MemoryRequirements BeadsProcessor::GetMemoryRequirements(float sample_rate) {
    MemoryRequirements req;

    size_t impl_bytes = AlignUp(sizeof(Impl));
    size_t recording_bytes = RecordingBuffer::RequiredBytes(
        sample_rate, kDefaultBufferDuration, 2);
    size_t reverb_bytes = kReverbBufferSize * sizeof(float);

    req.total_bytes = impl_bytes + AlignUp(recording_bytes) + AlignUp(reverb_bytes);
    req.alignment = kImplAlignment;
    return req;
}

void BeadsProcessor::Init(void* memory, size_t /*memory_size*/, float sample_rate) {
    uint8_t* ptr = static_cast<uint8_t*>(memory);

    // Placement-new the Impl at the start of the memory block
    impl_ = new (ptr) Impl();
    ptr += AlignUp(sizeof(Impl));

    impl_->sample_rate = sample_rate;
    impl_->params = BeadsParameters{};
    impl_->feedback_sample = {0.0f, 0.0f};
    impl_->prev_freeze = false;
    impl_->delay_mode = false;

    // Allocate recording buffer
    size_t recording_bytes = RecordingBuffer::RequiredBytes(
        sample_rate, kDefaultBufferDuration, 2);
    size_t num_frames = static_cast<size_t>(sample_rate * kDefaultBufferDuration);
    impl_->recording_buffer.Init(reinterpret_cast<float*>(ptr), num_frames, 2);
    ptr += AlignUp(recording_bytes);

    // Allocate reverb delay memory
    impl_->reverb.Init(reinterpret_cast<float*>(ptr), kReverbBufferSize, sample_rate);
    ptr += AlignUp(kReverbBufferSize * sizeof(float));

    // Initialize sub-processors
    impl_->grain_engine.Init(sample_rate, &impl_->recording_buffer);
    impl_->delay_engine.Init(sample_rate, &impl_->recording_buffer);
    impl_->saturation.Init();
    impl_->quality_processor.Init(sample_rate);
    impl_->auto_gain.Init(sample_rate);
    impl_->wavetable_osc.Init(sample_rate);

    // Feedback HP filter at ~20Hz to remove DC
    impl_->feedback_hp_l.Init();
    impl_->feedback_hp_l.SetFrequencyHz(20.0f, sample_rate);
    impl_->feedback_hp_l.SetQ(0.707f);
    impl_->feedback_hp_r.Init();
    impl_->feedback_hp_r.SetFrequencyHz(20.0f, sample_rate);
    impl_->feedback_hp_r.SetQ(0.707f);
}

void BeadsProcessor::SetWavetableProvider(WavetableProvider* provider) {
    impl_->wavetable_osc.SetProvider(provider);
}

void BeadsProcessor::SetParameters(const BeadsParameters& params) {
    impl_->params = params;
    impl_->delay_mode = (params.size >= 1.0f);

    // Configure reverb from parameters
    impl_->reverb.SetAmount(params.reverb);
    impl_->reverb.SetDecay(0.3f + params.reverb * 0.65f);
    impl_->reverb.SetDiffusion(0.7f);

    // Quality mode affects reverb LP: Tape=warmest, HiFi=brightest
    float reverb_lp;
    switch (params.quality_mode) {
        case QualityMode::kTape:      reverb_lp = 0.3f; break;
        case QualityMode::kCleanLoFi: reverb_lp = 0.5f; break;
        case QualityMode::kClouds:    reverb_lp = 0.6f; break;
        default:                      reverb_lp = 0.7f; break;
    }
    impl_->reverb.SetLpCutoff(reverb_lp);
}

void BeadsProcessor::Process(const StereoFrame* input, StereoFrame* output,
                              size_t num_frames) {
    auto& s = *impl_;  // shorthand

    // Detect freeze transitions for crossfade
    if (s.params.freeze != s.prev_freeze) {
        s.recording_buffer.StartFreezeCrossfade();
        s.prev_freeze = s.params.freeze;
    }

    // --- Per-sample input processing (steps 1-4) ---
    for (size_t i = 0; i < num_frames; ++i) {
        StereoFrame in = input[i];

        // Check for wavetable mode activation (silence detection)
        if (s.wavetable_osc.ShouldActivate(&in, 1)) {
            StereoFrame wt_out;
            s.wavetable_osc.Process(s.params.pitch, s.params.feedback, &wt_out, 1);
            in = wt_out;
        } else if (s.wavetable_osc.IsActive()) {
            s.wavetable_osc.Deactivate();
        }

        // 1. Auto-gain
        in = s.auto_gain.Process(in, s.params.manual_gain_db);

        // 2. Quality input processing
        in = s.quality_processor.ProcessInput(in, s.params.quality_mode);

        // 3. Feedback mix
        float feedback_gain = s.params.feedback * s.params.feedback;
        StereoFrame fb = {
            s.feedback_hp_l.ProcessHP(s.feedback_sample.l),
            s.feedback_hp_r.ProcessHP(s.feedback_sample.r)
        };
        in += fb * feedback_gain;
        in = s.saturation.LimitFeedback(in, s.params.quality_mode);

        // 4. Record to buffer (unless frozen)
        if (!s.params.freeze) {
            s.recording_buffer.Write(in);
        }
        if (s.recording_buffer.crossfading()) {
            s.recording_buffer.ProcessFreezeCrossfade();
        }

        output[i] = {0.0f, 0.0f};
    }

    // --- Block-based wet signal generation (step 5) ---
    StereoFrame wet[kMaxBlockSize];
    size_t remaining = num_frames;
    size_t offset = 0;

    while (remaining > 0) {
        size_t block = std::min(remaining, kMaxBlockSize);
        if (s.delay_mode) {
            s.delay_engine.Process(s.params, wet + offset, block);
        } else {
            s.grain_engine.Process(s.params, wet + offset, block);
        }
        offset += block;
        remaining -= block;
    }

    // --- Per-sample output processing (steps 6-10) ---
    for (size_t i = 0; i < num_frames; ++i) {
        // 6. Quality output processing
        StereoFrame wet_frame = s.quality_processor.ProcessOutput(wet[i], s.params.quality_mode);

        // 7. Capture feedback sample (before reverb)
        s.feedback_sample = wet_frame;

        // 8. Dry/wet crossfade (equal-power)
        StereoFrame mixed = EqualPowerCrossfade(input[i], wet_frame, s.params.dry_wet);

        // 9. Reverb
        float rev_l, rev_r;
        s.reverb.Process(mixed.l, mixed.r, &rev_l, &rev_r);
        mixed = {rev_l, rev_r};

        // 10. Output
        output[i] = mixed;
    }
}

bool BeadsProcessor::IsDelayMode() const {
    return impl_->delay_mode;
}

bool BeadsProcessor::IsWavetableMode() const {
    return impl_->wavetable_osc.IsActive();
}

int BeadsProcessor::ActiveGrainCount() const {
    return impl_->grain_engine.ActiveGrainCount();
}

float BeadsProcessor::InputLevel() const {
    return impl_->auto_gain.InputLevel();
}

} // namespace beads
