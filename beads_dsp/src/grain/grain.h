#pragma once

#include "../../include/beads/types.h"

namespace beads {

class RecordingBuffer;

class Grain {
public:
    struct GrainParameters {
        float position;         // Buffer position in frames (where to read)
        float size;             // Duration in samples
        float pitch_ratio;      // Playback rate (1.0 = normal, 2.0 = octave up)
        float shape;            // 0-1 envelope shape
        float pan;              // -1 to +1 stereo pan
        int pre_delay;          // Sub-block start offset in samples
    };

    void Init();

    // Trigger a new grain
    void Start(const GrainParameters& params);

    // Begin a short fade-out for grain stealing (avoids click)
    void StartFadeOut();

    // Process one sample, reading from the recording buffer
    // Returns true if grain is still active
    bool Process(const RecordingBuffer& buffer, float* out_l, float* out_r);

    bool active() const { return active_; }
    bool fading_out() const { return fading_out_; }

private:
    bool active_ = false;
    bool fading_out_ = false;

    // Fade-out for grain stealing
    static constexpr int kStealFadeSamples = 32;
    int steal_fade_counter_ = 0;

    // Read position (fractional for sub-sample accuracy)
    float read_position_ = 0.0f;
    float phase_increment_ = 0.0f;

    // Envelope
    float envelope_phase_ = 0.0f;
    float envelope_increment_ = 0.0f;
    float shape_ = 0.5f;

    // Stereo
    float pan_l_ = 1.0f;
    float pan_r_ = 1.0f;

    // Pre-delay counter
    int pre_delay_ = 0;

    // Compute envelope value from phase (0-1) and shape parameter
    float ComputeEnvelope(float phase, float shape);
};

} // namespace beads
