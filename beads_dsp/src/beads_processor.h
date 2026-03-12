#pragma once

// Internal header — defines BeadsProcessor::Impl
// Not part of the public API.

#include "../include/beads/beads.h"
#include "buffer/recording_buffer.h"
#include "grain/grain_engine.h"
#include "delay/delay_engine.h"
#include "fx/reverb.h"
#include "fx/saturation.h"
#include "quality/quality_processor.h"
#include "input/auto_gain.h"
#include "wavetable/wavetable_oscillator.h"
#include "util/svf.h"

namespace beads {

// The Impl struct holds all sub-processors. It is placement-new'd into
// the front of the user-provided memory block by Init().
struct BeadsProcessor::Impl {
    // Sub-processors
    RecordingBuffer recording_buffer;
    GrainEngine grain_engine;
    DelayEngine delay_engine;
    Reverb reverb;
    Saturation saturation;
    QualityProcessor quality_processor;
    AutoGain auto_gain;
    WavetableOscillator wavetable_osc;

    // Feedback HP filter (removes DC from feedback path)
    StateVariableFilter feedback_hp_l;
    StateVariableFilter feedback_hp_r;

    // Current parameters
    BeadsParameters params;
    float sample_rate = 48000.0f;

    // Feedback sample (captured after dry/wet, before reverb)
    StereoFrame feedback_sample = {0.0f, 0.0f};

    // Previous freeze state for crossfade detection
    bool prev_freeze = false;

    // Delay mode flag
    bool delay_mode = false;

    static constexpr size_t kAlignment = 16;
};

} // namespace beads
