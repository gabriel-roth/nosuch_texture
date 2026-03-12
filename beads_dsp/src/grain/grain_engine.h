#pragma once

#include <cstddef>
#include "../../include/beads/types.h"
#include "../../include/beads/parameters.h"
#include "../random/random.h"
#include "../random/attenurandomizer.h"
#include "grain.h"
#include "grain_scheduler.h"

namespace beads {

class RecordingBuffer;

class GrainEngine {
public:
    void Init(float sample_rate, RecordingBuffer* buffer);

    // Process one block of audio
    void Process(const BeadsParameters& params, StereoFrame* output,
                 size_t num_frames);

    int ActiveGrainCount() const;

private:
    static constexpr int kMaxGrains = 30;

    Grain grains_[kMaxGrains];
    GrainScheduler scheduler_;
    Attenurandomizer ar_time_;
    Attenurandomizer ar_size_;
    Attenurandomizer ar_shape_;
    Attenurandomizer ar_pitch_;
    Random random_;

    RecordingBuffer* buffer_ = nullptr;
    float sample_rate_ = 48000.0f;

    // Overlap normalization
    float overlap_count_lp_ = 0.0f;  // Smoothed active grain count

    // Allocate a grain from the pool (returns nullptr if full after stealing)
    Grain* AllocateGrain();

    // Compute grain parameters from BeadsParameters + attenurandomizers
    Grain::GrainParameters ComputeGrainParams(const BeadsParameters& params,
                                               int pre_delay);
};

} // namespace beads
