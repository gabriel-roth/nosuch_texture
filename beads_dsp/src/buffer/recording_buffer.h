#pragma once

#include <cstddef>
#include "../../include/beads/types.h"

namespace beads {

// Circular stereo recording buffer with interpolated reads.
//
// Stores interleaved float samples (L, R, L, R, ...) with an extra tail of
// kInterpolationTail frames appended after the main buffer. The tail mirrors
// the first frames so Hermite interpolation at the wrap boundary needs no
// special-casing.
//
// Memory is externally allocated. Call RequiredBytes() to learn the size,
// allocate, then pass the pointer to Init().
class RecordingBuffer {
public:
    void Init(float* buffer, size_t num_frames, int num_channels = 2);

    // Write one frame and advance the write head.
    void Write(float left, float right);
    void Write(const StereoFrame& frame);

    // Read a single channel with Hermite cubic interpolation.
    // |position| is in frames (0 .. size_-1), fractional part drives
    // the interpolation.
    float ReadHermite(int channel, float position) const;

    // Read a single channel with linear interpolation (cheaper).
    float ReadLinear(int channel, float position) const;

    // Freeze-transition crossfade.  Call StartFreezeCrossfade() when the
    // freeze state changes, then call ProcessFreezeCrossfade() once per
    // sample during the fade.
    void StartFreezeCrossfade();
    void ProcessFreezeCrossfade();

    size_t size() const { return size_; }
    size_t write_head() const { return write_head_; }
    bool crossfading() const { return crossfading_; }

    // How many bytes the caller must allocate for a buffer of the given
    // sample rate and duration.
    static size_t RequiredBytes(float sample_rate,
                                float duration_seconds,
                                int channels = 2);

private:
    // Update the tail copies that follow the main buffer.
    // Must be called whenever a frame in the first kInterpolationTail
    // frames is written.
    void UpdateTail();

    float* buffer_ = nullptr;
    size_t size_ = 0;           // Number of frames (not samples)
    int channels_ = 2;
    size_t write_head_ = 0;

    // Freeze crossfade state
    bool crossfading_ = false;
    int crossfade_counter_ = 0;
    static constexpr int kCrossfadeSamples = 32;
};

} // namespace beads
