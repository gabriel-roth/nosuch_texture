#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>

#include "beads/types.h"
#include "buffer/recording_buffer.h"
#include "util/interpolation.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;
static constexpr float kDuration = 0.1f;  // Short buffer for tests

TEST_CASE("RecordingBuffer: RequiredBytes returns positive value", "[buffer]") {
    size_t bytes = RecordingBuffer::RequiredBytes(kSampleRate, kDuration);
    REQUIRE(bytes > 0);
    // Should be at least num_frames * channels * sizeof(float)
    size_t expected_min = static_cast<size_t>(kSampleRate * kDuration) * 2 * sizeof(float);
    REQUIRE(bytes >= expected_min);
}

TEST_CASE("RecordingBuffer: Write and ReadLinear roundtrip", "[buffer]") {
    size_t bytes = RecordingBuffer::RequiredBytes(kSampleRate, kDuration);
    std::vector<uint8_t> mem(bytes, 0);

    RecordingBuffer buf;
    size_t num_frames = static_cast<size_t>(kSampleRate * kDuration);
    buf.Init(reinterpret_cast<float*>(mem.data()), num_frames, 2);

    // Write a known pattern
    for (size_t i = 0; i < num_frames; ++i) {
        float val = static_cast<float>(i) / static_cast<float>(num_frames);
        buf.Write(val, -val);
    }

    // Read back at integer positions
    for (size_t i = 0; i < 100; ++i) {
        float pos = static_cast<float>(i);
        float l = buf.ReadLinear(0, pos);
        float r = buf.ReadLinear(1, pos);
        float expected = static_cast<float>(i) / static_cast<float>(num_frames);
        REQUIRE(l == Approx(expected).margin(0.001f));
        REQUIRE(r == Approx(-expected).margin(0.001f));
    }
}

TEST_CASE("RecordingBuffer: ReadHermite at integer positions matches samples", "[buffer]") {
    size_t bytes = RecordingBuffer::RequiredBytes(kSampleRate, kDuration);
    std::vector<uint8_t> mem(bytes, 0);

    RecordingBuffer buf;
    size_t num_frames = static_cast<size_t>(kSampleRate * kDuration);
    buf.Init(reinterpret_cast<float*>(mem.data()), num_frames, 2);

    // Write a sine wave
    for (size_t i = 0; i < num_frames; ++i) {
        float phase = static_cast<float>(i) / static_cast<float>(num_frames) * 2.0f * 3.14159265f;
        buf.Write(std::sin(phase), std::cos(phase));
    }

    // At integer positions, Hermite should closely match the written sample
    // (not exact due to interpolation overshoot near discontinuities)
    for (size_t i = 10; i < 100; ++i) {
        float pos = static_cast<float>(i);
        float l = buf.ReadHermite(0, pos);
        float phase = static_cast<float>(i) / static_cast<float>(num_frames) * 2.0f * 3.14159265f;
        REQUIRE(l == Approx(std::sin(phase)).margin(0.01f));
    }
}

TEST_CASE("RecordingBuffer: Write wraps around", "[buffer]") {
    size_t num_frames = 100;
    size_t bytes = (num_frames + kInterpolationTail) * 2 * sizeof(float);
    std::vector<uint8_t> mem(bytes, 0);

    RecordingBuffer buf;
    buf.Init(reinterpret_cast<float*>(mem.data()), num_frames, 2);

    // Write more than buffer size
    for (size_t i = 0; i < num_frames + 50; ++i) {
        buf.Write(static_cast<float>(i), 0.0f);
    }

    // Write head should have wrapped
    REQUIRE(buf.write_head() == 50);
}

TEST_CASE("RecordingBuffer: Freeze crossfade", "[buffer]") {
    size_t num_frames = 1000;
    size_t bytes = (num_frames + kInterpolationTail) * 2 * sizeof(float);
    std::vector<uint8_t> mem(bytes, 0);

    RecordingBuffer buf;
    buf.Init(reinterpret_cast<float*>(mem.data()), num_frames, 2);

    // Fill buffer
    for (size_t i = 0; i < 500; ++i) {
        buf.Write(1.0f, 1.0f);
    }

    buf.StartFreezeCrossfade();
    REQUIRE(buf.crossfading() == true);

    // Process crossfade samples
    for (int i = 0; i < 32; ++i) {
        buf.ProcessFreezeCrossfade();
    }
    REQUIRE(buf.crossfading() == false);
}

TEST_CASE("Interpolation: Hermite is exact for linear functions", "[interpolation]") {
    // For a linear function y = x, Hermite should interpolate exactly
    float y_1 = -1.0f, y0 = 0.0f, y1 = 1.0f, y2 = 2.0f;
    REQUIRE(InterpolateHermite(y_1, y0, y1, y2, 0.0f) == Approx(0.0f));
    REQUIRE(InterpolateHermite(y_1, y0, y1, y2, 0.25f) == Approx(0.25f));
    REQUIRE(InterpolateHermite(y_1, y0, y1, y2, 0.5f) == Approx(0.5f));
    REQUIRE(InterpolateHermite(y_1, y0, y1, y2, 0.75f) == Approx(0.75f));
}

TEST_CASE("Interpolation: Linear basic", "[interpolation]") {
    REQUIRE(InterpolateLinear(0.0f, 1.0f, 0.0f) == Approx(0.0f));
    REQUIRE(InterpolateLinear(0.0f, 1.0f, 0.5f) == Approx(0.5f));
    REQUIRE(InterpolateLinear(0.0f, 1.0f, 1.0f) == Approx(1.0f));
}
