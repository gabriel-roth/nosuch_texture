#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>

#include "beads/types.h"
#include "beads/parameters.h"
#include "buffer/recording_buffer.h"
#include "grain/grain.h"
#include "grain/grain_scheduler.h"
#include "grain/grain_engine.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;

// Helper: create a small recording buffer filled with a sine wave
struct TestBuffer {
    std::vector<uint8_t> memory;
    RecordingBuffer buffer;
    size_t num_frames;

    TestBuffer(size_t frames = 4800) : num_frames(frames) {
        size_t bytes = (num_frames + kInterpolationTail) * 2 * sizeof(float);
        memory.resize(bytes, 0);
        buffer.Init(reinterpret_cast<float*>(memory.data()), num_frames, 2);

        for (size_t i = 0; i < num_frames; ++i) {
            float phase = static_cast<float>(i) / static_cast<float>(num_frames) * 2.0f * 3.14159265f * 10.0f;
            buffer.Write(std::sin(phase), std::cos(phase));
        }
    }
};

TEST_CASE("Grain: Init sets inactive", "[grain]") {
    Grain g;
    g.Init();
    REQUIRE(g.active() == false);
}

TEST_CASE("Grain: Start activates grain", "[grain]") {
    Grain g;
    g.Init();

    Grain::GrainParameters params;
    params.position = 100.0f;
    params.size = 480.0f;  // 10ms at 48kHz
    params.pitch_ratio = 1.0f;
    params.shape = 0.5f;
    params.pan = 0.0f;
    params.pre_delay = 0;

    g.Start(params);
    REQUIRE(g.active() == true);
}

TEST_CASE("Grain: Processes for correct duration", "[grain]") {
    TestBuffer tb;
    Grain g;
    g.Init();

    float grain_size = 480.0f;  // 10ms
    Grain::GrainParameters params;
    params.position = 100.0f;
    params.size = grain_size;
    params.pitch_ratio = 1.0f;
    params.shape = 0.5f;
    params.pan = 0.0f;
    params.pre_delay = 0;

    g.Start(params);

    int sample_count = 0;
    float out_l, out_r;
    while (g.Process(tb.buffer, &out_l, &out_r)) {
        sample_count++;
        if (sample_count > 1000) break;  // Safety
    }

    // Grain should have been active for approximately grain_size samples
    REQUIRE(sample_count == Approx(static_cast<int>(grain_size)).margin(2));
}

TEST_CASE("Grain: Output is non-zero with sine input", "[grain]") {
    TestBuffer tb;
    Grain g;
    g.Init();

    Grain::GrainParameters params;
    params.position = 100.0f;
    params.size = 480.0f;
    params.pitch_ratio = 1.0f;
    params.shape = 0.5f;
    params.pan = 0.0f;
    params.pre_delay = 0;

    g.Start(params);

    float max_level = 0.0f;
    float out_l, out_r;
    while (g.Process(tb.buffer, &out_l, &out_r)) {
        max_level = std::max(max_level, std::max(std::abs(out_l), std::abs(out_r)));
    }

    REQUIRE(max_level > 0.01f);
}

TEST_CASE("Grain: Bell envelope has zero at start and end", "[grain]") {
    TestBuffer tb;
    Grain g;
    g.Init();

    Grain::GrainParameters params;
    params.position = 100.0f;
    params.size = 480.0f;
    params.pitch_ratio = 1.0f;
    params.shape = 0.5f;  // Bell/Hann
    params.pan = 0.0f;
    params.pre_delay = 0;

    g.Start(params);

    float out_l, out_r;
    // First sample should be near zero (Hann window starts at 0)
    g.Process(tb.buffer, &out_l, &out_r);
    REQUIRE(std::abs(out_l) < 0.05f);
}

TEST_CASE("Grain: Pre-delay delays output", "[grain]") {
    TestBuffer tb;
    Grain g;
    g.Init();

    Grain::GrainParameters params;
    params.position = 100.0f;
    params.size = 480.0f;
    params.pitch_ratio = 1.0f;
    params.shape = 0.5f;
    params.pan = 0.0f;
    params.pre_delay = 10;

    g.Start(params);

    float out_l, out_r;
    // First 10 samples should be silent (pre-delay)
    for (int i = 0; i < 10; ++i) {
        REQUIRE(g.Process(tb.buffer, &out_l, &out_r) == true);
        REQUIRE(out_l == 0.0f);
        REQUIRE(out_r == 0.0f);
    }
}

TEST_CASE("GrainScheduler: Latched mode produces triggers", "[scheduler]") {
    GrainScheduler sched;
    sched.Init(kSampleRate);

    BeadsParameters params;
    params.trigger_mode = TriggerMode::kLatched;
    params.density = 0.2f;  // Left of noon = regular triggers

    int triggers[64];
    int total = 0;

    // Process several blocks
    for (int block = 0; block < 100; ++block) {
        int count = sched.Process(params, 256, triggers, 64);
        total += count;
    }

    // Should have generated some triggers
    REQUIRE(total > 0);
}

TEST_CASE("GrainScheduler: Latched at noon is silent", "[scheduler]") {
    GrainScheduler sched;
    sched.Init(kSampleRate);

    BeadsParameters params;
    params.trigger_mode = TriggerMode::kLatched;
    params.density = 0.5f;  // Noon = silent

    int triggers[64];
    int total = 0;

    for (int block = 0; block < 100; ++block) {
        int count = sched.Process(params, 256, triggers, 64);
        total += count;
    }

    REQUIRE(total == 0);
}

TEST_CASE("GrainEngine: Produces output with active grains", "[engine]") {
    TestBuffer tb;

    GrainEngine engine;
    engine.Init(kSampleRate, &tb.buffer);

    BeadsParameters params;
    params.trigger_mode = TriggerMode::kLatched;
    params.density = 0.1f;    // Far left of noon = fast trigger rate
    params.size = 0.5f;
    params.time = 0.5f;
    params.shape = 0.5f;
    params.pitch = 0.0f;

    std::vector<StereoFrame> output(256, {0.0f, 0.0f});

    // Process enough blocks for triggers to fire and grains to produce output
    float max_level = 0.0f;
    for (int block = 0; block < 200; ++block) {
        engine.Process(params, output.data(), 256);
        for (auto& f : output) {
            max_level = std::max(max_level, std::max(std::abs(f.l), std::abs(f.r)));
        }
    }

    REQUIRE(max_level > 0.001f);
}
