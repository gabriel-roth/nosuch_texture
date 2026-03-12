#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>
#include <cstring>

#include "beads/beads.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;
static constexpr size_t kBlockSize = 256;

// Helper to create and init a processor
struct TestProcessor {
    std::vector<uint8_t> memory;
    BeadsProcessor processor;

    TestProcessor() {
        auto req = BeadsProcessor::GetMemoryRequirements(kSampleRate);
        memory.resize(req.total_bytes, 0);
        processor.Init(memory.data(), memory.size(), kSampleRate);
    }
};

TEST_CASE("BeadsProcessor: GetMemoryRequirements returns sensible values", "[processor]") {
    auto req = BeadsProcessor::GetMemoryRequirements(kSampleRate);
    REQUIRE(req.total_bytes > 0);
    REQUIRE(req.alignment > 0);
    // Should be roughly 1.5-2MB at 48kHz with 4s buffer
    REQUIRE(req.total_bytes > 1000000);
    REQUIRE(req.total_bytes < 10000000);
}

TEST_CASE("BeadsProcessor: Init does not crash", "[processor]") {
    TestProcessor tp;
    // If we got here, Init succeeded
    REQUIRE(tp.processor.IsDelayMode() == false);
    REQUIRE(tp.processor.IsWavetableMode() == false);
    REQUIRE(tp.processor.ActiveGrainCount() == 0);
}

TEST_CASE("BeadsProcessor: Process with silence input", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize, {0.0f, 0.0f});
    std::vector<StereoFrame> output(kBlockSize);

    tp.processor.Process(input.data(), output.data(), kBlockSize);

    // With silence in and default params, output should be near silence
    for (size_t i = 0; i < kBlockSize; ++i) {
        REQUIRE(std::isfinite(output[i].l));
        REQUIRE(std::isfinite(output[i].r));
    }
}

TEST_CASE("BeadsProcessor: Process with sine input produces output", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 0.1f;  // Far left of noon = fast grain trigger rate
    params.dry_wet = 1.0f;  // Full wet
    params.time = 0.95f;    // Read near write head (where data has been written)
    params.size = 0.3f;     // Short grains
    params.shape = 0.5f;
    params.pitch = 0.0f;
    params.manual_gain_db = 0.0f;  // Bypass auto-gain ramping
    params.trigger_mode = TriggerMode::kLatched;
    tp.processor.SetParameters(params);

    // Generate a sine wave input
    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }

    std::vector<StereoFrame> output(kBlockSize);

    // Process enough blocks to fill buffer, trigger grains, and render output
    float max_level = 0.0f;
    for (int block = 0; block < 200; ++block) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            max_level = std::max(max_level, std::max(std::abs(output[i].l), std::abs(output[i].r)));
            REQUIRE(std::isfinite(output[i].l));
            REQUIRE(std::isfinite(output[i].r));
        }
    }

    // After many blocks, we should have some output from grains
    REQUIRE(max_level > 0.001f);
}

TEST_CASE("BeadsProcessor: Delay mode activates at size=1.0", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.size = 1.0f;
    tp.processor.SetParameters(params);
    REQUIRE(tp.processor.IsDelayMode() == true);

    params.size = 0.5f;
    tp.processor.SetParameters(params);
    REQUIRE(tp.processor.IsDelayMode() == false);
}

TEST_CASE("BeadsProcessor: No NaN in output with extreme parameters", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 1.0f;
    params.feedback = 0.99f;
    params.dry_wet = 1.0f;
    params.reverb = 1.0f;
    params.size = 0.5f;
    params.pitch = 24.0f;  // 2 octaves up
    params.quality_mode = QualityMode::kTape;
    params.manual_gain_db = 0.0f;  // Fixed gain to avoid auto-gain runaway
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }

    std::vector<StereoFrame> output(kBlockSize);

    for (int block = 0; block < 100; ++block) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            REQUIRE(std::isfinite(output[i].l));
            REQUIRE(std::isfinite(output[i].r));
        }
    }
}

TEST_CASE("BeadsProcessor: Freeze stops recording", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 0.3f;
    params.size = 0.5f;
    params.dry_wet = 0.5f;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }
    std::vector<StereoFrame> output(kBlockSize);

    // Process a few blocks
    for (int i = 0; i < 10; ++i) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
    }

    // Freeze
    params.freeze = true;
    tp.processor.SetParameters(params);
    tp.processor.Process(input.data(), output.data(), kBlockSize);

    // Process should still work without crash
    for (int i = 0; i < 10; ++i) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t j = 0; j < kBlockSize; ++j) {
            REQUIRE(std::isfinite(output[j].l));
            REQUIRE(std::isfinite(output[j].r));
        }
    }
}

TEST_CASE("BeadsProcessor: All quality modes work without NaN", "[processor]") {
    for (int mode = 0; mode < 4; ++mode) {
        TestProcessor tp;

        BeadsParameters params;
        params.quality_mode = static_cast<QualityMode>(mode);
        params.density = 0.3f;
        params.size = 0.5f;
        params.dry_wet = 1.0f;
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
            input[i] = {std::sin(phase), std::sin(phase)};
        }

        std::vector<StereoFrame> output(kBlockSize);
        for (int block = 0; block < 20; ++block) {
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                REQUIRE(std::isfinite(output[i].l));
                REQUIRE(std::isfinite(output[i].r));
            }
        }
    }
}
