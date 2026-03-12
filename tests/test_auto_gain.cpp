#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>

#include "beads/types.h"
#include "input/auto_gain.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;

TEST_CASE("AutoGain: Quiet input gets gain greater than 1", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Feed quiet signal (0.01 amplitude) for enough samples for the
    // envelope to converge and auto-gain to ramp up
    StereoFrame last_out = {0.0f, 0.0f};
    for (int i = 0; i < 200000; ++i) {
        StereoFrame in = {0.01f, 0.01f};
        last_out = ag.Process(in, NAN);  // NaN = auto-gain mode
    }

    // With a 0.01 input, auto-gain should have boosted the signal
    // The gain should push the output above the raw input level
    float output_level = std::max(std::abs(last_out.l), std::abs(last_out.r));
    REQUIRE(output_level > 0.01f);
}

TEST_CASE("AutoGain: Loud input stays near unity", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Feed loud signal (1.0 amplitude) for a long time
    StereoFrame last_out = {0.0f, 0.0f};
    for (int i = 0; i < 200000; ++i) {
        float val = std::sin(static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f);
        StereoFrame in = {val, val};
        last_out = ag.Process(in, NAN);
    }

    // With a loud input (peaks at 1.0), auto-gain should be near unity.
    // The kMinGainDb is 0dB, so gain_ should be close to 1.0
    // Output should be roughly the same as input
    float input_val = std::sin(200000.0f / kSampleRate * 440.0f * 2.0f * 3.14159265f);
    float ratio = std::abs(last_out.l) / std::max(std::abs(input_val), 0.001f);

    // Should be near 1.0 (unity gain for loud signal)
    REQUIRE(ratio > 0.5f);
    REQUIRE(ratio < 3.0f);
}

TEST_CASE("AutoGain: Manual gain override works", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Feed moderate signal with manual gain of +12dB
    float manual_gain_db = 12.0f;
    StereoFrame last_out = {0.0f, 0.0f};
    for (int i = 0; i < 100000; ++i) {
        StereoFrame in = {0.1f, 0.1f};
        last_out = ag.Process(in, manual_gain_db);
    }

    // With +12dB manual gain, the output should be roughly 0.1 * 10^(12/20) = 0.1 * ~3.98 = ~0.398
    float expected_gain = std::pow(10.0f, 12.0f / 20.0f);
    float expected_output = 0.1f * expected_gain;
    float actual = std::abs(last_out.l);

    REQUIRE(actual == Approx(expected_output).margin(0.05f));
}

TEST_CASE("AutoGain: NaN manual_gain_db activates auto mode", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // In auto mode (NaN manual_gain_db), quiet signal should get boosted
    // First verify with NaN (auto mode)
    for (int i = 0; i < 200000; ++i) {
        StereoFrame in = {0.01f, 0.01f};
        ag.Process(in, NAN);
    }
    StereoFrame auto_out = ag.Process({0.01f, 0.01f}, NAN);

    // Then create a new instance with manual gain = 0dB
    AutoGain ag2;
    ag2.Init(kSampleRate);
    for (int i = 0; i < 200000; ++i) {
        StereoFrame in = {0.01f, 0.01f};
        ag2.Process(in, 0.0f);
    }
    StereoFrame manual_out = ag2.Process({0.01f, 0.01f}, 0.0f);

    // Auto mode with quiet input should produce higher output than 0dB manual
    REQUIRE(std::abs(auto_out.l) > std::abs(manual_out.l));
}

TEST_CASE("AutoGain: InputLevel reflects signal level", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Initially, input level should be near zero
    REQUIRE(ag.InputLevel() == Approx(0.0f).margin(0.001f));

    // Feed signal and check that level increases
    for (int i = 0; i < 10000; ++i) {
        StereoFrame in = {0.5f, 0.5f};
        ag.Process(in, NAN);
    }

    REQUIRE(ag.InputLevel() > 0.1f);
}

TEST_CASE("AutoGain: Output is always finite", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Test with various input levels including near-zero
    float test_levels[] = {0.0f, 0.001f, 0.01f, 0.1f, 1.0f};
    for (float level : test_levels) {
        for (int i = 0; i < 1000; ++i) {
            StereoFrame in = {level, -level};
            StereoFrame out = ag.Process(in, NAN);
            REQUIRE(std::isfinite(out.l));
            REQUIRE(std::isfinite(out.r));
        }
    }
}

TEST_CASE("AutoGain: Manual gain is clamped to valid range", "[autogain]") {
    AutoGain ag;
    ag.Init(kSampleRate);

    // Even with extreme manual gain values, output should be finite
    float extreme_gains[] = {-100.0f, 0.0f, 32.0f, 100.0f};
    for (float gain_db : extreme_gains) {
        for (int i = 0; i < 1000; ++i) {
            StereoFrame in = {0.1f, 0.1f};
            StereoFrame out = ag.Process(in, gain_db);
            REQUIRE(std::isfinite(out.l));
            REQUIRE(std::isfinite(out.r));
        }
    }
}
