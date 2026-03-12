#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>

#include "beads/types.h"
#include "quality/quality_processor.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;

TEST_CASE("QualityModes: Clouds output quantization is detectable", "[quality]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);

    // Feed a ramp of small values and check that the Clouds output shows
    // quantization steps (round to 12-bit = 1/2048 steps)
    int step_changes = 0;
    float prev_out = 0.0f;
    bool first = true;

    for (int i = 0; i < 1000; ++i) {
        float val = static_cast<float>(i) / 1000.0f * 0.1f;  // 0 to 0.1 ramp
        StereoFrame in = {val, val};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kClouds);

        if (!first && out.l != prev_out) {
            step_changes++;
        }
        prev_out = out.l;
        first = false;
    }

    // A ramp from 0 to 0.1 at 12-bit resolution spans ~205 steps (0.1 * 2048)
    // The quantized output should have fewer unique transitions than the input
    REQUIRE(step_changes > 0);
    REQUIRE(step_changes < 999);  // Fewer than every sample changing (quantized)
}

TEST_CASE("QualityModes: Tape mode reduces high-frequency content", "[quality]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);

    // Generate a high-frequency signal (20kHz) well above tape LP at 8kHz
    float hifi_energy = 0.0f;
    float tape_energy = 0.0f;

    QualityProcessor qp_hifi;
    qp_hifi.Init(kSampleRate);

    for (int i = 0; i < 10000; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 20000.0f * 2.0f * kPi;
        float val = std::sin(phase);
        StereoFrame in = {val, val};

        StereoFrame hifi_out = qp_hifi.ProcessInput(in, QualityMode::kHiFi);
        StereoFrame tape_out = qp.ProcessInput(in, QualityMode::kTape);

        hifi_energy += hifi_out.l * hifi_out.l;
        tape_energy += tape_out.l * tape_out.l;
    }

    // Tape LP filter at 8kHz should heavily attenuate 20kHz
    REQUIRE(tape_energy < hifi_energy * 0.5f);
}

TEST_CASE("QualityModes: HiFi ProcessInput is near-passthrough", "[quality]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);

    // HiFi input should pass through unchanged
    float max_error = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        float val = std::sin(static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * kPi);
        StereoFrame in = {val, -val};
        StereoFrame out = qp.ProcessInput(in, QualityMode::kHiFi);
        max_error = std::max(max_error, std::abs(out.l - val));
        max_error = std::max(max_error, std::abs(out.r - (-val)));
    }

    REQUIRE(max_error < 0.001f);
}

TEST_CASE("QualityModes: HiFi ProcessOutput is near-passthrough", "[quality]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);

    float max_error = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        float val = std::sin(static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * kPi);
        StereoFrame in = {val, -val};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kHiFi);
        max_error = std::max(max_error, std::abs(out.l - val));
        max_error = std::max(max_error, std::abs(out.r - (-val)));
    }

    REQUIRE(max_error < 0.001f);
}

TEST_CASE("QualityModes: Each mode produces finite output", "[quality]") {
    for (int mode = 0; mode < 4; ++mode) {
        QualityProcessor qp;
        qp.Init(kSampleRate);

        QualityMode qm = static_cast<QualityMode>(mode);

        for (int i = 0; i < 5000; ++i) {
            float val = std::sin(static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * kPi);
            StereoFrame in = {val, val};

            StereoFrame out_in = qp.ProcessInput(in, qm);
            StereoFrame out_out = qp.ProcessOutput(in, qm);

            REQUIRE(std::isfinite(out_in.l));
            REQUIRE(std::isfinite(out_in.r));
            REQUIRE(std::isfinite(out_out.l));
            REQUIRE(std::isfinite(out_out.r));
        }
    }
}

TEST_CASE("QualityModes: CleanLoFi output LP attenuates high frequencies", "[quality]") {
    QualityProcessor qp_lofi, qp_hifi;
    qp_lofi.Init(kSampleRate);
    qp_hifi.Init(kSampleRate);

    // Generate a high-frequency signal (18kHz) and pass through output processing
    float hifi_energy = 0.0f;
    float lofi_energy = 0.0f;

    for (int i = 0; i < 10000; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 18000.0f * 2.0f * kPi;
        float val = std::sin(phase);
        StereoFrame in = {val, val};

        StereoFrame hifi_out = qp_hifi.ProcessOutput(in, QualityMode::kHiFi);
        StereoFrame lofi_out = qp_lofi.ProcessOutput(in, QualityMode::kCleanLoFi);

        hifi_energy += hifi_out.l * hifi_out.l;
        lofi_energy += lofi_out.l * lofi_out.l;
    }

    // CleanLoFi LP at 10kHz should attenuate 18kHz
    REQUIRE(lofi_energy < hifi_energy * 0.5f);
}

TEST_CASE("QualityModes: Mode crossfade produces smooth transition", "[quality]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);

    // Process several samples in HiFi mode
    for (int i = 0; i < 100; ++i) {
        StereoFrame in = {0.5f, 0.5f};
        qp.ProcessInput(in, QualityMode::kHiFi);
        qp.ProcessOutput(in, QualityMode::kHiFi);
    }

    // Switch to Clouds and check the transition frames
    float prev_l = 0.5f;
    float max_delta = 0.0f;
    for (int i = 0; i < 100; ++i) {
        StereoFrame in = {0.5f, 0.5f};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kClouds);
        float delta = std::abs(out.l - prev_l);
        max_delta = std::max(max_delta, delta);
        prev_l = out.l;
    }

    // The crossfade should prevent a large jump at the mode switch
    REQUIRE(max_delta < 0.1f);
}
