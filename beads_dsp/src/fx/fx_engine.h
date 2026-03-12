#pragma once
#include <cstddef>
#include <cmath>
#include "../../include/beads/types.h"

namespace beads {

// FxEngine manages a block of memory used as delay lines for reverb
// Based on the Clouds fx_engine pattern
class FxEngine {
public:
    // Context provides read/write access to the engine's delay memory
    // Used inside the reverb Process() to read/write delay taps
    class Context {
    public:
        Context(float* buffer, size_t size, size_t write_ptr)
            : buffer_(buffer), size_(size), write_ptr_(write_ptr) {}

        // Write accumulator to current position with optional gain
        void Write(float value, float scale = 1.0f) {
            buffer_[write_ptr_] = value * scale;
        }

        // Write with interpolated position (for modulated delays)
        void WriteAllpass(float value, float scale = 1.0f) {
            buffer_[write_ptr_] = value * scale;
        }

        // Read from delay line at offset samples behind write pointer
        float Read(size_t offset) const {
            size_t idx = (write_ptr_ + size_ - offset) % size_;
            return buffer_[idx];
        }

        // Read with linear interpolation for fractional delays
        float ReadInterpolated(float offset) const {
            float int_part;
            float frac = std::modf(offset, &int_part);
            size_t idx0 = (write_ptr_ + size_ - static_cast<size_t>(int_part)) % size_;
            size_t idx1 = (idx0 + size_ - 1) % size_;
            return buffer_[idx0] + frac * (buffer_[idx1] - buffer_[idx0]);
        }

    private:
        float* buffer_;
        size_t size_;
        size_t write_ptr_;
    };

    void Init(float* buffer, size_t size) {
        buffer_ = buffer;
        size_ = size;
        write_ptr_ = 0;
        Clear();
    }

    void Clear() {
        if (buffer_) {
            for (size_t i = 0; i < size_; ++i) buffer_[i] = 0.0f;
        }
    }

    Context GetContext() {
        return Context(buffer_, size_, write_ptr_);
    }

    void Advance() {
        write_ptr_ = (write_ptr_ + 1) % size_;
    }

private:
    float* buffer_ = nullptr;
    size_t size_ = 0;
    size_t write_ptr_ = 0;
};

} // namespace beads
