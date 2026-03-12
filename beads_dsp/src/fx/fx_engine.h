#pragma once
#include <cstddef>
#include <cmath>

namespace beads {

// Individual delay line for use in reverb and other effects.
// Operates on a sub-region of a shared buffer.
class DelayLine {
public:
    void Init(float* buffer, size_t size) {
        buffer_ = buffer;
        size_ = size;
        write_ptr_ = 0;
        if (buffer_) {
            for (size_t i = 0; i < size_; ++i) buffer_[i] = 0.0f;
        }
    }

    // Write value at current write position
    void Write(float value) {
        if (size_ == 0) return;
        buffer_[write_ptr_] = value;
    }

    // Read from offset samples behind write pointer.
    // offset=1 gives the most recently written sample,
    // offset=size gives the oldest sample (at write_ptr, before overwrite).
    float Read(size_t offset) const {
        if (size_ == 0) return 0.0f;
        size_t idx = (write_ptr_ + size_ - (offset % size_)) % size_;
        return buffer_[idx];
    }

    // Read with linear interpolation for fractional/modulated delays
    float ReadInterpolated(float offset) const {
        if (size_ == 0) return 0.0f;
        if (offset < 1.0f) offset = 1.0f;
        float max_off = static_cast<float>(size_);
        if (offset > max_off) offset = max_off;

        size_t off_int = static_cast<size_t>(offset);
        float frac = offset - static_cast<float>(off_int);

        float s0 = Read(off_int);
        float s1 = Read(off_int + 1);
        return s0 + frac * (s1 - s0);
    }

    // Advance write pointer by one position
    void Advance() {
        if (size_ == 0) return;
        write_ptr_ = (write_ptr_ + 1) % size_;
    }

    size_t size() const { return size_; }

private:
    float* buffer_ = nullptr;
    size_t size_ = 0;
    size_t write_ptr_ = 0;
};

} // namespace beads
