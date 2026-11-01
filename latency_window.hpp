#pragma once

// Rolling latency histogram (T0 diagnostics).
//
// `PLAN-camera-timing.md` T0 needs the cost of one `UpdateCamera` call as a
// percentile, not as an average: a camera tick that is usually 0.2 ms but
// occasionally 8 ms is a different bug from one that is uniformly 4 ms, and
// only p99/max tell them apart. Sorting a window of samples on the Game thread
// would cost more than the thing being measured, so this keeps a fixed
// histogram plus a ring of bucket indices for O(1) insert and eviction.
//
// Quantisation is kBucketMilliseconds (50 us). Averages are exact: every
// sample is also kept in a float ring so the sum can be decremented on
// eviction. Maximum is reported as the upper edge of the highest non-empty
// bucket, i.e. within one bucket of the true peak.

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nfs2015_chase_cam {

class LatencyWindow final {
public:
    static constexpr std::size_t kCapacity = 512;
    static constexpr std::size_t kBuckets = 128;
    static constexpr double kBucketMilliseconds = 0.05;

    void Add(double milliseconds) noexcept {
        if (!std::isfinite(milliseconds) || milliseconds < 0.0) {
            milliseconds = 0.0;
        }
        std::size_t bucket = static_cast<std::size_t>(milliseconds / kBucketMilliseconds);
        if (bucket >= kBuckets) bucket = kBuckets - 1U;

        if (count_ == kCapacity) {
            const std::size_t evicted = ring_[head_];
            --buckets_[evicted];
            sum_ -= values_[head_];
            --count_;
        }
        ring_[head_] = static_cast<std::uint8_t>(bucket);
        values_[head_] = static_cast<float>(milliseconds);
        ++buckets_[bucket];
        sum_ += milliseconds;
        ++count_;
        head_ = (head_ + 1U) % kCapacity;
    }

    void Reset() noexcept {
        for (std::size_t index = 0; index < kBuckets; ++index) buckets_[index] = 0;
        head_ = 0;
        count_ = 0;
        sum_ = 0.0;
    }

    [[nodiscard]] std::size_t Count() const noexcept { return count_; }

    [[nodiscard]] double Average() const noexcept {
        return count_ == 0 ? 0.0 : sum_ / static_cast<double>(count_);
    }

    // Upper edge of the highest bucket that holds a sample.
    [[nodiscard]] double Maximum() const noexcept {
        for (std::size_t index = kBuckets; index > 0; --index) {
            if (buckets_[index - 1U] != 0) {
                return static_cast<double>(index) * kBucketMilliseconds;
            }
        }
        return 0.0;
    }

    // `fraction` in [0, 1]. Returns the upper edge of the bucket that holds the
    // requested rank, so the result never under-reports the sample.
    [[nodiscard]] double Percentile(const double fraction) const noexcept {
        if (count_ == 0) return 0.0;
        double clamped = fraction;
        if (!std::isfinite(clamped) || clamped <= 0.0) clamped = 0.0;
        if (clamped >= 1.0) clamped = 1.0;
        const std::size_t target = static_cast<std::size_t>(
            std::ceil(clamped * static_cast<double>(count_)));
        std::size_t seen = 0;
        for (std::size_t index = 0; index < kBuckets; ++index) {
            seen += buckets_[index];
            if (seen >= target && buckets_[index] != 0) {
                return static_cast<double>(index + 1U) * kBucketMilliseconds;
            }
        }
        return static_cast<double>(kBuckets) * kBucketMilliseconds;
    }

private:
    std::uint16_t buckets_[kBuckets]{};
    std::uint8_t ring_[kCapacity]{};
    float values_[kCapacity]{};
    std::size_t head_{};
    std::size_t count_{};
    double sum_{};
};

}  // namespace nfs2015_chase_cam
