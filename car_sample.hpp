#pragma once

// Car samples for a render-clock chase camera.
//
// The game updates the car once per Game tick, but the camera is evaluated once
// per rendered frame, and the two rates are not related. Presenting a pose that
// was produced on the tick clock always leaves a step at every tick boundary,
// no matter how the pose is blended afterwards: blending can only interpolate
// between two tick poses, so the camera's velocity still jumps at each tick.
//
// The fix is to run the camera model itself on the render clock and feed it the
// car state for the time being rendered. This header is the plumbing for that:
// a POD sample plus a fixed-capacity ring the Game tick writes and the render
// thread reads. It has no clocks, locks or policy - the caller decides the
// delay and who advances the model.

#include "chase_cam.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace nfs2015_chase {

// Everything the model and the pose transform need for one instant. `car`
// already carries the geometry; the rest is what `PublishView` used to add.
struct CarSample final {
  std::int64_t time_ns{};
  CarState car{};
  // Pawn root, cm. The pose is reported relative to this so the renderer can
  // tell where the car is, and so diagnostics can measure camera-vs-car motion.
  double anchor_cm[3]{};
  // Model origin used by the camera-scale transform, cm.
  double origin_cm[3]{};
  double camera_scale{1.0};
  double aspect_ratio{16.0 / 9.0};
  bool fov_enabled{};
  bool driving{};
  // Bumped by the caller whenever the pose must not be blended across the
  // boundary (world change, teleport, preset switch).
  std::uint64_t epoch{};
};

// Interpolate two samples. Yaw takes the shortest arc so the result never spins
// the long way when the car crosses the +-180 degree seam. `alpha` is not
// clamped here; callers pass 0..1.
[[nodiscard]] inline CarSample BlendCarSample(const CarSample &from,
                                              const CarSample &to,
                                              const double alpha) noexcept {
  CarSample result = to;
  const double t = alpha;
  result.time_ns = from.time_ns + static_cast<std::int64_t>(
                                      static_cast<double>(to.time_ns -
                                                          from.time_ns) *
                                      t);
  result.car.position = Add(from.car.position,
                            Mul(Sub(to.car.position, from.car.position), t));
  result.car.velocity = Add(from.car.velocity,
                            Mul(Sub(to.car.velocity, from.car.velocity), t));
  result.car.acceleration =
      Add(from.car.acceleration,
          Mul(Sub(to.car.acceleration, from.car.acceleration), t));
  result.car.yaw_rate = Lerp(from.car.yaw_rate, to.car.yaw_rate, t);
  result.car.speed = Lerp(from.car.speed, to.car.speed, t);
  result.car.slip_angle = Lerp(from.car.slip_angle, to.car.slip_angle, t);
  result.car.brake = Lerp(from.car.brake, to.car.brake, t);
  // Angles are radians here, so WrapPi is the right shortest-arc helper.
  result.car.yaw = from.car.yaw + WrapPi(to.car.yaw - from.car.yaw) * t;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result.anchor_cm[axis] =
        Lerp(from.anchor_cm[axis], to.anchor_cm[axis], t);
    result.origin_cm[axis] =
        Lerp(from.origin_cm[axis], to.origin_cm[axis], t);
  }
  result.camera_scale = Lerp(from.camera_scale, to.camera_scale, t);
  // Booleans and identities come from the newer sample: blending them would be
  // meaningless, and a differing epoch is a discontinuity the caller must see.
  return result;
}

// Single-writer, multi-reader ring of recent samples. The writer is the Game
// tick, readers are render threads. The whole ring sits behind one seqlock:
// writes copy a small POD so the window is short, and a reader that loses the
// race retries a few times and then reports failure rather than tearing.
class SampleRing final {
 public:
  static constexpr std::size_t kCapacity = 8U;
  static constexpr int kReadAttempts = 8;

  // Game tick only.
  void Write(const CarSample &sample) noexcept {
    const std::uint64_t base = sequence_.load(std::memory_order_relaxed);
    sequence_.store(base + 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    slots_[next_] = sample;
    next_ = (next_ + 1U) % kCapacity;
    if (count_ < kCapacity) {
      ++count_;
    }
    sequence_.store(base + 2U, std::memory_order_release);
  }

  void Clear() noexcept {
    const std::uint64_t base = sequence_.load(std::memory_order_relaxed);
    sequence_.store(base + 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    next_ = 0U;
    count_ = 0U;
    sequence_.store(base + 2U, std::memory_order_release);
  }

  [[nodiscard]] bool Latest(CarSample &out) const noexcept {
    Snapshot view;
    if (!Copy(view) || view.count == 0U) {
      return false;
    }
    out = view.slots[(view.next + kCapacity - 1U) % kCapacity];
    return true;
  }

  // Sample at `target_ns`, interpolating when it falls between two samples and
  // continuing the newest pair when it falls beyond them.
  //
  // At() clamps in that last case, which is right for "always return a pose" and
  // wrong for a renderer: a clamp freezes the eye until the next Game tick and
  // then releases the whole accumulated distance in one jump. A tick that
  // arrives late is a late sample, not a stopped car, so carry on at the
  // velocity of the newest pair for at most one span of it. Beyond that the
  // sample is old enough that the honest answer is to stop.
  [[nodiscard]] bool AtExtrapolating(const std::int64_t target_ns,
                                     CarSample &out) const noexcept {
    if (!At(target_ns, out)) {
      return false;
    }
    Snapshot view;
    if (!Copy(view) || view.count < 2U) {
      return true;
    }
    const std::size_t newest = (view.next + kCapacity - 1U) % kCapacity;
    const CarSample &newer = view.slots[newest];
    if (target_ns <= newer.time_ns) {
      return true;
    }
    const std::size_t older_index = (newest + kCapacity - 1U) % kCapacity;
    const CarSample &older = view.slots[older_index];
    const std::int64_t span = newer.time_ns - older.time_ns;
    if (span <= 0 || newer.epoch != older.epoch) {
      return true;
    }
    double ahead = static_cast<double>(target_ns - newer.time_ns) /
                   static_cast<double>(span);
    if (!(ahead > 0.0)) {
      return true;
    }
    if (ahead > 1.0) {
      ahead = 1.0;
    }
    out = BlendCarSample(newer, older, -ahead);
    out.time_ns = target_ns;
    return true;
  }

  // Sample at `time_ns`, interpolating between the two samples that bracket it.
  // Times outside the stored range clamp to the oldest or newest sample: the
  // renderer must always get a pose, even if the Game thread stalled.
  [[nodiscard]] bool At(const std::int64_t time_ns,
                        CarSample &out) const noexcept {
    Snapshot snapshot;
    if (!Copy(snapshot) || snapshot.count == 0U) {
      return false;
    }
    const std::size_t newest =
        (snapshot.next + kCapacity - 1U) % kCapacity;
    if (snapshot.count == 1U) {
      out = snapshot.slots[newest];
      return true;
    }
    // Walk back from the newest until we straddle `time_ns`.
    std::size_t newer_index = newest;
    for (std::size_t step = 1U; step < snapshot.count; ++step) {
      const std::size_t older_index =
          (newest + kCapacity - step) % kCapacity;
      const CarSample &older = snapshot.slots[older_index];
      const CarSample &newer = snapshot.slots[newer_index];
      if (time_ns >= older.time_ns) {
        if (time_ns >= newer.time_ns || newer.time_ns <= older.time_ns) {
          out = newer;
          return true;
        }
        const double span =
            static_cast<double>(newer.time_ns - older.time_ns);
        const double alpha =
            static_cast<double>(time_ns - older.time_ns) / span;
        out = BlendCarSample(older, newer, alpha);
        return true;
      }
      newer_index = older_index;
    }
    out = snapshot.slots[newest];
    return true;
  }

 private:
  struct Snapshot final {
    CarSample slots[kCapacity]{};
    std::size_t next{};
    std::size_t count{};
  };

  [[nodiscard]] bool Copy(Snapshot &out) const noexcept {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
      const std::uint64_t before = sequence_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }
      out.next = next_;
      out.count = count_;
      for (std::size_t index = 0; index < kCapacity; ++index) {
        out.slots[index] = slots_[index];
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      if (sequence_.load(std::memory_order_relaxed) == before) {
        return true;
      }
    }
    return false;
  }

  mutable std::atomic<std::uint64_t> sequence_{};
  CarSample slots_[kCapacity]{};
  std::size_t next_{};
  std::size_t count_{};
};

}  // namespace nfs2015_chase
