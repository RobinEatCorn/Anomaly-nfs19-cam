// Module-level declarations and the small helpers several modules share. This
// header grows as modules are moved out of plugin.cpp.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

namespace nfs2015_chase_cam::detail {

// Window extremes, kept lock-free: the Game thread resets these between
// windows, so a plain CAS loop is enough.
template <typename T>
void LowerFloor(std::atomic<T> &floor, T value) noexcept {
  T previous = floor.load(std::memory_order_relaxed);
  while (value < previous &&
         !floor.compare_exchange_weak(previous, value,
                                     std::memory_order_relaxed)) {
  }
}

template <typename T>
void RaisePeak(std::atomic<T> &peak, T value) noexcept {
  T previous = peak.load(std::memory_order_relaxed);
  while (value > previous &&
         !peak.compare_exchange_weak(previous, value,
                                     std::memory_order_relaxed)) {
  }
}

// Body pitch (nose up positive) from the vehicle proxy's forward/nose vector.
// The vector may be fully normalised (z = sin pitch) or horizontally
// normalised (z = tan pitch, the historical publish format), so atan2 is the
// correct inverse for both; asin(z) would only work for the former.
inline bool BodyPitchFromForward(const double forward[3],
                                 double &pitch) noexcept {
  if (!std::isfinite(forward[0]) || !std::isfinite(forward[1]) ||
      !std::isfinite(forward[2])) {
    return false;
  }
  const double horizontal_squared =
      forward[0] * forward[0] + forward[1] * forward[1];
  if (!(horizontal_squared > 1e-8)) {
    return false;
  }
  pitch = std::atan2(forward[2], std::sqrt(horizontal_squared));
  return std::isfinite(pitch);
}

}  // namespace nfs2015_chase_cam::detail
