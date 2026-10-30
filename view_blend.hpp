#pragma once

// Render-side interpolation of the last two published camera views.
//
// The game only ticks the chase-camera model at its Game tick rate. The camera
// manager's view-point detour, however, runs at the render rate, so when the
// city drops below 60 Game ticks the view would otherwise step between ticks.
// This helper blends two already-published poses; it has no clocks or locks so
// the timing/threading policy stays in plugin.cpp.

#include "chase_cam.hpp"

#include <cstdint>

namespace nfs2015_chase {

struct ViewPose final {
  Vec3 position{};
  double pitch_degrees{};
  double yaw_degrees{};
  double roll_degrees{};
  double fov_degrees{};
};

// Blend `from` -> `to`. Yaw takes the shortest path across +/-180 degrees.
// Callers decide whether the FOV fields are safe to blend.
[[nodiscard]] inline ViewPose BlendViewPose(const ViewPose from,
                                            const ViewPose to,
                                            const double alpha) noexcept {
  const double t = Clamp(alpha, 0.0, 1.0);
  ViewPose result;
  result.position = Add(from.position, Mul(Sub(to.position, from.position), t));
  result.pitch_degrees = Lerp(from.pitch_degrees, to.pitch_degrees, t);
  result.roll_degrees = Lerp(from.roll_degrees, to.roll_degrees, t);
  result.fov_degrees = Lerp(from.fov_degrees, to.fov_degrees, t);

  const double yaw_delta = WrapPi(
      (to.yaw_degrees - from.yaw_degrees) * kRadiansPerDegree);
  result.yaw_degrees =
      from.yaw_degrees + yaw_delta * kDegreesPerRadian * t;
  result.yaw_degrees =
      WrapPi(result.yaw_degrees * kRadiansPerDegree) * kDegreesPerRadian;
  return result;
}

// Choose the blend factor for a render time. The pair is presented one
// (capped) update interval behind the newest snapshot: immediately after a
// publish alpha is 0, midway between ticks it is 0.5, and at the next tick it
// reaches 1. A long hitch is capped so the camera does not trail indefinitely.
[[nodiscard]] inline double ViewInterpolationAlpha(
    const std::int64_t previous_ns, const std::int64_t current_ns,
    const std::int64_t now_ns,
    const double maximum_delay_seconds) noexcept {
  if (current_ns <= previous_ns) {
    return 1.0;
  }
  const double interval = static_cast<double>(current_ns - previous_ns) * 1e-9;
  const double delay =
      Clamp(interval, 0.0, maximum_delay_seconds);
  return Clamp(
      (static_cast<double>(now_ns - previous_ns) * 1e-9 - delay) / interval,
      0.0, 1.0);
}

}  // namespace nfs2015_chase
