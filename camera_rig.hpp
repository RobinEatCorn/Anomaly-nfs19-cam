#pragma once

// Per-axis camera rig factors for the two user-facing metric sliders.
//
// The chase model's rest pose is the denominator; the presenter and route B
// multiply the model offset by these factors around the fixed ground origin.
// Keeping the math in one portable header lets the offline tests pin the
// default and per-preset cases without dragging in the Windows plugin state.

#include "chase_cam.hpp"

#include <cmath>

namespace nfs2015_chase {

struct CameraRigFactors final {
  double distance{1.0};
  double height{1.0};
};

// `rest` is the active preset's local rest pose (metres). `distance_m` and
// `height_m` are the user-facing values at rest. The static eye is exactly
// (-distance_m, 0, height_m) for either preset because the rest vector is
// multiplied by its own axis ratio. A zero/missing rest axis falls back to 1.0
// instead of dividing by zero.
[[nodiscard]] inline CameraRigFactors CameraRigFactorsFor(
    const Vec3 &rest, const double distance_m,
    const double height_m) noexcept {
  const double base_distance = -rest.x;
  const double base_height = rest.z;
  double distance = 1.0;
  double height = 1.0;
  if (std::isfinite(distance_m) && base_distance > 1e-6) {
    distance = distance_m / base_distance;
  }
  if (std::isfinite(height_m) && base_height > 1e-6) {
    height = height_m / base_height;
  }
  if (!std::isfinite(distance)) distance = 1.0;
  if (!std::isfinite(height)) height = 1.0;
  return {Clamp(distance, 0.05, 10.0), Clamp(height, 0.05, 10.0)};
}

}  // namespace nfs2015_chase
