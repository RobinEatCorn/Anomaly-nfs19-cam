#pragma once

// Car samples shared by the Game tick and the scene-view camera.
//
// The Game tick publishes one sample per tick with the derivative state and
// rig geometry. At the scene-view phase the camera replaces its position and
// yaw with the live VehicleMesh ComponentToWorld, so the model and the car
// about to be drawn share one transform. This header is only the POD sample
// and its blend helper; there is no ring, clock or thread policy.

#include "chase_cam.hpp"

#include <cstdint>

namespace nfs2015_chase {

// Burnout state published by the Game tick. `active` is the already-filtered
// gameplay condition; `left`/`right` are the per-rear-wheel flags and
// `intensity` is the smoothed 0..1 strength used by the shot animation.
// `input_active` is a separate, unfiltered fact: any of throttle/brake/
// steering/handbrake is deflected. The parked-orbit model uses it to decide
// whether the car is really parked; it is deliberately not derived from
// `active` because a burnout is an input, not a parked car.
struct BurnoutSample final {
  bool active{};
  bool left{};
  bool right{};
  // True when the driver is touching any of the four vehicle inputs.
  bool input_active{};
  double intensity{};
  double throttle{};
  double handbrake{};
  double steering{};
  double speed_mps{};
};

// Everything the model and the pose transform need for one instant. `car`
// already carries the geometry; the rest is what `PublishView` used to add.
struct CarSample final {
  std::int64_t time_ns{};
  CarState car{};
  // Game-tick burnout facts. Phase 2 fills this; Phase 3 consumes it on the
  // render clock. Booleans are taken from the newer sample by
  // BlendCarSample(), never interpolated.
  BurnoutSample burnout{};
  // Pawn root, cm. The pose is reported relative to this so the renderer can
  // tell where the car is, and so diagnostics can measure camera-vs-car motion.
  double anchor_cm[3]{};
  // Model origin used by the camera-scale transform, cm.
  double origin_cm[3]{};
  // User-facing rig geometry in metres. The presenter turns these into per-axis
  // factors against the active preset's rest pose; the game-tick copy is the
  // static value, and the live eye height is reported separately.
  double camera_distance_m{5.10};
  double camera_height_m{1.49};
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
  // Body pitch stays well inside +-90 degrees, so a plain linear blend is
  // correct; WrapPi would only introduce a seam the angle does not have.
  result.car.pitch = Lerp(from.car.pitch, to.car.pitch, t);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result.anchor_cm[axis] =
        Lerp(from.anchor_cm[axis], to.anchor_cm[axis], t);
    result.origin_cm[axis] =
        Lerp(from.origin_cm[axis], to.origin_cm[axis], t);
  }
  result.camera_distance_m =
      Lerp(from.camera_distance_m, to.camera_distance_m, t);
  result.camera_height_m =
      Lerp(from.camera_height_m, to.camera_height_m, t);
  // Burnout facts are event flags plus a smoothed strength. A boolean halfway
  // between true and false is meaningless, so active/left/right/input_active
  // come from the newer sample; the continuous values are safe to interpolate.
  result.burnout.active = to.burnout.active;
  result.burnout.left = to.burnout.left;
  result.burnout.right = to.burnout.right;
  result.burnout.input_active = to.burnout.input_active;
  result.burnout.intensity =
      Lerp(from.burnout.intensity, to.burnout.intensity, t);
  result.burnout.throttle =
      Lerp(from.burnout.throttle, to.burnout.throttle, t);
  result.burnout.handbrake =
      Lerp(from.burnout.handbrake, to.burnout.handbrake, t);
  result.burnout.steering =
      Lerp(from.burnout.steering, to.burnout.steering, t);
  result.burnout.speed_mps =
      Lerp(from.burnout.speed_mps, to.burnout.speed_mps, t);
  // Booleans and identities come from the newer sample: blending them would be
  // meaningless, and a differing epoch is a discontinuity the caller must see.
  return result;
}

}  // namespace nfs2015_chase
