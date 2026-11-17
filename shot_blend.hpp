#pragma once

// Shared blend between the driving chase pose and a car-local "shot" pose.
//
// Both cinematic shots in this plugin (the burnout wheel close-up and the
// parked orbit) are authored in car space and then blended into the chase
// camera. NFS 2015 uses EULER_ORBIT for those transitions: the eye travels on
// a cylinder around a car-local pivot instead of along a straight line, the
// look direction is slerped, and the up vector is re-orthogonalised. Keeping
// that one implementation here means the two shots cannot drift apart.
//
// This header is pure math: no SDK, no game state, no clock. It is extracted
// verbatim from BurnoutShot so `BlendBurnoutCamera` stays bit-for-bit
// identical; the burnout wrapper only forwards its historical default pivot.

#include "car_sample.hpp"

#include <algorithm>
#include <cmath>

namespace nfs2015_chase {

namespace shot_blend_detail {

[[nodiscard]] inline double Finite(const double value,
                                   const double fallback) noexcept {
  return std::isfinite(value) ? value : fallback;
}

[[nodiscard]] inline Vec3 SafeVec(const Vec3 value,
                                  const Vec3 fallback = {}) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
                 std::isfinite(value.z)
             ? value
             : fallback;
}

// Rotate one direction toward another by the shortest path. Opposite vectors
// have no unique shortest path, so pick a stable perpendicular axis; this only
// happens for a pathological caller and keeps the result finite.
[[nodiscard]] inline Vec3 SlerpDirection(Vec3 from, Vec3 to,
                                         const double t) noexcept {
  from = Normalize(from);
  to = Normalize(to);
  const double cosine = Clamp(Dot(from, to), -1.0, 1.0);
  if (cosine > 0.9995) {
    return Normalize(Add(Mul(from, 1.0 - t), Mul(to, t)));
  }
  if (cosine < -0.9995) {
    Vec3 axis = std::fabs(from.z) < 0.9
                    ? Cross(from, Vec3{0.0, 0.0, 1.0})
                    : Cross(from, Vec3{0.0, 1.0, 0.0});
    if (Length(axis) < 1e-9) {
      axis = Cross(from, Vec3{1.0, 0.0, 0.0});
    }
    axis = Normalize(axis);
    const double angle = kPi * t;
    return Normalize(Add(Mul(from, std::cos(angle)),
                         Mul(axis, std::sin(angle))));
  }
  const double angle = std::acos(cosine);
  const double sine = std::sin(angle);
  if (std::fabs(sine) < 1e-9) {
    return from;
  }
  return Normalize(
      Add(Mul(from, std::sin((1.0 - t) * angle) / sine),
          Mul(to, std::sin(t * angle) / sine)));
}

[[nodiscard]] inline Vec3 OrthonormalUp(Vec3 up, const Vec3 forward,
                                        const Vec3 fallback) noexcept {
  up = Sub(up, Mul(forward, Dot(up, forward)));
  if (Length(up) < 1e-9) {
    up = Sub(fallback, Mul(forward, Dot(fallback, forward)));
  }
  if (Length(up) < 1e-9) {
    up = std::fabs(forward.z) < 0.9 ? Vec3{0.0, 0.0, 1.0}
                                    : Vec3{1.0, 0.0, 0.0};
    up = Sub(up, Mul(forward, Dot(up, forward)));
  }
  return Normalize(up);
}

}  // namespace shot_blend_detail

// Blend the base (chase) view and a car-local shot around a car-local pivot.
//
// The shot is not a straight lerp: NFS uses EULER_ORBIT, so the eye travels in
// cylindrical coordinates around the pivot. The look direction uses a slerp and
// the up vector is re-orthogonalised, keeping the blend free of the usual
// target-length/pitch pops. WheelSpin and the parked orbit both have Roll 0.0
// and no DUTCH controller, so the visible tilt recovery is this one orbit
// blend, not a roll spring.
//
// `blend == 0` returns `base` bit-for-bit (no normalisation), and `blend == 1`
// returns `shot` bit-for-bit; only the interior takes the orbit path.
[[nodiscard]] inline CameraState BlendShotAroundCarPivot(
    const CameraState& base, const CameraState& shot, const CarState& car,
    const double blend, const double pivot_height_m) noexcept {
  const double t =
      Clamp(shot_blend_detail::Finite(blend, 0.0), 0.0, 1.0);
  if (t <= 0.0) {
    return base;
  }
  if (t >= 1.0) {
    return shot;
  }

  using namespace shot_blend_detail;
  const Vec3 car_position = SafeVec(car.position);
  const double yaw = Finite(car.yaw, 0.0);
  const double pitch = Finite(car.pitch, 0.0);
  const CarBasis basis = MakeCarBasis(yaw, pitch);
  const Vec3 pivot =
      Add(car_position, Mul(basis.up, (std::max)(0.0, pivot_height_m)));

  struct Cylindrical final {
    double radius{};
    double theta{};
    double height{};
  };
  const auto to_cylindrical = [&](const Vec3 point) {
    const Vec3 offset = Sub(point, pivot);
    const double x = Dot(offset, basis.forward);
    const double y = Dot(offset, basis.right);
    return Cylindrical{std::hypot(x, y), std::atan2(y, x),
                       Dot(offset, basis.up)};
  };

  const Cylindrical from = to_cylindrical(base.position);
  const Cylindrical to = to_cylindrical(shot.position);
  const double radius = Lerp(from.radius, to.radius, t);
  const double theta = from.theta + WrapPi(to.theta - from.theta) * t;
  const double height = Lerp(from.height, to.height, t);
  const Vec3 position =
      Add(Add(pivot, Add(Mul(basis.forward, radius * std::cos(theta)),
                         Mul(basis.right, radius * std::sin(theta)))),
          Mul(basis.up, height));

  Vec3 base_direction = Sub(base.target, base.position);
  Vec3 shot_direction = Sub(shot.target, shot.position);
  if (Length(base_direction) < 1e-9) {
    base_direction = ForwardFromYaw(yaw);
  }
  if (Length(shot_direction) < 1e-9) {
    shot_direction = ForwardFromYaw(yaw);
  }
  const Vec3 direction =
      SlerpDirection(base_direction, shot_direction, t);

  Vec3 up = Normalize(Add(Mul(base.up, 1.0 - t), Mul(shot.up, t)));
  up = OrthonormalUp(up, direction,
                     Length(shot.up) > 1e-9 ? shot.up : basis.up);

  CameraState result;
  result.position = position;
  result.target = Add(position, direction);
  result.up = up;
  result.fov_y_degrees = Lerp(base.fov_y_degrees, shot.fov_y_degrees, t);
  result.roll = Lerp(base.roll, shot.roll, t);
  result.roll_degrees = result.roll * kDegreesPerRadian;
  result.pitch_degrees =
      std::asin(Clamp(direction.z, -1.0, 1.0)) * kDegreesPerRadian;
  result.yaw_degrees =
      std::atan2(direction.y, direction.x) * kDegreesPerRadian;
  return result;
}

}  // namespace nfs2015_chase
