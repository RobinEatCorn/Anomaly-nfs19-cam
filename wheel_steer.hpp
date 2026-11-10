#pragma once

// Visual front-wheel steering for the NFS 2015 drift look.
//
// NTE drives the front wheels by rotating the wheel socket, so the wheel
// component's RelativeRotation stays zero. To override that visual angle we
// cannot write a fixed yaw: the socket's local axes are not the car's axes.
// Instead we read the current wheel world quaternion, work out how far its axle
// has to rotate to reach the target forward direction, and apply that delta in
// the socket's local frame.
//
// The math is deliberately pure here so it can be tested without the game. The
// plugin owns the memory reads/writes; this file owns the geometry.

#include "chase_cam.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nfs2015_chase {

struct Quat final {
  double x{};
  double y{};
  double z{};
  double w{1.0};
};

[[nodiscard]] inline Quat QuatMul(const Quat a, const Quat b) noexcept {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

[[nodiscard]] inline Quat QuatConjugate(const Quat q) noexcept {
  return {-q.x, -q.y, -q.z, q.w};
}

[[nodiscard]] inline Quat QuatNormalize(const Quat q) noexcept {
  const double length =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(length > 1e-12) || !std::isfinite(length)) {
    return {};
  }
  const double inverse = 1.0 / length;
  return {q.x * inverse, q.y * inverse, q.z * inverse, q.w * inverse};
}

// q * v * q^-1, the convention used by the validated probe script.
[[nodiscard]] inline Vec3 QuatRotate(const Quat q, const Vec3 v) noexcept {
  const Quat rotation =
      QuatMul(QuatMul(q, {v.x, v.y, v.z, 0.0}), QuatConjugate(q));
  return {rotation.x, rotation.y, rotation.z};
}

[[nodiscard]] inline Quat AxisAngle(const Vec3 axis, const double radians) noexcept {
  const Vec3 unit = Normalize(axis);
  const double half = radians * 0.5;
  const double sine = std::sin(half);
  return {unit.x * sine, unit.y * sine, unit.z * sine, std::cos(half)};
}

// UE FRotator is (Pitch, Yaw, Roll), in degrees. These two conversions mirror
// the probe script that was validated against the live wheel transforms.
[[nodiscard]] inline Quat QuatFromRotatorDegrees(
    const double rotator[3]) noexcept {
  const double pitch = rotator[0] * kRadiansPerDegree;
  const double yaw = rotator[1] * kRadiansPerDegree;
  const double roll = rotator[2] * kRadiansPerDegree;
  const double sp = std::sin(pitch * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cr = std::cos(roll * 0.5);
  return {cr * sp * sy - sr * cp * cy,
          -cr * sp * cy - sr * cp * sy,
          cr * cp * sy - sr * sp * cy,
          cr * cp * cy + sr * sp * sy};
}

inline void RotatorDegreesFromQuat(const Quat q, double out[3]) noexcept {
  // This is UE's FQuat::Rotator(), not a generic XYZ Euler decomposition. The
  // generic form is not the inverse of FRotator::Quaternion() and can produce a
  // completely different orientation when the wheel roll puts the quaternion
  // near the Euler singularity. That mismatch is what made the parked wheel
  // look "sprained" only at some roll phases.
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  const double w = q.w;
  const double singularity = z * x - w * y;
  const double yaw_y = 2.0 * (w * z + x * y);
  const double yaw_x = 1.0 - 2.0 * (y * y + z * z);
  constexpr double kSingularityThreshold = 0.4999995;
  if (singularity < -kSingularityThreshold) {
    out[0] = -90.0;
    out[1] = std::atan2(yaw_y, yaw_x) * kDegreesPerRadian;
    out[2] = -out[1] -
             2.0 * std::atan2(x, w) * kDegreesPerRadian;
  } else if (singularity > kSingularityThreshold) {
    out[0] = 90.0;
    out[1] = std::atan2(yaw_y, yaw_x) * kDegreesPerRadian;
    out[2] = out[1] -
             2.0 * std::atan2(x, w) * kDegreesPerRadian;
  } else {
    out[0] = std::asin((std::max)(-1.0, (std::min)(1.0, 2.0 * singularity))) *
             kDegreesPerRadian;
    out[1] = std::atan2(yaw_y, yaw_x) * kDegreesPerRadian;
    out[2] = std::atan2(-2.0 * (w * x + y * z),
                        1.0 - 2.0 * (x * x + y * y)) *
             kDegreesPerRadian;
  }
}

[[nodiscard]] inline double MoveTowards(const double current,
                                        const double target,
                                        const double maximum_delta) noexcept {
  if (maximum_delta <= 0.0) {
    return target;
  }
  const double difference = target - current;
  if (difference > maximum_delta) {
    return current + maximum_delta;
  }
  if (difference < -maximum_delta) {
    return current - maximum_delta;
  }
  return target;
}

struct WheelSteerConfig final {
  bool enabled{true};
  double max_angle_radians{40.0 * kRadiansPerDegree};
  double gain{1.0};
  double enter_slip_radians{6.5 * kRadiansPerDegree};
  double exit_slip_radians{3.0 * kRadiansPerDegree};
  double full_slip_radians{35.0 * kRadiansPerDegree};
  double rate_radians_per_second{120.0 * kRadiansPerDegree};
  double minimum_speed_mps{30.0 / kMphPerMetrePerSecond};
  double hold_seconds{0.5};
  double exit_hold_seconds{0.25};
  double blend_in_seconds{0.20};
  double blend_out_seconds{0.30};
  double stock_max_angle_radians{35.0 * kRadiansPerDegree};
  // A zero countersteer target means "no countersteer yet", not "center the
  // wheel". At low slip, preserving the stock angle avoids straightening a
  // driver's full steering input during a normal high-speed turn.
  double counter_authority_radians{6.0 * kRadiansPerDegree};
  double restore_tolerance_radians{1.5 * kRadiansPerDegree};
};

enum class WheelSteerPhase : std::uint8_t {
  grip = 0,
  entering,
  drifting,
  releasing,
};

// Persistent part of the visual drift steering state machine. Keeping this
// separate from the game-memory state makes the timing and hysteresis testable
// without a running game.
struct WheelSteerDriftState final {
  WheelSteerPhase phase{WheelSteerPhase::grip};
  double hold_seconds{};
  double exit_hold_seconds{};
  double blend{};
  // Last countersteer target from a valid drift tick. The release phase keeps
  // using it while the blend returns to the stock angle; recomputing it from a
  // zeroed low-speed travel vector would first steer the wheel straight and
  // then back to stock.
  double counter_angle{};
};

struct WheelSteerTarget final {
  // True while the plugin should take over the wheel pose. It waits until the
  // countersteer target has meaningful authority, so a normal turn does not
  // freeze the stock animation just because the travel-angle state entered a
  // drift. Once taken over, the plugin keeps the pose through release.
  bool active{};
  // True only while the drift state machine is entering or holding a drift.
  bool engaged{};
  // Final target after blending the stock angle and the countersteer angle.
  double angle_radians{};
  // Raw velocity-facing countersteer target, before the stock-angle blend.
  double counter_angle_radians{};
  // The stock front-wheel estimate from the player's steering input.
  double stock_angle_radians{};
  // Blend after both the state blend and counter-authority gate.
  double angle_blend{};
  double blend{};
  WheelSteerPhase phase{WheelSteerPhase::grip};
};

// Advance the NFS-style drift state machine and return the visual front-wheel
// target for this tick.
//
// The NFS 2015 entry gate is speed plus rear slip. NTE does not expose a
// reliable rear tire slip value here, so the body travel angle is used as the
// fallback. The 30 mph speed gate is what keeps a low-speed U-turn or a
// standstill donut on the stock steering: below that speed the car is not in a
// drift, even if its yaw and velocity directions differ.
//
// `steering_input` is the raw player steering, -1..1, positive to the right in
// UE axes. It is used for the stock-angle handoff, not for the countersteer
// target. The countersteer target still comes from the actual travel direction.
[[nodiscard]] inline WheelSteerTarget AdvanceWheelSteerTarget(
    WheelSteerDriftState& state, const Vec3 car_forward, const Vec3 car_up,
    const Vec3 world_velocity, const double forward_speed_mps,
    const double speed_mps, const double steering_input,
    const bool steering_known, const double dt,
    const WheelSteerConfig& config) noexcept {
  WheelSteerTarget result;
  result.stock_angle_radians =
      steering_known
          ? Clamp(steering_input, -1.0, 1.0) *
                config.stock_max_angle_radians
          : 0.0;

  const double step = std::isfinite(dt) ? Clamp(dt, 0.0, 0.25) : 0.0;

  // The speed gate is the NFS 2015 `Minimum_speed_to_enter_drift` (30 mph).
  // Requiring a positive forward component also excludes reverse and most
  // handbrake pivots at a standstill.
  bool valid = config.enabled && std::isfinite(speed_mps) &&
               std::isfinite(forward_speed_mps) &&
               speed_mps >= config.minimum_speed_mps &&
               forward_speed_mps > 0.1;

  double travel_angle = 0.0;
  if (valid) {
    const Vec3 up = Normalize(car_up);
    Vec3 forward = Sub(car_forward, Mul(up, Dot(car_forward, up)));
    Vec3 travel = Sub(world_velocity, Mul(up, Dot(world_velocity, up)));
    if (Length(forward) < 1e-6 || Length(travel) < 1e-6) {
      valid = false;
    } else {
      forward = Normalize(forward);
      travel = Normalize(travel);
      travel_angle =
          std::atan2(Dot(up, Cross(forward, travel)), Dot(forward, travel));
    }
  }

  const double absolute_slip = std::fabs(travel_angle);
  const bool start_requested =
      valid && absolute_slip >= config.enter_slip_radians;
  const bool below_exit =
      valid && absolute_slip < config.exit_slip_radians;

  const double full_slip = (std::max)(
      config.full_slip_radians, config.enter_slip_radians + 1e-6);
  const double normalized_slip =
      Clamp((absolute_slip - config.enter_slip_radians) /
                (full_slip - config.enter_slip_radians),
            0.0, 1.0);
  const double counter_scale =
      normalized_slip * normalized_slip * (3.0 - 2.0 * normalized_slip);
  const double raw_counter =
      Clamp(travel_angle * config.gain * counter_scale,
            -config.max_angle_radians, config.max_angle_radians);

  switch (state.phase) {
    case WheelSteerPhase::grip:
      if (start_requested) {
        state.phase = WheelSteerPhase::entering;
        state.hold_seconds = 0.0;
        state.exit_hold_seconds = 0.0;
        state.blend = 0.0;
        state.counter_angle = raw_counter;
      }
      break;

    case WheelSteerPhase::entering:
    case WheelSteerPhase::drifting:
      state.hold_seconds += step;
      // Freeze the last countersteer target as soon as the slip falls below
      // the exit threshold. Updating it to zero during the exit-hold window
      // would first steer the wheel straight, then send it to the stock angle
      // during the release blend.
      if (valid && !below_exit) {
        state.counter_angle = raw_counter;
      }
      if (!valid) {
        // Low speed, reverse or lost ground speed is an immediate release.
        // The blend still runs, so the visual wheel does not snap back.
        state.phase = WheelSteerPhase::releasing;
      } else {
        if (below_exit) {
          state.exit_hold_seconds += step;
        } else {
          state.exit_hold_seconds = 0.0;
        }
        const bool exit_allowed =
            state.hold_seconds >= config.hold_seconds &&
            state.exit_hold_seconds >= config.exit_hold_seconds;
        if (exit_allowed) {
          state.phase = WheelSteerPhase::releasing;
        } else {
          state.blend = MoveTowards(
              state.blend, 1.0,
              step / (std::max)(config.blend_in_seconds, 1e-6));
          if (state.phase == WheelSteerPhase::entering &&
              state.blend >= 1.0 - 1e-9) {
            state.phase = WheelSteerPhase::drifting;
          }
        }
      }
      break;

    case WheelSteerPhase::releasing:
      state.blend = MoveTowards(
          state.blend, 0.0,
          step / (std::max)(config.blend_out_seconds, 1e-6));
      if (state.blend <= 1e-9) {
        state.blend = 0.0;
        state.phase = WheelSteerPhase::grip;
        state.hold_seconds = 0.0;
        state.exit_hold_seconds = 0.0;
        state.counter_angle = 0.0;
      }
      break;
  }

  result.counter_angle_radians = state.counter_angle;

  const double smooth_blend =
      state.blend * state.blend * (3.0 - 2.0 * state.blend);
  const double normalized_authority = Clamp(
      std::fabs(result.counter_angle_radians) /
          (std::max)(config.counter_authority_radians, 1e-6),
      0.0, 1.0);
  const double counter_authority =
      normalized_authority * normalized_authority *
      (3.0 - 2.0 * normalized_authority);
  result.angle_blend = smooth_blend * counter_authority;
  result.angle_radians =
      Lerp(result.stock_angle_radians, result.counter_angle_radians,
           result.angle_blend);
  result.blend = state.blend;
  result.phase = state.phase;
  // Do not park the stock animation for a drift-state tick whose countersteer
  // target is still effectively zero. That would freeze the stock wheels
  // during a normal high-speed turn without producing any visible countersteer.
  result.active = state.phase != WheelSteerPhase::grip &&
                  state.blend * counter_authority > 0.35;
  result.engaged = state.phase == WheelSteerPhase::entering ||
                   state.phase == WheelSteerPhase::drifting;
  return result;
}

// Return the wheel RelativeRotation that makes the wheel's axle perpendicular
// to `target_forward_world`. UE applies the relative quaternion directly:
// wheel_world = socket * relative. `wheel_world` is the wheel component's
// current ComponentToWorld rotation and `relative_old` is the local quaternion
// currently in the component. The socket is therefore
// `wheel_world * conjugate(relative_old)`.
[[nodiscard]] inline bool ComputeSteeredRelativeRotation(
    const Vec3 car_forward, const Vec3 car_up,
    const Vec3 target_forward_world, const Quat wheel_world,
    const Quat relative_old, Quat& relative_new) noexcept {
  const Vec3 up = Normalize(car_up);
  Vec3 forward = Sub(car_forward, Mul(up, Dot(car_forward, up)));
  Vec3 target = Sub(target_forward_world, Mul(up, Dot(target_forward_world, up)));
  if (Length(forward) < 1e-6 || Length(target) < 1e-6) {
    return false;
  }
  forward = Normalize(forward);
  target = Normalize(target);
  const Quat socket =
      QuatNormalize(QuatMul(wheel_world, QuatConjugate(relative_old)));
  const Vec3 axle = Normalize(QuatRotate(wheel_world, {0.0, 1.0, 0.0}));
  const Vec3 desired_axle = Normalize(Cross(up, target));
  const double delta =
      std::atan2(Dot(up, Cross(axle, desired_axle)), Dot(axle, desired_axle));
  const Vec3 local_up = QuatRotate(QuatConjugate(socket), up);
  const Quat delta_rotation = AxisAngle(local_up, delta);
  relative_new = QuatNormalize(QuatMul(delta_rotation, relative_old));
  return std::isfinite(relative_new.x) && std::isfinite(relative_new.y) &&
         std::isfinite(relative_new.z) && std::isfinite(relative_new.w);
}

// Measure the stock front-wheel angle from its world rotation. This is used
// once, before the animation controller is parked, so the first manual pose
// starts from the wheel's real stock angle instead of a zero-angle guess.
// The wheel axle is local Y. A mirrored wheel can have the opposite axle sign,
// so the axle is normalized to the car's right side before the angle is taken.
[[nodiscard]] inline bool ComputeWheelSteerAngleFromWorld(
    const Vec3 car_forward, const Vec3 car_up, const Quat wheel_world,
    double& angle_radians) noexcept {
  const Vec3 up = Normalize(car_up);
  Vec3 forward = Sub(car_forward, Mul(up, Dot(car_forward, up)));
  if (Length(forward) < 1e-6) {
    return false;
  }
  forward = Normalize(forward);
  const Vec3 right = Normalize(Cross(up, forward));
  Vec3 axle = QuatRotate(wheel_world, {0.0, 1.0, 0.0});
  axle = Sub(axle, Mul(up, Dot(axle, up)));
  if (Length(axle) < 1e-6) {
    return false;
  }
  axle = Normalize(axle);
  if (Dot(axle, right) < 0.0) {
    axle = Mul(axle, -1.0);
  }
  const double cosine = Clamp(Dot(axle, right), -1.0, 1.0);
  const double sine = Dot(axle, forward);
  angle_radians = std::atan2(-sine, cosine);
  return std::isfinite(angle_radians);
}

// Compose the visual wheel rotation for the parked-wheel-animation mode.
// `socket_world` is the frozen wheel socket (the transform the game would have
// used before its wheel controller ran). UE applies the relative quaternion
// directly: wheel_world = socket * relative. Rotating the wheel around its
// local Y axle must therefore be right-multiplied by the steer rotation, so the
// axle stays on the steered heading for every roll phase.
[[nodiscard]] inline bool ComputeVisualWheelRelativeRotation(
    const Vec3 car_forward, const Vec3 car_up,
    const Vec3 target_forward_world, const Quat socket_world,
    const double roll_radians, Quat& relative_new) noexcept {
  Quat steer{};
  if (!ComputeSteeredRelativeRotation(car_forward, car_up,
                                      target_forward_world, socket_world, {},
                                      steer)) {
    return false;
  }
  const Quat roll = AxisAngle({0.0, 1.0, 0.0}, roll_radians);
  relative_new = QuatNormalize(QuatMul(steer, roll));
  return std::isfinite(relative_new.x) && std::isfinite(relative_new.y) &&
         std::isfinite(relative_new.z) && std::isfinite(relative_new.w);
}

[[nodiscard]] inline Quat ComputeVisualWheelRollRotation(
    const double roll_radians) noexcept {
  return AxisAngle({0.0, 1.0, 0.0}, roll_radians);
}

// Remove the wheel's own spin phase from a socket rotation. The game's wheel
// controller animates the spin on the socket/bone, and if the flag is parked
// while the wheel is at a random phase, the socket still carries that phase.
// Steering around `car_up` with a spun socket makes the wheel rotate around a
// tilted/fore-aft axis (the "universal joint" look). The spin axis is the wheel
// local Y, so keep the swing and discard the Y twist.
[[nodiscard]] inline Quat RemoveWheelSpinTwist(const Quat q) noexcept {
  const double yw_length = std::sqrt(q.y * q.y + q.w * q.w);
  if (yw_length < 1e-9) {
    // A pure 180-degree spin has no Y/W projection. A rotation with no Y
    // component at all has no spin to remove.
    return std::fabs(q.y) > 0.999 ? Quat{} : q;
  }
  const Quat twist{0.0, q.y / yw_length, 0.0, q.w / yw_length};
  return QuatNormalize(QuatMul(q, QuatConjugate(twist)));
}

}  // namespace nfs2015_chase
