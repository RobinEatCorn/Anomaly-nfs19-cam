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
  double max_angle_radians{32.0 * kRadiansPerDegree};
  double gain{1.0};
  double enter_slip_radians{6.5 * kRadiansPerDegree};
  double exit_slip_radians{3.0 * kRadiansPerDegree};
  double full_slip_radians{40.0 * kRadiansPerDegree};
  double rate_radians_per_second{120.0 * kRadiansPerDegree};
  // NFS 2015 only checks this speed when entering a drift. Once the state
  // machine is engaged, speed is not an exit condition; a large drift angle
  // can make forward speed approach zero while the car is still sliding fast.
  double entry_speed_mps{40.0 / 3.6};
  double hold_seconds{0.5};
  double exit_hold_seconds{0.25};
  double blend_in_seconds{0.20};
  double blend_out_seconds{0.30};
  double stock_max_angle_radians{35.0 * kRadiansPerDegree};
  // A zero countersteer target means "no countersteer yet", not "center the
  // wheel". Entry waits until the target has enough authority, so a normal
  // high-speed turn does not park the stock animation just because the slip
  // crossed the entry threshold.
  double counter_authority_radians{5.0 * kRadiansPerDegree};
  double restore_tolerance_radians{1.5 * kRadiansPerDegree};
  // Time spent showing the player's turn-in angle before the countersteer
  // blend starts. This is the visual equivalent of the NFS steering phase
  // before RemapSteeringForDrift hands the wheel to the countersteer target.
  double turn_in_seconds{0.18};
  double turn_in_min_seconds{0.10};
  double turn_in_max_seconds{0.30};
  // Small drifts should not flash the full stock angle. Scale the turn-in
  // target and duration by the rear-axle slip between these two thresholds.
  // Below the start threshold the turn-in phase is skipped entirely.
  double turn_in_start_slip_radians{8.0 * kRadiansPerDegree};
  double turn_in_full_slip_radians{22.0 * kRadiansPerDegree};
  double turn_in_min_alpha{0.08};
  // The explicit turn-in sweep is separate from the countersteer rate. It has
  // to reach the scaled stock target before the turn-in timer expires, even
  // when the stock wheel animation was still ramping up at takeover.
  double turn_in_rate_radians_per_second{300.0 * kRadiansPerDegree};
  double turn_in_max_rate_radians_per_second{900.0 * kRadiansPerDegree};
  double turn_in_release_slip_radians{18.0 * kRadiansPerDegree};
  // After a real release, do not immediately re-enter on the next tick. This
  // is not a speed threshold; it prevents state chatter at the exit slip.
  double reentry_cooldown_seconds{0.5};
  // Numerical degeneracy guard, not a driving-speed floor. When the total
  // velocity vector is almost zero there is no direction from which to compute
  // a countersteer target; hold the last target briefly while the car recovers.
  double standstill_speed_mps{0.5};
  double standstill_grace_seconds{0.5};
};

enum class WheelSteerPhase : std::uint8_t {
  grip = 0,
  turn_in,
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
  double turn_in_seconds{};
  // Captured at entry. Keeping the entry slip scale stable avoids the turn-in
  // target moving around while the rear slip is still crossing the threshold.
  double turn_in_duration{};
  double turn_in_alpha{};
  double blend{};
  // Last countersteer target from a valid drift tick. The release phase keeps
  // using it while the blend returns to the stock angle; recomputing it from a
  // zeroed low-speed travel vector would first steer the wheel straight and
  // then back to stock.
  double counter_angle{};
  double reentry_cooldown{};
  double standstill_seconds{};
};

struct WheelSteerTarget final {
  // True from turn-in through release. The plugin keeps the wheel pose until
  // the release blend has returned to the stock angle.
  bool active{};
  // True while the state machine is entering or holding a drift.
  bool engaged{};
  // Final target after blending the stock angle and the countersteer angle.
  double angle_radians{};
  // Raw velocity-facing countersteer target, before the stock-angle blend.
  double counter_angle_radians{};
  // The stock front-wheel estimate from the player's steering input.
  double stock_angle_radians{};
  // Blend after the state blend. The countersteer authority gate is applied
  // when entering, so this is the final visual blend.
  double angle_blend{};
  double blend{};
  // Entry-slip scale captured by the state machine. The plugin uses it to
  // blend between the measured stock pose and the raw stock target, and to
  // choose how long the turn-in phase should last.
  double turn_in_alpha{};
  double turn_in_duration_seconds{};
  WheelSteerPhase phase{WheelSteerPhase::grip};
};

struct WheelSteerTravel final {
  double front_angle_radians{};
  double rear_angle_radians{};
  bool front_valid{};
  bool rear_valid{};
};

// Compute the travel angle at a point offset from the velocity reference. The
// rigid-body velocity is v(P) = v(center) + omega x r. Using the front-axle
// centre instead of the body centre keeps the visual countersteer tied to the
// wheel contact path; using the rear-axle centre makes the entry/exit slip
// closer to the NFS 2015 rear-wheel slip signal.
[[nodiscard]] inline bool ComputeWheelSteerTravelAngle(
    const Vec3 car_forward, const Vec3 car_up, const Vec3 world_velocity,
    double& angle_radians) noexcept {
  const Vec3 up = Normalize(car_up);
  if (Length(up) < 1e-6) {
    return false;
  }
  Vec3 forward = Sub(car_forward, Mul(up, Dot(car_forward, up)));
  Vec3 travel = Sub(world_velocity, Mul(up, Dot(world_velocity, up)));
  if (Length(forward) < 1e-6 || Length(travel) < 1e-6) {
    return false;
  }
  forward = Normalize(forward);
  travel = Normalize(travel);
  angle_radians =
      std::atan2(Dot(up, Cross(forward, travel)), Dot(forward, travel));
  return std::isfinite(angle_radians);
}

[[nodiscard]] inline WheelSteerTravel ComputeWheelSteerTravel(
    const Vec3 car_forward, const Vec3 car_up, const Vec3 world_velocity,
    const double yaw_rate, const Vec3 front_axle_offset,
    const Vec3 rear_axle_offset) noexcept {
  WheelSteerTravel result;
  const Vec3 up = Normalize(car_up);
  if (Length(up) < 1e-6 || !std::isfinite(yaw_rate)) {
    return result;
  }

  const Vec3 omega = Mul(up, yaw_rate);
  const Vec3 front_velocity =
      Add(world_velocity, Cross(omega, front_axle_offset));
  const Vec3 rear_velocity =
      Add(world_velocity, Cross(omega, rear_axle_offset));

  double body_angle = 0.0;
  const bool body_valid = ComputeWheelSteerTravelAngle(
      car_forward, car_up, world_velocity, body_angle);

  result.front_valid = ComputeWheelSteerTravelAngle(
      car_forward, car_up, front_velocity, result.front_angle_radians);
  if (!result.front_valid && body_valid) {
    result.front_angle_radians = body_angle;
    result.front_valid = true;
  }
  result.rear_valid = ComputeWheelSteerTravelAngle(
      car_forward, car_up, rear_velocity, result.rear_angle_radians);
  if (!result.rear_valid && body_valid) {
    result.rear_angle_radians = body_angle;
    result.rear_valid = true;
  }
  return result;
}

// Advance the NFS-style drift state machine and return the visual front-wheel
// target for this tick.
//
// `front_travel` is the front-axle travel angle and `rear_travel` is the
// rear-axle travel angle. Entry/exit use the rear angle; the countersteer
// magnitude uses the front angle. If the caller cannot read wheel positions it
// can pass the same body angle for both.
//
// The speed gate is entry-only. Once `turn_in`, `entering`, `drifting` or
// `releasing` is active, speed and forward speed are not exit conditions. A
// large drift can make `forward_speed_mps` approach zero or go negative while
// the total velocity is still high; that is a normal part of the drift, not a
// reason to straighten the wheel. `reversing` only blocks entry; it is not an
// exit gate, because a >90 degree slide can look like reverse to a signed
// forward-speed signal.
[[nodiscard]] inline WheelSteerTarget AdvanceWheelSteerTarget(
    WheelSteerDriftState& state, const double front_travel_angle,
    const double rear_travel_angle,
    const double speed_mps, const double forward_speed_mps,
    const double steering_input, const bool steering_known,
    const bool reversing, const double dt,
    const WheelSteerConfig& config) noexcept {
  WheelSteerTarget result;
  result.stock_angle_radians =
      steering_known
          ? Clamp(steering_input, -1.0, 1.0) *
                config.stock_max_angle_radians
          : 0.0;

  const double step = std::isfinite(dt) ? Clamp(dt, 0.0, 0.25) : 0.0;
  const double finite_speed = std::isfinite(speed_mps) ? speed_mps : 0.0;
  const double finite_forward =
      std::isfinite(forward_speed_mps) ? forward_speed_mps : 0.0;
  const bool finite_angles =
      std::isfinite(front_travel_angle) &&
      std::isfinite(rear_travel_angle);
  const bool motion_valid =
      config.enabled && std::isfinite(speed_mps) &&
      finite_speed >= config.standstill_speed_mps;
  const bool entry_valid =
      motion_valid && !reversing && finite_angles &&
      finite_speed >= config.entry_speed_mps && finite_forward > 0.1;

  const double absolute_slip =
      finite_angles ? std::fabs(rear_travel_angle) : 0.0;
  // The turn-in pose is only needed for a deliberate drift entry. Scale it by
  // rear slip so a small slide does not show the same full-lock stock angle as
  // a high-speed U-turn.
  const double turn_in_start_slip =
      (std::max)(0.0, config.turn_in_start_slip_radians);
  const double turn_in_full_slip = (std::max)(
      config.turn_in_full_slip_radians, turn_in_start_slip + 1e-6);
  const double turn_in_alpha =
      finite_angles
          ? SmoothStep(turn_in_start_slip, turn_in_full_slip,
                       absolute_slip)
          : 0.0;
  const bool start_requested =
      entry_valid && absolute_slip >= config.enter_slip_radians;
  const bool below_exit =
      motion_valid && finite_angles &&
      absolute_slip < config.exit_slip_radians;

  const double full_slip = (std::max)(
      config.full_slip_radians, config.enter_slip_radians + 1e-6);
  const double normalized_slip =
      Clamp((absolute_slip - config.enter_slip_radians) /
                (full_slip - config.enter_slip_radians),
            0.0, 1.0);
  const double counter_scale =
      normalized_slip * normalized_slip * (3.0 - 2.0 * normalized_slip);
  const double raw_counter =
      finite_angles
          ? Clamp(front_travel_angle * config.gain * counter_scale,
                  -config.max_angle_radians, config.max_angle_radians)
          : 0.0;
  const bool has_authority =
      std::fabs(raw_counter) >= config.counter_authority_radians;

  state.reentry_cooldown =
      (std::max)(0.0, state.reentry_cooldown - step);
  if (motion_valid) {
    state.standstill_seconds = 0.0;
  } else if (state.phase != WheelSteerPhase::grip &&
             state.phase != WheelSteerPhase::releasing) {
    state.standstill_seconds += step;
  }
  const bool standstill_expired =
      !motion_valid && state.phase != WheelSteerPhase::grip &&
      state.phase != WheelSteerPhase::releasing &&
      state.standstill_seconds >= config.standstill_grace_seconds;

  const auto update_counter = [&]() {
    if (motion_valid && finite_angles && !below_exit) {
      state.counter_angle = raw_counter;
    }
  };
  const auto enter_release = [&]() {
    state.phase = WheelSteerPhase::releasing;
  };
  const auto enter_entering = [&]() {
    state.phase = WheelSteerPhase::entering;
    state.turn_in_seconds = 0.0;
    state.turn_in_duration = 0.0;
  };

  switch (state.phase) {
    case WheelSteerPhase::grip:
      state.blend = 0.0;
      state.hold_seconds = 0.0;
      state.exit_hold_seconds = 0.0;
      state.turn_in_seconds = 0.0;
      state.turn_in_duration = 0.0;
      state.turn_in_alpha = 0.0;
      state.standstill_seconds = 0.0;
      if (start_requested && state.reentry_cooldown <= 0.0 &&
          has_authority) {
        const bool steering_into =
            steering_known && steering_input * raw_counter < 0.0;
        // If the driver is already countersteering there is nothing to turn
        // into, so keep the entry scale at zero and blend straight to the
        // countersteer target. For an into-corner entry, low slip skips the
        // phase entirely and high slip gets the full stock sweep.
        state.turn_in_alpha = steering_into ? turn_in_alpha : 0.0;
        if (steering_into &&
            state.turn_in_alpha >= config.turn_in_min_alpha) {
          state.phase = WheelSteerPhase::turn_in;
          const double scaled_duration =
              config.turn_in_seconds * state.turn_in_alpha;
          state.turn_in_duration = Clamp(
              scaled_duration, config.turn_in_min_seconds,
              config.turn_in_max_seconds);
        } else {
          state.phase = WheelSteerPhase::entering;
          state.turn_in_duration = 0.0;
        }
        state.hold_seconds = 0.0;
        state.exit_hold_seconds = 0.0;
        state.turn_in_seconds = 0.0;
        state.counter_angle = raw_counter;
      } else {
        state.counter_angle = 0.0;
      }
      break;

    case WheelSteerPhase::turn_in:
      state.hold_seconds += step;
      state.turn_in_seconds += step;
      update_counter();
      if (below_exit) {
        state.exit_hold_seconds += step;
      } else {
        state.exit_hold_seconds = 0.0;
      }
      if (standstill_expired) {
        enter_release();
      } else if (below_exit &&
                 state.hold_seconds >= config.hold_seconds &&
                 state.exit_hold_seconds >= config.exit_hold_seconds) {
        enter_release();
      } else {
        const bool steering_into =
            steering_known && steering_input * raw_counter < 0.0;
        const bool slip_high =
            finite_angles &&
            std::fabs(rear_travel_angle) >=
                config.turn_in_release_slip_radians &&
            state.turn_in_seconds >= config.turn_in_min_seconds;
        if (!steering_into || slip_high ||
            state.turn_in_seconds >= config.turn_in_max_seconds ||
            state.turn_in_seconds >= state.turn_in_duration) {
          enter_entering();
        }
      }
      break;

    case WheelSteerPhase::entering:
    case WheelSteerPhase::drifting:
      state.hold_seconds += step;
      update_counter();
      if (below_exit) {
        state.exit_hold_seconds += step;
      } else {
        state.exit_hold_seconds = 0.0;
      }
      if (standstill_expired) {
        enter_release();
      } else if (below_exit &&
                 state.hold_seconds >= config.hold_seconds &&
                 state.exit_hold_seconds >= config.exit_hold_seconds) {
        enter_release();
      } else {
        state.blend = MoveTowards(
            state.blend, 1.0,
            step / (std::max)(config.blend_in_seconds, 1e-6));
        if (state.phase == WheelSteerPhase::entering &&
            state.blend >= 1.0 - 1e-9) {
          state.phase = WheelSteerPhase::drifting;
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
        state.turn_in_seconds = 0.0;
        state.turn_in_duration = 0.0;
        state.turn_in_alpha = 0.0;
        state.counter_angle = 0.0;
        state.standstill_seconds = 0.0;
        state.reentry_cooldown = config.reentry_cooldown_seconds;
      }
      break;
  }

  result.counter_angle_radians = state.counter_angle;
  const double smooth_blend =
      state.blend * state.blend * (3.0 - 2.0 * state.blend);
  result.angle_blend = smooth_blend;
  result.angle_radians =
      Lerp(result.stock_angle_radians, result.counter_angle_radians,
           result.angle_blend);
  result.blend = state.blend;
  result.turn_in_alpha = state.turn_in_alpha;
  result.turn_in_duration_seconds = state.turn_in_duration;
  result.phase = state.phase;
  // The authority gate is applied when entering. Once the state machine has
  // committed to a drift, keep the wheel override through release so the
  // handoff back to the stock animation stays continuous.
  result.active = state.phase != WheelSteerPhase::grip;
  result.engaged = state.phase == WheelSteerPhase::turn_in ||
                   state.phase == WheelSteerPhase::entering ||
                   state.phase == WheelSteerPhase::drifting;
  return result;
}

// The state machine's target is expressed against the raw steering input. The
// plugin knows the wheel's actual pose at takeover, so it blends from that
// measured pose to the scaled stock target before applying the countersteer
// blend. Small drifts stay near the measured pose; a high-slip U-turn reaches
// the full stock angle and then hands over to countersteer.
[[nodiscard]] inline double ComposeWheelSteerStockAngle(
    const double entry_stock_angle, const WheelSteerTarget& target) noexcept {
  if (target.phase == WheelSteerPhase::grip ||
      target.phase == WheelSteerPhase::releasing) {
    return entry_stock_angle;
  }
  return Lerp(entry_stock_angle, target.stock_angle_radians,
              target.turn_in_alpha);
}

[[nodiscard]] inline double ComposeWheelSteerTargetAngle(
    const double entry_stock_angle, const WheelSteerTarget& target) noexcept {
  const double stock =
      ComposeWheelSteerStockAngle(entry_stock_angle, target);
  if (target.phase == WheelSteerPhase::grip) {
    return stock;
  }
  return Lerp(stock, target.counter_angle_radians, target.angle_blend);
}

// Advance the visual wheel angle for one tick. Keeping this rate calculation
// here lets the turn-in sweep be tested without the game, and keeps the plugin
// from accidentally using the slower countersteer rate during turn-in.
[[nodiscard]] inline double AdvanceWheelSteerAppliedAngle(
    const double current_angle, const WheelSteerTarget& target,
    const double dt, const WheelSteerConfig& config) noexcept {
  const double step = std::isfinite(dt) ? Clamp(dt, 0.0, 0.25) : 0.0;
  double rate = config.rate_radians_per_second;
  if (target.phase == WheelSteerPhase::turn_in) {
    // A high-slip entry can leave turn_in after turn_in_min_seconds even when
    // the scaled duration is longer. Use the earliest possible handoff time so
    // the stock sweep is complete before the countersteer blend begins.
    const double sweep_time = (std::max)(
        1e-3, (std::min)(target.turn_in_duration_seconds,
                         config.turn_in_min_seconds));
    const double needed =
        std::fabs(target.angle_radians - current_angle) / sweep_time;
    rate = (std::max)(
        rate,
        (std::max)(config.turn_in_rate_radians_per_second,
                   (std::min)(needed,
                              config.turn_in_max_rate_radians_per_second)));
  }
  return MoveTowards(current_angle, target.angle_radians, rate * step);
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
