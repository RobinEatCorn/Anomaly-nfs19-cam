// Front-wheel visual counter-steer: resolve the wheel sockets through the
// reflected scene component, take the animation over, and publish the pose the
// model needs while the driver is drifting.
#include "internal.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace nfs2015_chase_cam::detail {

bool ResolveWheelSteerGeometry(Context &context) noexcept {
  WheelSteerState &state = context.wheel_steer;
  const bool had_geometry = state.resolved && state.wheel_count > 0;
  state.next_resolve = context.elapsed + (had_geometry ? 0.5 : 2.0);
  if (state.vehicle == 0 || context.core == nullptr) {
    if (state.animation_disabled) {
      RestoreWheelPose(context);
    }
    state.resolved = false;
    state.wheel_count = 0;
    state.front_count = 0;
    state.rear_count = 0;
    return false;
  }

  std::uintptr_t mesh = 0;
  std::uintptr_t anim = 0;
  std::uintptr_t front_data = 0;
  std::uintptr_t rear_data = 0;
  std::int32_t front_count = 0;
  std::int32_t rear_count = 0;
  const bool geometry_valid =
      ReadPointerAtOffset(context.core, state.vehicle, kVehicleMeshOffset,
                          mesh) &&
      ReadPointerAtOffset(context.core, mesh,
                          kSceneComponentAnimScriptInstanceOffset, anim) &&
      Read(context.core, state.vehicle + kVehicleFrontWheelsOffset,
           front_data) &&
      front_data != 0 &&
      Read(context.core,
           state.vehicle + kVehicleFrontWheelsOffset + kTArrayNumOffset,
           front_count) &&
      Read(context.core, state.vehicle + kVehicleRearWheelsOffset,
           rear_data) &&
      rear_data != 0 &&
      Read(context.core,
           state.vehicle + kVehicleRearWheelsOffset + kTArrayNumOffset,
           rear_count) &&
      front_count > 0 && rear_count > 0 &&
      front_count <= static_cast<std::int32_t>(kMaximumFrontWheels) &&
      rear_count <= static_cast<std::int32_t>(kMaximumRearWheels) &&
      static_cast<std::size_t>(front_count) +
              static_cast<std::size_t>(rear_count) <=
          kMaximumWheelSteerWheels;
  if (!geometry_valid) {
    if (state.animation_disabled) {
      RestoreWheelPose(context);
    }
    if (!state.failure_logged) {
      state.failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: wheel steering geometry unavailable; visual "
          "countersteer stays off");
    }
    state.resolved = false;
    state.wheel_count = 0;
    state.front_count = 0;
    state.rear_count = 0;
    return false;
  }

  std::array<std::uintptr_t, kMaximumWheelSteerWheels> wheels{};
  std::array<bool, kMaximumWheelSteerWheels> front{};
  std::size_t total = 0;
  for (std::int32_t index = 0; index < front_count; ++index) {
    std::uintptr_t wheel = 0;
    nfs2015_chase::Quat transform{};
    if (!Read(context.core,
              front_data + static_cast<std::uintptr_t>(index) *
                               sizeof(std::uintptr_t),
              wheel) ||
        wheel == 0 ||
        !Read(context.core, wheel + kSceneComponentTransformOffset,
              transform)) {
      if (state.animation_disabled) {
        RestoreWheelPose(context);
      }
      if (!state.failure_logged) {
        state.failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: front wheel component changed; visual "
            "countersteer stays off");
      }
      state.resolved = false;
      state.wheel_count = 0;
      state.front_count = 0;
      state.rear_count = 0;
      return false;
    }
    wheels[total] = wheel;
    front[total] = true;
    ++total;
  }
  for (std::int32_t index = 0; index < rear_count; ++index) {
    std::uintptr_t wheel = 0;
    nfs2015_chase::Quat transform{};
    if (!Read(context.core,
              rear_data + static_cast<std::uintptr_t>(index) *
                              sizeof(std::uintptr_t),
              wheel) ||
        wheel == 0 ||
        !Read(context.core, wheel + kSceneComponentTransformOffset,
              transform)) {
      if (state.animation_disabled) {
        RestoreWheelPose(context);
      }
      if (!state.failure_logged) {
        state.failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: rear wheel component changed; visual "
            "countersteer stays off");
      }
      state.resolved = false;
      state.wheel_count = 0;
      state.front_count = 0;
      state.rear_count = 0;
      return false;
    }
    wheels[total] = wheel;
    front[total] = false;
    ++total;
  }

  bool changed = mesh != state.mesh || anim != state.anim ||
                 static_cast<std::size_t>(front_count) != state.front_count ||
                 static_cast<std::size_t>(rear_count) != state.rear_count;
  for (std::size_t index = 0; index < total && !changed; ++index) {
    changed = state.wheels[index] != wheels[index] ||
              state.front[index] != front[index];
  }
  if (changed && state.animation_disabled) {
    RestoreWheelPose(context);
  }
  if (changed) {
    state.base_captured.fill(false);
    state.socket_captured.fill(false);
    state.socket_local.fill({});
  }

  state.mesh = mesh;
  state.anim = anim;
  for (std::size_t index = 0; index < kMaximumWheelSteerWheels; ++index) {
    state.wheels[index] = index < total ? wheels[index] : 0;
    state.front[index] = index < total ? front[index] : false;
  }
  state.wheel_count = total;
  state.front_count = static_cast<std::size_t>(front_count);
  state.rear_count = static_cast<std::size_t>(rear_count);
  state.resolved = true;
  state.failure_logged = false;
  if (!state.geometry_logged) {
    state.geometry_logged = true;
    char message[224]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: wheel steering geometry ready "
                  "(front=%zu rear=%zu mesh=0x%llx anim=0x%llx)",
                  state.front_count, state.rear_count,
                  static_cast<unsigned long long>(state.mesh),
                  static_cast<unsigned long long>(state.anim));
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }
  return true;
}

void RestoreWheelPose(Context &context) noexcept {
  WheelSteerState &state = context.wheel_steer;
  const bool restoring_animation =
      state.animation_saved || state.animation_disabled;
  const bool has_socket_state = [&state]() {
    for (const bool captured : state.socket_captured) {
      if (captured) {
        return true;
      }
    }
    return false;
  }();
  if (!state.active && !state.engaged && !restoring_animation &&
      !has_socket_state && state.applied_angle == 0.0) {
    return;
  }

  // Put the stock socket-relative pose back first. The game's wheel controller
  // is still parked while these writes happen, so the wheel does not see a
  // frame of old manual roll mixed with the stock animation.
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    const std::uintptr_t wheel = state.wheels[index];
    if (wheel == 0 || !state.base_captured[index]) {
      continue;
    }
    static_cast<void>(Write(context.core,
                            wheel + kSceneComponentRelativeRotationOffset,
                            state.base_rotator[index]));
  }

  bool animation_restored = !state.animation_saved;
  if (state.animation_saved) {
    animation_restored =
        state.anim != 0 &&
        Write(context.core, state.anim + kVehicleAnimWheelAnimationOffset,
              state.animation_was_enabled);
  }
  if (animation_restored) {
    state.animation_saved = false;
    state.animation_disabled = false;
    state.anim = 0;
    state.base_captured.fill(false);
  }
  state.socket_captured.fill(false);
  state.socket_local.fill({});
  state.settle_ticks = 0;
  state.roll_angle = 0.0;
  state.applied_angle = 0.0;
  state.target_angle = 0.0;
  state.target_blend = 0.0;
  state.stock_angle = 0.0;
  state.entry_stock_angle = 0.0;
  state.entry_stock_valid = false;
  state.front_travel_angle = 0.0;
  state.rear_travel_angle = 0.0;
  state.yaw_rate = 0.0;
  state.front_axle_offset = {};
  state.rear_axle_offset = {};
  state.axle_offsets_valid = false;
  state.last_mesh = {};
  state.have_last_mesh = false;
  // Preserve the re-entry cooldown across a normal pose restore. The pure
  // state machine sets it when releasing into grip; clearing it here would let
  // the plugin re-enter on the very next tick.
  const double reentry_cooldown = state.drift.reentry_cooldown;
  state.drift = {};
  state.drift.reentry_cooldown = reentry_cooldown;
  state.phase = nfs2015_chase::WheelSteerPhase::grip;
  state.engaged = false;
  state.active = false;
}

void ReleaseWheelSteer(Context &context) noexcept {
  RestoreWheelPose(context);
  context.wheel_steer = WheelSteerState{};
}

bool CaptureWheelAnimation(Context &context) noexcept {
  WheelSteerState &state = context.wheel_steer;
  if (state.animation_saved) {
    return true;
  }
  if (state.anim == 0) {
    return false;
  }
  std::uint8_t enabled = 0;
  if (!Read(context.core, state.anim + kVehicleAnimWheelAnimationOffset,
            enabled)) {
    return false;
  }
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    const std::uintptr_t wheel = state.wheels[index];
    if (wheel == 0) {
      state.base_captured[index] = false;
      continue;
    }
    double relative[3]{};
    if (Read(context.core,
             wheel + kSceneComponentRelativeRotationOffset, relative)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        state.base_rotator[index][axis] = relative[axis];
      }
      state.base_captured[index] = true;
    } else {
      state.base_captured[index] = false;
    }
  }
  state.animation_was_enabled = enabled;
  state.animation_saved = true;
  return true;
}

bool EnsureWheelAnimationDisabled(Context &context) noexcept {
  WheelSteerState &state = context.wheel_steer;
  if (!state.animation_saved && !CaptureWheelAnimation(context)) {
    return false;
  }
  if (!Write(context.core, state.anim + kVehicleAnimWheelAnimationOffset,
             std::uint8_t{0})) {
    return false;
  }
  if (!state.animation_disabled) {
    state.animation_disabled = true;
    state.settle_ticks = 4;
    state.roll_angle = 0.0;
    state.socket_captured.fill(false);
    state.socket_local.fill({});
  }
  return true;
}

bool CaptureWheelSockets(Context &context) noexcept {
  WheelSteerState &state = context.wheel_steer;
  nfs2015_chase::Quat mesh{};
  if (!Read(context.core, state.mesh + kSceneComponentTransformOffset, mesh)) {
    return false;
  }
  mesh = nfs2015_chase::QuatNormalize(mesh);
  if (!std::isfinite(mesh.x) || !std::isfinite(mesh.y) ||
      !std::isfinite(mesh.z) || !std::isfinite(mesh.w)) {
    return false;
  }
  bool captured_any = false;
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    if (!state.front[index]) {
      continue;
    }
    const std::uintptr_t wheel = state.wheels[index];
    nfs2015_chase::Quat wheel_world{};
    if (wheel == 0 ||
        !Read(context.core, wheel + kSceneComponentTransformOffset,
              wheel_world)) {
      return false;
    }
    wheel_world = nfs2015_chase::QuatNormalize(wheel_world);
    nfs2015_chase::Quat base{};
    if (state.base_captured[index]) {
      base = nfs2015_chase::QuatNormalize(
          nfs2015_chase::QuatFromRotatorDegrees(
              state.base_rotator[index].data()));
    }
    // UE applies the relative quaternion directly:
    // wheel_world = socket_world * relative. Multiplying by the conjugate of
    // the captured stock relative therefore recovers the frozen socket.
    const nfs2015_chase::Quat socket_world = nfs2015_chase::QuatNormalize(
        nfs2015_chase::QuatMul(
            wheel_world,
            nfs2015_chase::QuatConjugate(base)));
    state.socket_local[index] = nfs2015_chase::RemoveWheelSpinTwist(
        nfs2015_chase::QuatNormalize(
            nfs2015_chase::QuatMul(
                nfs2015_chase::QuatConjugate(mesh), socket_world)));
    state.socket_captured[index] = true;
    captured_any = true;
  }
  return captured_any;
}

const char *WheelSteerPhaseName(
    const nfs2015_chase::WheelSteerPhase phase) noexcept {
  switch (phase) {
    case nfs2015_chase::WheelSteerPhase::grip:
      return "grip";
    case nfs2015_chase::WheelSteerPhase::turn_in:
      return "turn_in";
    case nfs2015_chase::WheelSteerPhase::entering:
      return "entering";
    case nfs2015_chase::WheelSteerPhase::drifting:
      return "drifting";
    case nfs2015_chase::WheelSteerPhase::releasing:
      return "releasing";
  }
  return "unknown";
}

// Read the front-wheel stock angle before parking the animation controller.
// This is only called on the first frame of a drift, while the stock socket
// still holds the player's current steering. Averaging the two front wheels
// also gives the small Ackermann difference a neutral handoff point.
bool MeasureStockWheelSteer(Context &context, const WheelSteerState &state,
                            const nfs2015_chase::Vec3 car_forward,
                            const nfs2015_chase::Vec3 car_up,
                            double &angle_radians) noexcept {
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    if (!state.front[index] || state.wheels[index] == 0) {
      continue;
    }
    nfs2015_chase::Quat wheel_world{};
    if (!Read(context.core,
              state.wheels[index] + kSceneComponentTransformOffset,
              wheel_world)) {
      continue;
    }
    double angle = 0.0;
    if (!nfs2015_chase::ComputeWheelSteerAngleFromWorld(
            car_forward, car_up,
            nfs2015_chase::QuatNormalize(wheel_world), angle)) {
      continue;
    }
    sum += angle;
    ++count;
  }
  if (count == 0) {
    return false;
  }
  angle_radians = sum / static_cast<double>(count);
  return std::isfinite(angle_radians);
}

// Average the front/rear wheel ComponentToWorld translations and express them
// relative to the vehicle mesh. The wheel controller animates suspension, so
// this is sampled every tick rather than cached with the component pointers.
// Returns false when the translations are unavailable; the caller then falls
// back to the body-centre travel angle.
bool ReadWheelAxleOffsets(Context &context, const WheelSteerState &state,
                          nfs2015_chase::Vec3 &front_offset,
                          nfs2015_chase::Vec3 &rear_offset) noexcept {
  double mesh_translation[3]{};
  if (!Read(context.core,
            state.mesh + kSceneComponentTransformTranslationOffset,
            mesh_translation)) {
    return false;
  }
  for (double &value : mesh_translation) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  double front_sum[3]{};
  double rear_sum[3]{};
  std::size_t front_count = 0;
  std::size_t rear_count = 0;
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    if (state.wheels[index] == 0) {
      continue;
    }
    double translation[3]{};
    if (!Read(context.core,
              state.wheels[index] +
                  kSceneComponentTransformTranslationOffset,
              translation)) {
      return false;
    }
    bool finite = true;
    for (double value : translation) {
      finite = finite && std::isfinite(value);
    }
    if (!finite) {
      return false;
    }
    double *sum = state.front[index] ? front_sum : rear_sum;
    std::size_t &count = state.front[index] ? front_count : rear_count;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sum[axis] += translation[axis];
    }
    ++count;
  }
  if (front_count == 0 || rear_count == 0) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    front_sum[axis] /= static_cast<double>(front_count);
    rear_sum[axis] /= static_cast<double>(rear_count);
  }
  front_offset = {(front_sum[0] - mesh_translation[0]) / 100.0,
                  (front_sum[1] - mesh_translation[1]) / 100.0,
                  (front_sum[2] - mesh_translation[2]) / 100.0};
  rear_offset = {(rear_sum[0] - mesh_translation[0]) / 100.0,
                 (rear_sum[1] - mesh_translation[1]) / 100.0,
                 (rear_sum[2] - mesh_translation[2]) / 100.0};
  if (!std::isfinite(front_offset.x) || !std::isfinite(front_offset.y) ||
      !std::isfinite(front_offset.z) || !std::isfinite(rear_offset.x) ||
      !std::isfinite(rear_offset.y) || !std::isfinite(rear_offset.z)) {
    return false;
  }
  // Plausibility guard: a wheel-to-mesh offset is roughly one to three metres.
  // If the component transform is not in world space (or a stale pointer was
  // read), a huge offset would produce a wildly wrong front-axle velocity.
  const double front_length =
      std::sqrt(front_offset.x * front_offset.x +
                front_offset.y * front_offset.y +
                front_offset.z * front_offset.z);
  const double rear_length =
      std::sqrt(rear_offset.x * rear_offset.x +
                rear_offset.y * rear_offset.y +
                rear_offset.z * rear_offset.z);
  return front_length >= 0.2 && front_length <= 5.0 &&
         rear_length >= 0.2 && rear_length <= 5.0;
}

// Estimate yaw rate from the vehicle mesh quaternion. This is deliberately
// independent of the camera yaw: during a drift the camera lags, while the
// front-axle velocity correction must follow the body. Fall back to the
// existing camera/vehicle yaw rate on the first tick or a bad sample.
double EstimateMeshYawRate(WheelSteerState &state,
                           const nfs2015_chase::Quat mesh,
                           const nfs2015_chase::Vec3 car_up,
                           const double fallback_yaw_rate,
                           const double dt) noexcept {
  double raw_rate = fallback_yaw_rate;
  if (state.have_last_mesh && std::isfinite(dt) && dt > 1e-5) {
    nfs2015_chase::Quat delta = nfs2015_chase::QuatMul(
        mesh, nfs2015_chase::QuatConjugate(state.last_mesh));
    if (delta.w < 0.0) {
      delta.x = -delta.x;
      delta.y = -delta.y;
      delta.z = -delta.z;
      delta.w = -delta.w;
    }
    const double vector_length =
        std::sqrt(delta.x * delta.x + delta.y * delta.y +
                  delta.z * delta.z);
    if (vector_length > 1e-9) {
      const double angle =
          2.0 * std::atan2(vector_length, delta.w);
      const nfs2015_chase::Vec3 axis{
          delta.x / vector_length, delta.y / vector_length,
          delta.z / vector_length};
      raw_rate = nfs2015_chase::Dot(axis, car_up) * angle / dt;
    }
  }
  state.last_mesh = mesh;
  state.have_last_mesh = true;
  if (!std::isfinite(raw_rate)) {
    raw_rate = 0.0;
  }
  raw_rate = nfs2015_chase::Clamp(raw_rate, -kMaximumYawRate,
                                  kMaximumYawRate);
  state.yaw_rate = std::isfinite(state.yaw_rate)
                       ? nfs2015_chase::Damp(state.yaw_rate, raw_rate, 15.0,
                                             (std::max)(dt, 0.0))
                       : raw_rate;
  return state.yaw_rate;
}

// Game-tick visual override. The game's own wheel animation is parked while the
// drift override is active; the front wheels get a scripted steer plus roll and
// the rear wheels get the matching roll. No physics field is written.
//
// Entry and exit are not binary. `AdvanceWheelSteerTarget` owns the NFS-style
// speed/slip gates, hold time and blend; this function owns the game pose and
// hands the wheel back to the stock animation only after the manual angle has
// converged to the stock-input estimate.
void UpdateWheelSteer(Context &context, const std::uintptr_t vehicle,
                      const nfs2015_chase::Vec3 world_velocity,
                      const double forward_speed_mps,
                      const double speed_mps, const double steering_input,
                      const bool steering_known, const bool reversing,
                      const double dt) noexcept {
  WheelSteerState &state = context.wheel_steer;
  const bool enabled =
      context.wheel_steer_enabled.load(std::memory_order_acquire);
  if (!enabled || vehicle == 0) {
    if (state.vehicle != 0 || state.active || state.wheel_count != 0 ||
        state.animation_disabled) {
      ReleaseWheelSteer(context);
    }
    return;
  }
  if (state.vehicle != vehicle) {
    ReleaseWheelSteer(context);
    state.vehicle = vehicle;
  }
  if (!state.resolved || context.elapsed >= state.next_resolve) {
    if (!ResolveWheelSteerGeometry(context)) {
      return;
    }
  }
  if (state.wheel_count == 0 || state.mesh == 0 || state.anim == 0) {
    if (state.animation_disabled) {
      RestoreWheelPose(context);
    }
    return;
  }
  nfs2015_chase::Quat mesh{};
  if (!Read(context.core, state.mesh + kSceneComponentTransformOffset, mesh)) {
    if (state.animation_disabled) {
      RestoreWheelPose(context);
    }
    state.resolved = false;
    state.next_resolve = context.elapsed + 1.0;
    return;
  }
  if (!std::isfinite(mesh.x) || !std::isfinite(mesh.y) ||
      !std::isfinite(mesh.z) || !std::isfinite(mesh.w)) {
    if (state.animation_disabled) {
      RestoreWheelPose(context);
    }
    state.resolved = false;
    state.next_resolve = context.elapsed + 1.0;
    return;
  }
  mesh = nfs2015_chase::QuatNormalize(mesh);
  const nfs2015_chase::Vec3 car_forward =
      nfs2015_chase::QuatRotate(mesh, {1.0, 0.0, 0.0});
  const nfs2015_chase::Vec3 car_up =
      nfs2015_chase::QuatRotate(mesh, {0.0, 0.0, 1.0});

  nfs2015_chase::WheelSteerConfig config;
  const double max_degrees = nfs2015_chase::Clamp(
      context.wheel_steer_max_degrees.load(std::memory_order_acquire),
      kMinimumWheelSteerMaxDegrees, kMaximumWheelSteerMaxDegrees);
  const double full_slip_degrees = nfs2015_chase::Clamp(
      context.wheel_steer_full_slip_degrees.load(std::memory_order_acquire),
      kMinimumWheelSteerFullSlipDegrees,
      kMaximumWheelSteerFullSlipDegrees);
  const double gain = nfs2015_chase::Clamp(
      context.wheel_steer_gain.load(std::memory_order_acquire),
      kMinimumWheelSteerGain, kMaximumWheelSteerGain);
  const double rate_degrees_per_second = nfs2015_chase::Clamp(
      context.wheel_steer_rate_degrees_per_second.load(
          std::memory_order_acquire),
      kMinimumWheelSteerRateDegreesPerSecond,
      kMaximumWheelSteerRateDegreesPerSecond);
  const double entry_speed_kmh = nfs2015_chase::Clamp(
      context.wheel_steer_entry_speed_kmh.load(std::memory_order_acquire),
      kMinimumWheelSteerEntrySpeedKmh, kMaximumWheelSteerEntrySpeedKmh);
  config.enabled = true;
  config.max_angle_radians =
      max_degrees * nfs2015_chase::kRadiansPerDegree;
  config.gain = gain;
  config.full_slip_radians =
      full_slip_degrees * nfs2015_chase::kRadiansPerDegree;
  config.enter_slip_radians =
      kWheelSteerEnterSlipDegrees * nfs2015_chase::kRadiansPerDegree;
  config.exit_slip_radians =
      kWheelSteerExitSlipDegrees * nfs2015_chase::kRadiansPerDegree;
  config.rate_radians_per_second =
      rate_degrees_per_second * nfs2015_chase::kRadiansPerDegree;
  config.entry_speed_mps = entry_speed_kmh / 3.6;
  config.hold_seconds = kWheelSteerHoldSeconds;
  config.exit_hold_seconds = kWheelSteerExitHoldSeconds;
  config.blend_in_seconds = kWheelSteerBlendInSeconds;
  config.blend_out_seconds = kWheelSteerBlendOutSeconds;
  config.stock_max_angle_radians =
      kWheelSteerStockMaxDegrees * nfs2015_chase::kRadiansPerDegree;
  config.counter_authority_radians =
      kWheelSteerCounterAuthorityDegrees * nfs2015_chase::kRadiansPerDegree;
  config.restore_tolerance_radians =
      kWheelSteerRestoreToleranceDegrees * nfs2015_chase::kRadiansPerDegree;
  config.turn_in_seconds = kWheelSteerTurnInSeconds;
  config.turn_in_min_seconds = kWheelSteerTurnInMinSeconds;
  config.turn_in_max_seconds = kWheelSteerTurnInMaxSeconds;
  config.turn_in_start_slip_radians =
      kWheelSteerTurnInStartSlipDegrees *
      nfs2015_chase::kRadiansPerDegree;
  config.turn_in_full_slip_radians =
      kWheelSteerTurnInFullSlipDegrees *
      nfs2015_chase::kRadiansPerDegree;
  config.turn_in_min_alpha = kWheelSteerMinimumTurnInAlpha;
  config.turn_in_rate_radians_per_second =
      kWheelSteerTurnInRateDegreesPerSecond *
      nfs2015_chase::kRadiansPerDegree;
  config.turn_in_max_rate_radians_per_second =
      kWheelSteerTurnInMaxRateDegreesPerSecond *
      nfs2015_chase::kRadiansPerDegree;
  config.turn_in_release_slip_radians =
      kWheelSteerTurnInReleaseSlipDegrees *
      nfs2015_chase::kRadiansPerDegree;
  config.reentry_cooldown_seconds = kWheelSteerReentryCooldownSeconds;
  config.standstill_speed_mps = kWheelSteerStandstillSpeedMps;
  config.standstill_grace_seconds = kWheelSteerStandstillGraceSeconds;

  // The body's own forward axis is used for the entry direction and for the
  // visual wheel roll. Reverse itself comes from the vehicle gear signal; a
  // lateral slide can legitimately have a negative forward component.
  const double mesh_forward_speed = nfs2015_chase::Dot(
      world_velocity, nfs2015_chase::Normalize(car_forward));
  const double wheel_forward_speed =
      std::isfinite(mesh_forward_speed) ? mesh_forward_speed
                                        : forward_speed_mps;
  state.front_travel_angle = 0.0;
  state.rear_travel_angle = 0.0;
  state.front_axle_offset = {};
  state.rear_axle_offset = {};
  state.axle_offsets_valid =
      ReadWheelAxleOffsets(context, state, state.front_axle_offset,
                           state.rear_axle_offset);
  if (!state.axle_offsets_valid) {
    state.front_axle_offset = {};
    state.rear_axle_offset = {};
  }
  state.yaw_rate = EstimateMeshYawRate(state, mesh, car_up,
                                       context.yaw_rate, dt);
  const nfs2015_chase::WheelSteerTravel travel =
      nfs2015_chase::ComputeWheelSteerTravel(
          car_forward, car_up, world_velocity, state.yaw_rate,
          state.front_axle_offset, state.rear_axle_offset);
  state.front_travel_angle = travel.front_angle_radians;
  state.rear_travel_angle = travel.rear_angle_radians;
  const nfs2015_chase::WheelSteerPhase previous_phase = state.phase;
  nfs2015_chase::WheelSteerTarget target =
      nfs2015_chase::AdvanceWheelSteerTarget(
          state.drift, travel.front_angle_radians, travel.rear_angle_radians,
          speed_mps, wheel_forward_speed, steering_input, steering_known,
          reversing, dt, config);

  // A new drift takes over from the stock pose at the stock angle. Starting
  // from zero is what produced the old "wheel snaps straight, then turns"
  // entry jump. Read the real wheel heading while the stock animation is still
  // running and keep it only as the visual starting pose; the state machine's
  // raw stock target remains the sweep target. Overwriting the target with the
  // measured pose is what used to freeze turn_in at a few degrees while NTE's
  // stock animation was still ramping up.
  if (target.active && !state.animation_disabled) {
    double measured_stock = 0.0;
    const bool measured = MeasureStockWheelSteer(
        context, state, car_forward, car_up, measured_stock);
    state.entry_stock_angle = nfs2015_chase::Clamp(
        measured ? measured_stock : target.stock_angle_radians,
        -config.max_angle_radians, config.max_angle_radians);
    state.entry_stock_valid = true;
    state.applied_angle = state.entry_stock_angle;
  }

  // Compose the final visual target. The entry stock pose is only the start;
  // turn_in_alpha decides how much of the raw stock angle the turn-in phase
  // should show. The release phase deliberately returns to the entry pose so
  // the handoff to the stock animation does not jump to the raw steering angle.
  const double entry_stock = state.entry_stock_valid
      ? state.entry_stock_angle
      : target.stock_angle_radians;
  if (state.entry_stock_valid) {
    target.angle_radians = nfs2015_chase::ComposeWheelSteerTargetAngle(
        entry_stock, target);
    state.stock_angle = nfs2015_chase::ComposeWheelSteerStockAngle(
        entry_stock, target);
  } else {
    state.stock_angle = target.stock_angle_radians;
  }

  state.target_angle = target.angle_radians;
  state.target_blend = target.blend;
  state.engaged = target.engaged;
  state.phase = target.phase;

  if (target.phase != previous_phase) {
    char message[256]{};
    std::snprintf(
        message, sizeof(message),
        "nfs2015 chase cam: wheel drift %s -> %s speed=%.2f "
        "steer=%.2f front=%.1f rear=%.1f yaw=%.1f stock=%.1f "
        "counter=%.1f blend=%.2f",
        WheelSteerPhaseName(previous_phase), WheelSteerPhaseName(target.phase),
        speed_mps, steering_known ? steering_input : 0.0,
        state.front_travel_angle * kRadiansToDegrees,
        state.rear_travel_angle * kRadiansToDegrees,
        state.yaw_rate * kRadiansToDegrees,
        state.stock_angle * kRadiansToDegrees,
        target.counter_angle_radians * kRadiansToDegrees, target.blend);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }

  if (!target.active && !state.animation_disabled) {
    return;
  }

  if (!EnsureWheelAnimationDisabled(context)) {
    RestoreWheelPose(context);
    return;
  }
  if (state.settle_ticks > 0) {
    --state.settle_ticks;
    if (state.settle_ticks > 0) {
      return;
    }
  }

  // turn_in is a visible sweep, not a hold. It uses its own rate and is
  // guaranteed to finish before the earliest possible handoff; countersteer
  // keeps the existing user-configurable rate.
  state.applied_angle = nfs2015_chase::AdvanceWheelSteerAppliedAngle(
      state.applied_angle, target, dt, config);

  // The release blend has reached zero, but the manual wheel may still be
  // travelling toward the stock angle. Keep the animation parked until that
  // handoff is complete; restoring earlier is the exit snap.
  if (!target.active && target.phase == nfs2015_chase::WheelSteerPhase::grip &&
      std::fabs(state.applied_angle - target.angle_radians) <=
          config.restore_tolerance_radians) {
    RestoreWheelPose(context);
    return;
  }

  bool have_all_front_sockets = true;
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    if (state.front[index] && !state.socket_captured[index]) {
      have_all_front_sockets = false;
      break;
    }
  }
  if (!have_all_front_sockets && !CaptureWheelSockets(context)) {
    RestoreWheelPose(context);
    return;
  }

  if (std::isfinite(wheel_forward_speed) && std::isfinite(dt)) {
    state.roll_angle -=
        wheel_forward_speed / kWheelVisualRollRadiusMeters * dt;
    state.roll_angle =
        std::fmod(state.roll_angle, 2.0 * nfs2015_chase::kPi);
  }
  const nfs2015_chase::Quat roll_rotation =
      nfs2015_chase::ComputeVisualWheelRollRotation(state.roll_angle);
  const nfs2015_chase::Vec3 target_forward = nfs2015_chase::QuatRotate(
      nfs2015_chase::AxisAngle(nfs2015_chase::Normalize(car_up),
                               state.applied_angle),
      car_forward);
  bool wrote_any = false;
  for (std::size_t index = 0; index < state.wheel_count; ++index) {
    const std::uintptr_t wheel = state.wheels[index];
    if (wheel == 0) {
      continue;
    }
    nfs2015_chase::Quat relative{};
    if (state.front[index]) {
      if (!state.socket_captured[index]) {
        continue;
      }
      const nfs2015_chase::Quat socket_world = nfs2015_chase::QuatNormalize(
          nfs2015_chase::QuatMul(mesh, state.socket_local[index]));
      if (!nfs2015_chase::ComputeVisualWheelRelativeRotation(
              car_forward, car_up, target_forward, socket_world,
              state.roll_angle, relative)) {
        continue;
      }
    } else {
      relative = roll_rotation;
    }
    double new_rotator[3]{};
    nfs2015_chase::RotatorDegreesFromQuat(relative, new_rotator);
    if (Write(context.core,
              wheel + kSceneComponentRelativeRotationOffset, new_rotator)) {
      wrote_any = true;
    }
  }
  if (wrote_any) {
    if (!state.activation_logged) {
      state.activation_logged = true;
      char message[320]{};
      std::snprintf(
          message, sizeof(message),
          "nfs2015 chase cam: visual wheel override active "
          "(front=%zu rear=%zu phase=%s target=%.1f deg applied=%.1f deg "
          "stock=%.1f deg counter=%.1f deg blend=%.2f front_slip=%.1f "
          "rear_slip=%.1f offsets=%d max=%.1f)",
          state.front_count, state.rear_count,
          WheelSteerPhaseName(target.phase),
          state.target_angle * kRadiansToDegrees,
          state.applied_angle * kRadiansToDegrees,
          state.stock_angle * kRadiansToDegrees,
          target.counter_angle_radians * kRadiansToDegrees, target.blend,
          state.front_travel_angle * kRadiansToDegrees,
          state.rear_travel_angle * kRadiansToDegrees,
          state.axle_offsets_valid ? 1 : 0, max_degrees);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    }
    state.active = true;
  } else {
    RestoreWheelPose(context);
    state.resolved = false;
    state.next_resolve = context.elapsed + 0.5;
  }
}

}  // namespace nfs2015_chase_cam::detail
