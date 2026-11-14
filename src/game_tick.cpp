// Per-frame game-side model driver: read the player snapshot, integrate the
// chase-camera kinematics from the game's own camera yaw, and publish the
// snapshot the detour will serve.
#include "internal.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace nfs2015_chase_cam::detail {

// Q2 answer (PLAN-camera-timing.md): the game-camera pose the entity frame
// once exposed is already in the player ESP snapshot, which is a cached copy
// the plugin reads every tick anyway (`camera_position` / `camera_rotation`).
// There is no need to lease the entity service just to observe it.

// ---------------------------------------------------------------------------
// Game tick
// ---------------------------------------------------------------------------

bool QueryPlayer(Context &context, double origin_cm[3], double base_cm[3],
                 std::uint64_t &sequence,
                 AnomalyGenerationHandleV1 &handle) noexcept {
  sequence = 0;
  handle = {};
  context.camera_yaw_valid = false;
  context.camera_yaw_radians = 0.0;
  context.probe_bounds_dx_cm = 0.0;
  context.probe_bounds_dy_cm = 0.0;
  if (!PlayerReady(context.player)) {
    return false;
  }
  AnomalyNtePlayerSnapshotV1 player{sizeof(player)};
  if (context.player->snapshot(context.player->user, &player).code !=
          ANOMALY_STATUS_V1_OK ||
      !SnapshotValid(player.flags)) {
    return false;
  }
  sequence = player.sequence;
  handle = player.handle;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    base_cm[axis] = player.position[axis];
  }
  double extent_z = kFallbackPlayerCenterHeightCm;
  AnomalyNtePlayerEspSnapshotV1 esp{sizeof(esp)};
  const bool has_esp =
      HasField<AnomalyNtePlayerServiceV1,
               decltype(AnomalyNtePlayerServiceV1::esp_snapshot)>(
          context.player,
          offsetof(AnomalyNtePlayerServiceV1, esp_snapshot)) &&
      context.player->esp_snapshot != nullptr;
  if (has_esp &&
      context.player->esp_snapshot(context.player->user, &esp).code ==
          ANOMALY_STATUS_V1_OK) {
    // PARTIAL means the optional ESP tail is missing. Its bounds fields are
    // not a trustworthy origin, even when they happen to be finite. Also
    // reject a stale ESP sample that does not describe the current pawn.
    const bool complete =
        SnapshotValid(esp.flags) &&
        (esp.flags & ANOMALY_NTE_SNAPSHOT_V1_PARTIAL) == 0;
    const bool finite =
        std::isfinite(esp.bounds_center[0]) &&
        std::isfinite(esp.bounds_center[1]) &&
        std::isfinite(esp.bounds_center[2]) &&
        std::isfinite(esp.bounds_extent[2]);
    const double dx = esp.bounds_center[0] - player.position[0];
    const double dy = esp.bounds_center[1] - player.position[1];
    const double dz = esp.bounds_center[2] - player.position[2];
    // Probe input: the horizontal lever arm of the bounds centre in the car's
    // own frame. Kept even when the bounds are not plausible enough to use as
    // an origin, because the probe is asking whether the direction of this
    // vector is stable, not whether its length is right.
    if (complete && finite) {
      context.probe_bounds_dx_cm = dx;
      context.probe_bounds_dy_cm = dy;
    }
    const bool plausible =
        complete && finite && dx * dx + dy * dy < 500.0 * 500.0 &&
        std::fabs(dz) < 500.0 && esp.bounds_extent[2] > 0.5F &&
        esp.bounds_extent[2] < 1000.0F;
    if (plausible) {
      extent_z = esp.bounds_extent[2];
      // The probe measures how much this radius moves tick to tick. It is the
      // only mesh-bound quantity that still reaches the eye.
      context.probe_extent_z_cm = static_cast<double>(extent_z);
    }
    // T3.1: the game camera's yaw, sampled by the host on the same Game tick
    // as the car position and delivered as a cached copy. The detour's
    // observation is the same rotation read on the render thread at ~1400 Hz,
    // which the 50 Hz Game tick then samples, so it aliases. Independent of
    // the bounds sanity check above: the camera tail is usable even when the
    // bounds jump.
    if (complete && std::isfinite(esp.camera_rotation[1])) {
      context.camera_yaw_radians =
          esp.camera_rotation[1] * kDegreesToRadians;
      context.camera_yaw_valid = true;
    }
  }
  // The camera model is anchored to the rigid Pawn root. The mesh bounds
  // centre and extent move with body roll and pitch; anchoring the camera to
  // them made the eye inherit that motion 1:1, which reads as twitch. Keep the
  // bounds fields only for the probe, never for the model origin.
  // `base_cm` is the body centre reported by the anchor; the chase model's
  // local rest is ground-relative, so the model origin is always the ground.
  origin_cm[0] = base_cm[0];
  origin_cm[1] = base_cm[1];
  origin_cm[2] = base_cm[2] - kVehicleAnchorCenterHeightCm;
  // The frame callback projects this. The Pawn root, so the measurement is of
  // what the game actually moved, with no mesh bound in it.
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.probe_car_cm[axis].store(player.position[axis],
                                     std::memory_order_relaxed);
  }
  context.probe_car_valid.store(std::isfinite(player.position[0]) &&
                                    std::isfinite(player.position[1]) &&
                                    std::isfinite(player.position[2]),
                                std::memory_order_relaxed);
  return std::isfinite(origin_cm[0]) && std::isfinite(origin_cm[1]) &&
         std::isfinite(origin_cm[2]) && std::isfinite(base_cm[0]) &&
         std::isfinite(base_cm[1]) && std::isfinite(base_cm[2]);
}

double WrappedAngle(const double angle) noexcept {
  return nfs2015_chase::WrapPi(angle);
}

// A world change, teleport, or model reset must not carry the old slope's
// pitch into the new frame. The floor also rejects a still-fresh snapshot that
// was published before the reset.
void ResetRenderPitch(Context &context) noexcept {
  context.render_pitch_valid = false;
  context.render_pitch_radians = 0.0;
  context.render_pitch_ns = 0;
  context.render_pitch_floor_ns = NowNanoseconds();
  context.render_pitch_proxy = 0;
}

bool WorldChanged(Context &context) noexcept {
  if (context.session == nullptr || context.session->snapshot == nullptr) {
    return false;
  }
  AnomalyNteSessionSnapshotV1 snapshot{sizeof(snapshot)};
  if (context.session->snapshot(context.session->user, &snapshot).code !=
          ANOMALY_STATUS_V1_OK ||
      snapshot.state != ANOMALY_NTE_SESSION_V1_WORLD_READY ||
      snapshot.world.id == 0 || snapshot.world.generation == 0) {
    return false;
  }
  if (context.world.id == 0) {
    context.world = snapshot.world;
    return true;
  }
  if (context.world.id != snapshot.world.id ||
      context.world.generation != snapshot.world.generation) {
    context.world = snapshot.world;
    return true;
  }
  return false;
}

void ResetMotion(Context &context) noexcept {
  NoteViewSnap(context, kSnapReasonMotion);
  context.yaw_source = kYawSourceNone;
  context.have_last_position = false;
  context.have_last_yaw = false;
  context.reversing = false;
  context.speed_sign_known = true;
  context.last_vehicle_parent = 0;
  context.last_vehicle_speed_mps = -1.0;
  context.last_vehicle_speed_time = 0.0;
  context.last_speed_exact = false;
  context.brake_deceleration_mps2 = 0.0;
  context.brake_input = 0.0;
  context.brake_hold_until = 0.0;
  context.vehicle_acceleration_mps2 = 0.0;
  context.last_player_sequence = 0;
  context.last_player_time = context.elapsed;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.velocity_cmps[axis] = 0.0;
    context.acceleration_cmps2[axis] = 0.0;
  }
  context.yaw_rate = 0.0;
  RequestPresenterReset(context);
}

void UpdateBrakeInput(Context &context, const bool speed_valid,
                      const bool speed_fresh, const bool derived,
                      const double speed_kmh, const double dt) noexcept {
  const double input_smoothing =
      derived ? kDerivedBrakingInputSmoothing : kBrakingInputSmoothing;
  const double deceleration_smoothing =
      derived ? kDerivedBrakingDecelerationSmoothing
              : kBrakingDecelerationSmoothing;
  if (!speed_valid) {
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
        context.vehicle_acceleration_mps2, 0.0, deceleration_smoothing, dt);
    return;
  }
  if (!speed_fresh) {
    // The motion sample is held between host snapshots, so there is no new
    // speed delta to differentiate. Differentiating it at tick rate would read
    // as zero acceleration on the held ticks and a spike on the sampled one,
    // i.e. a sawtooth in the braking pitch.
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input,
        context.elapsed <= context.brake_hold_until
            ? context.brake_last_target
            : 0.0,
        input_smoothing, dt);
    return;
  }
  // Braking pitch follows speed magnitude: reversing decelerates too, and the
  // camera model already receives the signed speed separately.
  const double speed_mps = std::fabs(speed_kmh) / 3.6;
  if (context.last_vehicle_speed_mps < 0.0) {
    context.last_vehicle_speed_mps = speed_mps;
    context.last_vehicle_speed_time = context.elapsed;
    return;
  }
  const double elapsed = context.elapsed - context.last_vehicle_speed_time;
  const double previous_mps = context.last_vehicle_speed_mps;
  context.last_vehicle_speed_mps = speed_mps;
  context.last_vehicle_speed_time = context.elapsed;
  if (elapsed <= 1e-4) {
    return;
  }
  if (derived && elapsed > kBrakingMaxSampleIntervalSeconds) {
    // Only the differentiated position is invalid across a hitch; the exact
    // speed getter is a direct sample and can span a longer poll gap.
    context.brake_deceleration_mps2 = 0.0;
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    return;
  }
  const double raw_acceleration = (speed_mps - previous_mps) / elapsed;
  context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
      context.vehicle_acceleration_mps2,
      nfs2015_chase::Clamp(raw_acceleration, -30.0, 30.0),
      deceleration_smoothing, elapsed);
  const double deceleration =
      nfs2015_chase::Clamp(-raw_acceleration, 0.0, 30.0);
  context.brake_deceleration_mps2 = nfs2015_chase::Damp(
      context.brake_deceleration_mps2, deceleration,
      deceleration_smoothing, elapsed);
  const double deadzone =
      derived ? kDerivedBrakingPitchDeadzoneMps2
              : kBrakingPitchDeadzoneMps2;
  const double excess =
      (std::max)(0.0, context.brake_deceleration_mps2 - deadzone);
  const double target = nfs2015_chase::Clamp(
      excess / (kBrakingPitchFullMps2 - deadzone), 0.0, 1.0);
  if (target > 0.05) {
    context.brake_hold_until = context.elapsed + kBrakingHoldSeconds;
  }
  const double gated =
      context.elapsed <= context.brake_hold_until ? target : 0.0;
  context.brake_last_target = gated;
  context.brake_input = nfs2015_chase::Damp(
      context.brake_input, gated, input_smoothing, dt);
}

void UpdateMotion(Context &context, const double origin_cm[3],
                  const std::uint64_t sequence) noexcept {
  if (!context.have_last_position || sequence != context.last_player_sequence) {
    // Q5: does the player sequence advance once per tick? A jump means the
    // position derivative below is divided by the wrong elapsed time.
    if (context.last_player_sequence != 0 &&
        sequence != context.last_player_sequence &&
        sequence != context.last_player_sequence + 1U) {
      context.diag.player_sequence_jumps.fetch_add(1,
                                                  std::memory_order_relaxed);
    }
    if (context.have_last_position && sequence != context.last_player_sequence) {
      const double elapsed = context.elapsed - context.last_player_time;
      if (elapsed > 1e-4) {
        std::array<double, 3> raw{};
        double distance_squared = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          raw[axis] = (origin_cm[axis] - context.last_position_cm[axis]) /
                      elapsed;
          distance_squared +=
              (origin_cm[axis] - context.last_position_cm[axis]) *
              (origin_cm[axis] - context.last_position_cm[axis]);
        }
        const double raw_speed = std::hypot(raw[0], raw[1]);
        if (distance_squared < kMaximumTeleportStepCm * kMaximumTeleportStepCm &&
            std::isfinite(raw_speed) &&
            raw_speed < kMaximumPlausibleSpeedCmps) {
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double previous = context.velocity_cmps[axis];
            context.velocity_cmps[axis] = nfs2015_chase::Damp(
                context.velocity_cmps[axis], raw[axis], kVelocitySmoothing,
                elapsed);
            context.acceleration_cmps2[axis] = nfs2015_chase::Damp(
                context.acceleration_cmps2[axis],
                (context.velocity_cmps[axis] - previous) / elapsed,
                kAccelerationSmoothing, elapsed);
          }
        } else {
          NoteViewSnap(context, kSnapReasonTeleport);
          ResetRenderPitch(context);
          // A teleport is not a brake. Clear the derived-speed history so the
          // next sample cannot turn the jump to zero into a full pitch input.
          context.last_vehicle_speed_mps = -1.0;
          context.last_vehicle_speed_time = context.elapsed;
          context.brake_deceleration_mps2 = 0.0;
          context.brake_input = 0.0;
          context.brake_hold_until = 0.0;
          context.brake_last_target = 0.0;
          context.vehicle_acceleration_mps2 = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            context.velocity_cmps[axis] = 0.0;
            context.acceleration_cmps2[axis] = 0.0;
          }
        }
      }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      context.last_position_cm[axis] = origin_cm[axis];
    }
    context.last_player_sequence = sequence;
    context.last_player_time = context.elapsed;
    context.have_last_position = true;
  }

  const double acceleration_length =
      std::sqrt(context.acceleration_cmps2[0] *
                    context.acceleration_cmps2[0] +
                context.acceleration_cmps2[1] *
                    context.acceleration_cmps2[1] +
                context.acceleration_cmps2[2] *
                    context.acceleration_cmps2[2]);
  if (acceleration_length > kMaximumPlausibleAccelerationCmps2) {
    const double scale = kMaximumPlausibleAccelerationCmps2 / acceleration_length;
    for (double &value : context.acceleration_cmps2) {
      value *= scale;
    }
  }
}

struct ObservedPose final {
  std::array<double, 3> location{};
  std::array<double, 3> rotation{};
};

// Try-read the detour's copy of the game camera pose. The detour writes it on
// another thread; a torn or in-progress write is reported as unavailable
// instead of spinning the Game thread.
[[nodiscard]] bool ReadObservedPose(Context &context,
                                    ObservedPose &pose) noexcept {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t before =
        context.observed_sequence.load(std::memory_order_acquire);
    if ((before & 1U) != 0) {
      continue;
    }
    if (!context.observed_valid.load(std::memory_order_relaxed)) {
      return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      pose.location[axis] =
          context.observed_location[axis].load(std::memory_order_relaxed);
      pose.rotation[axis] =
          context.observed_rotation[axis].load(std::memory_order_relaxed);
    }
    const std::uint64_t after =
        context.observed_sequence.load(std::memory_order_acquire);
    if (before == after) {
      return true;
    }
  }
  context.diag.observed_read_failures.fetch_add(1,
                                                std::memory_order_relaxed);
  return false;
}

double SelectCarYaw(Context &context, const double velocity_yaw,
                    const double velocity_mps, const bool have_velocity,
                    const bool exact_speed,
                    const double signed_speed_kmh) noexcept {
  // Prefer the Game-tick camera yaw: same clock as the car position, so the
  // reference cannot alias against a faster writer. The detour's render-thread
  // observation is only a fallback for profiles without the ESP tail.
  double camera_yaw = 0.0;
  bool have_camera_yaw = false;
  if (context.camera_yaw_valid) {
    camera_yaw = context.camera_yaw_radians;
    have_camera_yaw = true;
  } else {
    ObservedPose observed{};
    if (ReadObservedPose(context, observed)) {
      const double raw = observed.rotation[1];
      if (std::isfinite(raw)) {
        camera_yaw = raw * kDegreesToRadians;
        have_camera_yaw = true;
      }
    }
    context.diag.camera_yaw_esp_misses.fetch_add(1,
                                                std::memory_order_relaxed);
  }
  context.speed_sign_known = exact_speed || have_camera_yaw;
  if (exact_speed) {
    // GetForwardSpeedKmH is signed, so reverse is stateless again exactly as
    // it was in 2.6. The derived-speed hysteresis is only the fallback.
    context.reversing = signed_speed_kmh < -kReverseSpeedKmh;
  }
  if (!have_camera_yaw) {
    NoteYawSource(context, kYawSourceNoObservation);
    if (!have_velocity) {
      return context.have_last_yaw ? context.last_car_yaw : 0.0;
    }
    // Without a camera reference the sign is unknown. Reuse the last known
    // reverse state rather than flipping the camera to the nose in one tick.
    return context.reversing
               ? WrappedAngle(velocity_yaw + nfs2015_chase::kPi)
               : velocity_yaw;
  }
  if (!have_velocity) {
    // Too slow for the travel direction to mean anything: the camera is the
    // only continuous reference left.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  // Reversing travels along the nose plus 180 degrees, so the camera stays
  // aligned with the nose instead of ending up in front of the car. When the
  // exact signed speed is unavailable, infer the sign from the angle between
  // travel and the game-camera yaw. The two thresholds form a hysteresis band
  // so a drift or a camera swing cannot chatter the state; below walking speed
  // the last decision is held.
  const double alignment =
      std::cos(WrappedAngle(velocity_yaw - camera_yaw));
  if (!exact_speed && velocity_mps >= kReverseDecisionMinSpeedMps) {
    if (context.reversing) {
      if (alignment > kReverseExitCosine) {
        context.reversing = false;
      }
    } else if (alignment < kReverseEnterCosine) {
      context.reversing = true;
    }
  }
  const bool reversing = context.reversing;
  if (!reversing && velocity_mps < kReverseDecisionMinSpeedMps &&
      alignment < kReverseEnterCosine) {
    // Below the reverse decision speed the travel direction is too noisy to
    // commit to a gear. The camera still points along the nose, so keep it
    // instead of flipping the rig to the noisy velocity vector.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  const double travel_yaw =
      reversing ? WrappedAngle(velocity_yaw + nfs2015_chase::kPi)
                : velocity_yaw;
  if (std::fabs(WrappedAngle(camera_yaw - travel_yaw)) >
      kMaximumCameraYawDifference) {
    NoteYawSource(context, kYawSourceVelocity);
    return travel_yaw;
  }
  NoteYawSource(context, kYawSourceObserved);
  return camera_yaw;
}

void PublishInactive(Context &context) noexcept {
  const std::uint64_t sequence =
      context.view_sequence.load(std::memory_order_relaxed);
  context.view_sequence.store(sequence + 1U, std::memory_order_release);
  context.view_active.store(false, std::memory_order_release);
  context.view_sequence.store(sequence + 2U, std::memory_order_release);
  // Re-entering the camera must not blend from the view that was active
  // before the player stopped driving.
  NoteViewSnap(context, kSnapReasonInactive);
  if (context.last_view_active && context.last_fov_enabled) {
    context.fov_restore.store(true, std::memory_order_release);
  }
  context.last_view_active = false;
  context.last_fov_enabled = false;
}

void ActivateView(Context &context, const bool fov_enabled) noexcept {
  context.last_view_active = true;
  context.last_fov_enabled = fov_enabled;
  context.view_active.store(true, std::memory_order_release);
}

[[nodiscard]] bool ViewHistoryJumped(
    const std::array<double, 3> &old_location,
    const std::array<double, 3> &old_rotation,
    const std::array<double, 3> &new_location,
    const std::array<double, 3> &new_rotation) noexcept {
  const double dx = new_location[0] - old_location[0];
  const double dy = new_location[1] - old_location[1];
  const double dz = new_location[2] - old_location[2];
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      dx * dx + dy * dy + dz * dz >
          kViewHistorySnapDistanceCm * kViewHistorySnapDistanceCm) {
    return true;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(old_rotation[axis]) ||
        !std::isfinite(new_rotation[axis])) {
      return true;
    }
    const double delta = WrappedAngle(
        (new_rotation[axis] - old_rotation[axis]) * kDegreesToRadians);
    if (std::fabs(delta) >
        kViewHistorySnapAngleDegrees * kDegreesToRadians) {
      return true;
    }
  }
  return false;
}

}  // namespace nfs2015_chase_cam::detail
