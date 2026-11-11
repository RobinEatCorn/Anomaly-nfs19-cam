// NFS 2015 Chase Cam — third-party Anomaly plugin (ABI v1).
//
// This package is built against the installed Anomaly SDK only: it includes no
// host-internal headers and carries no JSON library of its own. Settings are
// persisted through `anomaly.config` and decoded through `anomaly.json`.
//
// Feeds the clean-room NFS 2015 chase-camera model in `chase_cam.hpp` from
// Anomaly's NTE snapshots, then replaces the view point produced by the game's
// camera manager. The game's own third-person camera is used as the car-yaw
// sensor; the player snapshot supplies the car position. Anomaly 2.7.0 removed
// `anomaly.nte.vehicle`, so vehicle presence is recovered by calling the same
// reflected getters that service used, through `ue5.objects` +
// `ue5.process-event`; the player capsule's AttachParent remains the anchor and
// the fallback. Speed comes from differentiating the player position and is
// signed against the resolved car yaw, with hysteresis for reverse. No GObjects
// scan.
//
// The `Probe` block below is the only code that reaches into the engine: it
// resolves three reflected functions, takes the Pawn object handle and calls
// `K2_GetActorRotation` from inside the once-per-frame AHUD callback. It is
// read-only, it never feeds the camera, and every result is shown in the
// diagnostics panel. See PLAN-camera-timing.md §8 P0.
//
// Thread domains:
//   on_update = Game. Reads NTE services, integrates the camera, publishes an
//               immutable view snapshot and persists settings.
//   detour    = whatever thread asks the camera manager for its view point.
//               It only calls the original, copies the published snapshot to
//               the two out-parameters, and writes the optional lens.
//   on_draw   = Render. Copies the display snapshot under a short lock and
//               calls UI outside it.

#include "localization.hpp"

#include "anomaly/sdk/cpp.hpp"
#include "anomaly/sdk/services/core.h"
#include "anomaly/sdk/services/interop.h"
#include "anomaly/sdk/services/json.h"
#include "anomaly/sdk/services/nte.h"
#include "anomaly/sdk/services/ui.h"
#include "anomaly/sdk/services/ui_resources.h"
#include "anomaly/sdk/services/ue5.h"

#include "chase_cam.hpp"
#include "wheel_steer.hpp"
#include "latency_window.hpp"
#include "car_sample.hpp"
#include "burnout_shot.hpp"
#include "view_blend.hpp"

#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <thread>
#include <system_error>

#include "src/constants.hpp"
#include "src/internal.hpp"
#include "src/sdk.hpp"
#include "src/state.hpp"

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
  const double origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
  // The camera model is anchored to the rigid Pawn root. The mesh bounds
  // centre and extent move with body roll and pitch; anchoring the camera to
  // them made the eye inherit that motion 1:1, which reads as twitch. Keep the
  // bounds fields only for the probe, never for the model origin.
  origin_cm[0] = base_cm[0];
  origin_cm[1] = base_cm[1];
  origin_cm[2] = base_cm[2] + origin_offset_cm;
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

// ---- T2 presenter -----------------------------------------------------------
//
// The model used to be advanced on the Game tick and then blended on the render
// thread. That cannot be made smooth: blending only interpolates between two
// tick poses, so the camera's *velocity* still jumps at every tick boundary.
// Measured against the car, the eye sat one tick of travel behind and caught up
// in bursts (655 m/s against a 670 mm tick step).
//
// Now the Game tick only publishes samples and the render thread advances the
// springs, so velocity is continuous by construction.

bool TryLockPresenter(Context &context) noexcept {
  return !context.presenter_lock.test_and_set(std::memory_order_acquire);
}

void UnlockPresenter(Context &context) noexcept {
  context.presenter_lock.clear(std::memory_order_release);
}

// The Game tick may block briefly: the render side only holds the lock for a
// handful of spring steps.
void LockPresenter(Context &context) noexcept {
  while (!TryLockPresenter(context)) {
    std::this_thread::yield();
  }
}

// Ask the render side to restart the model. The epoch travels with the samples,
// so the renderer notices even if it was not running at the time.
void RequestPresenterReset(Context &context) noexcept {
  ResetRenderPitch(context);
  context.chase_initialized = false;
  ++context.sample_epoch;
}

// The model owns its state; other render threads must read a consistent pose.
void PublishPresenterPose(Context &context, const double location[3],
                          const double rotation[3], const double car_cm[3],
                          const float lens, const bool fov_enabled) noexcept {
  const std::uint64_t sequence =
      context.presenter_pose_sequence.load(std::memory_order_relaxed);
  context.presenter_pose_sequence.store(sequence + 1U,
                                        std::memory_order_release);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.presenter_pose_location[axis].store(location[axis],
                                                std::memory_order_relaxed);
    context.presenter_pose_rotation[axis].store(rotation[axis],
                                               std::memory_order_relaxed);
    context.presenter_car_cm[axis].store(car_cm[axis],
                                        std::memory_order_relaxed);
  }
  context.presenter_pose_lens.store(lens, std::memory_order_relaxed);
  context.presenter_pose_fov.store(fov_enabled, std::memory_order_relaxed);
  context.presenter_pose_valid.store(true, std::memory_order_relaxed);
  context.presenter_pose_sequence.store(sequence + 2U,
                                        std::memory_order_release);
}

[[nodiscard]] bool ReadPresenterPose(const Context &context, double location[3],
                                     double rotation[3], float *lens,
                                     bool *fov_enabled,
                                     double car_cm[3]) noexcept {
  for (int attempt = 0; attempt < 8; ++attempt) {
    const std::uint64_t before =
        context.presenter_pose_sequence.load(std::memory_order_acquire);
    if ((before & 1U) != 0U) {
      continue;
    }
    if (!context.presenter_pose_valid.load(std::memory_order_relaxed)) {
      return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      location[axis] = context.presenter_pose_location[axis].load(
          std::memory_order_relaxed);
      rotation[axis] = context.presenter_pose_rotation[axis].load(
          std::memory_order_relaxed);
      if (car_cm != nullptr) {
        car_cm[axis] = context.presenter_car_cm[axis].load(
            std::memory_order_relaxed);
      }
    }
    *lens = context.presenter_pose_lens.load(std::memory_order_relaxed);
    *fov_enabled =
        context.presenter_pose_fov.load(std::memory_order_relaxed);
    if (context.presenter_pose_sequence.load(std::memory_order_relaxed) ==
        before) {
      return true;
    }
  }
  return false;
}

void PublishRouteBModelPose(Context &context,
                            const nfs2015_chase::CarSample &sample,
                            const nfs2015_chase::CameraState &view,
                            const std::int64_t now_ns) noexcept {
  RenderXProbe &probe = context.render_x;
  probe.route_b_model_sequence.fetch_add(1, std::memory_order_acq_rel);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double model_car =
        axis == 0 ? sample.car.position.x
                  : (axis == 1 ? sample.car.position.y
                               : sample.car.position.z);
    const double model_eye =
        axis == 0 ? view.position.x
                  : (axis == 1 ? view.position.y : view.position.z);
    const double model_rotation =
        axis == 0 ? view.pitch_degrees
                  : (axis == 1 ? view.yaw_degrees : view.roll_degrees);
    probe.route_b_model_car_cm[axis].store(model_car * 100.0,
                                           std::memory_order_relaxed);
    probe.route_b_model_eye_cm[axis].store(model_eye * 100.0,
                                           std::memory_order_relaxed);
    probe.route_b_model_rotation[axis].store(model_rotation,
                                             std::memory_order_relaxed);
  }
  probe.route_b_model_camera_scale.store(
      nfs2015_chase::Clamp(sample.camera_scale, kMinimumCameraScale,
                           kMaximumCameraScale),
      std::memory_order_relaxed);
  probe.route_b_model_origin_offset_cm.store(
      context.origin_offset_cm.load(std::memory_order_acquire),
      std::memory_order_relaxed);
  probe.route_b_model_lens.store(
      sample.fov_enabled
          ? static_cast<float>(
                view.HorizontalFovDegrees(sample.aspect_ratio))
          : 0.0F,
      std::memory_order_relaxed);
  probe.route_b_model_ns.store(now_ns, std::memory_order_relaxed);
  probe.route_b_model_valid.store(true, std::memory_order_relaxed);
  probe.route_b_model_sequence.fetch_add(1, std::memory_order_release);
}

// Advance the model to `now_ns` and produce the pose for this render call.
// Returns false when there is nothing to present (no sample yet, or another
// thread is advancing and no cached pose exists).
bool AdvancePresenter(Context &context, const std::int64_t now_ns,
                      const double *game_eye_cm, double location[3],
                      double rotation[3], float *lens, bool *fov_enabled,
                      bool *advanced) noexcept {
  *advanced = false;
  if (!TryLockPresenter(context)) {
    return ReadPresenterPose(context, location, rotation, lens, fov_enabled);
  }
  struct Unlock final {
    Context &context;
    ~Unlock() { UnlockPresenter(context); }
  } unlock{context};

  // The detour runs once per render pass, and one frame has many passes. If
  // every pass advanced the model, the model time would be the pass-to-pass
  // interval (a noisy fraction of a millisecond), not the frame interval.
  // Advance once per frame and hand every later pass of that frame the cached
  // pose. The threshold is smaller than the shortest frame at 60 fps
  // (16.7 ms) and larger than the longest observed within-frame pass gap.
  if (context.presenter_pose_valid.load(std::memory_order_relaxed)) {
    const std::int64_t since = now_ns - context.presenter_pose_ns;
    if (since >= -kPresenterSameFrameNs && since < kPresenterSameFrameNs) {
      return ReadPresenterPose(context, location, rotation, lens, fov_enabled,
                              nullptr);
    }
  }

  // Look back one and a half mean intervals. More than one interval keeps the
  // target strictly inside a bracket, so the ring interpolates instead of
  // clamping at its edge; the mean rather than the last measurement keeps the
  // target from dancing with the interval's own jitter. When a tick arrives
  // late the target runs past the newest sample, and the ring continues along
  // the newest pair rather than freezing there.
  const double mean_interval_ns =
      context.sample_mean_interval_ns.load(std::memory_order_relaxed);
  constexpr double kSampleDelayFactor = 1.5;
  const double delay_ns = mean_interval_ns * kSampleDelayFactor;
  nfs2015_chase::CarSample sample;
  const bool zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_relaxed);
  // The curved sample, not the linear one. Linear interpolation is continuous
  // in position and discontinuous in velocity, so the eye's speed steps at every
  // Game tick and the whole world's flow rate steps with it. The projection
  // instrument cannot see that: it projects the car, which travels with the
  // camera, so the car stays still while the picture jolts. W, the camera's own
  // speed jump, is the number that sees it, and this curve is what it is for.
  const bool sampled =
      zero_lag ? context.car_ring.Latest(sample)
               : context.car_ring.AtSmooth(
                     now_ns - static_cast<std::int64_t>(delay_ns), sample);
  if (!sampled) {
    return false;
  }
  if (!sample.driving) {
    // The car disappeared (exit / world change). Do not leave a latched shot
    // waiting to resume at full blend when the next vehicle appears.
    context.burnout_shot.Reset();
    context.presenter_burnout_blend.store(0.0, std::memory_order_relaxed);
    context.presenter_burnout_side.store(1, std::memory_order_relaxed);
    return false;
  }

  // A bumped epoch means the model must restart (world change, teleport,
  // preset switch). Restart the offset estimate too, so it initialises from
  // the current frame instead of converging over a third of a second.
  if (context.presenter_initialized && context.presenter_epoch != sample.epoch) {
    context.game_offset_valid = false;
    context.game_offset_last_ns = 0;
  }

  // Anchor the model to the car position produced at this detour instant.
  // Four sources, selected by `anchor_source`:
  //   X       the player root+280 read the same detour took (task group 1),
  //   Vehicle the player capsule's AttachParent (VehicleMesh) scene proxy
  //           LocalToWorld, with the component transform as fallback,
  //   Borrow  the old game-camera estimate,
  //   Model   the untouched Game-tick sample.
  // X only enters the anchor; velocity and yaw still come from the Game tick.
  if (game_eye_cm != nullptr) {
    const double our_car_cm[3] = {
        sample.car.position.x * 100.0, sample.car.position.y * 100.0,
        sample.car.position.z * 100.0};
    const double borrow_strength =
        context.borrow_strength.load(std::memory_order_relaxed);
    const std::uint32_t anchor_source =
        context.anchor_source.load(std::memory_order_relaxed);
    double raw_est_cm[3]{};

    bool x_valid = false;
    if (anchor_source == kAnchorSourceX ||
        anchor_source == kAnchorSourceVehicle) {
      x_valid =
          context.render_x.anchor_enabled.load(std::memory_order_acquire) &&
          context.render_x.anchor_valid.load(std::memory_order_acquire);
      if (x_valid) {
        const std::int64_t age =
            now_ns - context.render_x.anchor_ns.load(std::memory_order_relaxed);
        x_valid = age >= 0 && age <= kAnchorSnapshotMaxAgeNs;
      }
    }
    bool vehicle_valid = false;
    if (anchor_source == kAnchorSourceVehicle) {
      vehicle_valid =
          context.render_x.anchor_enabled.load(std::memory_order_acquire) &&
          context.render_x.vehicle_anchor_valid.load(
              std::memory_order_acquire);
      if (vehicle_valid) {
        const std::int64_t age =
            now_ns - context.render_x.vehicle_anchor_ns.load(
                         std::memory_order_relaxed);
        vehicle_valid = age >= 0 && age <= kAnchorSnapshotMaxAgeNs;
      }
      if (!vehicle_valid) {
        context.render_x.vehicle_fallbacks.fetch_add(
            1, std::memory_order_relaxed);
      }
    }

    if (anchor_source == kAnchorSourceVehicle && vehicle_valid) {
      // The vehicle source reads the capsule's AttachParent (the `VehicleMesh`
      // SkeletalMesh) in the same detour pass. Its ComponentToWorld is the
      // chassis transform the renderer uses; the capsule's own origin is the
      // seat, a child transform that can lag the rendered car.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] = context.render_x.vehicle_anchor_cm[axis].load(
            std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateVehicle,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceX && x_valid) {
      // X is the raw `root+280`; the model's car origin is that value plus the
      // user's `originOffsetCm`, applied on Z exactly as `QueryPlayer` does.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] =
            context.render_x.anchor_cm[axis].load(std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      // The borrow offset is not tracking in this mode. If the X path falls
      // back, the offset re-initialises from the current Game camera instead
      // of resuming a stale estimate.
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateX,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceVehicle && x_valid) {
      // The vehicle parent was unavailable this frame (on foot, or a read that
      // did not validate). The character X is a better fallback than borrow.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] =
            context.render_x.anchor_cm[axis].load(std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateX,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceModel) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] = our_car_cm[axis];
      }
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateDisabled,
                                       std::memory_order_relaxed);
    } else {
      if (anchor_source == kAnchorSourceX ||
          anchor_source == kAnchorSourceVehicle) {
        context.render_x.anchor_fallbacks.fetch_add(
            1, std::memory_order_relaxed);
      }
      const double borrow_dt =
          context.game_offset_last_ns == 0
              ? 0.0
              : static_cast<double>(now_ns - context.game_offset_last_ns) * 1e-9;
      context.game_offset_last_ns = now_ns;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double offset = game_eye_cm[axis] - our_car_cm[axis];
        if (!context.game_offset_valid) {
          context.game_offset_cm[axis] = offset;
        } else if (borrow_dt > 1e-5 && borrow_dt < 0.2) {
          constexpr double kGameOffsetRate = 3.3;
          const double blend =
              1.0 - std::exp(-kGameOffsetRate * borrow_dt);
          context.game_offset_cm[axis] +=
              (offset - context.game_offset_cm[axis]) * blend;
        }
        raw_est_cm[axis] =
            game_eye_cm[axis] - context.game_offset_cm[axis];
      }
      context.game_offset_valid = true;
      context.render_x.last_used.store(kAnchorStateBorrow,
                                       std::memory_order_relaxed);
    }

    // `borrow_strength` still interpolates between the plain tick sample and
    // the selected source, so 0 is the pure model and 1 is fully anchored.
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double car_cm =
          our_car_cm[axis] +
          (raw_est_cm[axis] - our_car_cm[axis]) * borrow_strength;
      if (axis == 0) {
        sample.car.position.x = car_cm / 100.0;
      } else if (axis == 1) {
        sample.car.position.y = car_cm / 100.0;
      } else {
        sample.car.position.z = car_cm / 100.0;
      }
      sample.anchor_cm[axis] = car_cm;
      sample.origin_cm[axis] = car_cm;
    }
  }

  if (!context.presenter_initialized || context.presenter_epoch != sample.epoch ||
      !context.chase.initialized()) {
    context.chase.Reset(sample.car);
    context.burnout_shot.Reset();
    context.presenter_burnout_blend.store(0.0, std::memory_order_relaxed);
    context.presenter_burnout_side.store(1, std::memory_order_relaxed);
    context.presenter_initialized = true;
    context.presenter_epoch = sample.epoch;
    context.presenter_last_ns = now_ns;
  }

  double elapsed =
      static_cast<double>(now_ns - context.presenter_last_ns) * 1e-9;
  if (!(elapsed > 0.0) || elapsed > kMaximumPresenterStepSeconds) {
    elapsed = 0.0;
  }
  context.presenter_last_ns = now_ns;
  // Step the model by the real time that passed. The springs settle with
  // exp(-rate * dt) (chase_cam.hpp), which solves the differential equation
  // exactly for any dt, so a variable step is not an approximation here and
  // there is nothing to keep frame-rate independent.
  //
  // The previous version accumulated this time and spent it in fixed 1/120 s
  // substeps. That made the number of substeps per frame 120 divided by the
  // frame rate, which is only a whole number at 30, 40 and 60 fps. At any other
  // rate the count alternates between the two neighbours and the camera's own
  // advance is quantised: at 45 fps it takes 2, 3, 3 substeps, so the model
  // moves 16.7, 25, 25 ms of distance in frames that were all equally long, and
  // the car drifts up to 17 cm back and forth in frame at 30 m/s. That is the
  // judder: invisible at 60, worst at 45, and proportional to speed, which is
  // exactly how it has been reported.
  const nfs2015_chase::CameraState chase_view =
      context.chase.Update(elapsed, sample.car);

  // Phase 3/4: advance the burnout shot on the same render clock and blend
  // its pose into the chase view before publishing anything. The shot consumes
  // the same anchored car sample as the chase, so the wheel stays attached to
  // the rendered chassis. A zero blend returns the chase view exactly.
  const bool burnout_enabled =
      context.burnout_enabled.load(std::memory_order_acquire);
  nfs2015_chase::CameraState view = chase_view;
  if (burnout_enabled) {
    nfs2015_chase::BurnoutShotConfig shot_config =
        context.burnout_shot.config();
    shot_config.entry_delay =
        context.burnout_entry_delay.load(std::memory_order_acquire);
    shot_config.entry_time =
        context.burnout_entry_time.load(std::memory_order_acquire);
    shot_config.exit_delay =
        context.burnout_exit_delay.load(std::memory_order_acquire);
    shot_config.exit_time =
        context.burnout_exit_time.load(std::memory_order_acquire);
    shot_config.launch_exit_time =
        context.burnout_launch_exit_time.load(std::memory_order_acquire);
    shot_config.horizontal_fov_degrees =
        context.burnout_fov.load(std::memory_order_acquire);
    shot_config.shake_scale =
        context.burnout_shake.load(std::memory_order_acquire);
    shot_config.side_mode = static_cast<int>(
        context.burnout_side_mode.load(std::memory_order_acquire));
    shot_config.entry_speed_mps =
        context.burnout_entry_speed.load(std::memory_order_acquire);
    shot_config.hold_speed_mps =
        context.burnout_hold_speed.load(std::memory_order_acquire);
    shot_config.slope_follow =
        context.slope_follow.load(std::memory_order_acquire);
    // The presenter / route B still applies the user's cameraScale once,
    // around the car origin, after the blend. Scaling the shot locally as well
    // would square that value (0.6 -> 0.36) and pull the close-up into the car.
    shot_config.camera_scale = 1.0;
    context.burnout_shot.SetConfig(shot_config);
    const nfs2015_chase::BurnoutShotResult burnout =
        context.burnout_shot.Update(elapsed, sample.car, sample.burnout,
                                   sample.aspect_ratio);
    context.presenter_burnout_blend.store(burnout.blend,
                                          std::memory_order_relaxed);
    context.presenter_burnout_side.store(burnout.side,
                                         std::memory_order_relaxed);
    view = nfs2015_chase::BlendBurnoutCamera(chase_view, burnout.view,
                                             sample.car, burnout.blend);
  } else {
    // Disabling the feature must cut back to the exact chase pose instead of
    // leaving a latched shot to resume at full blend when it is re-enabled.
    context.burnout_shot.Reset();
    context.presenter_burnout_blend.store(0.0, std::memory_order_relaxed);
    context.presenter_burnout_side.store(1, std::memory_order_relaxed);
  }
  // A disabled game FOV must not be changed by the shot. The route B publisher
  // already writes lens=0 in that case; restoring the chase vertical FOV here
  // keeps the CameraState consistent for any other consumer.
  if (!sample.fov_enabled) {
    view.fov_y_degrees = chase_view.fov_y_degrees;
  }
  PublishRouteBModelPose(context, sample, view, now_ns);

  // Scale around the model origin, exactly as the old publish path did, and
  // report the result in world centimetres. The anchor cancels out of this
  // transform; it is kept in the sample for diagnostics.
  const double scale = nfs2015_chase::Clamp(
      sample.camera_scale, kMinimumCameraScale, kMaximumCameraScale);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double origin = sample.origin_cm[axis];
    const double model = (&view.position.x)[axis] * 100.0;
    location[axis] = origin + (model - origin) * scale;
  }
  rotation[0] = view.pitch_degrees;
  rotation[1] = view.yaw_degrees;
  rotation[2] = view.roll_degrees;
  *fov_enabled = sample.fov_enabled;
  *lens = sample.fov_enabled
              ? static_cast<float>(
                    view.HorizontalFovDegrees(sample.aspect_ratio))
              : 0.0F;

  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.presenter_anchor[axis].store(sample.anchor_cm[axis],
                                         std::memory_order_relaxed);
  }
  const double car_cm[3] = {
      sample.car.position.x * 100.0,
      sample.car.position.y * 100.0,
      sample.car.position.z * 100.0,
  };
  PublishPresenterPose(context, location, rotation, car_cm, *lens,
                       *fov_enabled);
  context.presenter_pose_ns = now_ns;
  context.presenter_drift.store(context.chase.drift(),
                                std::memory_order_relaxed);
  double distance2 = 0.0;
  double offset_cm[3]{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    offset_cm[axis] = location[axis] - sample.anchor_cm[axis];
    distance2 += offset_cm[axis] * offset_cm[axis];
  }
  const double distance_cm = std::sqrt(distance2);
  context.presenter_distance_cm.store(distance_cm, std::memory_order_relaxed);
  *advanced = true;

  return true;
}

void UpdateCamera(Context &context, const double dt) noexcept {
  const bool enabled = context.enabled.load(std::memory_order_acquire);
  const std::uint32_t preset = context.preset.load(std::memory_order_acquire);
  bool fov_enabled = context.fov_enabled.load(std::memory_order_acquire);
  const double camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
  const double position_spring =
      context.position_spring.load(std::memory_order_acquire);
  const double yaw_spring =
      context.yaw_spring.load(std::memory_order_acquire);
  const double drift_yaw_spring =
      context.drift_yaw_spring.load(std::memory_order_acquire);
  const double look_yaw_spring =
      context.look_yaw_spring.load(std::memory_order_acquire);
  const double yaw_return_spring =
      context.yaw_return_spring.load(std::memory_order_acquire);
  const double pitch_spring =
      context.pitch_spring.load(std::memory_order_acquire);
  const double roll_spring =
      context.roll_spring.load(std::memory_order_acquire);
  const double aspect_ratio =
      context.aspect_ratio.load(std::memory_order_acquire);
  const double speed_distance_scale =
      context.speed_distance_scale.load(std::memory_order_acquire);
  const double roll_scale =
      context.roll_scale.load(std::memory_order_acquire);
  const double pitch_scale =
      context.pitch_scale.load(std::memory_order_acquire);
  const double shake_scale =
      context.shake_scale.load(std::memory_order_acquire);
  const bool slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  const double slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  const double slope_max_gradient_degrees =
      context.slope_max_gradient_degrees.load(std::memory_order_acquire);
  const double slope_up_look_at =
      context.slope_up_look_at.load(std::memory_order_acquire);
  const double slope_up_rest =
      context.slope_up_rest.load(std::memory_order_acquire);
  const double slope_down_look_at =
      context.slope_down_look_at.load(std::memory_order_acquire);
  const double slope_down_rest =
      context.slope_down_rest.load(std::memory_order_acquire);
  bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  // The bisect switch: no attitude, no lens. Position and yaw only.
  const bool diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  const double effective_roll_scale = diagnostic_flat_pose ? 0.0 : roll_scale;
  const double effective_pitch_scale = diagnostic_flat_pose ? 0.0 : pitch_scale;
  const double effective_shake_scale = diagnostic_flat_pose ? 0.0 : shake_scale;
  const bool effective_slope_follow = !diagnostic_flat_pose && slope_follow;
  if (diagnostic_flat_pose) {
    fov_enabled = false;
    roll_only_in_drift = true;
  }
  // The model lives on the render clock now, so configuration changes are made
  // under its lock.
  LockPresenter(context);
  if (preset != context.configured_preset) {
    context.configured_preset = preset;
    context.chase.SetConfig(preset == 0 ? nfs2015_chase::GamePreset()
                                        : nfs2015_chase::CinematicPreset());
    RequestPresenterReset(context);
    NoteViewSnap(context, kSnapReasonPreset);
  }
  // The spring sliders are Game-preset tuning aids; the cinematic preset
  // keeps its own values.
  if (preset == 0) {
    context.chase.SetPositionSpring(position_spring);
    context.chase.SetYawSpring(yaw_spring);
    context.chase.SetDriftYawSpring(drift_yaw_spring);
    context.chase.SetLookYawSpring(look_yaw_spring);
    context.chase.SetYawReturnSpring(yaw_return_spring);
    context.chase.SetPitchSpring(pitch_spring);
    context.chase.SetRollSpring(roll_spring);
  }
  // These are cheap setters that do not reset the springs.
  context.chase.SetSpeedDistanceScale(speed_distance_scale);
  context.chase.SetRollScale(effective_roll_scale);
  context.chase.SetPitchScale(effective_pitch_scale);
  context.chase.SetShakeScale(effective_shake_scale);
  context.chase.SetSlopeFollow(effective_slope_follow);
  context.chase.SetSlopeStrength(slope_strength);
  context.chase.SetSlopeMaxGradientDegrees(slope_max_gradient_degrees);
  context.chase.SetSlopeGradientParams(
      {slope_up_look_at, slope_up_rest},
      {slope_down_look_at, slope_down_rest});
  context.chase.SetRollOnlyInDrift(roll_only_in_drift);
  UnlockPresenter(context);
  if (context.reset_requested.exchange(false, std::memory_order_acq_rel)) {
    RequestPresenterReset(context);
    context.have_last_yaw = false;
    NoteViewSnap(context, kSnapReasonUserReset);
  }
  if (WorldChanged(context)) {
    ResetMotion(context);
  }

  double origin_cm[3]{};
  double base_cm[3]{};
  std::uint64_t player_sequence = 0;
  AnomalyGenerationHandleV1 player_handle{};
  const bool player_valid = QueryPlayer(context, origin_cm, base_cm,
                                        player_sequence, player_handle);
  // Task group 0 probe: cache the pawn chain and publish the snapshot the
  // render-thread read is compared with. Read-only; the camera ignores it.
  UpdateRenderXProbe(context, base_cm, player_valid);
  // The camera anchor is the Pawn root location that `QueryPlayer` already
  // fills. `base_cm` cancels out of the published world position (the detour
  // adds it back), so it only has to be stable and cheap. Resolving the
  // rendered body through `component_bounds` used to cost ~2 ms of the Game
  // tick: the host walks the class's whole reflected property list by name on
  // every call, decoding an FName twice per property and reading it through
  // ReadProcessMemory. Nothing about the camera needs that.
  // Anomaly 2.7.0 removed `anomaly.nte.vehicle`, not the game's vehicle
  // getters. Poll the same reflected functions the old Host snapshot used:
  // BP_GetCurrentDriveVehicle / BP_GetCurrentDirvingVehicle /
  // GetBoardWaterVehicle. Their answer is authoritative even when parked; the
  // AttachParent sample remains the anchor source and the fallback when the
  // generic UE5 services are unavailable.
  RenderXProbe &render_x = context.render_x;
  const VehiclePresence vehicle_query =
      enabled && player_valid ? QueryVehiclePresence(context)
                              : VehiclePresence{};
  const std::int64_t now_ns = NowNanoseconds();
  const std::int64_t vehicle_sample_ns =
      render_x.vehicle_sample_ns.load(std::memory_order_relaxed);
  const bool attach_sample_fresh =
      vehicle_sample_ns > 0 && now_ns >= vehicle_sample_ns &&
      now_ns - vehicle_sample_ns <= kVehiclePresenceFreshNs;
  const bool attach_present_fresh =
      attach_sample_fresh &&
      render_x.vehicle_present.load(std::memory_order_acquire);
  const bool presence_known = vehicle_query.known || attach_sample_fresh;
  const bool presence_now =
      vehicle_query.known ? vehicle_query.present : attach_present_fresh;
  const std::uintptr_t vehicle_parent =
      render_x.vehicle_parent.load(std::memory_order_relaxed);
  if (presence_now) {
    context.last_vehicle_present_time = context.elapsed;
  }
  const bool vehicle_recent =
      context.last_vehicle_present_time > 0.0 &&
      context.elapsed >= context.last_vehicle_present_time &&
      context.elapsed - context.last_vehicle_present_time <=
          kVehiclePresenceGraceSeconds;
  const bool vehicle_now = presence_now || vehicle_recent;
  const std::uintptr_t presence_identity =
      vehicle_query.known
          ? vehicle_query.object
          : (attach_present_fresh ? vehicle_parent : 0);

  bool identity_changed = false;
  if (player_valid && player_handle.id != 0 &&
      (player_handle.id != context.player_handle.id ||
       player_handle.generation != context.player_handle.generation)) {
    context.player_handle = player_handle;
    identity_changed = true;
  }
  if (presence_identity != 0 && context.last_vehicle_parent != 0 &&
      context.last_vehicle_parent != presence_identity) {
    identity_changed = true;
  }
  // Entering at a standstill must discard walking velocity, but a presence
  // sample that appears after the speed fallback has already taken over must
  // not reset the model mid-drive.
  const bool vehicle_entered = presence_now &&
                               !context.vehicle_present_last &&
                               !context.last_view_active;
  if (identity_changed || vehicle_entered) {
    ResetMotion(context);
  }
  if (presence_identity != 0) {
    context.last_vehicle_parent = presence_identity;
  } else if (!vehicle_now) {
    context.last_vehicle_parent = 0;
  }
  context.vehicle_present_last = vehicle_now;

  if (!enabled || !player_valid) {
    ReleaseWheelSteer(context);
    PublishInactive(context);
    ResetMotion(context);
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.reversing = false;
    context.display.player_valid = player_valid;
    context.display.speed_source = kSpeedSourceUnavailable;
    context.display.speed_sign_known = false;
    return;
  }

  // Keep the motion history alive while the camera is inactive: the fallback
  // speed gate is built from consecutive positions, and resetting every idle
  // tick would make it impossible for that speed to ever rise above zero.
  const bool had_motion = context.have_last_position;
  const std::uint64_t previous_sequence = context.last_player_sequence;
  UpdateMotion(context, origin_cm, player_sequence);
  const bool player_sample_fresh =
      !had_motion || player_sequence != previous_sequence;

  double velocity_length =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const double velocity_mps = velocity_length / 100.0;
  const bool speed_plausible =
      velocity_mps >= kMinimumVehicleSpeedMps && velocity_mps <
          kMaximumPlausibleSpeedCmps / 100.0;
  // Speed is a last resort for hosts where the getter and the AttachParent
  // sample are both unavailable. It must never turn sprinting on foot into a
  // car when a presence answer exists.
  const bool driving = vehicle_now || (speed_plausible && !presence_known);
  if (!driving) {
    ReleaseWheelSteer(context);
    const bool was_active = context.last_view_active;
    PublishInactive(context);
    if (was_active) {
      ResetMotion(context);
    }
    context.vehicle_present_last = false;
    context.last_vehicle_present_time = 0.0;
    context.last_vehicle_parent = 0;
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.reversing = false;
    context.display.player_valid = true;
    context.display.speed_source = kSpeedSourceUnavailable;
    context.display.speed_sign_known = false;
    context.display.speed_kmh = 0.0;
    return;
  }

  // The exact forward speed is the 2.6 signal: use it for magnitude and
  // reverse, and keep the position direction only for the lateral component
  // (slip/roll). That is what the old service did when HAS_SPEED was set.
  const bool exact_speed = vehicle_query.present && vehicle_query.speed_known;
  const bool exact_speed_fresh = exact_speed && vehicle_query.speed_fresh;
  if (exact_speed) {
    context.speed_sign_known = true;
    const double target_cmps =
        std::fabs(vehicle_query.speed_kmh) * (100000.0 / 3600.0);
    if (velocity_length > 50.0) {
      const double scale = target_cmps / velocity_length;
      if (scale > 0.5 && scale < 2.0) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          context.velocity_cmps[axis] *= scale;
        }
        velocity_length =
            std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
      }
    }
  }
  const double velocity_yaw =
      velocity_length > 1.0
          ? std::atan2(context.velocity_cmps[1], context.velocity_cmps[0])
          : (context.have_last_yaw ? context.last_car_yaw : 0.0);
  double car_yaw =
      SelectCarYaw(context, velocity_yaw, velocity_length / 100.0,
                   velocity_length > 50.0, exact_speed,
                   vehicle_query.speed_kmh);
  if (context.have_last_yaw && dt > 1e-5) {
    const double maximum_step = kMaximumBodyYawRate * dt;
    car_yaw =
        context.last_car_yaw +
        nfs2015_chase::Clamp(WrappedAngle(car_yaw - context.last_car_yaw),
                             -maximum_step, maximum_step);
  }
  if (context.have_last_yaw && dt > 1e-5) {
    const double raw_rate = WrappedAngle(car_yaw - context.last_car_yaw) / dt;
    context.yaw_rate = nfs2015_chase::Damp(
        context.yaw_rate,
        nfs2015_chase::Clamp(raw_rate, -kMaximumYawRate, kMaximumYawRate),
        kYawRateSmoothing, dt);
  }
  context.last_car_yaw = car_yaw;
  context.have_last_yaw = true;

  const double speed_mps = velocity_length / 100.0;
  const nfs2015_chase::Vec3 world_velocity{
      context.velocity_cmps[0] / 100.0, context.velocity_cmps[1] / 100.0,
      context.velocity_cmps[2] / 100.0};
  const double forward_speed =
      nfs2015_chase::Dot(world_velocity,
                         nfs2015_chase::ForwardFromYaw(car_yaw));
  double slip_angle = 0.0;
  if (speed_mps >= 0.75 && forward_speed >= 0.0) {
    slip_angle = nfs2015_chase::Clamp(
        WrappedAngle(velocity_yaw - car_yaw), -nfs2015_chase::kPi * 0.5,
        nfs2015_chase::kPi * 0.5);
  }
  // Experiment (T3.4): the slip angle from the position derivative. The
  // signed speed above comes from the same derivative, so the reverse gate and
  // the model see one consistent sign.
  const double slip_position_degrees =
      slip_angle * kRadiansToDegrees;
  context.diag.slip_from_position_degrees.store(
      slip_position_degrees, std::memory_order_relaxed);
  {
    // A real drift holds one slip sign for as long as it lasts. Counting the
    // sign changes is an objective proxy for "the drift reading flickers",
    // which is what a twitch in the push-in and roll would come from.
    const int sign =
        slip_position_degrees > 3.0 ? 1
                                    : (slip_position_degrees < -3.0 ? -1 : 0);
    const int previous = context.diag.slip_last_sign.load(
        std::memory_order_relaxed);
    if (sign != 0 && previous != 0 && sign != previous) {
      context.diag.slip_sign_changes.fetch_add(1, std::memory_order_relaxed);
    }
    if (sign != 0) {
      context.diag.slip_last_sign.store(sign, std::memory_order_relaxed);
    }
  }
  {
    // Experiment (T3.4): how far the slip angle moves in a single tick. The
    // camera model consumes this number for the drift blend, the roll and the
    // push-in, so a step here becomes a visible twitch.
    const double step = context.have_last_slip
                            ? std::fabs(slip_position_degrees -
                                        context.last_slip_degrees)
                            : 0.0;
    context.last_slip_degrees = slip_position_degrees;
    context.have_last_slip = true;
    const std::uint64_t step_ddeg =
        static_cast<std::uint64_t>(step * 10.0);
    std::uint64_t previous_peak =
        context.diag.slip_max_step_ddeg.load(std::memory_order_relaxed);
    while (step_ddeg > previous_peak &&
           !context.diag.slip_max_step_ddeg.compare_exchange_weak(
               previous_peak, step_ddeg, std::memory_order_relaxed)) {
    }
  }

  if (exact_speed != context.last_speed_exact) {
    // Do not differentiate an exact-speed sample against a position-derived
    // one (or the reverse); that step would read as a phantom brake.
    context.last_vehicle_speed_mps = -1.0;
    context.last_vehicle_speed_time = context.elapsed;
    context.brake_deceleration_mps2 = 0.0;
    context.last_speed_exact = exact_speed;
  }
  // Exact speed is fresh when the 20 Hz vehicle poll dispatched. If it is not
  // available (water vehicle or a failed getter), keep the position-derived
  // fallback, which is fresh when the player sequence advanced.
  UpdateBrakeInput(context, true,
                   exact_speed ? exact_speed_fresh : player_sample_fresh,
                   !exact_speed,
                   exact_speed ? vehicle_query.speed_kmh : speed_mps * 3.6,
                   dt);

  nfs2015_chase::CarState car;
  car.position = {origin_cm[0] / 100.0, origin_cm[1] / 100.0,
                  origin_cm[2] / 100.0};
  car.yaw = car_yaw;
  // The Game-tick snapshot has no body attitude; the rendered vehicle proxy is
  // the only source for the slope frame. A fresh pitch is held briefly across
  // a missed render frame and zeroed after a reset or teleport.
  car.pitch = UpdateRenderPitch(context);
  car.velocity = world_velocity;
  car.yaw_rate = context.yaw_rate;
  nfs2015_chase::Vec3 acceleration{
      context.acceleration_cmps2[0] / 100.0,
      context.acceleration_cmps2[1] / 100.0,
      context.acceleration_cmps2[2] / 100.0};
  if (exact_speed) {
    // Restore the 2.6 split: exact longitudinal acceleration, position-derived
    // lateral acceleration. The longitudinal derivative of position is the
    // noisy part that pumped the dolly/pitch filters.
    const nfs2015_chase::Vec3 forward =
        nfs2015_chase::ForwardFromYaw(car_yaw);
    const nfs2015_chase::Vec3 right =
        nfs2015_chase::RightFromYaw(car_yaw);
    const double lateral = nfs2015_chase::Dot(acceleration, right);
    acceleration = nfs2015_chase::Add(
        nfs2015_chase::Mul(forward, context.vehicle_acceleration_mps2),
        nfs2015_chase::Mul(right, lateral));
  }
  car.acceleration = acceleration;
  car.speed = forward_speed;
  car.slip_angle = slip_angle;
  car.brake = context.brake_input;
  car.on_ground = true;

  // Phase 0/2: read the reflected burnout/input signals at 20 Hz, then turn
  // them into the sample facts. The camera still ignores the result.
  UpdateBurnoutProbe(context, vehicle_query, speed_mps, forward_speed);
  const nfs2015_chase::BurnoutSample burnout_sample =
      UpdateBurnoutDetection(context, car.on_ground, speed_mps, forward_speed,
                             dt);
  // NFS 2015-style visual countersteer. This writes only the front wheel
  // components' RelativeRotation; it never touches the movement component.
  UpdateWheelSteer(context, vehicle_query.object, world_velocity,
                   forward_speed, speed_mps, burnout_sample.steering,
                   context.burnout_signals.steering_known, context.reversing,
                   dt);

  // T2: the tick no longer advances the model - it publishes a car sample and
  // the render thread integrates the springs on its own clock.
  {
    nfs2015_chase::CarSample sample;
    sample.time_ns = NowNanoseconds();
    sample.car = car;
    sample.burnout = burnout_sample;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sample.anchor_cm[axis] = base_cm[axis];
      sample.origin_cm[axis] = origin_cm[axis];
    }
    sample.camera_scale = camera_scale;
    sample.aspect_ratio = aspect_ratio;
    sample.fov_enabled = fov_enabled;
    sample.driving = true;
    sample.epoch = context.sample_epoch;
    if (context.sample_last_ns > 0 && sample.time_ns > context.sample_last_ns) {
      const std::int64_t interval_ns =
          sample.time_ns - context.sample_last_ns;
      context.sample_interval_ns.store(interval_ns,
                                       std::memory_order_relaxed);
      {
        const double previous =
            context.sample_mean_interval_ns.load(std::memory_order_relaxed);
        const double measured = static_cast<double>(interval_ns);
        constexpr double kTrackingGain = 0.02;
        context.sample_mean_interval_ns.store(
            previous <= 0.0
                ? measured
                : previous + (measured - previous) * kTrackingGain,
            std::memory_order_relaxed);
      }
      // Probe: how steady is the interval the delayed interpolation window is
      // built from. A jumpy interval puts a per-tick speed change into every
      // linearly interpolated quantity.
      const auto interval = static_cast<std::uint64_t>(interval_ns);
      RaisePeak(context.diag.tick_interval_max_ns, interval);
      LowerFloor(context.diag.tick_interval_min_ns, interval);
    }
    context.sample_last_ns = sample.time_ns;
    context.car_ring.Write(sample);
    // The detour gates on this: without it the game camera shows through.
    context.last_view_active = true;
    context.last_fov_enabled = fov_enabled;
    context.view_active.store(true, std::memory_order_release);
  }
  {
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = true;
    context.display.reversing = context.reversing;
    context.display.player_valid = true;
    context.display.speed_source =
        exact_speed ? kSpeedSourceExact : kSpeedSourceDerived;
    context.display.speed_sign_known = context.speed_sign_known;
    context.display.speed_kmh =
        exact_speed ? std::fabs(vehicle_query.speed_kmh) : speed_mps * 3.6;
    context.display.slip_degrees = slip_angle * kRadiansToDegrees;
    context.display.wheel_steer_enabled =
        context.wheel_steer_enabled.load(std::memory_order_relaxed);
    context.display.wheel_steer_active = context.wheel_steer.active;
    context.display.wheel_steer_engaged = context.wheel_steer.engaged;
    context.display.wheel_steer_wheels =
        static_cast<std::uint32_t>(context.wheel_steer.wheel_count);
    context.display.wheel_steer_angle_degrees =
        context.wheel_steer.applied_angle * kRadiansToDegrees;
    context.display.wheel_steer_target_degrees =
        context.wheel_steer.target_angle * kRadiansToDegrees;
    context.display.wheel_steer_entry_speed_kmh =
        context.wheel_steer_entry_speed_kmh.load(std::memory_order_relaxed);
    context.display.wheel_steer_max_degrees =
        context.wheel_steer_max_degrees.load(std::memory_order_relaxed);
    context.display.wheel_steer_gain =
        context.wheel_steer_gain.load(std::memory_order_relaxed);
    context.display.wheel_steer_full_slip_degrees =
        context.wheel_steer_full_slip_degrees.load(
            std::memory_order_relaxed);
    context.display.wheel_steer_rate_degrees_per_second =
        context.wheel_steer_rate_degrees_per_second.load(
            std::memory_order_relaxed);
    context.display.drift =
        context.presenter_drift.load(std::memory_order_relaxed);
    const float presenter_lens =
        context.presenter_pose_lens.load(std::memory_order_relaxed);
    context.display.fov_horizontal =
        fov_enabled && presenter_lens > 1.0F && presenter_lens < 179.0F
            ? static_cast<double>(presenter_lens)
            : 0.0;
    context.display.camera_distance_m =
        context.presenter_distance_cm.load(std::memory_order_relaxed) / 100.0;
    context.display.camera_distance_raw_m =
        context.display.camera_distance_m;
    const BurnoutProbeState &burnout = context.burnout_probe;
    context.display.burnout_probe_is_burnout_known =
        burnout.is_burnout_known;
    context.display.burnout_probe_is_burnout = burnout.is_burnout;
    context.display.burnout_probe_throttle_known = burnout.throttle_known;
    context.display.burnout_probe_throttle = burnout.throttle;
    context.display.burnout_probe_brake_known = burnout.brake_known;
    context.display.burnout_probe_brake = burnout.brake;
    context.display.burnout_probe_steering_known = burnout.steering_known;
    context.display.burnout_probe_steering = burnout.steering;
    context.display.burnout_probe_skid_known = burnout.skid_known;
    context.display.burnout_probe_skid_angle = burnout.skid_angle;
    context.display.burnout_probe_player_throttle_known =
        burnout.player_throttle_known;
    context.display.burnout_probe_player_throttle = burnout.player_throttle;
    context.display.burnout_probe_handbrake12_known =
        burnout.handbrake12_known;
    context.display.burnout_probe_handbrake12 = burnout.handbrake12;
    context.display.burnout_probe_handbrake0_known =
        burnout.handbrake0_known;
    context.display.burnout_probe_handbrake0 = burnout.handbrake0;
    context.display.burnout_probe_handbrake_known = burnout.handbrake_known;
    context.display.burnout_probe_handbrake = burnout.handbrake;
    context.display.burnout_probe_wheel_count_known =
        burnout.wheel_count_known;
    context.display.burnout_probe_wheel_count = burnout.wheel_count;
    for (std::size_t wheel = 0; wheel < kBurnoutProbeMaxWheels; ++wheel) {
      context.display.burnout_probe_wheel_burnout_known[wheel] =
          burnout.wheel_burnout_known[wheel];
      context.display.burnout_probe_wheel_burnout[wheel] =
          burnout.wheel_burnout[wheel];
      context.display.burnout_probe_wheel_rep_known[wheel] =
          burnout.wheel_rep_known[wheel];
      context.display.burnout_probe_wheel_rep[wheel] =
          burnout.wheel_rep[wheel];
      context.display.burnout_probe_wheel_angular_known[wheel] =
          burnout.wheel_angular_known[wheel];
      context.display.burnout_probe_wheel_angular[wheel] =
          burnout.wheel_angular[wheel];
    }
    context.display.burnout_probe_input_raw_size = burnout.input_raw_size;
    std::memcpy(context.display.burnout_probe_input_raw, burnout.input_raw,
                burnout.input_raw_size);
    std::uint32_t resolved = 0;
    std::uint32_t ok = 0;
    std::uint32_t failed = 0;
    for (std::size_t index = 0; index < kBurnoutProbeCount; ++index) {
      const BurnoutProbeResult &result = burnout.results[index];
      context.display.burnout_probe_status[index] = result.status;
      context.display.burnout_probe_code[index] = result.last_code;
      if (result.function.id != 0) {
        ++resolved;
      }
      if (result.status == kBurnoutProbeOk) {
        ++ok;
      }
      if (result.failures != 0) {
        ++failed;
      }
    }
    context.display.burnout_probe_resolved = resolved;
    context.display.burnout_probe_ok = ok;
    context.display.burnout_probe_failed = failed;

    const BurnoutSignals &signals = context.burnout_signals;
    const BurnoutDetectionState &detection = context.burnout_detection;
    context.display.burnout_signal_known = signals.known;
    context.display.burnout_direct = signals.direct_known && signals.direct;
    context.display.burnout_fallback = detection.fallback;
    context.display.burnout_active = detection.active;
    context.display.burnout_left = signals.left_known && signals.left;
    context.display.burnout_right = signals.right_known && signals.right;
    context.display.burnout_intensity = detection.intensity;
    context.display.burnout_throttle = burnout_sample.throttle;
    context.display.burnout_handbrake = burnout_sample.handbrake;
    context.display.burnout_steering = burnout_sample.steering;
    context.display.burnout_speed_mps = burnout_sample.speed_mps;
    context.display.burnout_hold_remaining =
        detection.active && detection.hold_until > context.elapsed
            ? detection.hold_until - context.elapsed
            : 0.0;
  }
}

void UpdateDisplay(Context &context) noexcept {
  std::scoped_lock lock(context.display_mutex);
  context.display.started = context.started.load(std::memory_order_acquire);
  context.display.enabled = context.enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_enabled =
      context.wheel_steer_enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_active = context.wheel_steer.active;
  context.display.wheel_steer_engaged = context.wheel_steer.engaged;
  context.display.wheel_steer_wheels =
      static_cast<std::uint32_t>(context.wheel_steer.wheel_count);
  context.display.wheel_steer_angle_degrees =
      context.wheel_steer.applied_angle * kRadiansToDegrees;
  context.display.wheel_steer_target_degrees =
      context.wheel_steer.target_angle * kRadiansToDegrees;
  context.display.wheel_steer_entry_speed_kmh =
      context.wheel_steer_entry_speed_kmh.load(std::memory_order_acquire);
  context.display.wheel_steer_max_degrees =
      context.wheel_steer_max_degrees.load(std::memory_order_acquire);
  context.display.wheel_steer_gain =
      context.wheel_steer_gain.load(std::memory_order_acquire);
  context.display.wheel_steer_full_slip_degrees =
      context.wheel_steer_full_slip_degrees.load(
          std::memory_order_acquire);
  context.display.wheel_steer_rate_degrees_per_second =
      context.wheel_steer_rate_degrees_per_second.load(
          std::memory_order_acquire);
  context.display.speed_distance_scale =
      context.speed_distance_scale.load(std::memory_order_acquire);
  context.display.hook_ready =
      context.hook_ready.load(std::memory_order_acquire);
  context.display.active =
      context.view_active.load(std::memory_order_acquire) &&
      context.hook_ready.load(std::memory_order_acquire);
  context.display.update_hz = context.update_hz;
  context.display.view_output_hz = context.view_output_hz;
  context.display.interpolation_ratio = context.interpolation_ratio;
  context.display.detour_hz = context.detour_hz;
  context.display.tick_ms_last = context.camera_ms_last;
  context.display.tick_ms_avg = context.camera_cost.Average();
  context.display.tick_ms_p99 = context.camera_cost.Percentile(0.99);
  context.display.tick_ms_max = context.camera_cost.Maximum();
  const Diagnostics &diag = context.diag;
  std::uint64_t snap_total = 0;
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    const std::uint64_t count =
        diag.snap_by_reason[index].load(std::memory_order_relaxed);
    context.display.snap_by_reason[index] = count;
    snap_total += count;
  }
  context.display.snap_total = snap_total;
  context.display.yaw_source_switches =
      diag.yaw_source_switches.load(std::memory_order_relaxed);
  context.display.observed_read_failures =
      diag.observed_read_failures.load(std::memory_order_relaxed);
  context.display.lease_failures =
      diag.lease_failures.load(std::memory_order_relaxed);
  context.display.view_read_torn =
      diag.view_read_torn.load(std::memory_order_relaxed);
  context.display.detour_calls =
      diag.detour_calls.load(std::memory_order_relaxed);
  context.display.detour_no_publish =
      diag.detour_no_publish.load(std::memory_order_relaxed);
  context.display.detour_thread_count =
      diag.detour_thread_count.load(std::memory_order_relaxed);
  context.display.player_sequence_jumps =
      diag.player_sequence_jumps.load(std::memory_order_relaxed);
  context.display.camera_yaw_esp_misses =
      diag.camera_yaw_esp_misses.load(std::memory_order_relaxed);
  context.display.slip_from_position_degrees =
      diag.slip_from_position_degrees.load(std::memory_order_relaxed);
  context.display.slip_sign_changes =
      diag.slip_sign_changes.load(std::memory_order_relaxed);
  context.display.slip_max_step_degrees =
      static_cast<double>(
          diag.slip_max_step_ddeg.load(std::memory_order_relaxed)) /
      10.0;
  const std::uint64_t speed_floor_raw =
      diag.render_speed_floor_mmps.load(std::memory_order_relaxed);
  // The floor starts at UINT64_MAX so the first sample sets it; an untouched
  // window must not print that sentinel.
  context.display.render_speed_floor_mps =
      speed_floor_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(speed_floor_raw) / 1000.0;
  context.display.render_speed_valid = speed_floor_raw != UINT64_MAX;
  context.display.render_speed_peak_mps =
      static_cast<double>(
          diag.render_speed_peak_mmps.load(std::memory_order_relaxed)) /
      1000.0;
  const std::uint64_t turn_floor_raw =
      diag.render_turn_floor_cdps.load(std::memory_order_relaxed);
  context.display.render_turn_floor_dps =
      turn_floor_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(turn_floor_raw) / 100.0;
  context.display.render_turn_valid = turn_floor_raw != UINT64_MAX;
  context.display.render_turn_peak_dps =
      static_cast<double>(
          diag.render_turn_peak_cdps.load(std::memory_order_relaxed)) /
      100.0;
  context.display.car_step_cm =
      static_cast<double>(diag.car_step_dcm.load(std::memory_order_relaxed)) /
      10.0;
  const Probe &probe = context.probe;
  context.display.probe_ahud_service =
      context.ahud != nullptr && ProbeAhudReady(context.ahud);
  context.display.probe_subscribe_state = probe.subscribe_state;
  context.display.probe_subscribe_code = probe.subscribe_code;
  context.display.probe_frame_hz = probe.frame_hz;
  context.display.probe_frame_calls = probe.frame_calls;
  context.display.probe_camera_yaw_step_max_degrees =
      probe.camera_yaw_step_max_ddeg;
  context.display.probe_camera_yaw_step_avg_degrees =
      probe.camera_yaw_step_count == 0
          ? 0.0
          : probe.camera_yaw_step_sum_ddeg /
                static_cast<double>(probe.camera_yaw_step_count);
  context.display.probe_camera_yaw_valid = probe.camera_yaw_valid;
  context.display.probe_camera_yaw_degrees = probe.camera_yaw_degrees;
  context.display.probe_bounds_offset_cm = probe.bounds_offset_cm;
  context.display.probe_bounds_yaw_valid = probe.bounds_yaw_valid;

  context.display.probe_camera_minus_bounds_degrees =
      probe.camera_minus_bounds_valid ? probe.camera_minus_bounds_degrees
                                      : 0.0;
  context.display.resets_in_window =
      diag.reset_window.load(std::memory_order_relaxed);
  {
    const std::uint64_t flow_count =
        diag.flow_count.load(std::memory_order_relaxed);
    const std::uint64_t flow_sum =
        diag.flow_sum_ccm.load(std::memory_order_relaxed);
    context.display.flow_valid = flow_count != 0;
    context.display.flow_peak_cm =
        static_cast<double>(
            diag.flow_peak_ccm.load(std::memory_order_relaxed)) /
        100.0;
    context.display.flow_avg_cm =
        flow_count == 0 ? 0.0
                        : static_cast<double>(flow_sum) /
                              static_cast<double>(flow_count) / 100.0;
    const std::uint64_t frame_count =
        diag.frame_count.load(std::memory_order_relaxed);
    const std::uint64_t frame_sum =
        diag.frame_sum_us.load(std::memory_order_relaxed);
    context.display.frame_valid = frame_count != 0;
    context.display.frame_peak_ms =
        static_cast<double>(
            diag.frame_peak_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.frame_mean_ms =
        frame_count == 0 ? 0.0
                         : static_cast<double>(frame_sum) /
                               static_cast<double>(frame_count) / 1000.0;
  }
  const std::uint64_t judder_count =
      diag.judder_count.load(std::memory_order_relaxed);
  const std::uint64_t judder_sum =
      diag.judder_sum_dpx.load(std::memory_order_relaxed);
  context.display.judder_valid = judder_count != 0;
  context.display.judder_peak_px =
      static_cast<double>(
          diag.judder_peak_dpx.load(std::memory_order_relaxed)) /
      10.0;
  context.display.judder_avg_px =
      judder_count == 0
          ? 0.0
          : static_cast<double>(judder_sum) /
                static_cast<double>(judder_count) / 10.0;
  {
    const std::uint64_t pitch_count =
        diag.pitch_count.load(std::memory_order_relaxed);
    const std::uint64_t pitch_sum =
        diag.pitch_sum_cdeg.load(std::memory_order_relaxed);
    const std::uint64_t roll_count =
        diag.roll_count.load(std::memory_order_relaxed);
    const std::uint64_t roll_sum =
        diag.roll_sum_cdeg.load(std::memory_order_relaxed);
    const std::uint64_t lens_count =
        diag.lens_count.load(std::memory_order_relaxed);
    const std::uint64_t lens_sum =
        diag.lens_sum_cdeg.load(std::memory_order_relaxed);
    context.display.rotation_valid = pitch_count != 0 && roll_count != 0;
    context.display.pitch_peak_degrees =
        static_cast<double>(
            diag.pitch_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.pitch_avg_degrees =
        pitch_count == 0 ? 0.0
                         : static_cast<double>(pitch_sum) /
                               static_cast<double>(pitch_count) / 100.0;
    context.display.roll_peak_degrees =
        static_cast<double>(
            diag.roll_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.roll_avg_degrees =
        roll_count == 0 ? 0.0
                        : static_cast<double>(roll_sum) /
                              static_cast<double>(roll_count) / 100.0;
    context.display.lens_valid = lens_count != 0;
    context.display.lens_peak_degrees =
        static_cast<double>(
            diag.lens_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.lens_avg_degrees =
        lens_count == 0 ? 0.0
                        : static_cast<double>(lens_sum) /
                              static_cast<double>(lens_count) / 100.0;
  }
  const std::uint64_t judder_car_count =
      diag.judder_car_count.load(std::memory_order_relaxed);
  const std::uint64_t judder_car_sum =
      diag.judder_car_sum_dpx.load(std::memory_order_relaxed);
  context.display.judder_car_valid = judder_car_count != 0;
  context.display.judder_car_peak_px =
      static_cast<double>(
          diag.judder_car_peak_dpx.load(std::memory_order_relaxed)) /
      10.0;
  context.display.judder_car_avg_px =
      judder_car_count == 0
          ? 0.0
          : static_cast<double>(judder_car_sum) /
                static_cast<double>(judder_car_count) / 10.0;
  context.display.probe_height_step_cm = probe.height_step_cm;
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint64_t samples =
        render_x.sample_count.load(std::memory_order_relaxed);
    const std::uint64_t sum_dcm =
        render_x.sum_dcm.load(std::memory_order_relaxed);
    const std::uint64_t sum_dms =
        render_x.sum_dms.load(std::memory_order_relaxed);
    const std::int64_t lead_count =
        render_x.lead_count.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dcm =
        render_x.lead_sum_dcm.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dms =
        render_x.lead_sum_dms.load(std::memory_order_relaxed);
    const std::uint64_t speed_sum_cmps =
        render_x.speed_sum_cmps.load(std::memory_order_relaxed);
    context.display.render_x_state = render_x.resolve_state;
    context.display.render_x_code = render_x.resolve_code;
    context.display.render_x_failures =
        render_x.read_failures.load(std::memory_order_relaxed);
    context.display.render_x_samples = samples;
    context.display.render_x_valid = samples != 0;
    context.display.render_x_peak_cm =
        static_cast<double>(
            render_x.peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_cm =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dcm) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_peak_ms =
        static_cast<double>(
            render_x.peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_ms =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dms) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_speed_kmh =
        samples == 0 ? 0.0
                     : static_cast<double>(speed_sum_cmps) /
                           static_cast<double>(samples) * 0.036;
    context.display.render_x_lead_cm =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dcm) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_lead_ms =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dms) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_chain_cm = render_x.chain_max_cm;
    context.display.render_x_chain_mismatches = render_x.chain_mismatches;
    context.display.render_x_gt_samples =
        render_x.samples_on_game_thread.load(std::memory_order_relaxed);
    const std::uint64_t proxy_count =
        render_x.render_proxy_count.load(std::memory_order_relaxed);
    const std::uint64_t proxy_time_count =
        render_x.render_proxy_time_count.load(std::memory_order_relaxed);
    const std::int64_t proxy_lead_count =
        render_x.render_proxy_lead_count.load(std::memory_order_relaxed);
    const std::uint64_t age_count =
        render_x.render_age_count.load(std::memory_order_relaxed);
    context.display.render_proxy_valid = proxy_count != 0;
    context.display.render_proxy_peak_cm =
        static_cast<double>(
            render_x.render_proxy_peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_cm =
        proxy_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dcm.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_count) / 10.0;
    context.display.render_proxy_peak_ms =
        static_cast<double>(
            render_x.render_proxy_peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_ms =
        proxy_time_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dms.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_time_count) / 10.0;
    context.display.render_proxy_lead_cm =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dcm.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_proxy_lead_ms =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dms.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_age_valid = age_count != 0;
    context.display.render_age_peak_ms =
        static_cast<double>(
            render_x.render_age_peak_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.render_age_avg_ms =
        age_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_age_sum_us.load(std::memory_order_relaxed)) /
                  static_cast<double>(age_count) / 1000.0;
  }
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint32_t source =
        context.anchor_source.load(std::memory_order_acquire);
    const std::uint8_t insurance = render_x.insurance_state;
    const std::uint8_t last_used =
        render_x.last_used.load(std::memory_order_relaxed);
    context.display.anchor_source = source;
    context.display.anchor_fallbacks =
        render_x.anchor_fallbacks.load(std::memory_order_relaxed);
    context.display.anchor_read_failures =
        render_x.anchor_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_anchor_valid =
        render_x.vehicle_anchor_valid.load(std::memory_order_relaxed);
    context.display.vehicle_present =
        context.vehicle_query_known
            ? context.vehicle_query_present
            : render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_presence_known =
        context.vehicle_query_known ||
        render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_proxy_used =
        render_x.vehicle_proxy_used.load(std::memory_order_relaxed);
    context.display.vehicle_render_used =
        render_x.vehicle_render_used.load(std::memory_order_relaxed);
    context.display.transform_hook_valid =
        context.transform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.transform_updates =
        render_x.transform_updates.load(std::memory_order_relaxed);
    context.display.view_uniform_hook_valid =
        context.view_uniform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.view_uniform_calls =
        render_x.view_uniform_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_render_calls =
        render_x.view_uniform_render_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_game_calls =
        render_x.view_uniform_game_calls.load(std::memory_order_relaxed);
    context.display.render_thread_id =
        render_x.render_thread_id.load(std::memory_order_relaxed);
    const std::uint64_t view_age_count =
        render_x.view_uniform_age_count.load(std::memory_order_relaxed);
    context.display.view_uniform_age_valid = view_age_count != 0;
    context.display.view_uniform_age_avg_ms =
        view_age_count == 0
            ? 0.0
            : static_cast<double>(render_x.view_uniform_age_sum_us.load(
                  std::memory_order_relaxed)) /
                  static_cast<double>(view_age_count) / 1000.0;
    context.display.view_uniform_age_peak_ms =
        static_cast<double>(render_x.view_uniform_age_peak_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_applied =
        render_x.route_b_applied.load(std::memory_order_relaxed);
    context.display.route_b_fallbacks =
        render_x.route_b_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_witness_failures =
        render_x.route_b_witness_failures.load(std::memory_order_relaxed);
    context.display.route_b_early_applied =
        render_x.route_b_early_applied.load(std::memory_order_relaxed);
    context.display.route_b_early_fallbacks =
        render_x.route_b_early_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_last_age_ms =
        static_cast<double>(
            render_x.route_b_last_age_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_early_last_age_ms =
        static_cast<double>(render_x.route_b_early_last_age_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.prepare_calls =
        render_x.prepare_calls.load(std::memory_order_relaxed);
    context.display.prepare_thread_mismatch =
        render_x.prepare_thread_mismatch.load(std::memory_order_relaxed);
    context.display.prepare_anchor_off =
        render_x.prepare_anchor_off.load(std::memory_order_relaxed);
    context.display.prepare_no_views =
        render_x.prepare_no_views.load(std::memory_order_relaxed);
    context.display.prepare_pose_fail =
        render_x.prepare_pose_fail.load(std::memory_order_relaxed);
    context.display.prepare_fullscreen_views =
        render_x.prepare_fullscreen_views.load(std::memory_order_relaxed);
    context.display.route_b_early_direct =
        render_x.route_b_early_direct.load(std::memory_order_relaxed);
    context.display.route_b_early_held =
        render_x.route_b_early_held.load(std::memory_order_relaxed);
    context.display.vehicle_fallbacks =
        render_x.vehicle_fallbacks.load(std::memory_order_relaxed);
    context.display.vehicle_read_failures =
        render_x.vehicle_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_proxy_fallbacks =
        render_x.vehicle_proxy_fallbacks.load(std::memory_order_relaxed);
    std::uint8_t gate = kAnchorGateOff;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        gate = kAnchorGateHash;
      } else if (insurance == kAnchorInsuranceServiceMissing) {
        gate = kAnchorGateService;
      } else if (insurance == kAnchorInsuranceReadFailed) {
        gate = kAnchorGateRead;
      } else if (render_x.chain_failed) {
        gate = kAnchorGateOffset;
      } else if (render_x.resolve_state == kRenderXFailed) {
        gate = kAnchorGateResolve;
      } else if (insurance == kAnchorInsuranceReady &&
                 render_x.resolve_state == kRenderXReady) {
        gate = kAnchorGateOk;
      }
    }
    context.display.anchor_gate = gate;
    std::uint8_t effective = kAnchorStateDisabled;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        effective = kAnchorStateHashMismatch;
      } else if (render_x.resolve_state == kRenderXFailed) {
        effective = kAnchorStateResolveFailed;
      } else {
        effective = last_used;
      }
    } else if (source == kAnchorSourceBorrow) {
      effective = kAnchorStateBorrow;
    }
    context.display.anchor_effective = effective;
  }
  const std::uint64_t interval_min_raw =
      diag.tick_interval_min_ns.load(std::memory_order_relaxed);
  context.display.tick_interval_valid = interval_min_raw != UINT64_MAX;
  context.display.tick_interval_min_ms =
      interval_min_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(interval_min_raw) * 1e-6;
  context.display.tick_interval_max_ms =
      static_cast<double>(
          diag.tick_interval_max_ns.load(std::memory_order_relaxed)) *
      1e-6;
  context.display.preset = context.preset.load(std::memory_order_acquire);
  context.display.fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  context.display.camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
  context.display.position_spring =
      context.position_spring.load(std::memory_order_acquire);
  context.display.yaw_spring =
      context.yaw_spring.load(std::memory_order_acquire);
  context.display.drift_yaw_spring =
      context.drift_yaw_spring.load(std::memory_order_acquire);
  context.display.look_yaw_spring =
      context.look_yaw_spring.load(std::memory_order_acquire);
  context.display.yaw_return_spring =
      context.yaw_return_spring.load(std::memory_order_acquire);
  context.display.pitch_spring =
      context.pitch_spring.load(std::memory_order_acquire);
  context.display.roll_spring =
      context.roll_spring.load(std::memory_order_acquire);
  context.display.origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
  context.display.aspect_ratio =
      context.aspect_ratio.load(std::memory_order_acquire);
  context.display.speed_distance_scale =
      context.speed_distance_scale.load(std::memory_order_acquire);
  context.display.roll_scale =
      context.roll_scale.load(std::memory_order_acquire);
  context.display.pitch_scale =
      context.pitch_scale.load(std::memory_order_acquire);
  context.display.shake_scale =
      context.shake_scale.load(std::memory_order_acquire);
  context.display.burnout_enabled =
      context.burnout_enabled.load(std::memory_order_acquire);
  context.display.burnout_entry_delay =
      context.burnout_entry_delay.load(std::memory_order_acquire);
  context.display.burnout_entry_time =
      context.burnout_entry_time.load(std::memory_order_acquire);
  context.display.burnout_exit_delay =
      context.burnout_exit_delay.load(std::memory_order_acquire);
  context.display.burnout_exit_time =
      context.burnout_exit_time.load(std::memory_order_acquire);
  context.display.burnout_launch_exit_time =
      context.burnout_launch_exit_time.load(std::memory_order_acquire);
  context.display.burnout_fov =
      context.burnout_fov.load(std::memory_order_acquire);
  context.display.burnout_shake =
      context.burnout_shake.load(std::memory_order_acquire);
  context.display.burnout_side_mode =
      context.burnout_side_mode.load(std::memory_order_acquire);
  context.display.burnout_entry_speed =
      context.burnout_entry_speed.load(std::memory_order_acquire);
  context.display.burnout_hold_speed =
      context.burnout_hold_speed.load(std::memory_order_acquire);
  context.display.burnout_shot_blend =
      context.presenter_burnout_blend.load(std::memory_order_relaxed);
  context.display.burnout_shot_side =
      context.presenter_burnout_side.load(std::memory_order_relaxed);
  context.display.burnout_shot_fallbacks =
      context.presenter_burnout_fallbacks.load(std::memory_order_relaxed);
  context.display.slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  context.display.slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  context.display.roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  context.display.diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  context.display.diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  context.display.borrow_strength =
      context.borrow_strength.load(std::memory_order_acquire);
  context.display.diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  context.display.diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
}

void RetryHookIfNeeded(Context &context) noexcept {
  const bool view_point_ready =
      context.view_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_point_pattern_unavailable;
  const bool transform_ready =
      context.transform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.transform_pattern_unavailable;
  const bool view_uniform_ready =
      context.view_uniform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_uniform_pattern_unavailable;
  const bool early_route_b_ready =
      context.prepare_view_state_hook_id.load(std::memory_order_acquire) != 0 ||
      context.early_route_b_unavailable;
  if (view_point_ready && transform_ready && view_uniform_ready &&
      early_route_b_ready) {
    return;
  }
  if (context.hook_retry_requested.exchange(false, std::memory_order_acq_rel)) {
    context.next_hook_retry = 0.0;
  }
  if (context.elapsed < context.next_hook_retry) {
    return;
  }
  context.next_hook_retry = context.elapsed + kHookRetrySeconds;
  // The profile hash is the layout witness for the route B hooks. Patterns
  // prove the entry point exists; they cannot prove that FViewInfo or
  // FSceneRenderer still have the fields we write. If the gate is closed,
  // stay out of the game instead of guessing. The legacy view-point hook is
  // kept only when the host genuinely has no build service (2.5).
  const bool version_gate =
      context.render_x.version_gate.load(std::memory_order_acquire);
  const bool legacy_without_build =
      context.render_x.insurance_state == kAnchorInsuranceServiceMissing;
  if (!version_gate && !legacy_without_build) {
    return;
  }
  if (version_gate) {
    static_cast<void>(EnsurePrepareViewStateHook(context));
    static_cast<void>(EnsureTransformHook(context));
    static_cast<void>(EnsureViewUniformHook(context));
  }
  static_cast<void>(EnsureHook(context));
}

void LogViewUniformHookDiagnostics(Context &context) noexcept {
  if (context.view_uniform_diag_logged) {
    return;
  }
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t calls =
      probe.view_uniform_calls.load(std::memory_order_relaxed);
  const std::uint64_t render_calls =
      probe.view_uniform_render_calls.load(std::memory_order_relaxed);
  if (render_calls < 10 && calls < 200) {
    return;
  }
  const std::uint64_t age_count =
      probe.view_uniform_age_count.load(std::memory_order_relaxed);
  const double age_avg_ms =
      age_count == 0
          ? 0.0
          : static_cast<double>(
                probe.view_uniform_age_sum_us.load(std::memory_order_relaxed)) /
                static_cast<double>(age_count) / 1000.0;
  const double age_peak_ms =
      static_cast<double>(
          probe.view_uniform_age_peak_us.load(std::memory_order_relaxed)) /
      1000.0;
  char message[192]{};
  std::snprintf(
      message, sizeof(message),
      "nfs2015 chase cam: view hook calls=%llu render=%llu game=%llu "
      "transform_thread=%u age avg/peak=%.2f/%.2f ms samples=%llu",
      static_cast<unsigned long long>(calls),
      static_cast<unsigned long long>(render_calls),
      static_cast<unsigned long long>(
          probe.view_uniform_game_calls.load(std::memory_order_relaxed)),
      render_calls == 0
          ? 0U
          : probe.render_thread_id.load(std::memory_order_relaxed),
      age_avg_ms, age_peak_ms,
      static_cast<unsigned long long>(age_count));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.view_uniform_diag_logged = true;
}

void LogRouteBDiagnostics(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t applied =
      probe.route_b_applied.load(std::memory_order_relaxed);
  const std::uint64_t early_applied =
      probe.route_b_early_applied.load(std::memory_order_relaxed);
  const std::uint64_t prepare_calls =
      probe.prepare_calls.load(std::memory_order_relaxed);
  if (!context.route_b_early_failure_logged && early_applied == 0 &&
      applied == 0 && prepare_calls >= 120) {
    context.route_b_early_failure_logged = true;
    char warning[320]{};
    std::snprintf(
        warning, sizeof(warning),
        "nfs2015 chase cam: route B early path has not applied yet: "
        "prep=%llu off=%llu nv=%llu pf=%llu fs=%llu dir=%llu mm=%llu fb=%llu",
        static_cast<unsigned long long>(prepare_calls),
        static_cast<unsigned long long>(
            probe.prepare_anchor_off.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_no_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_pose_fail.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_fullscreen_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_direct.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_thread_mismatch.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_fallbacks.load(std::memory_order_relaxed)));
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, warning);
  }
  if (context.route_b_diag_logged) {
    return;
  }
  if (applied + early_applied < 10) {
    return;
  }
  char message[256]{};
  std::snprintf(message, sizeof(message),
                "nfs2015 chase cam: route B early=%llu late=%llu "
                "fallbacks=%llu witness=%llu early_age=%.2f ms "
                "late_age=%.2f ms presenter=%llu direct=%llu held=%llu",
                static_cast<unsigned long long>(early_applied),
                static_cast<unsigned long long>(applied),
                static_cast<unsigned long long>(
                    probe.route_b_early_fallbacks.load(
                        std::memory_order_relaxed) +
                    probe.route_b_fallbacks.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_witness_failures.load(
                        std::memory_order_relaxed)),
                static_cast<double>(probe.route_b_early_last_age_us.load(
                    std::memory_order_relaxed)) /
                    1000.0,
                static_cast<double>(
                    probe.route_b_last_age_us.load(std::memory_order_relaxed)) /
                    1000.0,
                static_cast<unsigned long long>(
                    probe.route_b_early_presenter.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_direct.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_held.load(
                        std::memory_order_relaxed)));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.route_b_diag_logged = true;
}

// TEMP read-only timing report. It only logs values already captured by the
// early/transform/late callbacks; it does not change any camera state.
void LogRouteBTiming(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const auto early = probe.timing_early_count.load(std::memory_order_relaxed);
  if (early == 0 || context.route_b_timing_windows >= 15) {
    return;
  }
  const auto now = GetTickCount64();
  if (context.route_b_timing_last_log_ms != 0 &&
      now - context.route_b_timing_last_log_ms < 2000) {
    return;
  }
  context.route_b_timing_last_log_ms = now;
  ++context.route_b_timing_windows;

  const auto late = probe.timing_late_count.load(std::memory_order_relaxed);
  const auto car_count =
      probe.timing_car_delta_count.load(std::memory_order_relaxed);
  const auto view_count =
      probe.timing_view_delta_count.load(std::memory_order_relaxed);
  const double car_avg =
      car_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_car_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(car_count) / 10.0;
  const double car_peak =
      static_cast<double>(probe.timing_car_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;
  const double view_avg =
      view_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_view_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(view_count) / 10.0;
  const double view_peak =
      static_cast<double>(probe.timing_view_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;

  char line[384]{};
  std::snprintf(
      line, sizeof(line),
      "nfs2015 chase cam: timing early=%llu late=%llu "
      "transform before/between/after=%llu/%llu/%llu "
      "car_delta avg/peak=%.2f/%.2f cm view_delta avg/peak=%.2f/%.2f cm",
      static_cast<unsigned long long>(early),
      static_cast<unsigned long long>(late),
      static_cast<unsigned long long>(
          probe.timing_transform_before.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_between.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_after.load(std::memory_order_relaxed)),
      car_avg, car_peak, view_avg, view_peak);
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, line);

  char sample[384]{};
  std::snprintf(
      sample, sizeof(sample),
      "nfs2015 chase cam: timing sample early_car=(%.2f,%.2f,%.2f) "
      "late_car=(%.2f,%.2f,%.2f) presenter_car=(%.2f,%.2f,%.2f) "
      "early_view=(%.2f,%.2f,%.2f) late_view=(%.2f,%.2f,%.2f)",
      probe.timing_early_car[0].load(std::memory_order_relaxed),
      probe.timing_early_car[1].load(std::memory_order_relaxed),
      probe.timing_early_car[2].load(std::memory_order_relaxed),
      probe.timing_late_car[0].load(std::memory_order_relaxed),
      probe.timing_late_car[1].load(std::memory_order_relaxed),
      probe.timing_late_car[2].load(std::memory_order_relaxed),
      probe.timing_presenter_car[0].load(std::memory_order_relaxed),
      probe.timing_presenter_car[1].load(std::memory_order_relaxed),
      probe.timing_presenter_car[2].load(std::memory_order_relaxed),
      probe.timing_early_view[0].load(std::memory_order_relaxed),
      probe.timing_early_view[1].load(std::memory_order_relaxed),
      probe.timing_early_view[2].load(std::memory_order_relaxed),
      probe.timing_late_view[0].load(std::memory_order_relaxed),
      probe.timing_late_view[1].load(std::memory_order_relaxed),
      probe.timing_late_view[2].load(std::memory_order_relaxed));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, sample);
}

// ---------------------------------------------------------------------------
// Plugin lifecycle
// ---------------------------------------------------------------------------

AnomalyStatusV1 ANOMALY_CALL Load(const AnomalyHostApiV1 *host,
                                  void **plugin_context) {
  if (host == nullptr || plugin_context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  *plugin_context = nullptr;
  auto *context = new (std::nothrow) Context();
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_FAILED);
  }
  context->localizer = nfs2015_chase_cam::Localizer(host);
  context->core = Query<AnomalyCoreServiceV1>(
      host, ANOMALY_CORE_SERVICE_V1_ID, ANOMALY_CORE_SERVICE_V1_VERSION);
  context->config = Query<AnomalyConfigServiceV1>(
      host, ANOMALY_CONFIG_SERVICE_V1_ID, ANOMALY_CONFIG_SERVICE_V1_VERSION);
  context->json = Query<AnomalyJsonServiceV1>(
      host, ANOMALY_JSON_SERVICE_V1_ID, ANOMALY_JSON_SERVICE_V1_VERSION);
  context->input = Query<AnomalyInputServiceV1>(
      host, ANOMALY_INPUT_SERVICE_V1_ID, ANOMALY_INPUT_SERVICE_V1_VERSION);
  context->ui = Query<AnomalyUiServiceV1>(
      host, ANOMALY_UI_SERVICE_V1_ID, ANOMALY_UI_SERVICE_V1_VERSION);
  // 2.6.0 managed windows. Optional: 2.5.0 has no `anomaly.window`, in which
  // case Draw falls back to the direct `anomaly.ui` begin/end pair.
  context->window = Query<AnomalyWindowServiceV1>(
      host, ANOMALY_WINDOW_SERVICE_V1_ID, ANOMALY_WINDOW_SERVICE_V1_VERSION);
  context->signature = Query<AnomalySignatureServiceV1>(
      host, ANOMALY_SIGNATURE_SERVICE_V1_ID,
      ANOMALY_SIGNATURE_SERVICE_V1_VERSION);
  context->hook = Query<AnomalyHookServiceV1>(
      host, ANOMALY_HOOK_SERVICE_V1_ID, ANOMALY_HOOK_SERVICE_V1_VERSION);
  context->session = Query<AnomalyNteSessionServiceV1>(
      host, ANOMALY_NTE_SESSION_SERVICE_V1_ID,
      ANOMALY_NTE_SESSION_SERVICE_V1_VERSION);
  context->player = Query<AnomalyNtePlayerServiceV1>(
      host, ANOMALY_NTE_PLAYER_SERVICE_V1_ID,
      ANOMALY_NTE_PLAYER_SERVICE_V1_VERSION);
  // 2.6's anomaly.nte.vehicle service is gone. The getters it called are still
  // ordinary reflected UFunctions, so resolve them through the generic UE5
  // services instead of guessing from motion.
  context->objects = Query<AnomalyUe5ObjectsServiceV1>(
      host, ANOMALY_UE5_OBJECTS_SERVICE_V1_ID,
      ANOMALY_UE5_OBJECTS_SERVICE_V1_VERSION);
  context->events = Query<AnomalyUe5ProcessEventServiceV1>(
      host, ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_ID,
      ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_VERSION);
  // Optional engine access for the probe. Declared optional in the manifest, so
  // a host without `ue5.ahud` simply leaves this null and the probe reports
  // "off" instead of failing the load.
  context->ahud = Query<AnomalyUe5AhudServiceV1>(
      host, ANOMALY_UE5_AHUD_SERVICE_V1_ID, ANOMALY_UE5_AHUD_SERVICE_V1_VERSION);
  // Version insurance for the X anchor. Optional: a host without the build
  // service leaves the X path disabled and the borrow estimate in charge.
  context->build = Query<AnomalyUe5BuildServiceV1>(
      host, ANOMALY_UE5_BUILD_SERVICE_V1_ID,
      ANOMALY_UE5_BUILD_SERVICE_V1_VERSION);
  // The entities/actors services are deliberately not leased. Every one of
  // their reflection calls (`component_bounds`, `frame`, `page`) runs a
  // name-keyed walk inside the host on every call, and the camera needs none
  // of them: the Pawn root is the vehicle actor while driving, and its
  // position and bounds arrive through the player service's cached snapshots.
  if (!CoreReady(context->core) || !ConfigReady(context->config) ||
      !UiReady(context->ui) || !SignatureReady(context->signature) ||
      !HookReady(context->hook)) {
    delete context;
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE,
                  "required camera services are unavailable");
  }
  const auto schema_status = context->config->register_schema(
      context->config->user, StringView(kSettingsSchemaId),
      kSettingsSchemaVersion, Bytes(kSettingsSchema), &context->settings_schema);
  if (schema_status.code != ANOMALY_STATUS_V1_OK ||
      context->settings_schema.id == 0 || !LoadSettings(*context)) {
    ReleaseSettingsSchema(*context);
    delete context;
    return Status(ANOMALY_STATUS_V1_FAILED, "camera settings are invalid");
  }
  *plugin_context = context;
  return anomaly::sdk::Ok();
}

void RegisterSettingsWindow(Context &context) noexcept {
  if (!WindowReady(context.window) || context.window_handle.id != 0) {
    return;
  }
  try {
    AnomalyWindowSpecV1 spec{};
    spec.struct_size = sizeof(spec);
    spec.flags = 0;
    spec.id = StringView(kSettingsWindowId);
    const std::string title =
        context.localizer.Text("window.title", "NFS 2015 Chase Cam");
    spec.title = StringView(title);
    spec.initial_width = 430.0F;
    spec.initial_height = 720.0F;
    spec.minimum_width = 320.0F;
    spec.minimum_height = 240.0F;
    spec.maximum_width = 900.0F;
    spec.maximum_height = 1200.0F;
    spec.default_open = 1;
    const AnomalyStatusV1 status = context.window->register_window(
        context.window->user, &spec, &context.window_handle);
    if (status.code != ANOMALY_STATUS_V1_OK ||
        context.window_handle.id == 0) {
      context.window_handle = {};
    }
  } catch (...) {
    context.window_handle = {};
  }
}

void ReleaseSettingsWindow(Context &context) noexcept {
  if (context.window_handle.id != 0 && WindowReady(context.window)) {
    static_cast<void>(context.window->release_window(
        context.window->user, context.window_handle));
  }
  context.window_handle = {};
}

AnomalyStatusV1 ANOMALY_CALL Start(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->elapsed = 0.0;
  context->update_rate_window = 0.0;
  context->update_rate_frames = 0;
  context->update_hz = 0.0;
  context->view_output_hz = 0.0;
  context->interpolation_ratio = 0.0;
  context->view_output_frames.store(0, std::memory_order_relaxed);
  context->configured_preset =
      context->preset.load(std::memory_order_acquire);
  context->chase.SetConfig(context->configured_preset == 0
                               ? nfs2015_chase::GamePreset()
                               : nfs2015_chase::CinematicPreset());
  context->reset_requested.store(true, std::memory_order_release);
  // Signature learning is a Lifecycle-domain operation; cache the target here
  // so the Game tick only has to create the hook.
  std::uintptr_t view_point_target = 0;
  if (ResolveSignature(*context, kCameraViewPointPattern, view_point_target)) {
    context->hook_target.store(view_point_target, std::memory_order_release);
  }
  ResetMotion(*context);
  context->wheel_steer = WheelSteerState{};
  context->vehicle_present_last = false;
  context->last_vehicle_present_time = 0.0;
  context->vehicle_getter_handles = {};
  context->next_vehicle_getter_retry = {};
  context->next_vehicle_query_time = 0.0;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->observed_valid.store(false, std::memory_order_release);
  PublishInactive(*context);
  // T0: a new session starts from a clean counter baseline so the recorded
  // numbers describe this run only.
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    context->diag.snap_by_reason[index].store(0, std::memory_order_relaxed);
  }
  context->diag.yaw_source_switches.store(0, std::memory_order_relaxed);
  context->diag.observed_read_failures.store(0, std::memory_order_relaxed);
  context->diag.lease_failures.store(0, std::memory_order_relaxed);
  context->diag.view_read_torn.store(0, std::memory_order_relaxed);
  context->diag.detour_calls.store(0, std::memory_order_relaxed);
  context->diag.detour_calls_window.store(0, std::memory_order_relaxed);
  context->diag.detour_no_publish.store(0, std::memory_order_relaxed);
  context->diag.player_sequence_jumps.store(0, std::memory_order_relaxed);
  context->diag.camera_yaw_esp_misses.store(0, std::memory_order_relaxed);
  context->diag.slip_from_position_degrees.store(0.0,
                                                 std::memory_order_relaxed);
  context->diag.slip_sign_changes.store(0, std::memory_order_relaxed);
  context->diag.slip_last_sign.store(0, std::memory_order_relaxed);
  context->diag.slip_max_step_ddeg.store(0, std::memory_order_relaxed);
  context->last_slip_degrees = 0.0;
  context->have_last_slip = false;
  context->diag.render_speed_floor_mmps.store(UINT64_MAX,
                                              std::memory_order_relaxed);
  context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
  context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                             std::memory_order_relaxed);
  context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
  context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
  context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                           std::memory_order_relaxed);
  context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
  context->camera_cost.Reset();
  context->camera_ms_last = 0.0;
  // A fresh session re-runs the P0 probe from scratch. Stop already dropped the
  // old subscription, so nothing can be in flight while this resets.
  context->probe = Probe{};
  // Task group 1: check the profile hash, then resolve `ue5.GWorld` from the
  // Lifecycle domain where signature learning is allowed. The Game tick
  // retries a failed pattern scan, but never bypasses the hash gate.
  context->render_x.Reset();
  EvaluateAnchorInsurance(*context);
  context->detour_hz = 0.0;
  context->brake_last_target = 0.0;
  static_cast<void>(RegisterToggleHotkey(*context));
  RegisterSettingsWindow(*context);
  // Install the hook from the Game tick, not from this lifecycle callback: the
  // target function is called on the Game thread, so create/enable cannot
  // re-enter the detour before the original pointer is published.
  context->next_hook_retry = 0.0;
  context->started.store(true, std::memory_order_release);
  return anomaly::sdk::Ok();
}

AnomalyStatusV1 ANOMALY_CALL Stop(void *plugin_context,
                                  std::uint32_t /*deadline_milliseconds*/) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->started.store(false, std::memory_order_release);
  // Put the front wheels back to their stock relative rotation before the
  // vehicle getter caches are dropped. If the game is already tearing down the
  // car the writes fail harmlessly; a normal stop restores the visual pose.
  ReleaseWheelSteer(*context);
  // Both callbacks check `started` or `g_active` first, so dropping these two
  // before any release means the revocation below has nothing to wait for.
  // Draining a callback that is still running the model is what turns a stop
  // into a timeout, and a timeout into a quarantined plugin that the Host will
  // not load again until the process restarts.
  g_active.store(nullptr, std::memory_order_release);
  context->chase_initialized = false;
  ++context->sample_epoch;
  context->sample_mean_interval_ns.store(0.0, std::memory_order_relaxed);
  context->game_offset_valid = false;
  context->game_offset_last_ns = 0;
  context->car_ring.Clear();
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
  ReleaseProbeSubscription(*context);
  ReleaseSettingsWindow(*context);
  ReleaseHook(*context);
  if (context->settings_revision.load(std::memory_order_acquire) !=
      context->persisted_settings_revision.load(std::memory_order_acquire)) {
    static_cast<void>(PersistSettings(*context));
  }
  std::scoped_lock lock(context->display_mutex);
  context->display.started = false;
  context->display.hook_ready = false;
  context->display.active = false;
  context->display.driving = false;
  context->display.reversing = false;
  context->display.speed_source = kSpeedSourceUnavailable;
  context->display.speed_sign_known = false;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->next_vehicle_query_time = 0.0;
  return anomaly::sdk::Ok();
}

void ANOMALY_CALL Unload(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  static_cast<void>(Stop(context, 0));
  ReleaseSettingsSchema(*context);
  delete context;
}

void ANOMALY_CALL Update(void *plugin_context, const double delta_seconds) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  try {
    if (!context->started.load(std::memory_order_acquire)) {
      UpdateDisplay(*context);
      return;
    }
    const double dt =
        std::isfinite(delta_seconds)
            ? nfs2015_chase::Clamp(delta_seconds, 0.0, 0.25)
            : 0.0;
    context->elapsed += dt;
    if (dt > 0.0) {
      context->update_rate_window += dt;
      ++context->update_rate_frames;
      if (context->update_rate_window >= kUpdateRateWindowSeconds) {
        context->update_hz =
            static_cast<double>(context->update_rate_frames) /
            context->update_rate_window;
        const std::uint64_t outputs =
            context->view_output_frames.exchange(0, std::memory_order_relaxed);
        // Every detour call that produced a pose is now a render-clock model
        // step, so the "interpolated" share is simply the presented share.
        const std::uint64_t interpolated = outputs;
        context->view_output_hz =
            static_cast<double>(outputs) / context->update_rate_window;
        context->interpolation_ratio =
            outputs != 0
                ? static_cast<double>(interpolated) /
                      static_cast<double>(outputs)
                : 0.0;
        // Q1/Q4: how often the presenter is actually asked for a pose, next
        // to the Game tick rate. These two should track the render rate once
        // T2 moves the model off the Game tick.
        const std::uint64_t detour_calls =
            context->diag.detour_calls_window.exchange(
                0, std::memory_order_relaxed);
        context->detour_hz =
            static_cast<double>(detour_calls) / context->update_rate_window;
        context->diag.render_speed_floor_mmps.store(
            UINT64_MAX, std::memory_order_relaxed);
        context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
        context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                                   std::memory_order_relaxed);
        context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
        context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
        context->probe.camera_yaw_step_max_ddeg = 0.0;
        context->probe.camera_yaw_step_sum_ddeg = 0.0;
        context->probe.camera_yaw_step_count = 0;
        context->probe.height_step_cm = 0.0;
        context->probe.track_anchor.count = 0;
        context->probe.eye_count = 0;
        context->probe.track_car.count = 0;
        context->diag.judder_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_count.store(0, std::memory_order_relaxed);
        context->diag.judder_car_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_count.store(0, std::memory_order_relaxed);
        context->diag.reset_window.store(0, std::memory_order_relaxed);
        ResetResidual(context->diag.flow_peak_ccm,
                      context->diag.flow_sum_ccm,
                      context->diag.flow_count);
        ResetResidual(context->diag.frame_peak_us,
                      context->diag.frame_sum_us,
                      context->diag.frame_count);
        ResetResidual(context->diag.pitch_peak_cdeg,
                      context->diag.pitch_sum_cdeg,
                      context->diag.pitch_count);
        ResetResidual(context->diag.roll_peak_cdeg,
                      context->diag.roll_sum_cdeg,
                      context->diag.roll_count);
        ResetResidual(context->diag.lens_peak_cdeg,
                      context->diag.lens_sum_cdeg,
                      context->diag.lens_count);
        context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                                 std::memory_order_relaxed);
        context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
        context->render_x.ResetWindow();
        context->diag.slip_max_step_ddeg.store(0,
                                               std::memory_order_relaxed);
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    RetryHookIfNeeded(*context);
    LogViewUniformHookDiagnostics(*context);
    LogRouteBDiagnostics(*context);
    LogRouteBTiming(*context);
    static_cast<void>(ResolveCameraPov(*context));
    // T0: time the Game-tick work that the camera itself owns. The host
    // callback as a whole also persists settings and resolves the POV, so only
    // UpdateCamera is bracketed here.
    const std::int64_t camera_begin_ns = NowNanoseconds();
    UpdateCamera(*context, dt);
    context->camera_ms_last =
        static_cast<double>(NowNanoseconds() - camera_begin_ns) * 1e-6;
    context->camera_cost.Add(context->camera_ms_last);
    // Probe: read-only reconnaissance, one Game tick at a time. It runs after
    // UpdateCamera so the yaw it compares is this tick's sample.
    RunProbe(*context, dt);
    UpdateDisplay(*context);
    const auto revision =
        context->settings_revision.load(std::memory_order_acquire);
    if (revision !=
        context->persisted_settings_revision.load(std::memory_order_acquire)) {
      const auto now = GetTickCount64();
      const auto changed_at =
          context->settings_changed_at.load(std::memory_order_acquire);
      if (now - changed_at >= kSettingsSaveDelayMilliseconds) {
        if (!PersistSettings(*context)) {
          // Back off instead of retrying a failing config write every tick.
          context->settings_changed_at.store(now, std::memory_order_release);
        }
      }
    }
  } catch (...) {
    // Host callback boundary: plugin exceptions must not escape.
  }
}

}  // namespace nfs2015_chase_cam::detail

ANOMALY_SDK_EXPORT AnomalyStatusV1 ANOMALY_CALL AnomalyPluginEntryV1(
    AnomalyPluginDescriptorV1 *descriptor) {
  if (descriptor == nullptr ||
      descriptor->struct_size < sizeof(*descriptor)) {
    return {ANOMALY_STATUS_V1_INVALID_ARGUMENT, 0, {}};
  }
  using namespace nfs2015_chase_cam::detail;
  *descriptor = {
      sizeof(*descriptor),
      ANOMALY_PLUGIN_API_V1_MAJOR,
      ANOMALY_PLUGIN_API_V1_MINOR,
      StringView(kPluginId),
      StringView("NFS 2015 Chase Cam"),
      StringView("RobinEatCorn"),
      StringView("2.0.3"),
      Load,
      Start,
      Stop,
      Unload,
      Update,
      Draw};
  return anomaly::sdk::Ok();
}
