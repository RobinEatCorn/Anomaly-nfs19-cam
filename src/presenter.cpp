// The T2 presenter feed and the camera integration it drives. This block turns
// the model state into the pose the detour serves, through a seqlock the Game
// thread writes and the detour thread reads.
#include "internal.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>

namespace nfs2015_chase_cam::detail {

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
    if (!enabled) {
      // Plan SS2.3.3 "plugin disabled": the chase wish and any pending
      // entryView die with it, and the detour passes Z straight through.
      NoteVehicleExit(context);
      // Plan SS2.3.3: enabling while driving counts as a fresh entry, so the
      // gap saturates here. `+= dt` would make a quick off/on (the switch
      // flipped twice inside the threshold) silently leave the chase off.
      context.view_off_seconds = kChaseEntryGapSeconds;
    }
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
    // Plan SS2.3.3 "left the vehicle": overlay off, chase wish off.
    NoteVehicleExit(context);
    context.view_off_seconds += dt;
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

  // The chase slot. `in_vehicle` is the answer the toggle detour gates on.
  //
  // Entering is debounced (`kChaseEntryGapSeconds`): the vehicle getter and the
  // pawn chain both drop out for a tick now and then, and a gap must not look
  // like "got out and back in" - that would re-apply `entryView` and bring the
  // chase camera back seconds after the player dismissed it with Z. Losing the
  // vehicle for long enough re-arms it, which is also what makes enabling the
  // plugin while driving behave like a fresh entry (plan SS2.3.3).
  context.view_mode.in_vehicle.store(true, std::memory_order_release);
  if (context.view_off_seconds >= kChaseEntryGapSeconds) {
    ApplyEntryView(context);
  } else if (context.view_mode.pending_entry) {
    // A cutscene (Fixed / Seat) held the camera when the vehicle was entered;
    // retry every tick until the game reports Third or First again.
    ApplyEntryView(context);
  }
  context.view_off_seconds = 0.0;

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
    // The sample above is published in every slot, not only the chase one: the
    // spring model has to be warm when FP -> chase flips this flag inside the
    // toggle call, or the pose would start from a stale anchor.
    //
    // `view_active` is the whole difference between "chase camera" and "plain
    // third person", because both have the same game tag. It is true only while
    // the game really is in ThirdPerson (plan SS2.4).
    const bool chase_slot = ChaseSlotAllowed(context);
    if (chase_slot) {
      // Without this the game camera shows through.
      ActivateView(context, fov_enabled);
    } else if (context.last_view_active) {
      // Restores the game lens, notes the snap and bumps the view sequence, so
      // the pose is rebuilt from the car rather than blended from the old one.
      PublishInactive(context);
    }
    if (chase_slot != context.view_slot_chase_last) {
      context.view_slot_chase_last = chase_slot;
      char message[192]{};
      std::snprintf(message, sizeof(message),
                    "viewmode slot: chase=%d tag=%s wanted=%d veh=1 en=%d",
                    chase_slot ? 1 : 0,
                    ViewModeTagLabel(
                        context,
                        context.view_mode.published_tag.load(
                            std::memory_order_relaxed)),
                    context.view_mode.chase_wanted ? 1 : 0, enabled ? 1 : 0);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
    }
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
    context.display.wheel_steer_force_turn = context.wheel_steer.force_turn;
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
    context.display.wheel_steer_force_turn_delay_seconds =
        context.wheel_steer_force_turn_delay_seconds.load(
            std::memory_order_relaxed);
    context.display.wheel_steer_sample_percent =
        context.wheel_steer_sample_percent.load(std::memory_order_relaxed);
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

}  // namespace nfs2015_chase_cam::detail
