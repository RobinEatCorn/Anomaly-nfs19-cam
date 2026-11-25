// Game-thread camera path (2.1.0). The detours run on the engine's own
// GetViewPoint call chain, so the chase model and the car share the same tick.
// Nothing here writes a game object: the only writes are the caller's
// FMinimalViewInfo fields and the frame-gate fields in our own Context.
#include "internal.hpp"
#include "view_point.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace nfs2015_chase_cam::detail {

namespace {

Context *ActiveContext() noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  return context;
}

bool BeginViewPointLease(Context &context,
                         AnomalyGenerationHandleV1 &lease) noexcept {
  const std::uint64_t hook_id =
      context.game_view_point_hook_id.load(std::memory_order_acquire);
  if (hook_id == 0 || !HookReady(context.hook)) {
    return false;
  }
  const AnomalyGenerationHandleV1 handle{
      hook_id,
      context.game_view_point_hook_generation.load(std::memory_order_acquire)};
  bool leased = false;
  try {
    leased = context.hook
                 ->begin_callback(context.hook->user, handle, &lease)
                 .code == ANOMALY_STATUS_V1_OK;
  } catch (...) {
    leased = false;
  }
  if (!leased) {
    context.diag.lease_failures.fetch_add(1, std::memory_order_relaxed);
  }
  return leased;
}

bool BeginSceneViewInitLease(Context &context,
                             AnomalyGenerationHandleV1 &lease) noexcept {
  const std::uint64_t hook_id =
      context.scene_view_init_hook_id.load(std::memory_order_acquire);
  if (hook_id == 0 || !HookReady(context.hook)) {
    return false;
  }
  const AnomalyGenerationHandleV1 handle{
      hook_id, context.scene_view_init_hook_generation.load(
                   std::memory_order_acquire)};
  bool leased = false;
  try {
    leased = context.hook
                 ->begin_callback(context.hook->user, handle, &lease)
                 .code == ANOMALY_STATUS_V1_OK;
  } catch (...) {
    leased = false;
  }
  if (!leased) {
    context.diag.lease_failures.fetch_add(1, std::memory_order_relaxed);
  }
  return leased;
}

void EndLease(Context &context, AnomalyGenerationHandleV1 lease,
              bool leased) noexcept {
  if (leased && context.hook != nullptr &&
      context.hook->end_callback != nullptr) {
    static_cast<void>(
        context.hook->end_callback(context.hook->user, lease));
  }
}

// Task group 6: model a short gap in the live ComponentToWorld read. The
// last successful scene sample still carries the Game-tick velocity and yaw
// rate; advance those facts on the scene clock for at most two frames. This is
// not a second smoothing layer: it keeps the camera on the car's known
// trajectory while the transform read is unavailable, instead of freezing or
// snapping to the game camera. A longer gap is a hitch/stream/pause; it
// requests one cut and drops the history.
bool ExtrapolateSceneCarSample(Context &context,
                               const std::int64_t now_ns,
                               nfs2015_chase::CarSample &sample) noexcept {
  if (!context.last_scene_car_valid) {
    return false;
  }
  const nfs2015_chase::CarSample &last = context.last_scene_car_sample;
  if (last.epoch != context.sample_epoch) {
    return false;
  }
  const std::int64_t age_ns = now_ns - last.time_ns;
  if (age_ns <= 0) {
    return false;
  }
  if (age_ns > kSceneExtrapolationMaxAgeNs ||
      context.scene_extrapolation_frames >= kSceneExtrapolationMaxFrames) {
    NoteViewSnap(context, kSnapReasonMotion);
    context.diag.extrapolation_timeouts.fetch_add(1,
                                                  std::memory_order_relaxed);
    context.last_scene_car_valid = false;
    context.scene_extrapolation_frames = 0;
    return false;
  }
  const double dt = static_cast<double>(age_ns) * 1e-9;
  sample = last;
  sample.time_ns = now_ns;
  sample.epoch = context.sample_epoch;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double position =
        axis == 0 ? last.car.position.x
                  : (axis == 1 ? last.car.position.y : last.car.position.z);
    const double velocity =
        axis == 0 ? last.car.velocity.x
                  : (axis == 1 ? last.car.velocity.y : last.car.velocity.z);
    const double step = velocity * dt;
    const double next = position + step;
    if (axis == 0) {
      sample.car.position.x = next;
    } else if (axis == 1) {
      sample.car.position.y = next;
    } else {
      sample.car.position.z = next;
    }
    // The anchor and origin are world centimetres, while CarState::position
    // and velocity are metres.
    sample.anchor_cm[axis] += step * 100.0;
    sample.origin_cm[axis] += step * 100.0;
  }
  sample.car.yaw =
      nfs2015_chase::WrapPi(last.car.yaw + last.car.yaw_rate * dt);
  context.scene_extrapolation_frames += 1U;
  context.diag.extrapolated_frames.fetch_add(1, std::memory_order_relaxed);
  return true;
}

// Task group 5: build the sample the model consumes at the scene-view
// advance. The last completed Game-tick sample supplies the derivative state
// (velocity, acceleration, yaw rate, brake, burnout, rig geometry); the live
// VehicleMesh ComponentToWorld supplies the position, yaw and body pitch. This
// is the transform after the vehicle has run, so the camera and the car about
// to be drawn share it; there is no render-clock interpolation.
bool ReadGameThreadCarSample(Context &context, const std::int64_t now_ns,
                             nfs2015_chase::CarSample &sample,
                             bool &extrapolated) noexcept {
  extrapolated = false;
  if (!ReadLatestCarSample(context, sample)) {
    extrapolated = ExtrapolateSceneCarSample(context, now_ns, sample);
    return extrapolated;
  }
  double car_cm[3]{};
  double forward[3]{};
  if (!ReadVehicleComponentTransform(context, car_cm, forward)) {
    extrapolated = ExtrapolateSceneCarSample(context, now_ns, sample);
    return extrapolated;
  }
  if (!std::isfinite(car_cm[0]) || !std::isfinite(car_cm[1]) ||
      !std::isfinite(car_cm[2]) || !std::isfinite(forward[0]) ||
      !std::isfinite(forward[1]) || !std::isfinite(forward[2])) {
    extrapolated = ExtrapolateSceneCarSample(context, now_ns, sample);
    return extrapolated;
  }
  double pitch = 0.0;
  if (!BodyPitchFromForward(forward, pitch)) {
    extrapolated = ExtrapolateSceneCarSample(context, now_ns, sample);
    return extrapolated;
  }
  if (!std::isfinite(now_ns) || now_ns <= 0) {
    return false;
  }
  // D30: the teleport check runs on consecutive scene-phase car transforms,
  // not on the on_update position. If the physics moved the car this frame,
  // the cut must be produced in this frame, before CalcSceneViewInitOptions
  // returns, so the engine can apply it to the view being built.
  if (context.last_scene_car_valid &&
      context.last_scene_car_sample.epoch == sample.epoch) {
    const std::int64_t age_ns = now_ns - context.last_scene_car_sample.time_ns;
    if (age_ns > 0) {
      double squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double delta =
            car_cm[axis] - context.last_scene_car_sample.anchor_cm[axis];
        squared += delta * delta;
      }
      const double step = std::sqrt(squared);
      const double dt = static_cast<double>(age_ns) * 1e-9;
      if (!std::isfinite(step) || step > kMaximumTeleportStepCm ||
          (dt > 0.0 &&
           step / dt > kMaximumPlausibleSpeedCmps)) {
        NoteViewSnap(context, kSnapReasonTeleport);
      }
    }
  }
  // The derivative facts come from the Game tick, but this pose is evaluated
  // at the scene-view phase, when the VehicleMesh ComponentToWorld is the
  // transform the renderer is about to draw. Advancing on this timestamp keeps
  // the model's step tied to the car step that just happened.
  sample.time_ns = now_ns;
  sample.car.position = {car_cm[0] / 100.0, car_cm[1] / 100.0,
                         car_cm[2] / 100.0};
  sample.car.yaw = std::atan2(forward[1], forward[0]);
  sample.car.pitch = pitch;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    sample.anchor_cm[axis] = car_cm[axis];
    sample.origin_cm[axis] = car_cm[axis];
  }
  // Keep the derivative facts from the Game tick, but express the signed speed
  // and slip in the freshly read body frame. The old sample was built with the
  // previous frame's yaw; mixing the two frames would tilt the slip angle by
  // the yaw change even though the car did not slip.
  const nfs2015_chase::Vec3 velocity = sample.car.velocity;
  const nfs2015_chase::Vec3 body_forward =
      nfs2015_chase::ForwardFromYaw(sample.car.yaw);
  const nfs2015_chase::Vec3 body_right =
      nfs2015_chase::RightFromYaw(sample.car.yaw);
  const double forward_speed =
      nfs2015_chase::Dot(velocity, body_forward);
  const double speed = std::hypot(velocity.x, velocity.y);
  sample.car.speed = forward_speed;
  if (speed < 0.75) {
    sample.car.slip_angle = 0.0;
  } else if (forward_speed >= 0.0) {
    sample.car.slip_angle = nfs2015_chase::Clamp(
        std::atan2(nfs2015_chase::Dot(velocity, body_right), forward_speed),
        -nfs2015_chase::kPi * 0.5, nfs2015_chase::kPi * 0.5);
  }
  sample.driving = true;
  sample.epoch = context.sample_epoch;
  context.last_scene_car_sample = sample;
  context.last_scene_car_valid = true;
  context.scene_extrapolation_frames = 0;
  return true;
}

void WritePose(Context &context, MinimalViewInfo &view,
               const double location[3], const double rotation[3],
               float lens, bool fov_enabled) noexcept {
  const bool skip_position =
      context.diagnostic_skip_position.load(std::memory_order_relaxed);
  const bool skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_relaxed);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!skip_position) {
      view.location[axis] = location[axis];
    }
    if (!skip_rotation) {
      view.rotation[axis] = rotation[axis];
    }
  }
  // The game's GetViewPoint sets DesiredFOV = FOV at the end. Write both so the
  // projection matrix built from this view uses the same lens as the chase
  // model; writing only FOV would leave the engine's idea of the requested lens
  // at the game value.
  if (fov_enabled && std::isfinite(lens) && lens > 1.0F && lens < 179.0F) {
    view.fov = lens;
    view.desired_fov = lens;
  }
  context.diag.view_applies.fetch_add(1, std::memory_order_relaxed);
}

// Task group 6: consume one pending cut after the game has written its own
// bInCameraCut. The field is a plain bool in FSceneViewInitOptions; use byte
// copies so this stays independent of the C++ bool ABI at an opaque offset.
// If the original did not produce a view, keep the request for the next one.
void ApplyPendingCut(Context &context, void *out, const bool result) noexcept {
  if (!context.pending_cut.load(std::memory_order_acquire)) {
    return;
  }
  if (!result || out == nullptr) {
    context.diag.cut_skipped_no_view.fetch_add(1,
                                               std::memory_order_relaxed);
    return;
  }
  if (!context.view_active.load(std::memory_order_acquire) ||
      !ChaseCameraGuardAllows(context)) {
    // The camera is inactive or the game is in a photo/cutscene slot. Keep the
    // request pending: the first view after re-entry is the one that needs it.
    return;
  }
  if (!context.pending_cut.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  auto *field = reinterpret_cast<std::uint8_t *>(out) +
                kSceneViewInitOptionsCameraCutOffset;
  std::uint8_t current = 0;
  std::memcpy(&current, field, sizeof(current));
  if (current == 0) {
    const std::uint8_t cut = 1;
    std::memcpy(field, &cut, sizeof(cut));
  }
  if (context.diag.cut_applied.fetch_add(1, std::memory_order_relaxed) == 0) {
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
        std::string_view("nfs2015 chase cam: first camera cut applied"));
  }
  // The snap reason has been delivered to the engine. Clear it so the
  // diagnostic state describes pending work only.
  context.view_snap = false;
  context.view_snap_reasons = 0;
}

}  // namespace

bool AdvanceScenePose(Context &context, const std::int64_t now_ns) noexcept {
  if (!context.view_active.load(std::memory_order_acquire)) {
    // D29: no published pose may survive an inactive interval. The early UI
    // on the next active frame must fail open until the scene has advanced.
    InvalidatePresenterPose(context);
    ClearSceneCarTracking(context);
    return false;
  }
  if (!ChaseCameraGuardAllows(context)) {
    context.diag.tag_veto_frames.fetch_add(1, std::memory_order_relaxed);
    InvalidatePresenterPose(context);
    ClearSceneCarTracking(context);
    return false;
  }
  if (context.pose_attempted.exchange(true, std::memory_order_acq_rel)) {
    // A second scene view in the same frame (stereo) reuses the first pose.
    return context.pose_ready.load(std::memory_order_acquire);
  }
  context.pose_ready.store(false, std::memory_order_release);
  nfs2015_chase::CarSample sample;
  bool extrapolated = false;
  if (!ReadGameThreadCarSample(context, now_ns, sample, extrapolated)) {
    // The scene advance failed. Invalidate the previous pose so the next
    // frame's early UI cannot mix it with a scene that has fallen back to the
    // game camera.
    InvalidatePresenterPose(context);
    context.diag.detour_no_publish.fetch_add(1,
                                             std::memory_order_relaxed);
    return false;
  }
  double location[3]{};
  double rotation[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  bool advanced = false;
  if (!AdvancePresenter(context, now_ns, sample, location, rotation,
                        &lens, &fov_enabled, &advanced)) {
    InvalidatePresenterPose(context);
    context.diag.detour_no_publish.fetch_add(1,
                                             std::memory_order_relaxed);
    return false;
  }
  context.pose_ready.store(true, std::memory_order_release);
  context.view_output_frames.fetch_add(1, std::memory_order_relaxed);
  SamplePresenterMotion(context, now_ns, location, rotation, advanced);
  (void)extrapolated;
  return true;
}

bool ApplyViewPoint(Context &context, MinimalViewInfo &view) noexcept {
  if (!context.view_active.load(std::memory_order_acquire)) {
    return false;
  }
  if (!ChaseCameraGuardAllows(context)) {
    context.diag.tag_veto_frames.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  // The scene view is the phase that owns the advance (called from
  // CalcSceneViewInitOptions before the original). Every consumer before that
  // point, including the UMG/Slate projection in World Tick, uses the pose the
  // previous scene view published, because its car transform is also the
  // previous frame's final transform. Once the scene pose is ready, the scene
  // view and the HUD after it copy it. If the scene advance failed this frame,
  // the scene side fails open to the game camera instead of mixing a stale
  // pose with the current car.
  const bool scene_phase =
      context.view_build_depth.load(std::memory_order_acquire) > 0;
  const bool attempted =
      context.pose_attempted.load(std::memory_order_acquire);
  const bool ready = context.pose_ready.load(std::memory_order_acquire);
  if ((scene_phase && !ready) || (attempted && !ready)) {
    // The scene advance failed. Do not let the HUD or any later consumer mix
    // the previous pose with this frame's car; the whole post-scene frame
    // fails open to the game camera.
    context.diag.query_skips.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  double location[3]{};
  double rotation[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  if (!ReadPresenterPose(context, location, rotation, &lens, &fov_enabled)) {
    context.diag.detour_no_publish.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  WritePose(context, view, location, rotation, lens, fov_enabled);
  return true;
}

void ANOMALY_CALL LocalPlayerViewPointDetour(const void *self,
                                             void *out) noexcept {
  Context *context = ActiveContext();
  if (context != nullptr) {
    context->diag.detour_calls.fetch_add(1, std::memory_order_relaxed);
    context->diag.detour_calls_window.fetch_add(1,
                                               std::memory_order_relaxed);
    NoteDetourThread(*context);
  }

  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  if (context != nullptr) {
    leased = BeginViewPointLease(*context, lease);
  }

  LocalPlayerGetViewPointFn original =
      g_local_player_get_view_point_original.load(std::memory_order_acquire);
  bool original_called = false;
  try {
    if (original != nullptr) {
      original(self, out);
      original_called = true;
    }
  } catch (...) {
  }

  if (context != nullptr && out != nullptr && original_called && leased) {
    auto &view = *static_cast<MinimalViewInfo *>(out);
    try {
      static_cast<void>(ApplyViewPoint(*context, view));
    } catch (...) {
    }
  }

  if (context != nullptr) {
    EndLease(*context, lease, leased);
  }
}

bool ANOMALY_CALL CalcSceneViewInitOptionsDetour(
    void *self, void *out, void *viewport, void *drawer,
    std::int32_t stereo) noexcept {
  Context *context = ActiveContext();
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  if (context != nullptr) {
    leased = BeginSceneViewInitLease(*context, lease);
    context->view_build_depth.fetch_add(1, std::memory_order_acq_rel);
    // This is the first point in the frame where the vehicle's
    // ComponentToWorld is the transform the renderer is about to draw. Advance
    // before the original so both GetProjectionData and the second direct
    // GetViewPoint inside CalcSceneView see this frame's pose. The earlier UMG
    // projection uses the previous frame's published pose, which matches the
    // previous-frame car transform it sees in TG_PrePhysics.
    if (leased) {
      AdvanceScenePose(*context, NowNanoseconds());
    }
  }

  bool result = false;
  try {
    CalcSceneViewInitOptionsFn original =
        g_calc_scene_view_init_options_original.load(
            std::memory_order_acquire);
    if (original != nullptr) {
      result = original(self, out, viewport, drawer, stereo);
    }
  } catch (...) {
  }

  if (context != nullptr) {
    // Write the cut after the original returned, so the game's own
    // bInCameraCut value is preserved and ORed with ours. This is the same
    // field the engine reads in LocalPlayer.cpp:767.
    ApplyPendingCut(*context, out, result);
    context->view_build_depth.fetch_sub(1, std::memory_order_acq_rel);
  }
  if (context != nullptr) {
    EndLease(*context, lease, leased);
  }
  return result;
}

}  // namespace nfs2015_chase_cam::detail
