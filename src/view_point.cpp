// Game-thread camera path (2.1.0). The detours run on the engine's own
// GetViewPoint call chain, so the chase model and the car share the same tick.
// Nothing here writes a game object: the only writes are the caller's
// FMinimalViewInfo fields and the frame-gate fields in our own Context.
#include "internal.hpp"
#include "view_point.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

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

void PublishObservedView(const MinimalViewInfo &view,
                         Context &context) noexcept {
  const double *location = view.location;
  const double *rotation = view.rotation;
  const bool finite =
      std::isfinite(location[0]) && std::isfinite(location[1]) &&
      std::isfinite(location[2]) && std::isfinite(rotation[0]) &&
      std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
  if (!finite) {
    return;
  }
  const std::uint64_t sequence =
      context.observed_sequence.load(std::memory_order_relaxed);
  context.observed_sequence.store(sequence + 1U,
                                  std::memory_order_release);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.observed_location[axis].store(location[axis],
                                          std::memory_order_relaxed);
    context.observed_rotation[axis].store(rotation[axis],
                                          std::memory_order_relaxed);
  }
  context.observed_valid.store(true, std::memory_order_relaxed);
  context.observed_sequence.store(sequence + 2U,
                                  std::memory_order_release);
}

// Task group 5: build the sample the model consumes at the scene-view
// advance. The last completed Game-tick sample supplies the derivative state
// (velocity, acceleration, yaw rate, brake, burnout, rig geometry); the live
// VehicleMesh ComponentToWorld supplies the position, yaw and body pitch. This
// is the transform after the vehicle has run, so the camera and the car about
// to be drawn share it; there is no render-clock interpolation.
bool ReadGameThreadCarSample(Context &context, const std::int64_t now_ns,
                             nfs2015_chase::CarSample &sample) noexcept {
  if (!ReadLatestCarSample(context, sample)) {
    return false;
  }
  double car_cm[3]{};
  double forward[3]{};
  if (!ReadVehicleComponentTransform(context, car_cm, forward)) {
    return false;
  }
  if (!std::isfinite(car_cm[0]) || !std::isfinite(car_cm[1]) ||
      !std::isfinite(car_cm[2]) || !std::isfinite(forward[0]) ||
      !std::isfinite(forward[1]) || !std::isfinite(forward[2])) {
    return false;
  }
  double pitch = 0.0;
  if (!BodyPitchFromForward(forward, pitch)) {
    return false;
  }
  if (!std::isfinite(now_ns) || now_ns <= 0) {
    return false;
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
  const double speed =
      std::hypot(velocity.x, velocity.y);
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

}  // namespace

bool AdvanceScenePose(Context &context, const std::int64_t now_ns) noexcept {
  if (!context.view_active.load(std::memory_order_acquire)) {
    return false;
  }
  if (!ChaseCameraGuardAllows(context)) {
    context.diag.tag_veto_frames.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (context.pose_attempted.exchange(true, std::memory_order_acq_rel)) {
    // A second scene view in the same frame (stereo) reuses the first pose.
    return context.pose_ready.load(std::memory_order_acquire);
  }
  context.pose_ready.store(false, std::memory_order_release);
  nfs2015_chase::CarSample sample;
  if (!ReadGameThreadCarSample(context, now_ns, sample)) {
    context.diag.detour_no_publish.fetch_add(1,
                                             std::memory_order_relaxed);
    return false;
  }
  double location[3]{};
  double rotation[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  bool advanced = false;
  if (!AdvancePresenter(context, now_ns, nullptr, location, rotation,
                        &lens, &fov_enabled, &advanced,
                        /*dedupe_same_frame=*/false, &sample)) {
    context.diag.detour_no_publish.fetch_add(1,
                                             std::memory_order_relaxed);
    return false;
  }
  context.pose_ready.store(true, std::memory_order_release);
  context.view_output_frames.fetch_add(1, std::memory_order_relaxed);
  SamplePresenterMotion(context, now_ns, location, rotation, advanced);
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

  if (context != nullptr && out != nullptr && original_called) {
    auto &view = *static_cast<MinimalViewInfo *>(out);
    // Keep the same observation the old render-thread detour exposed, so the
    // diagnostics and the borrow input can be compared across the A/B.
    try {
      PublishObservedView(view, *context);
    } catch (...) {
    }
    if (leased) {
      try {
        static_cast<void>(ApplyViewPoint(*context, view));
      } catch (...) {
      }
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
    // Task group 6 writes `out + kSceneViewInitOptionsCameraCutOffset` here,
    // after the original returned (so the game's own cut is preserved) and
    // before the depth drops. This task group only owns the gate.
    context->view_build_depth.fetch_sub(1, std::memory_order_acq_rel);
  }
  if (context != nullptr) {
    EndLease(*context, lease, leased);
  }
  return result;
}

}  // namespace nfs2015_chase_cam::detail
