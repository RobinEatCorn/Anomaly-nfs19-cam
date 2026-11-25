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

bool ApplyViewPoint(Context &context, MinimalViewInfo &view) noexcept {
  if (!context.view_active.load(std::memory_order_acquire)) {
    return false;
  }
  if (!ChaseCameraGuardAllows(context)) {
    context.diag.tag_veto_frames.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  const bool pose_ready =
      context.pose_ready.load(std::memory_order_acquire);
  if (!pose_ready) {
    // D27: the first GetViewPoint call in the frame owns the advance. It is
    // normally the UI projection in TG_PrePhysics; the vehicle is still at the
    // previous frame's final transform there, which is the same phase the
    // game's own camera manager uses. Every later consumer must see the pose
    // that this call produces, or UI and scene would use different cameras.
    if (context.pose_attempted.exchange(true, std::memory_order_acq_rel)) {
      // The advance already ran and failed this frame. Do not retry it for the
      // next query: one failure means the whole frame stays fail-open.
      context.diag.query_skips.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    const std::int64_t now_ns = NowNanoseconds();
    const double game_eye_cm[3] = {view.location[0], view.location[1],
                                   view.location[2]};
    double location[3]{};
    double rotation[3]{};
    float lens = 0.0F;
    bool fov_enabled = false;
    bool advanced = false;
    if (!AdvancePresenter(context, now_ns, game_eye_cm, location, rotation,
                          &lens, &fov_enabled, &advanced,
                          /*dedupe_same_frame=*/false)) {
      context.diag.detour_no_publish.fetch_add(1,
                                               std::memory_order_relaxed);
      return false;
    }
    context.pose_ready.store(true, std::memory_order_release);
    context.view_output_frames.fetch_add(1, std::memory_order_relaxed);
    SamplePresenterMotion(context, now_ns, location, rotation, advanced);
    WritePose(context, view, location, rotation, lens, fov_enabled);
    return true;
  }

  // Every later call in this frame (the UI projections, the direct
  // CalcSceneView GetViewPoint, and the HUD) copies the pose the first call
  // published. No model step, no second vehicle read.
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
