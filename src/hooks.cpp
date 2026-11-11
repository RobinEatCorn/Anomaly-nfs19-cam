// The five render-side detours plus the signature resolvers that install them.
// Every detour is a thin callback wrapper: it checks the lease the host handed
// out, copies what the model published, and calls the original.
#include "internal.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace nfs2015_chase_cam::detail {

bool ResolveSignature(Context &context, const std::string_view pattern,
                      std::uintptr_t &address) noexcept {
  address = 0;
  return SignatureReady(context.signature) &&
         context.signature
                 ->resolve(context.signature->user,
                           StringView("HTGame.exe"), StringView(".text"),
                           StringView(pattern), &address)
                 .code == ANOMALY_STATUS_V1_OK &&
         address != 0;
}

// Resolve the manager's cached FMinimalViewInfo. The getter's own body reaches
// it through a vtable slot holding a `lea rax,[rcx+disp32]; ret` accessor.
bool ResolveCameraPov(Context &context) noexcept {
  const std::uintptr_t manager =
      context.observed_manager.load(std::memory_order_acquire);
  if (manager == 0) {
    return false;
  }
  if (manager == context.resolved_manager &&
      context.camera_pov.load(std::memory_order_acquire) != 0) {
    return true;
  }
  std::uintptr_t vtable{};
  std::uintptr_t accessor{};
  if (!ReadPointerAtOffset(context.core, manager, 0, vtable) ||
      !ReadPointerAtOffset(context.core, vtable,
                           kCameraPovAccessorVtableOffset, accessor)) {
    return false;
  }
  std::array<std::uint8_t, 8> body{};
  if (!Read(context.core, accessor, body) || body[0] != 0x48 ||
      body[1] != 0x8D || body[2] != 0x81 || body[7] != 0xC3) {
    return false;
  }
  const std::uint32_t raw_displacement =
      static_cast<std::uint32_t>(body[3]) |
      (static_cast<std::uint32_t>(body[4]) << 8U) |
      (static_cast<std::uint32_t>(body[5]) << 16U) |
      (static_cast<std::uint32_t>(body[6]) << 24U);
  const std::int32_t displacement =
      static_cast<std::int32_t>(raw_displacement);
  const std::intptr_t resolved =
      static_cast<std::intptr_t>(manager) + displacement;
  if (resolved <= 0) {
    return false;
  }
  const std::uintptr_t pov = static_cast<std::uintptr_t>(resolved);
  std::uintptr_t fov_address{};
  float lens{};
  const bool lens_valid =
      AddAddress(pov, kCameraPovFovOffset, fov_address) &&
      Read(context.core, fov_address, lens) && std::isfinite(lens) &&
      lens > 1.0F && lens < 179.0F;
  context.resolved_manager = manager;
  if (!lens_valid) {
    // Do not keep a POV we could not validate: the detour would write the lens
    // through it. A later tick retries because camera_pov stays zero.
    context.camera_pov.store(0, std::memory_order_release);
    context.camera_pov_manager.store(0, std::memory_order_release);
    return false;
  }
  context.game_fov.store(lens, std::memory_order_release);
  context.camera_pov.store(pov, std::memory_order_release);
  context.camera_pov_manager.store(manager, std::memory_order_release);
  return true;
}

AnomalyStatusV1 ResolveSignatureStatus(const Context &context,
                                       const std::string_view pattern,
                                       std::uintptr_t &address) noexcept {
  address = 0;
  if (!SignatureReady(context.signature)) {
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE);
  }
  return context.signature->resolve(
      context.signature->user, StringView("HTGame.exe"), StringView(".text"),
      StringView(pattern), &address);
}

// Task group 3 A2. The render thread calls this once for every primitive whose
// transform changed. Most calls are other objects: the watched-proxy pointer
// check is the only work before the original call. For the vehicle proxy we
// copy the matrix's translation (last row, +0x60 in the FMatrix) into the
// probe's seqlock before returning.
void ANOMALY_CALL RenderTransformDetour(void *self,
                                           void *scene) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->transform_hook_id.load(std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->transform_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  double translation[3]{};
  double forward[3]{};
  bool publish = false;
  bool orientation_valid = false;
  if (context != nullptr && self != nullptr &&
      context->render_x.version_gate.load(std::memory_order_acquire) &&
      reinterpret_cast<std::uintptr_t>(self) ==
          context->render_x.watched_proxy.load(std::memory_order_relaxed)) {
    const auto *value = reinterpret_cast<const double *>(
        reinterpret_cast<std::uintptr_t>(self) +
        kVehicleProxyTranslationOffset);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      translation[axis] = value[axis];
    }
    // The proxy matrix is row-major; row 0 is the local +X (nose) basis.
    // Normalise the full vector, not only its horizontal part: consumers use
    // forward.z as sin(pitch) for the body frame. The proxy may carry a
    // component scale.
    const auto *matrix = reinterpret_cast<const double *>(
        reinterpret_cast<std::uintptr_t>(self) + 0xE0);
    forward[0] = matrix[0];
    forward[1] = matrix[1];
    forward[2] = matrix[2];
    const double length_squared = forward[0] * forward[0] +
                                  forward[1] * forward[1] +
                                  forward[2] * forward[2];
    if (std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
        std::isfinite(forward[2]) && length_squared > 1e-8) {
      const double inverse_length = 1.0 / std::sqrt(length_squared);
      forward[0] *= inverse_length;
      forward[1] *= inverse_length;
      forward[2] *= inverse_length;
      orientation_valid = true;
    }
    publish = std::isfinite(translation[0]) &&
              std::isfinite(translation[1]) &&
              std::isfinite(translation[2]);
  }

  OnTransformChangedFn original =
      g_on_transform_changed_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, scene);
    }
  } catch (...) {
  }

  if (publish) {
    RenderXProbe &probe = context->render_x;
    probe.render_thread_id.store(static_cast<std::uint32_t>(GetCurrentThreadId()),
                                 std::memory_order_release);
    probe.transform_sequence.fetch_add(1, std::memory_order_acq_rel);
    probe.transform_proxy.store(reinterpret_cast<std::uintptr_t>(self),
                                std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.transform_cm[axis].store(translation[axis],
                                     std::memory_order_relaxed);
      probe.transform_forward[axis].store(forward[axis],
                                          std::memory_order_relaxed);
    }
    probe.transform_orientation_valid.store(orientation_valid,
                                             std::memory_order_relaxed);
    probe.transform_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    probe.transform_updates.fetch_add(1, std::memory_order_relaxed);
    probe.transform_valid.store(true, std::memory_order_release);
    probe.transform_sequence.fetch_add(1, std::memory_order_release);
    // TEMP read-only timing diagnostics: classify this transform update
    // against the last early camera write. It never changes the camera.
    if (probe.timing_frame.load(std::memory_order_relaxed) != 0) {
      const std::int64_t early_ns =
          probe.timing_early_ns.load(std::memory_order_relaxed);
      const std::int64_t event_ns = NowNanoseconds();
      if (probe.timing_late_seen.load(std::memory_order_acquire)) {
        probe.timing_transform_after.fetch_add(1,
                                               std::memory_order_relaxed);
      } else if (early_ns != 0 && event_ns >= early_ns) {
        probe.timing_transform_between.fetch_add(1,
                                                 std::memory_order_relaxed);
      } else {
        probe.timing_transform_before.fetch_add(1,
                                                std::memory_order_relaxed);
      }
    }
    // The VehicleMesh ComponentToWorld is already available at the Game
    // camera point and matches the final proxy. Do not write the view here:
    // this point is after PrepareViewStateForVisibility, so a late camera
    // change would desynchronise the history and visibility state.
  }

  // Release the callback lease even if the host is already tearing the hook
  // service down. `begin_callback` succeeded, so this lease is ours; skipping
  // `end_callback` because `HookReady` turned false during shutdown would leak
  // an in-flight callback and make the host quarantine the next generation.
  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

// Diagnostic-only view hook. It records whether the renderer's late view setup
// runs on the same thread as the render transform hook, and how old the
// transform snapshot is. It deliberately never touches the camera: changing
// the pose after PrepareViewStateForVisibility would leave this frame's
// culling, motion vectors and TAA history describing a different camera, which
// is exactly the one-frame ghost this hook used to cause.
//
// The hook is kept installed because the transform/variable diagnostics are
// useful and because removing an established hook target would change the
// plugin's failure surface. Route B itself is applied only in
// ApplyRouteBEarly, before visibility.
bool IsRouteBEarlyView(const RenderXProbe &probe, void *view) noexcept {
  if (view == nullptr ||
      !probe.route_b_early_views_valid.load(std::memory_order_acquire)) {
    return false;
  }
  const std::uint32_t count =
      probe.route_b_early_view_count.load(std::memory_order_relaxed);
  const std::uint32_t limit =
      count < sizeof(probe.route_b_early_views) /
                  sizeof(probe.route_b_early_views[0])
          ? count
          : static_cast<std::uint32_t>(
                sizeof(probe.route_b_early_views) /
                sizeof(probe.route_b_early_views[0]));
  const auto address = reinterpret_cast<std::uintptr_t>(view);
  std::uintptr_t state = 0;
  std::memcpy(&state,
              reinterpret_cast<const void *>(address + kViewStateOffset),
              sizeof(state));
  for (std::uint32_t index = 0; index < limit; ++index) {
    if (probe.route_b_early_views[index].load(
            std::memory_order_relaxed) == address &&
        probe.route_b_early_view_states[index].load(
            std::memory_order_relaxed) == state) {
      return true;
    }
  }
  return false;
}

void ANOMALY_CALL ViewUniformDetour(void *self, void *view_matrices,
                                    void *prev_view_matrices,
                                    void *out_parameters,
                                    std::uint64_t flag,
                                    void *uniform_buffer) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->view_uniform_hook_id.load(std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->view_uniform_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  if (context != nullptr) {
    RenderXProbe &probe = context->render_x;
    probe.view_uniform_calls.fetch_add(1, std::memory_order_relaxed);
    const std::uint32_t thread = static_cast<std::uint32_t>(GetCurrentThreadId());
    const std::uint32_t render_thread =
        probe.render_thread_id.load(std::memory_order_acquire);
    if (render_thread != 0 && thread == render_thread) {
      probe.view_uniform_render_calls.fetch_add(1,
                                                std::memory_order_relaxed);
      if (probe.transform_valid.load(std::memory_order_acquire)) {
        const std::int64_t transform_ns =
            probe.transform_ns.load(std::memory_order_relaxed);
        const std::int64_t age = NowNanoseconds() - transform_ns;
        // Only a plausible in-flight age is a timing sample. A huge value is
        // a stale/never-written snapshot, not a camera frame.
        if (transform_ns != 0 && age >= 0 && age <= 1'000'000'000) {
          const auto age_us = static_cast<std::uint64_t>(age / 1000);
          RaisePeak(probe.view_uniform_age_peak_us, age_us);
          probe.view_uniform_age_sum_us.fetch_add(age_us,
                                                  std::memory_order_relaxed);
          probe.view_uniform_age_count.fetch_add(1,
                                                  std::memory_order_relaxed);
        }
      }
    } else {
      probe.view_uniform_game_calls.fetch_add(1,
                                               std::memory_order_relaxed);
    }
  }

  SetupUniformBufferFn original =
      g_setup_uniform_buffer_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, view_matrices, prev_view_matrices, out_parameters, flag,
               uniform_buffer);
    }
  } catch (...) {
  }

  // TEMP read-only timing diagnostics: compare the car/view at the late
  // uniform setup with the values captured by the early camera write.
  if (context != nullptr && IsRouteBEarlyView(context->render_x, self) &&
      context->render_x.timing_frame.load(std::memory_order_relaxed) != 0 &&
      !context->render_x.timing_late_seen.exchange(
          true, std::memory_order_acq_rel)) {
    RenderXProbe &probe = context->render_x;
    const auto address = reinterpret_cast<std::uintptr_t>(self);
    double late_view[3]{};
    std::memcpy(late_view,
                reinterpret_cast<const void *>(address +
                                               kViewLocationOffset),
                sizeof(late_view));
    double late_car[3]{};
    double late_forward[3]{};
    const bool have_car =
        ReadRouteBDirectCar(probe, late_car, late_forward);
    probe.timing_late_count.fetch_add(1, std::memory_order_relaxed);
    probe.timing_late_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    double car_distance = 0.0;
    double view_distance = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.timing_late_view[axis].store(late_view[axis],
                                         std::memory_order_relaxed);
      if (have_car) {
        probe.timing_late_car[axis].store(late_car[axis],
                                          std::memory_order_relaxed);
        const double car_delta =
            late_car[axis] -
            probe.timing_early_car[axis].load(std::memory_order_relaxed);
        car_distance += car_delta * car_delta;
      }
      const double view_delta =
          late_view[axis] -
          probe.timing_early_view[axis].load(std::memory_order_relaxed);
      view_distance += view_delta * view_delta;
    }
    const auto store_distance = [](std::atomic<std::uint64_t> &peak,
                                   std::atomic<std::uint64_t> &sum,
                                   std::atomic<std::uint64_t> &count,
                                   const double distance_cm) {
      if (!std::isfinite(distance_cm)) {
        return;
      }
      const auto value = static_cast<std::uint64_t>(
          distance_cm * 10.0 + 0.5);
      RaisePeak(peak, value);
      sum.fetch_add(value, std::memory_order_relaxed);
      count.fetch_add(1, std::memory_order_relaxed);
    };
    if (have_car) {
      store_distance(probe.timing_car_delta_peak_dmm,
                     probe.timing_car_delta_sum_dmm,
                     probe.timing_car_delta_count,
                     std::sqrt(car_distance));
    }
    store_distance(probe.timing_view_delta_peak_dmm,
                   probe.timing_view_delta_sum_dmm,
                   probe.timing_view_delta_count,
                   std::sqrt(view_distance));
  }

  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

// Pre-visibility route B. This runs immediately before culling tasks are
// launched and before PrepareViewStateForVisibility snapshots the matrices
// into the temporal history. It is the only camera writer; the late
// view-uniform hook is diagnostic and must never move a view.
void ANOMALY_CALL PrepareViewStateDetour(void *self,
                                         const void *scene_textures_config) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->prepare_view_state_hook_id.load(
                std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->prepare_view_state_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  if (context != nullptr) {
    context->render_x.prepare_calls.fetch_add(1, std::memory_order_relaxed);
    context->render_x.route_b_active_renderer.store(
        reinterpret_cast<std::uintptr_t>(self), std::memory_order_release);
    if (context->enabled.load(std::memory_order_acquire)) {
      static_cast<void>(ApplyRouteBEarly(*context, self));
    }
  }

  PrepareViewStateFn original =
      g_prepare_view_state_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, scene_textures_config);
    }
  } catch (...) {
  }

  if (context != nullptr &&
      context->render_x.route_b_active_renderer.load(
          std::memory_order_relaxed) ==
          reinterpret_cast<std::uintptr_t>(self)) {
    context->render_x.route_b_active_renderer.store(
        0, std::memory_order_release);
  }

  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

bool EnsurePrepareViewStateHook(Context &context) noexcept {
  if (context.early_route_b_unavailable) {
    return false;
  }
  const auto note_pattern_miss = [&context]() {
    ++context.prepare_hook_pattern_failures;
    if (context.prepare_hook_pattern_failures >=
        kPrepareHookMaxPatternFailures) {
      context.early_route_b_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: early route B pattern still unresolved after "
          "the retry window; route B disabled for this session");
    }
  };
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.prepare_view_state_hook_id.load(std::memory_order_acquire) !=
          0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }

  std::uintptr_t target =
      context.prepare_view_state_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kPrepareViewStatePattern, target)) {
      note_pattern_miss();
      if (!context.prepare_view_state_pattern_failure_logged) {
        context.prepare_view_state_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: PrepareViewStateForVisibility pattern not "
            "found; early route B is retrying, no late camera write");
      }
      return false;
    }
    context.prepare_view_state_hook_target.store(target,
                                                  std::memory_order_release);
  }

  if (g_scene_view_update_matrix.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t update_matrix = 0;
    if (!ResolveSignature(context, kSceneViewUpdateMatrixPattern,
                          update_matrix)) {
      note_pattern_miss();
      if (!context.route_b_pattern_failure_logged) {
        context.route_b_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: FSceneView::UpdateViewMatrix pattern not "
            "found; early route B is retrying, no late camera write");
      }
      return false;
    }
    g_scene_view_update_matrix.store(
        reinterpret_cast<SceneViewUpdateMatrixFn>(update_matrix),
        std::memory_order_release);
  }

  if (g_scene_view_setup_frustum.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t setup_frustum = 0;
    if (!ResolveSignature(context, kSceneViewSetupFrustumPattern,
                          setup_frustum)) {
      if (!context.route_b_pattern_failure_logged) {
        context.route_b_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: FSceneView::SetupViewFrustum pattern not "
            "found; CullingFrustum may trail route B");
      }
    } else {
      g_scene_view_setup_frustum.store(
          reinterpret_cast<SceneViewSetupFrustumFn>(setup_frustum),
          std::memory_order_release);
    }
  }

  // Route B rebuilds FViewMatrices through the engine constructor so every
  // derived matrix (InvProjection, ProjectionNoAA, ScreenToClip) matches the
  // lens it writes. Resolve the constructor here as well: the prepare hook can
  // be installed before the diagnostic view-uniform hook.
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t matrices_ctor = 0;
    if (ResolveSignature(context, kViewMatricesCtorPattern, matrices_ctor)) {
      g_view_matrices_ctor.store(
          reinterpret_cast<ViewMatricesCtorFn>(matrices_ctor),
          std::memory_order_release);
    }
  }
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    note_pattern_miss();
    if (!context.route_b_pattern_failure_logged) {
      context.route_b_pattern_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: FViewMatrices ctor pattern not found; early "
          "route B is retrying, no late camera write");
    }
    return false;
  }

  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&PrepareViewStateDetour);
  request.label = StringView("nfs2015-chase-cam-prepare-view-state");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.prepare_view_state_hook_id.store(0, std::memory_order_release);
    context.prepare_view_state_hook_generation.store(
        0, std::memory_order_release);
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_hook_id.load(std::memory_order_acquire) == 0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.prepare_view_state_hook_failure_logged) {
      context.prepare_view_state_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: prepare-view-state hook creation failed");
    }
    return false;
  }
  context.prepare_view_state_hook_generation.store(
      handle.generation, std::memory_order_release);
  g_prepare_view_state_original.store(
      reinterpret_cast<PrepareViewStateFn>(original),
      std::memory_order_release);
  context.prepare_view_state_hook_id.store(handle.id,
                                            std::memory_order_release);
  context.prepare_view_state_hook_failure_logged = false;
  context.prepare_hook_pattern_failures = 0;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: pre-visibility route B hook installed");
  return true;
}

bool EnsureViewUniformHook(Context &context) noexcept {
  if (context.view_uniform_pattern_unavailable) {
    return false;
  }
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.view_uniform_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.view_uniform_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kSetupUniformBufferPattern, target)) {
      context.view_uniform_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: SetupUniformBufferParameters pattern not found; "
          "view-uniform diagnostics disabled (route B is unaffected)");
      return false;
    }
    context.view_uniform_hook_target.store(target, std::memory_order_release);
  }
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t matrices_ctor = 0;
    if (ResolveSignature(context, kViewMatricesCtorPattern, matrices_ctor)) {
      g_view_matrices_ctor.store(
          reinterpret_cast<ViewMatricesCtorFn>(matrices_ctor),
          std::memory_order_release);
    } else if (!context.route_b_pattern_failure_logged) {
      context.route_b_pattern_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: FViewMatrices ctor pattern not found; route B "
          "will use the original camera");
    }
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&ViewUniformDetour);
  request.label = StringView("nfs2015-chase-cam-view-uniform");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.view_uniform_hook_id.store(0, std::memory_order_release);
    context.view_uniform_hook_generation.store(0, std::memory_order_release);
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.view_uniform_hook_failure_logged) {
      context.view_uniform_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: view uniform hook creation failed");
    }
    return false;
  }
  context.view_uniform_hook_generation.store(handle.generation,
                                               std::memory_order_release);
  g_setup_uniform_buffer_original.store(
      reinterpret_cast<SetupUniformBufferFn>(original),
      std::memory_order_release);
  context.view_uniform_hook_id.store(handle.id, std::memory_order_release);
  context.view_uniform_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: render view hook installed");
  return true;
}

bool EnsureTransformHook(Context &context) noexcept {
  if (context.transform_pattern_unavailable) {
    return false;
  }
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.transform_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.transform_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kOnTransformChangedPattern, target)) {
      context.transform_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: OnTransformChanged pattern not found; render "
          "transform hook disabled for this session");
      return false;
    }
    context.transform_hook_target.store(target, std::memory_order_release);
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&RenderTransformDetour);
  request.label = StringView("nfs2015-chase-cam-on-transform-changed");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.transform_hook_id.store(0, std::memory_order_release);
    context.transform_hook_generation.store(0, std::memory_order_release);
    if (context.view_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.transform_hook_failure_logged) {
      context.transform_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: render transform hook creation failed");
    }
    return false;
  }
  context.transform_hook_generation.store(handle.generation,
                                           std::memory_order_release);
  g_on_transform_changed_original.store(
      reinterpret_cast<OnTransformChangedFn>(original),
                                 std::memory_order_release);
  context.transform_hook_id.store(handle.id, std::memory_order_release);
  context.transform_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: render transform hook installed");
  return true;
}

bool EnsureHook(Context &context) noexcept {
  if (context.view_point_pattern_unavailable) {
    return false;
  }
  // 2.5 has no build service. Preserve its legacy view-point hook rather than
  // turning an optional service into a hard requirement; 2.6+ with a build
  // service present is fail-closed on a hash mismatch.
  const bool legacy_without_build =
      context.render_x.insurance_state == kAnchorInsuranceServiceMissing;
  if (!context.render_x.version_gate.load(std::memory_order_acquire) &&
      !legacy_without_build) {
    return false;
  }
  if (context.view_hook_id.load(std::memory_order_acquire) != 0 &&
      context.hook_ready.load(std::memory_order_acquire)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target = context.hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kCameraViewPointPattern, target)) {
      context.view_point_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: CameraManager view-point pattern not found; "
          "camera hook disabled for this session");
      return false;
    }
    context.hook_target.store(target, std::memory_order_release);
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&CameraViewPointDetour);
  request.label = StringView("nfs2015-chase-cam-view-point");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.view_hook_id.store(0, std::memory_order_release);
    context.view_hook_generation.store(0, std::memory_order_release);
    context.hook_ready.store(false, std::memory_order_release);
    if (!context.hook_failure_logged) {
      context.hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: view-point hook creation failed; another camera "
          "plugin may own the same getter");
    }
    return false;
  }
  context.view_hook_generation.store(handle.generation,
                                     std::memory_order_release);
  g_original.store(reinterpret_cast<CameraViewPointFn>(original),
                   std::memory_order_release);
  context.view_hook_id.store(handle.id, std::memory_order_release);
  context.hook_ready.store(true, std::memory_order_release);
  context.hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: view-point hook installed");
  return true;
}

void ReleaseHook(Context &context) noexcept {
  g_active.store(nullptr, std::memory_order_release);
  const std::uint64_t transform_id =
      context.transform_hook_id.load(std::memory_order_acquire);
  bool transform_released = false;
  if (transform_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        transform_id,
        context.transform_hook_generation.load(std::memory_order_acquire)};
    transform_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (transform_released) {
    g_on_transform_changed_original.store(nullptr,
                                          std::memory_order_release);
  }
  context.transform_hook_id.store(0, std::memory_order_release);
  context.transform_hook_generation.store(0, std::memory_order_release);
  const std::uint64_t view_uniform_id =
      context.view_uniform_hook_id.load(std::memory_order_acquire);
  bool view_uniform_released = false;
  if (view_uniform_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        view_uniform_id,
        context.view_uniform_hook_generation.load(std::memory_order_acquire)};
    view_uniform_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (view_uniform_released) {
    g_setup_uniform_buffer_original.store(nullptr, std::memory_order_release);
  }
  context.view_uniform_hook_id.store(0, std::memory_order_release);
  context.view_uniform_hook_generation.store(0, std::memory_order_release);
  const std::uint64_t prepare_view_state_id =
      context.prepare_view_state_hook_id.load(std::memory_order_acquire);
  bool prepare_view_state_released = false;
  if (prepare_view_state_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        prepare_view_state_id,
        context.prepare_view_state_hook_generation.load(
            std::memory_order_acquire)};
    prepare_view_state_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (prepare_view_state_released) {
    g_prepare_view_state_original.store(nullptr, std::memory_order_release);
  }
  context.prepare_view_state_hook_id.store(0, std::memory_order_release);
  context.prepare_view_state_hook_generation.store(0,
                                                    std::memory_order_release);
  const std::uint64_t hook_id =
      context.view_hook_id.load(std::memory_order_acquire);
  bool released = false;
  if (hook_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        hook_id, context.view_hook_generation.load(std::memory_order_acquire)};
    released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (released) {
    // Keep the trampoline if release failed: a late detour must still be able
    // to delegate to the game, and the host will drain the hook on unload.
    g_original.store(nullptr, std::memory_order_release);
  }
  context.view_hook_id.store(0, std::memory_order_release);
  context.view_hook_generation.store(0, std::memory_order_release);
  context.hook_ready.store(false, std::memory_order_release);
  context.view_active.store(false, std::memory_order_release);
  context.observed_valid.store(false, std::memory_order_release);
  context.observed_manager.store(0, std::memory_order_release);
  context.resolved_manager = 0;
  context.camera_pov.store(0, std::memory_order_release);
  context.camera_pov_manager.store(0, std::memory_order_release);
  context.game_fov.store(0.0F, std::memory_order_release);
  context.fov_restore.store(false, std::memory_order_release);
}

}  // namespace nfs2015_chase_cam::detail
