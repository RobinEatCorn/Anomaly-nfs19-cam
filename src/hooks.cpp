// The two game-thread camera detours and their signature resolvers. Every
// detour is a thin callback wrapper: it checks the lease the host handed out,
// copies what the model published, and calls the original.
#include "internal.hpp"

#include <cstddef>
#include <cstdint>
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

namespace {

bool AnyCameraHookInstalled(const Context &context) noexcept {
  return context.game_view_point_hook_id.load(std::memory_order_acquire) != 0 ||
         context.scene_view_init_hook_id.load(std::memory_order_acquire) != 0;
}

void ClearActiveIfNoHooks(Context &context) noexcept {
  if (!AnyCameraHookInstalled(context)) {
    g_active.store(nullptr, std::memory_order_release);
    context.hook_ready.store(false, std::memory_order_release);
  }
}

}  // namespace

bool EnsureViewPointHook(Context &context) noexcept {
  if (context.game_view_point_pattern_unavailable) {
    return false;
  }
  // The game-thread path is fail-closed on the profile hash. Its field offsets
  // were only verified on the accepted builds.
  if (!context.vehicle_runtime.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.game_view_point_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.game_view_point_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kLocalPlayerGetViewPointPattern, target)) {
      context.game_view_point_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: ULocalPlayer::GetViewPoint pattern not found; "
          "game-thread camera disabled for this session");
      return false;
    }
    context.game_view_point_hook_target.store(target,
                                               std::memory_order_release);
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&LocalPlayerViewPointDetour);
  request.label =
      StringView("nfs2015-chase-cam-local-player-view-point");
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
    context.game_view_point_hook_id.store(0, std::memory_order_release);
    context.game_view_point_hook_generation.store(0,
                                                   std::memory_order_release);
    ClearActiveIfNoHooks(context);
    if (!context.game_view_point_hook_failure_logged) {
      context.game_view_point_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: ULocalPlayer::GetViewPoint hook creation "
          "failed; another plugin may own the same function");
    }
    return false;
  }
  context.game_view_point_hook_generation.store(handle.generation,
                                                 std::memory_order_release);
  g_local_player_get_view_point_original.store(
      reinterpret_cast<LocalPlayerGetViewPointFn>(original),
      std::memory_order_release);
  context.game_view_point_hook_id.store(handle.id, std::memory_order_release);
  context.game_view_point_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: game-thread GetViewPoint hook installed");
  return true;
}

bool EnsureSceneViewInitOptionsHook(Context &context) noexcept {
  if (context.scene_view_init_pattern_unavailable) {
    return false;
  }
  if (!context.vehicle_runtime.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.scene_view_init_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.scene_view_init_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kCalcSceneViewInitOptionsPattern, target)) {
      context.scene_view_init_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: CalcSceneViewInitOptions pattern not found; "
          "game-thread camera disabled for this session");
      return false;
    }
    context.scene_view_init_hook_target.store(target,
                                               std::memory_order_release);
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&CalcSceneViewInitOptionsDetour);
  request.label =
      StringView("nfs2015-chase-cam-scene-view-init-options");
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
    context.scene_view_init_hook_id.store(0, std::memory_order_release);
    context.scene_view_init_hook_generation.store(0,
                                                   std::memory_order_release);
    ClearActiveIfNoHooks(context);
    if (!context.scene_view_init_hook_failure_logged) {
      context.scene_view_init_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: CalcSceneViewInitOptions hook creation failed; "
          "the view-point gate cannot open");
    }
    return false;
  }
  context.scene_view_init_hook_generation.store(handle.generation,
                                                 std::memory_order_release);
  g_calc_scene_view_init_options_original.store(
      reinterpret_cast<CalcSceneViewInitOptionsFn>(original),
      std::memory_order_release);
  context.scene_view_init_hook_id.store(handle.id,
                                        std::memory_order_release);
  context.scene_view_init_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: CalcSceneViewInitOptions gate hook installed");
  return true;
}

bool EnsureHook(Context &context) noexcept {
  // The two hooks are one path: the view-point hook writes the pose and the
  // scene hook advances it and writes the cut. Installing only one would leave
  // a half-active path, so a session that cannot resolve either pattern keeps
  // the native camera and does not keep retrying the other hook.
  if (context.game_view_point_pattern_unavailable ||
      context.scene_view_init_pattern_unavailable) {
    if (AnyCameraHookInstalled(context)) {
      ReleaseHook(context);
    }
    context.hook_ready.store(false, std::memory_order_release);
    return false;
  }
  const bool view_point = EnsureViewPointHook(context);
  bool scene_init = false;
  if (view_point) {
    scene_init = EnsureSceneViewInitOptionsHook(context);
  }
  const bool ready = view_point && scene_init;
  context.hook_ready.store(ready, std::memory_order_release);
  if (!ready && AnyCameraHookInstalled(context)) {
    ReleaseHook(context);
  }
  return ready;
}

void ReleaseHook(Context &context) noexcept {
  g_active.store(nullptr, std::memory_order_release);
  const std::uint64_t game_view_point_id =
      context.game_view_point_hook_id.load(std::memory_order_acquire);
  bool game_view_point_released = false;
  if (game_view_point_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        game_view_point_id,
        context.game_view_point_hook_generation.load(
            std::memory_order_acquire)};
    game_view_point_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (game_view_point_released) {
    g_local_player_get_view_point_original.store(nullptr,
                                                  std::memory_order_release);
  }
  context.game_view_point_hook_id.store(0, std::memory_order_release);
  context.game_view_point_hook_generation.store(0,
                                                 std::memory_order_release);
  const std::uint64_t scene_view_init_id =
      context.scene_view_init_hook_id.load(std::memory_order_acquire);
  bool scene_view_init_released = false;
  if (scene_view_init_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        scene_view_init_id,
        context.scene_view_init_hook_generation.load(
            std::memory_order_acquire)};
    scene_view_init_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (scene_view_init_released) {
    g_calc_scene_view_init_options_original.store(nullptr,
                                                   std::memory_order_release);
  }
  context.scene_view_init_hook_id.store(0, std::memory_order_release);
  context.scene_view_init_hook_generation.store(0,
                                                 std::memory_order_release);
  context.hook_ready.store(false, std::memory_order_release);
  context.view_build_depth.store(0, std::memory_order_release);
  context.pose_attempted.store(false, std::memory_order_release);
  context.pose_ready.store(false, std::memory_order_release);
  context.view_active.store(false, std::memory_order_release);
}

}  // namespace nfs2015_chase_cam::detail
