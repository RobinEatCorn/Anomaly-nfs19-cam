// Parked-orbit HUD hiding: guest-side resolution, readback and ProcessEvent
// calls. The pure Q2 state machine lives in orbit_ui_hide.hpp; this file is the
// Game-thread backend that feeds it.
//
// TG-12 established the guest API:
//   W_OverallUILayout_C::HideAllLayer    -> hide the whole HUD
//   W_OverallUILayout_C::RevertLayerShow -> restore it
// The root widget's Visibility does not change; the layer's RenderOpacity does.
// The readback therefore uses UWidget::GetRenderOpacity on the live
// GameplayMain_Stack / GameLayer_Stack children.
//
// Everything here is fail-open: if a service, class, function, instance or
// readback is unavailable, the camera keeps running. The state machine hides
// without readback but always restores on exit (Q2=C) when the readback is
// unavailable.

#include "internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace nfs2015_chase_cam::detail {
namespace {

constexpr std::string_view kLayoutClassPath =
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C";
constexpr std::string_view kHideAllLayerPath =
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:HideAllLayer";
constexpr std::string_view kRevertLayerShowPath =
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:RevertLayerShow";
constexpr std::string_view kGetRenderOpacityPath =
    "/Script/UMG.Widget:GetRenderOpacity";
constexpr std::string_view kSetRenderOpacityPath =
    "/Script/UMG.Widget:SetRenderOpacity";
constexpr std::string_view kSetRenderTranslationPath =
    "/Script/UMG.Widget:SetRenderTranslation";

// Runtime child paths are tried before falling back to a name scan. A
// WidgetTree child's path is the generated class path plus the WidgetTree
// subobject and the widget name.
constexpr std::array<std::string_view, 10> kReadbackPaths = {
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:WidgetTree.GameplayMain_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:WidgetTree.GameLayer_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:GameplayMain_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:GameLayer_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C.WidgetTree.GameplayMain_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C.WidgetTree.GameLayer_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:GameplayMain_Stack.GameplayMain_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:GameLayer_Stack.GameLayer_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:WidgetTree.MainOverlay.GameplayMain_Stack",
    "/Game/UI/W_OverallUILayout.W_OverallUILayout_C:WidgetTree.MainOverlay.GameLayer_Stack",
};

constexpr std::array<std::string_view, 2> kReadbackNames = {
    "GameplayMain_Stack",
    "GameLayer_Stack",
};

constexpr std::uint32_t kScanBatch = 8192;
constexpr double kResolveRetrySeconds = 2.0;
constexpr std::string_view kOrbitHudMarkerPath = "orbit-hud-hidden";
constexpr std::string_view kOrbitHudMarkerValue = "1";
constexpr double kStartupRestoreRetrySeconds = 2.0;
// Short enough to finish before the 0.25 s readback poll, long enough to read
// as the same simple fade the ESC menu uses.
constexpr double kOrbitUiFadeSeconds = 0.18;
// The ESC transition also gives the HUD a small slide. Positive Y is down in
// Slate; the HUD drops a little while fading out and returns on fade-in.
constexpr double kOrbitUiFadeTranslation = 18.0;

bool StorageReady(const AnomalyStorageServiceV1 *storage) noexcept {
  return storage != nullptr && storage->read != nullptr &&
         storage->write_atomic != nullptr;
}

void WriteOrbitHudMarker(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (!StorageReady(context.storage)) {
    if (!runtime.marker_error_logged) {
      runtime.marker_error_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: orbit UI recovery marker storage unavailable");
    }
    return;
  }
  const auto status = context.storage->write_atomic(
      context.storage->user, StringView(kOrbitHudMarkerPath),
      Bytes(kOrbitHudMarkerValue));
  if (status.code != ANOMALY_STATUS_V1_OK && !runtime.marker_error_logged) {
    runtime.marker_error_logged = true;
    char message[160]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: could not write orbit UI recovery "
                  "marker (%u)",
                  status.code);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
  }
}

void ClearOrbitHudMarker(Context &context) noexcept {
  if (!StorageReady(context.storage)) {
    return;
  }
  // Prefer overwriting over Delete: the host storage backend may reject a
  // delete of a state file on some installations. A zero byte is the cleared
  // state and LoadOrbitUiMarker only treats "1" as pending.
  const auto status = context.storage->write_atomic(
      context.storage->user, StringView(kOrbitHudMarkerPath),
      Bytes("0"));
  if (status.code != ANOMALY_STATUS_V1_OK &&
      !context.orbit_ui.marker_error_logged) {
    context.orbit_ui.marker_error_logged = true;
    char message[160]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: could not clear orbit UI recovery "
                  "marker (%u)",
                  status.code);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
  }
}

void ANOMALY_CALL ClearOrbitHudMarkerTask(
    void *user, AnomalyGenerationHandleV1 task) noexcept;

void ScheduleMarkerClear(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.marker_clear_task.id != 0 || context.scheduler == nullptr ||
      context.scheduler->schedule == nullptr) {
    return;
  }
  AnomalyGenerationHandleV1 task{};
  const auto status = context.scheduler->schedule(
      context.scheduler->user, 0, ClearOrbitHudMarkerTask, &context, &task);
  if (status.code == ANOMALY_STATUS_V1_OK && task.id != 0) {
    runtime.marker_clear_task = task;
  }
}

bool ResolvePath(Context &context, const std::string_view path,
                 AnomalyGenerationHandleV1 &handle) noexcept {
  if (context.objects == nullptr || context.objects->find_exact == nullptr) {
    return false;
  }
  AnomalyGenerationHandleV1 found{};
  const auto status = context.objects->find_exact(
      context.objects->user, StringView(path), &found);
  if (status.code != ANOMALY_STATUS_V1_OK || found.id == 0) {
    return false;
  }
  handle = found;
  return true;
}

bool InvokeLayout(Context &context, const AnomalyGenerationHandleV1 function,
                  const char *label) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.layout.id == 0 || function.id == 0 ||
      context.events == nullptr || context.events->invoke == nullptr) {
    return false;
  }
  const auto status = context.events->invoke(
      context.events->user, runtime.layout, function, nullptr, 0);
  if (status.code != ANOMALY_STATUS_V1_OK) {
    char message[192]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: orbit UI %s failed (%u)", label,
                  status.code);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    return false;
  }
  char message[160]{};
  std::snprintf(message, sizeof(message),
                "nfs2015 chase cam: orbit UI %s ok", label);
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  return true;
}

bool RefreshGeneration(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (!Ue5ObjectSnapshotReady(context.objects)) {
    return false;
  }
  const std::uint64_t generation =
      context.objects->generation(context.objects->user);
  if (generation == 0) {
    return false;
  }
  if (runtime.generation == generation) {
    return true;
  }

  if (runtime.generation != 0) {
    // A registry rebuild invalidates every guest handle. If the HUD was hidden
    // by us, try the old handles once before dropping them; the game may have
    // already rebuilt the layout, in which case the call simply fails.
    if (runtime.machine.state().hidden_by_us) {
      if (InvokeLayout(context, runtime.revert_layer_show,
                       "RevertLayerShow (generation change)")) {
        static_cast<void>(runtime.machine.Reset());
      } else {
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: orbit UI could not restore before object "
            "generation change");
      }
    }
    runtime = OrbitUiRuntimeState{};
  }
  runtime.generation = generation;
  return true;
}

bool EnsureStatics(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (!Ue5ObjectsReady(context.objects) ||
      !Ue5ProcessEventInvokeReady(context.events)) {
    if (!runtime.services_unavailable_logged) {
      runtime.services_unavailable_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: orbit UI services unavailable; HUD stays "
          "native");
    }
    return false;
  }
  if (runtime.layout_class.id != 0 && runtime.hide_all_layer.id != 0 &&
      runtime.revert_layer_show.id != 0 && runtime.layout_name_id != 0) {
    return true;
  }
  if (context.elapsed < runtime.next_resolve) {
    return false;
  }
  runtime.next_resolve = context.elapsed + kResolveRetrySeconds;

  if (runtime.layout_class.id == 0 &&
      !ResolvePath(context, kLayoutClassPath, runtime.layout_class)) {
    if (!runtime.resolve_failed_logged) {
      runtime.resolve_failed_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: W_OverallUILayout_C not resolved");
    }
    return false;
  }
  if (runtime.hide_all_layer.id == 0 &&
      !ResolvePath(context, kHideAllLayerPath, runtime.hide_all_layer)) {
    return false;
  }
  if (runtime.revert_layer_show.id == 0 &&
      !ResolvePath(context, kRevertLayerShowPath,
                   runtime.revert_layer_show)) {
    return false;
  }
  if (runtime.layout_name_id == 0) {
    AnomalyUe5ObjectSnapshotV1 snapshot{sizeof(snapshot)};
    if (context.objects->snapshot_by_handle(
            context.objects->user, runtime.layout_class, &snapshot)
            .code != ANOMALY_STATUS_V1_OK ||
        snapshot.name_id == 0) {
      return false;
    }
    runtime.layout_name_id = snapshot.name_id;
  }
  return true;
}

bool AddReadback(Context &context, const AnomalyGenerationHandleV1 handle,
                 const bool menu) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (handle.id == 0) {
    return false;
  }
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (runtime.readback[i].id == handle.id &&
        runtime.readback[i].generation == handle.generation) {
      return false;
    }
  }
  if (runtime.readback_count >= runtime.readback.size()) {
    return false;
  }
  const std::uint32_t index = runtime.readback_count;
  runtime.readback[index] = handle;
  const std::uint32_t handle_index = ANOMALY_UE5_OBJECT_HANDLE_INDEX(handle);
  const std::uint32_t layout_index = runtime.layout_resolved
                                         ? ANOMALY_UE5_OBJECT_HANDLE_INDEX(
                                               runtime.layout)
                                         : 0U;
  runtime.readback_live[index] =
      runtime.layout_resolved && handle_index > layout_index;
  runtime.readback_menu[index] = menu;
  ++runtime.readback_count;
  return true;
}

bool ResolveReadbackTargets(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.readback_ready) {
    return true;
  }
  if (runtime.get_render_opacity.id == 0) {
    static_cast<void>(ResolvePath(context, kGetRenderOpacityPath,
                                  runtime.get_render_opacity));
  }
  if (runtime.set_render_opacity.id == 0) {
    static_cast<void>(ResolvePath(context, kSetRenderOpacityPath,
                                  runtime.set_render_opacity));
  }
  if (runtime.set_render_translation.id == 0) {
    static_cast<void>(ResolvePath(context, kSetRenderTranslationPath,
                                  runtime.set_render_translation));
  }
  if (runtime.get_render_opacity.id == 0) {
    runtime.readback_ready = true;
    if (!runtime.readback_unavailable_logged) {
      runtime.readback_unavailable_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: Widget::GetRenderOpacity unavailable; orbit UI "
          "readback disabled (Q2=C)");
    }
    return true;
  }

  if (!runtime.readback_paths_tried) {
    runtime.readback_paths_tried = true;
    for (const std::string_view path : kReadbackPaths) {
      AnomalyGenerationHandleV1 handle{};
      if (ResolvePath(context, path, handle)) {
        // ESC's phone menu is parented under GameplayMain_Stack; the HUD is
        // the GameLayer_Stack. Keep this in one place so the split and the
        // readback scan cannot disagree.
        const bool menu =
            path.find(kReadbackNames[0]) != std::string_view::npos;
        if (AddReadback(context, handle, menu)) {
          char message[320]{};
          std::snprintf(message, sizeof(message),
                        "nfs2015 chase cam: orbit UI readback resolved: %.*s",
                        static_cast<int>(path.size()), path.data());
          Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
        }
      }
    }
  }

  // The exact object path can resolve the class's CDO WidgetTree rather than
  // the live instance. The live layout and its WidgetTree children are
  // allocated contiguously, so scan a bounded window immediately after the
  // live layout handle and resolve each object name through the v1 name
  // service. This does not depend on a fixed registry index or a hardcoded
  // child address.
  if (context.names == nullptr || context.names->resolve_utf8 == nullptr) {
    if (runtime.readback_count != 0) {
      runtime.readback_ready = true;
      return true;
    }
    runtime.readback_ready = true;
    if (!runtime.readback_unavailable_logged) {
      runtime.readback_unavailable_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: orbit UI readback target scan needs the name "
          "service; readback disabled (Q2=C)");
    }
    return true;
  }

  if (!runtime.readback_scanning) {
    runtime.readback_scanning = true;
    const std::uint32_t layout_index =
        ANOMALY_UE5_OBJECT_HANDLE_INDEX(runtime.layout);
    const std::uint32_t total =
        context.objects->count(context.objects->user);
    runtime.readback_scan_index = layout_index + 1U;
    runtime.readback_scan_count = (std::min)(
        total, static_cast<std::uint32_t>(layout_index + 1U + kScanBatch));
    char message[224]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: orbit UI scanning child window %u..%u "
                  "for readback layers (%u path candidates)",
                  runtime.readback_scan_index, runtime.readback_scan_count,
                  runtime.readback_count);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }

  for (; runtime.readback_scan_index < runtime.readback_scan_count;
       ++runtime.readback_scan_index) {
    AnomalyUe5ObjectSnapshotV1 snapshot{sizeof(snapshot)};
    if (context.objects
            ->snapshot_at(context.objects->user, runtime.readback_scan_index,
                          &snapshot)
            .code != ANOMALY_STATUS_V1_OK) {
      continue;
    }
    char name[128]{};
    std::size_t name_size = sizeof(name);
    if (context.names
            ->resolve_utf8(context.names->user, snapshot.name_id, name,
                           &name_size)
            .code != ANOMALY_STATUS_V1_OK ||
        name_size == 0) {
      continue;
    }
    const std::size_t length = strnlen(name, sizeof(name));
    const bool gameplay_main =
        length == kReadbackNames[0].size() &&
        std::memcmp(name, kReadbackNames[0].data(), length) == 0;
    const bool gameplay_layer =
        length == kReadbackNames[1].size() &&
        std::memcmp(name, kReadbackNames[1].data(), length) == 0;
    if (gameplay_main || gameplay_layer) {
      static_cast<void>(AddReadback(context, snapshot.handle,
                                    gameplay_main));
    }
  }
  if (runtime.readback_scan_index >= runtime.readback_scan_count) {
    runtime.readback_scanning = false;
    runtime.readback_ready = true;
    if (runtime.readback_count == 0 && !runtime.readback_unavailable_logged) {
      runtime.readback_unavailable_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: orbit UI readback layer not found; readback "
          "disabled (Q2=C)");
    } else {
      char message[192]{};
      std::snprintf(message, sizeof(message),
                    "nfs2015 chase cam: orbit UI readback targets=%u",
                    runtime.readback_count);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    }
  }
  return runtime.readback_ready;
}

bool EnsureLayout(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.layout_resolved) {
    return true;
  }
  if (!runtime.layout_scanning) {
    runtime.layout_scanning = true;
    runtime.layout_scan_index = 0;
    runtime.layout_scan_count =
        context.objects->count(context.objects->user);
    char message[192]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: orbit UI scanning %u objects for "
                  "W_OverallUILayout_C",
                  runtime.layout_scan_count);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }
  const std::uint32_t end = (std::min)(
      runtime.layout_scan_count, runtime.layout_scan_index + kScanBatch);
  for (; runtime.layout_scan_index < end; ++runtime.layout_scan_index) {
    AnomalyUe5ObjectSnapshotV1 snapshot{sizeof(snapshot)};
    if (context.objects
            ->snapshot_at(context.objects->user, runtime.layout_scan_index,
                          &snapshot)
            .code != ANOMALY_STATUS_V1_OK) {
      continue;
    }
    if (snapshot.name_id != runtime.layout_name_id ||
        (snapshot.handle.id == runtime.layout_class.id &&
         snapshot.handle.generation == runtime.layout_class.generation)) {
      continue;
    }
    runtime.layout = snapshot.handle;
    runtime.layout_resolved = true;
    runtime.layout_scanning = false;
    char message[224]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: orbit UI layout resolved at index %u",
                  runtime.layout_scan_index);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    return true;
  }
  if (runtime.layout_scan_index >= runtime.layout_scan_count) {
    runtime.layout_scanning = false;
    runtime.next_resolve = context.elapsed + kResolveRetrySeconds;
    if (!runtime.resolve_failed_logged) {
      runtime.resolve_failed_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: W_OverallUILayout_C instance not found");
    }
  }
  return false;
}

bool ReadOrbitUiVisible(Context &context, bool &visible) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.get_render_opacity.id == 0 || runtime.readback_count == 0 ||
      context.events == nullptr || context.events->invoke == nullptr) {
    return false;
  }
  bool any = false;
  bool any_hidden = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    alignas(4) std::array<std::uint8_t, 4> buffer{};
    const auto status = context.events->invoke(
        context.events->user, runtime.readback[i],
        runtime.get_render_opacity, buffer.data(), buffer.size());
    if (status.code != ANOMALY_STATUS_V1_OK) {
      continue;
    }
    float opacity = 1.0F;
    std::memcpy(&opacity, buffer.data(), sizeof(opacity));
    if (!std::isfinite(opacity)) {
      continue;
    }
    any = true;
    if (opacity < 0.5F) {
      any_hidden = true;
    }
  }
  if (!any) {
    return false;
  }
  // GameplayMain_Stack and GameLayer_Stack are transparent (0) while hidden.
  // The path may resolve an archetype as well as the live instance; if any
  // candidate is transparent, the live HUD is hidden.
  visible = !any_hidden;
  return true;
}

// HUD-only visibility: GameLayer_Stack is the gameplay HUD; GameplayMain_Stack
// holds the menu widgets used by ESC, F1-F5, B and C. Keeping the latter at its
// normal opacity is what lets the first UI hotkey open its screen.
bool ApplyOrbitUiHudOpacity(Context &context, const float opacity) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.set_render_opacity.id == 0 || context.events == nullptr ||
      context.events->invoke == nullptr || runtime.readback_count == 0) {
    return false;
  }
  bool any_live = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (runtime.readback_live[i]) {
      any_live = true;
      break;
    }
  }
  bool applied = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (any_live && !runtime.readback_live[i]) {
      continue;
    }
    if (runtime.readback_menu[i]) {
      continue;
    }
    float value = (std::clamp)(opacity, 0.0F, 1.0F);
    const auto status = context.events->invoke(
        context.events->user, runtime.readback[i],
        runtime.set_render_opacity, &value, sizeof(value));
    applied = applied || status.code == ANOMALY_STATUS_V1_OK;
  }
  return applied;
}

bool ApplyOrbitUiTranslation(Context &context, const double y) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.set_render_translation.id == 0 || context.events == nullptr ||
      context.events->invoke == nullptr || runtime.readback_count == 0) {
    return false;
  }
  bool any_live = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (runtime.readback_live[i]) {
      any_live = true;
      break;
    }
  }
  bool applied = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (any_live && !runtime.readback_live[i]) {
      continue;
    }
    if (runtime.readback_menu[i]) {
      continue;
    }
    // UMG's FVector2D is double-precision in this UE5 build; ParmsSize is 16.
    double translation[2]{0.0, y};
    const auto status = context.events->invoke(
        context.events->user, runtime.readback[i],
        runtime.set_render_translation, translation, sizeof(translation));
    applied = applied || status.code == ANOMALY_STATUS_V1_OK;
  }
  return applied;
}

// The native HideAllLayer changes more than the two RenderOpacity fields; it
// also hides the containers that own the ESC menu. A plain SetRenderOpacity on
// the menu stack therefore cannot bring the phone back. RevertLayerShow first
// restores the layer structure, then this split keeps the HUD transparent and
// the menu layer visible.
bool ApplyOrbitUiMenuSplit(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.set_render_opacity.id == 0 || runtime.readback_count == 0) {
    return false;
  }
  bool any_live = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (runtime.readback_live[i]) {
      any_live = true;
      break;
    }
  }
  bool applied = false;
  for (std::uint32_t i = 0; i < runtime.readback_count; ++i) {
    if (any_live && !runtime.readback_live[i]) {
      continue;
    }
    float value = runtime.readback_menu[i] ? 1.0F : 0.0F;
    const auto status = context.events->invoke(
        context.events->user, runtime.readback[i],
        runtime.set_render_opacity, &value, sizeof(value));
    applied = applied || status.code == ANOMALY_STATUS_V1_OK;
  }
  return applied;
}

// ESC is the game's menu, not an orbit restore. Restore the layer structure,
// then hide only the gameplay HUD. The state machine keeps ownership
// (`MenuOpened`), so the eventual orbit exit/stop still restores everything.
bool RevealOrbitUiMenu(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  runtime.fade = {};
  if (!InvokeLayout(context, runtime.revert_layer_show,
                    "RevertLayerShow (ESC menu)")) {
    return false;
  }
  return ApplyOrbitUiMenuSplit(context);
}

// Begin a short opacity ramp. Hide commits HideAllLayer when the ramp reaches
// zero; show calls RevertLayerShow first, then starts from zero so the native
// restore does not pop. The state machine sees the action as accepted
// immediately; its first readback poll is 0.25 s away, after this 0.18 s ramp.
bool BeginOrbitUiFade(Context &context, const bool hiding) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (runtime.set_render_opacity.id == 0 || runtime.readback_count == 0) {
    return false;
  }
  if (!hiding) {
    // RevertLayerShow also restores the menu layer; the opacity path never
    // hid it, so there is nothing to revert here.
    runtime.full_layer_hide = false;
  }
  runtime.fade.active = true;
  runtime.fade.hiding = hiding;
  runtime.fade.start = context.elapsed;
  runtime.fade.duration = kOrbitUiFadeSeconds;
  runtime.fade.from = hiding ? 1.0F : 0.0F;
  runtime.fade.to = hiding ? 0.0F : 1.0F;
  static_cast<void>(ApplyOrbitUiTranslation(
      context, hiding ? 0.0 : kOrbitUiFadeTranslation));
  if (!ApplyOrbitUiHudOpacity(context, runtime.fade.from)) {
    runtime.fade.active = false;
    return false;
  }
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      hiding ? "nfs2015 chase cam: orbit UI fade-out"
             : "nfs2015 chase cam: orbit UI fade-in");
  return true;
}

bool UpdateOrbitUiFade(Context &context, const double now) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (!runtime.fade.active) {
    return false;
  }
  const double duration =
      runtime.fade.duration > 1e-9 ? runtime.fade.duration : kOrbitUiFadeSeconds;
  double t = (now - runtime.fade.start) / duration;
  t = std::isfinite(t) ? (std::clamp)(t, 0.0, 1.0) : 1.0;
  // Smoothstep keeps both ends of the fade gentle; the game's ESC transition
  // reads as the same short ease rather than a linear pop.
  const float eased = static_cast<float>(t * t * (3.0 - 2.0 * t));
  const float opacity =
      runtime.fade.from + (runtime.fade.to - runtime.fade.from) * eased;
  static_cast<void>(ApplyOrbitUiHudOpacity(context, opacity));
  const double translation =
      runtime.fade.hiding
          ? kOrbitUiFadeTranslation * eased
          : kOrbitUiFadeTranslation * (1.0 - eased);
  static_cast<void>(ApplyOrbitUiTranslation(context, translation));
  if (t < 1.0) {
    return true;
  }
  runtime.fade = {};
  return true;
}

bool EscapePressed(Context &context) noexcept {
  if (context.input == nullptr || context.input->was_pressed == nullptr) {
    return false;
  }
  std::int32_t pressed = 0;
  constexpr std::uint32_t kEscapeVirtualKey = 0x1B;
  const auto status = context.input->was_pressed(
      context.input->user, kEscapeVirtualKey, &pressed);
  return status.code == ANOMALY_STATUS_V1_OK && pressed != 0;
}

bool Execute(Context &context, const nfs2015_chase::OrbitUiHideAction action) noexcept {
  if (action == nfs2015_chase::OrbitUiHideAction::kHide) {
    if (BeginOrbitUiFade(context, true)) {
      context.orbit_ui.full_layer_hide = false;
      return true;
    }
    context.orbit_ui.full_layer_hide = true;
    return InvokeLayout(context, context.orbit_ui.hide_all_layer,
                        "HideAllLayer");
  }
  if (action == nfs2015_chase::OrbitUiHideAction::kRestore) {
    if (BeginOrbitUiFade(context, false)) {
      context.orbit_ui.full_layer_hide = false;
      return true;
    }
    context.orbit_ui.full_layer_hide = false;
    return InvokeLayout(context, context.orbit_ui.revert_layer_show,
                        "RevertLayerShow");
  }
  return true;
}

void LogPhaseIfChanged(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  const auto phase = runtime.machine.state().phase;
  if (phase == runtime.logged_phase) {
    return;
  }
  runtime.logged_phase = phase;
  char message[256]{};
  const auto &state = runtime.machine.state();
  std::snprintf(message, sizeof(message),
                "nfs2015 chase cam: orbit UI phase=%s was_hidden=%d hidden=%d "
                "intervened=%d readback=%d hides=%u restores=%u errors=%u",
                nfs2015_chase::OrbitUiHidePhaseName(phase),
                state.was_hidden ? 1 : 0, state.hidden_by_us ? 1 : 0,
                state.intervened ? 1 : 0,
                state.readback_supported ? 1 : 0, state.hide_requests,
                state.restore_requests, state.errors);
  Log(context,
      phase == nfs2015_chase::OrbitUiHidePhase::kFailed
          ? ANOMALY_CORE_LOG_LEVEL_V1_WARNING
          : ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      message);
}

}  // namespace

namespace {
void ANOMALY_CALL ClearOrbitHudMarkerTask(
    void *user, AnomalyGenerationHandleV1 /*task*/) noexcept {
  auto *context = static_cast<Context *>(user);
  if (context != nullptr) {
    ClearOrbitHudMarker(*context);
  }
}
}  // namespace

void LoadOrbitUiMarker(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (!StorageReady(context.storage)) {
    return;
  }
  std::array<std::uint8_t, 16> buffer{};
  std::size_t size = buffer.size();
  const auto status = context.storage->read(
      context.storage->user, StringView(kOrbitHudMarkerPath),
      AnomalyMutableByteSpanV1{buffer.data(), buffer.size()}, &size);
  if (status.code == ANOMALY_STATUS_V1_OK && size != 0 &&
      buffer[0] == static_cast<std::uint8_t>('1')) {
    runtime.startup_restore_pending = true;
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
        "nfs2015 chase cam: orbit UI recovery marker found; restore pending");
  } else if (status.code != ANOMALY_STATUS_V1_OK &&
             status.code != ANOMALY_STATUS_V1_NOT_FOUND) {
    runtime.marker_error_logged = true;
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
        "nfs2015 chase cam: could not read orbit UI recovery marker");
  }
}

void ResetOrbitUi(Context &context) noexcept {
  const bool startup_restore_pending =
      context.orbit_ui.startup_restore_pending;
  context.orbit_ui = OrbitUiRuntimeState{};
  context.orbit_ui.startup_restore_pending = startup_restore_pending;
}

void RestoreOrbitUiOnStop(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  runtime.fade = {};
  const bool startup_pending = runtime.startup_restore_pending;
  const auto action = runtime.machine.Reset();
  bool restore_pending = startup_pending ||
                         action == nfs2015_chase::OrbitUiHideAction::kRestore;
  if (restore_pending) {
    // on_stop runs on a Host worker thread. ProcessEvent.invoke requires the
    // UE Game thread, so this call normally fails with CONFLICT(8). Persist a
    // marker first; the next plugin generation restores from its Game tick.
    // State storage is deliberately forbidden from Game/render callbacks, so
    // this worker thread is also the only place marker I/O can run. A pending
    // startup marker is preserved even when the local machine has no episode:
    // the previous generation may have hidden the HUD before its Game tick
    // could consume it.
    WriteOrbitHudMarker(context);
    const bool restored =
        InvokeLayout(context, runtime.revert_layer_show,
                     "RevertLayerShow (stop)");
    if (restored) {
      ClearOrbitHudMarker(context);
      restore_pending = false;
    }
  } else {
    // A clean stop while the HUD is visible clears any stale marker from a
    // previous generation.
    ClearOrbitHudMarker(context);
  }
  context.orbit_ui = OrbitUiRuntimeState{};
  // Keep the recovery bit for a Stop/Start cycle that does not unload the
  // module. The next Game tick will retry RevertLayerShow from the marker.
  context.orbit_ui.startup_restore_pending = restore_pending;
}

void UpdateOrbitUiHide(Context &context) noexcept {
  OrbitUiRuntimeState &runtime = context.orbit_ui;
  if (context.objects == nullptr || context.events == nullptr) {
    return;
  }
  if (!RefreshGeneration(context)) {
    return;
  }

  const std::uint32_t mode =
      context.presenter_orbit_mode.load(std::memory_order_relaxed);
  const auto orbit_phase =
      static_cast<nfs2015_chase::OrbitPhase>(mode);
  const bool driving =
      context.view_mode.in_vehicle.load(std::memory_order_relaxed);
  // `ChaseSlotAllowed` is the same gate the camera uses: it includes the
  // first-person/cinematic/photo override. Using `chase_wanted` alone left the
  // HUD hidden when the game's own camera took over without a Z press.
  const bool chase_view = ChaseSlotAllowed(context);
  const std::uint64_t view_epoch =
      context.view_epoch.load(std::memory_order_acquire);
  const bool view_epoch_changed =
      runtime.view_epoch_initialized && runtime.view_epoch != view_epoch;
  runtime.view_epoch = view_epoch;
  runtime.view_epoch_initialized = true;
  // The exit transition no longer owns the HUD. Restore on the first kExit
  // tick so the fade-in happens during the pull-out, not after it.
  const bool wants_hide =
      nfs2015_chase::OrbitPhaseHidesHud(orbit_phase) && driving &&
      chase_view && !view_epoch_changed &&
      context.orbit_enabled.load(std::memory_order_acquire) &&
      context.orbit_hide_ui.load(std::memory_order_acquire);

  const bool startup_pending = runtime.startup_restore_pending;
  const bool episode_open = runtime.machine.EpisodeOpen();
  const bool fade_active = runtime.fade.active;
  if (!wants_hide && !runtime.machine.state().hidden_by_us &&
      !episode_open && !startup_pending && !fade_active) {
    // Idle and not owning the HUD: no guest resolution is needed.
    return;
  }

  if (!EnsureStatics(context)) {
    return;
  }
  if (!EnsureLayout(context)) {
    return;
  }
  if (!runtime.layout_resolved) {
    // An exit needs the layout too; without it there is nothing safe to call.
    return;
  }

  // Keep the menu visible and the HUD hidden while the game's own ESC handler
  // settles. Without this, the handler can animate the HUD from visible to
  // hidden again after RevertLayerShow.
  if (runtime.full_layer_hide && runtime.menu_hold_until > context.elapsed) {
    static_cast<void>(ApplyOrbitUiMenuSplit(context));
  }

  // A ramp is presentation-only; advance it before the state machine decides
  // whether this tick still needs a native hide/restore call.
  if (runtime.fade.active) {
    static_cast<void>(UpdateOrbitUiFade(context, context.elapsed));
  }

  // HUD-only mode: the game restores its own HUD when an F1/F2/ESC menu
  // closes. Reassert the transparent gameplay layer every tick while the orbit
  // still owns it, so closing a menu does not leave the HUD behind the orbit.
  if (!runtime.full_layer_hide && wants_hide &&
      runtime.machine.state().hidden_by_us && !runtime.fade.active) {
    static_cast<void>(ApplyOrbitUiHudOpacity(context, 0.0F));
    static_cast<void>(ApplyOrbitUiTranslation(context,
                                              kOrbitUiFadeTranslation));
  }

  if ((wants_hide || startup_pending) &&
      !ResolveReadbackTargets(context)) {
    // Do not hide before the entry readback is available: Q2=A needs to know
    // whether the player already had the HUD hidden. Startup recovery also
    // waits for the readback scan so it can avoid a blind restore.
    return;
  }
  bool readback_valid = false;
  bool readback_visible = false;
  if (runtime.readback_ready) {
    readback_valid = ReadOrbitUiVisible(context, readback_visible);
  }

  if (startup_pending) {
    if (context.elapsed + 1e-12 < runtime.next_startup_restore) {
      return;
    }
    if (readback_valid && readback_visible) {
      // The marker is stale; the HUD is already visible.
      runtime.startup_restore_pending = false;
      ScheduleMarkerClear(context);
    } else {
      const bool restored = InvokeLayout(
          context, runtime.revert_layer_show,
          "RevertLayerShow (startup recovery)");
      if (restored) {
        runtime.startup_restore_pending = false;
        readback_valid = true;
        readback_visible = true;
        ScheduleMarkerClear(context);
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
            "nfs2015 chase cam: orbit UI startup recovery complete");
      } else {
        runtime.next_startup_restore =
            context.elapsed + kStartupRestoreRetrySeconds;
        if (!runtime.startup_restore_logged) {
          runtime.startup_restore_logged = true;
          Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
              "nfs2015 chase cam: orbit UI startup recovery failed; will "
              "retry");
        }
        return;
      }
    }
  }

  // An ESC intervention leaves the episode open with no hidden state. Keep
  // feeding the model while idle so that episode is closed before the next
  // orbit; the first early return above would otherwise latch `entered_`.
  if (!wants_hide && !runtime.machine.state().hidden_by_us && !episode_open) {
    return;
  }

  nfs2015_chase::OrbitUiHideInput input;
  input.orbit_mode = orbit_phase;
  input.orbit_enabled =
      context.orbit_enabled.load(std::memory_order_acquire);
  input.hide_ui_enabled =
      context.orbit_hide_ui.load(std::memory_order_acquire);
  input.driving = driving;
  input.chase_view = chase_view;
  // In HUD-only mode a visible readback is the game's own temporary HUD
  // restore (menu close), not a player intervention. Let the state machine
  // keep ownership and reassert transparency above.
  input.readback_valid = readback_valid && runtime.full_layer_hide;
  input.readback_visible = readback_visible;
  input.invoke_ok = runtime.last_invoke_ok;
  input.now_seconds = context.elapsed;

  if (runtime.full_layer_hide && runtime.machine.state().hidden_by_us &&
      EscapePressed(context)) {
    if (runtime.machine.MenuOpened()) {
      runtime.menu_hold_until = context.elapsed + 0.75;
      runtime.last_invoke_ok = RevealOrbitUiMenu(context);
      if (!runtime.last_invoke_ok) {
        // No SetRenderOpacity target: fall back to the old full restore so the
        // menu is still usable. This is the only path that can replay the
        // game's own ESC hide animation.
        runtime.last_invoke_ok = InvokeLayout(
            context, runtime.revert_layer_show,
            "RevertLayerShow (ESC fallback)");
      }
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
          "nfs2015 chase cam: orbit UI ESC menu; revealing menu layer");
      LogPhaseIfChanged(context);
      return;
    }
  }

  const auto action = runtime.machine.Update(input);
  if (action == nfs2015_chase::OrbitUiHideAction::kHide ||
      action == nfs2015_chase::OrbitUiHideAction::kRestore) {
    // Marker I/O is forbidden from the Game thread. Stop owns the marker:
    // it writes it while hidden and clears it while visible.
    runtime.last_invoke_ok = Execute(context, action);
  } else {
    runtime.last_invoke_ok = true;
  }
  LogPhaseIfChanged(context);
}

}  // namespace nfs2015_chase_cam::detail
