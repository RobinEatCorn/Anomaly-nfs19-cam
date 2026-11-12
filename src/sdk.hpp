// Anomaly SDK plumbing shared by every module: the detour function-pointer
// types, the service readiness predicates, the typed read/write wrappers, the
// log bridge, and the atomics that hold the installed detour originals.
#pragma once

#include "state.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace nfs2015_chase_cam::detail {

using anomaly::sdk::StringView;

using CameraViewPointFn = void(ANOMALY_CALL *)(void *, double *, double *);
// `HTPlayerController::ToggleSwitchCameraViewMode(bool bForward)`. Both the
// argument and the result are carried as raw bytes on purpose: the game's own
// body only ever tests the flag for zero, and taking it as `bool` would let the
// compiler canonicalise a value the game did not produce. The detour forwards
// and returns exactly the byte it received.
using ToggleViewModeFn = std::uint8_t(ANOMALY_CALL *)(void *, std::uint8_t);
// The virtual transform notification. The second argument is the scene/command
// context SetTransform forwards; it is passed through unchanged.
using OnTransformChangedFn = void(ANOMALY_CALL *)(void *, void *);
// FViewInfo::SetupUniformBufferParameters. The first four arguments are
// pointers; the fifth is a small flag and the sixth an output pointer. It runs
// late in the renderer's view setup, once per view, and is the stable place to
// observe the view the renderer is actually about to use.
using SetupUniformBufferFn = void(ANOMALY_CALL *)(void *, void *, void *,
                                                  void *, std::uint64_t,
                                                  void *) noexcept;
// FViewMatrices::FViewMatrices(const FSceneViewInitOptions&). The constructor
// derives all view/projection matrices from the supplied rotation/projection
// and origin. Calling it is preferable to reimplementing its derived matrices.
using ViewMatricesCtorFn = void(ANOMALY_CALL *)(void *, const void *) noexcept;
// FSceneRenderer::PrepareViewStateForVisibility. Called once per renderer
// immediately before visibility tasks are launched; the second argument points
// at the scene textures config embedded in the renderer.
using PrepareViewStateFn = void(ANOMALY_CALL *)(void *, const void *) noexcept;
// FSceneView::UpdateViewMatrix() and the static FSceneView::SetupViewFrustum().
// The former rebuilds ViewMatrices from ViewLocation/ViewRotation; the latter
// derives both ViewFrustum and CullingFrustum afterwards.
using SceneViewUpdateMatrixFn = void(ANOMALY_CALL *)(void *) noexcept;
using SceneViewSetupFrustumFn = void(ANOMALY_CALL *)(void *) noexcept;

template <typename Struct, typename Field>
bool HasField(const Struct *value, const std::size_t offset) noexcept {
  return value != nullptr && value->struct_size >= offset + sizeof(Field);
}

AnomalyStatusV1 Status(std::uint32_t code,
                       std::string_view message = {}) noexcept;
AnomalyByteSpanV1 Bytes(std::string_view value) noexcept;
[[nodiscard]] std::int64_t NowNanoseconds() noexcept;

bool CoreReady(const AnomalyCoreServiceV1 *service) noexcept;
bool ConfigReady(const AnomalyConfigServiceV1 *service) noexcept;
bool JsonReady(const AnomalyJsonServiceV1 *service) noexcept;
bool InputReady(const AnomalyInputServiceV1 *service) noexcept;
bool UiReady(const AnomalyUiServiceV1 *service) noexcept;
bool SignatureReady(const AnomalySignatureServiceV1 *service) noexcept;
bool HookReady(const AnomalyHookServiceV1 *service) noexcept;
bool WindowReady(const AnomalyWindowServiceV1 *service) noexcept;
bool PlayerReady(const AnomalyNtePlayerServiceV1 *service) noexcept;
bool PlayerHandlesReady(const AnomalyNtePlayerServiceV1 *service) noexcept;
bool Ue5ObjectsReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept;
// The names service shipped `find_utf8` in v2. A v1 host (or none at all)
// answers false here, and every consumer must treat that as "cannot name
// anything in this process" rather than as an empty pool.
bool Ue5NamesFindReady(const AnomalyUe5NamesServiceV1 *service) noexcept;
bool Ue5ProcessEventInvokeReady(
    const AnomalyUe5ProcessEventServiceV1 *service) noexcept;
bool Ue5ObjectSnapshotReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept;
bool SnapshotValid(std::uint32_t flags) noexcept;

template <typename Service>
const Service *Query(const AnomalyHostApiV1 *host, const char *id,
                     const std::uint32_t version) noexcept {
  if (!HasField<AnomalyHostApiV1, decltype(AnomalyHostApiV1::query_service)>(
          host, offsetof(AnomalyHostApiV1, query_service)) ||
      host->query_service == nullptr) {
    return nullptr;
  }
  const void *table{};
  if (host->query_service(host->host_context, StringView(id), version, &table)
              .code != ANOMALY_STATUS_V1_OK ||
      table == nullptr) {
    return nullptr;
  }
  const auto *service = static_cast<const Service *>(table);
  constexpr std::size_t prefix = offsetof(Service, user) + sizeof(void *);
  return service->struct_size >= prefix && service->service_version >= version
             ? service
             : nullptr;
}

template <typename T>
bool Read(const AnomalyCoreServiceV1 *core, const std::uintptr_t address,
          T &value) noexcept {
  if (core == nullptr || core->read_memory == nullptr || address == 0) {
    return false;
  }
  AnomalyMutableByteSpanV1 destination{reinterpret_cast<std::uint8_t *>(&value),
                                       sizeof(value)};
  return core->read_memory(core->user, address, destination).code ==
         ANOMALY_STATUS_V1_OK;
}

template <typename T>
bool Write(const AnomalyCoreServiceV1 *core, const std::uintptr_t address,
           const T &value) noexcept {
  if (core == nullptr || core->write_memory == nullptr || address == 0) {
    return false;
  }
  AnomalyByteSpanV1 source{reinterpret_cast<const std::uint8_t *>(&value),
                           sizeof(value)};
  return core->write_memory(core->user, address, source).code ==
         ANOMALY_STATUS_V1_OK;
}

bool AddAddress(std::uintptr_t base, std::uint64_t offset,
                std::uintptr_t &result) noexcept;
bool ReadPointerAtOffset(const AnomalyCoreServiceV1 *core,
                         std::uintptr_t base, std::uint32_t offset,
                         std::uintptr_t &value) noexcept;

void Log(const Context &context, std::uint32_t level,
         std::string_view message) noexcept;
void NoteViewSnap(Context &context, SnapReason reason) noexcept;
void NoteYawSource(Context &context, YawSource source) noexcept;
void NoteDetourThread(Context &context) noexcept;

// The installed detour originals. Written once under the hook lock while the
// hook is installed and read on the hook's own thread afterwards.
extern std::atomic<Context *> g_active;
extern std::atomic<CameraViewPointFn> g_original;
extern std::atomic<ToggleViewModeFn> g_toggle_view_mode_original;
// The toggle detour publishes its own context instead of sharing `g_active`:
// releasing any one camera hook clears `g_active`, and a toggle that silently
// stopped being observed would look exactly like a Z key that does nothing.
extern std::atomic<Context *> g_view_mode_context;
extern std::atomic<OnTransformChangedFn> g_on_transform_changed_original;
extern std::atomic<SetupUniformBufferFn> g_setup_uniform_buffer_original;
extern std::atomic<ViewMatricesCtorFn> g_view_matrices_ctor;
extern std::atomic<PrepareViewStateFn> g_prepare_view_state_original;
extern std::atomic<SceneViewUpdateMatrixFn> g_scene_view_update_matrix;
extern std::atomic<SceneViewSetupFrustumFn> g_scene_view_setup_frustum;

}  // namespace nfs2015_chase_cam::detail
