// Definitions for the SDK plumbing declared in sdk.hpp.
#include "sdk.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace nfs2015_chase_cam::detail {

AnomalyStatusV1 Status(const std::uint32_t code,
                       const std::string_view message) noexcept {
  return {code, 0, {message.data(), message.size()}};
}

AnomalyByteSpanV1 Bytes(const std::string_view value) noexcept {
  return {reinterpret_cast<const std::uint8_t *>(value.data()), value.size()};
}

[[nodiscard]] std::int64_t NowNanoseconds() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool CoreReady(const AnomalyCoreServiceV1 *service) noexcept {
  return HasField<AnomalyCoreServiceV1,
                  decltype(AnomalyCoreServiceV1::write_memory)>(
             service, offsetof(AnomalyCoreServiceV1, write_memory)) &&
         service->log != nullptr && service->read_memory != nullptr &&
         service->write_memory != nullptr;
}

bool ConfigReady(const AnomalyConfigServiceV1 *service) noexcept {
  return HasField<AnomalyConfigServiceV1,
                  decltype(AnomalyConfigServiceV1::unregister_schema)>(
             service, offsetof(AnomalyConfigServiceV1, unregister_schema)) &&
         service->register_schema != nullptr && service->read != nullptr &&
         service->write_atomic != nullptr &&
         service->unregister_schema != nullptr;
}

bool JsonReady(const AnomalyJsonServiceV1 *service) noexcept {
  return HasField<AnomalyJsonServiceV1,
                  decltype(AnomalyJsonServiceV1::serialize)>(
             service, offsetof(AnomalyJsonServiceV1, serialize)) &&
         service->parse != nullptr && service->release != nullptr &&
         service->kind != nullptr && service->boolean_value != nullptr &&
         service->number_value != nullptr &&
         service->object_find != nullptr &&
         service->object_size != nullptr;
}

bool UiReady(const AnomalyUiServiceV1 *service) noexcept {
  return HasField<AnomalyUiServiceV1,
                  decltype(AnomalyUiServiceV1::slider_float)>(
             service, offsetof(AnomalyUiServiceV1, slider_float)) &&
         service->set_next_window_size != nullptr &&
         service->begin_window != nullptr && service->end_window != nullptr &&
         service->text != nullptr && service->button != nullptr &&
         service->checkbox != nullptr && service->slider_float != nullptr &&
         service->separator != nullptr;
}

bool SignatureReady(const AnomalySignatureServiceV1 *service) noexcept {
  return service != nullptr && service->resolve != nullptr;
}

bool HookReady(const AnomalyHookServiceV1 *service) noexcept {
  return service != nullptr && service->create != nullptr &&
         service->release != nullptr && service->begin_callback != nullptr &&
         service->end_callback != nullptr;
}

bool WindowReady(const AnomalyWindowServiceV1 *service) noexcept {
  return service != nullptr && service->register_window != nullptr &&
         service->release_window != nullptr && service->set_open != nullptr &&
         service->state != nullptr && service->begin != nullptr &&
         service->end != nullptr;
}

bool PlayerReady(const AnomalyNtePlayerServiceV1 *service) noexcept {
  return service != nullptr && service->snapshot != nullptr;
}

bool PlayerHandlesReady(const AnomalyNtePlayerServiceV1 *service) noexcept {
  return HasField<AnomalyNtePlayerServiceV1,
                  decltype(AnomalyNtePlayerServiceV1::get_object_handles)>(
             service,
             offsetof(AnomalyNtePlayerServiceV1, get_object_handles)) &&
         service->get_object_handles != nullptr;
}

bool Ue5ObjectsReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept {
  // find_exact is the appended V1 tail; older hosts only expose the registry
  // snapshots and cannot resolve a UFunction by path.
  return HasField<AnomalyUe5ObjectsServiceV1,
                  decltype(AnomalyUe5ObjectsServiceV1::find_exact)>(
             service, offsetof(AnomalyUe5ObjectsServiceV1, find_exact)) &&
         service->find_exact != nullptr;
}

bool Ue5NamesFindReady(const AnomalyUe5NamesServiceV1 *service) noexcept {
  // The struct is size-versioned: a v1 host stops after resolve_ftext_utf8, so
  // the tail has to be checked before it is read.
  return HasField<AnomalyUe5NamesServiceV1,
                  decltype(AnomalyUe5NamesServiceV1::find_utf8)>(
             service, offsetof(AnomalyUe5NamesServiceV1, find_utf8)) &&
         service->find_utf8 != nullptr;
}

bool Ue5ProcessEventInvokeReady(
    const AnomalyUe5ProcessEventServiceV1 *service) noexcept {
  return HasField<AnomalyUe5ProcessEventServiceV1,
                  decltype(AnomalyUe5ProcessEventServiceV1::invoke)>(
             service, offsetof(AnomalyUe5ProcessEventServiceV1, invoke)) &&
         service->invoke != nullptr;
}

// Exact-speed support needs the registry snapshot call to turn the raw vehicle
// pointer returned by BP_GetCurrentDriveVehicle into a ProcessEvent handle.
bool Ue5ObjectSnapshotReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept {
  return Ue5ObjectsReady(service) && service->generation != nullptr &&
         service->snapshot_at != nullptr;
}

bool SnapshotValid(const std::uint32_t flags) noexcept {
  // STALE only means the sampler has not refreshed this tick (the interval is
  // configurable); the last sample is still the best known player/vehicle
  // state. PARTIAL means the optional ESP/camera tail is missing, and the
  // fallback origin handles that. Neither makes the position unusable.
  return (flags & ANOMALY_NTE_SNAPSHOT_V1_VALID) != 0;
}

bool AddAddress(const std::uintptr_t base, const std::uint64_t offset,
                std::uintptr_t &result) noexcept {
  if (base == 0 || offset > (std::numeric_limits<std::uintptr_t>::max)() - base) {
    return false;
  }
  result = base + static_cast<std::uintptr_t>(offset);
  return true;
}

bool ReadPointerAtOffset(const AnomalyCoreServiceV1 *core,
                         const std::uintptr_t base, const std::uint32_t offset,
                         std::uintptr_t &value) noexcept {
  std::uintptr_t address{};
  return AddAddress(base, offset, address) && Read(core, address, value) &&
         value != 0;
}

void Log(const Context &context, const std::uint32_t level,
         const std::string_view message) noexcept {
  if (context.core == nullptr || context.core->log == nullptr) {
    return;
  }
  context.core->log(context.core->user, level, StringView(message));
}

// Mark the published view history as discontinuous. The reason is accumulated
// and consumed by PublishView, so a reset requested while the camera is idle
// costs nothing and is not reported as a snap that actually happened.
void NoteViewSnap(Context &context, const SnapReason reason) noexcept {
  context.diag.reset_window.fetch_add(1, std::memory_order_relaxed);
  context.view_snap = true;
  context.view_snap_reasons |= static_cast<std::uint32_t>(reason);
}

void NoteYawSource(Context &context, const YawSource source) noexcept {
  const auto value = static_cast<std::uint32_t>(source);
  if (context.yaw_source == value) {
    return;
  }
  if (context.yaw_source != kYawSourceNone) {
    context.diag.yaw_source_switches.fetch_add(1, std::memory_order_relaxed);
  }
  context.yaw_source = value;
}

// Q1: a second calling thread would mean the single-clock presenter planned in
// T2 needs explicit serialisation, so record the identities now.
void NoteDetourThread(Context &context) noexcept {
  const auto thread = static_cast<std::uint32_t>(GetCurrentThreadId());
  for (std::size_t index = 0; index < kDiagnosticThreadSlots; ++index) {
    std::uint32_t slot =
        context.diag.detour_threads[index].load(std::memory_order_relaxed);
    if (slot == thread) {
      return;
    }
    if (slot == 0 &&
        context.diag.detour_threads[index].compare_exchange_strong(
            slot, thread, std::memory_order_relaxed,
            std::memory_order_relaxed)) {
      context.diag.detour_thread_count.fetch_add(1,
                                                std::memory_order_relaxed);
      return;
    }
    if (slot == thread) {
      return;
    }
  }
}

}  // namespace nfs2015_chase_cam::detail
