// Z-key view-mode recon (PLAN-zkey-phase2-h1.md, task group 0).
//
// Task group 0 answers one question before the cycle is written: which call
// site does the game's Z key actually reach, and with which `bForward`? The
// Phase 1 reconstruction found eight virtual call sites for
// `HTPlayerController::ToggleSwitchCameraViewMode` and could not tell which of
// them a key press takes, because the key arrives through a Blueprint graph.
//
// So this module is deliberately log-only. It installs one detour on the
// function, records the state the game is in when the call arrives, and then
// forwards the call untouched on every path. Nothing here changes the view
// mode, swallows a key, or writes memory; the worst case of a stale offset is
// a wrong number in the log.
//
// What the log is meant to settle:
//   * the return address inside the detour, matched against the eight known
//     call sites from REPORT-tg3-routes.md section 6.1;
//   * whether a key press arrives as `bForward == false` (the manual path with
//     its 0.5 s cooldown) or as `bForward == true` (the scripted path that
//     clears the cooldown);
//   * what `limitManualToggle` reads at that moment, which is the counter the
//     Blueprint side is expected to check before calling;
//   * whether entering or leaving a vehicle calls the toggle at all;
//   * whether the array is still exactly {ThirdPerson, FirstPerson}, which is
//     the precondition for taking the cycle over at all.
#include "internal.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace nfs2015_chase_cam::detail {
namespace {

// The caller's return address, i.e. the instruction after the call that landed
// in the detour. It is read in the detour's own frame: `__builtin_return_address`
// reports the frame it is compiled into, so a helper in this file would report
// its own caller and name a site inside the plugin.
#if defined(_MSC_VER)
#define NFS2015_RETURN_ADDRESS() \
  reinterpret_cast<std::uintptr_t>(_ReturnAddress())
#else
#define NFS2015_RETURN_ADDRESS() \
  reinterpret_cast<std::uintptr_t>(__builtin_return_address(0))
#endif

// Closes a `begin_callback` lease when it goes out of scope. The host's stop
// barrier waits for every lease it handed out, so a detour that returns without
// `end_callback` blocks the plugin's own unload and quarantines it.
class HookCallbackLease final {
 public:
  explicit HookCallbackLease(const AnomalyHookServiceV1 *hook) noexcept
      : hook_(hook) {}

  HookCallbackLease(const HookCallbackLease &) = delete;
  HookCallbackLease &operator=(const HookCallbackLease &) = delete;

  ~HookCallbackLease() {
    if (active_ && hook_ != nullptr && hook_->end_callback != nullptr) {
      static_cast<void>(hook_->end_callback(hook_->user, handle_));
    }
  }

  void Begin(const AnomalyGenerationHandleV1 hook_handle) noexcept {
    if (hook_ == nullptr || hook_->begin_callback == nullptr) {
      return;
    }
    active_ = hook_->begin_callback(hook_->user, hook_handle, &handle_).code ==
              ANOMALY_STATUS_V1_OK;
  }

  [[nodiscard]] bool active() const noexcept { return active_; }

 private:
  const AnomalyHookServiceV1 *hook_{};
  AnomalyGenerationHandleV1 handle_{};
  bool active_{};
};

// The PlayerController's view-mode block, copied out in one read so the fields
// cannot disagree with each other because the game moved between two reads.
struct ViewModeSnapshot final {
  std::uint64_t tag{};
  std::uintptr_t array{};
  std::int32_t count{};
  std::int32_t capacity{};
  std::int32_t limit{};
  float cooldown{};
  float grace{};
  double cooldown_clock{};
  std::uint8_t grace_used{};
  std::uint64_t modes[kMaximumLoggedViewModes]{};
  std::size_t modes_read{};
};

// Tag text. `kind` is the pre-resolved id table; the detour never asks the host
// to resolve a name, so an unknown tag is printed as a bare id and can be
// resolved offline with `anomaly-cli ue fname`.
struct TagNames final {
  std::uint32_t third_person{};
  std::uint32_t first_person{};
  std::uint32_t fixed{};
  std::uint32_t seat{};
  std::uint32_t driving{};
};

const char *TagLabel(const TagNames &names, const std::uint64_t tag) noexcept {
  const auto id = static_cast<std::uint32_t>(tag & 0xFFFFFFFFU);
  if (id == 0) {
    return "none";
  }
  if (id == names.third_person) {
    return "TP";
  }
  if (id == names.first_person) {
    return "FP";
  }
  if (id == names.fixed) {
    return "Fixed";
  }
  if (id == names.seat) {
    return "Seat";
  }
  if (id == names.driving) {
    return "Driving";
  }
  return "?";
}

TagNames ReadNames(const ViewModeState &state) noexcept {
  TagNames names{};
  names.third_person =
      state.tag_third_person.load(std::memory_order_relaxed);
  names.first_person = state.tag_first_person.load(std::memory_order_relaxed);
  names.fixed = state.tag_fixed.load(std::memory_order_relaxed);
  names.seat = state.tag_seat.load(std::memory_order_relaxed);
  names.driving = state.tag_driving.load(std::memory_order_relaxed);
  return names;
}

bool ReadViewModeState(const Context &context, const std::uintptr_t pc,
                       ViewModeSnapshot &snapshot) noexcept {
  if (pc == 0 || context.core == nullptr) {
    return false;
  }
  std::uintptr_t address{};
  if (!AddAddress(pc, kPcViewModeTagOffset, address)) {
    return false;
  }
  std::uint8_t raw[kPcViewModeStateSpan]{};
  AnomalyMutableByteSpanV1 destination{raw, sizeof(raw)};
  if (context.core->read_memory == nullptr ||
      context.core->read_memory(context.core->user, address, destination)
              .code != ANOMALY_STATUS_V1_OK) {
    return false;
  }
  // Offsets inside the copied block, i.e. relative to kPcViewModeTagOffset.
  std::memcpy(&snapshot.tag, raw + 0x00, sizeof(snapshot.tag));
  std::memcpy(&snapshot.array, raw + 0x08, sizeof(snapshot.array));
  std::memcpy(&snapshot.count, raw + 0x10, sizeof(snapshot.count));
  std::memcpy(&snapshot.capacity, raw + 0x14, sizeof(snapshot.capacity));
  std::memcpy(&snapshot.limit, raw + 0x18, sizeof(snapshot.limit));
  std::memcpy(&snapshot.cooldown, raw + 0x1C, sizeof(snapshot.cooldown));
  std::memcpy(&snapshot.grace, raw + 0x20, sizeof(snapshot.grace));
  std::memcpy(&snapshot.cooldown_clock, raw + 0x28,
              sizeof(snapshot.cooldown_clock));
  snapshot.grace_used = raw[0x30];

  if (snapshot.array == 0 || snapshot.count <= 0) {
    return true;
  }
  const auto readable = static_cast<std::size_t>(snapshot.count) >
                                kMaximumLoggedViewModes
                            ? kMaximumLoggedViewModes
                            : static_cast<std::size_t>(snapshot.count);
  std::uint8_t entries[kMaximumLoggedViewModes * kViewModeEntryStride]{};
  AnomalyMutableByteSpanV1 modes{entries,
                                 readable * kViewModeEntryStride};
  if (context.core->read_memory(context.core->user, snapshot.array, modes)
          .code != ANOMALY_STATUS_V1_OK) {
    return true;
  }
  snapshot.modes_read = readable;
  for (std::size_t index = 0; index < readable; ++index) {
    std::memcpy(&snapshot.modes[index],
                entries + index * kViewModeEntryStride,
                sizeof(snapshot.modes[index]));
  }
  return true;
}

void LogCall(Context &context, const std::uint8_t b_forward_raw,
             const void *self, const std::uintptr_t site,
             const ViewModeSnapshot &snapshot, const bool have_snapshot,
             const std::uint8_t result, const std::uint64_t call_index,
             const std::uint64_t tag_after, const bool have_tag_after,
             const char *decision) noexcept {
  const TagNames names = ReadNames(context.view_mode);
  const bool enabled = context.enabled.load(std::memory_order_relaxed);
  const bool view_active =
      context.view_active.load(std::memory_order_relaxed);
  // The presence poll writes these on the Game tick, which is also where the
  // toggle is called from; a stale-by-one-tick answer is exactly what the
  // camera itself uses, so it is the honest number to record.
  const int presence = context.vehicle_query_known
                           ? (context.vehicle_query_present ? 1 : 0)
                           : -1;

  char modes[64]{};
  std::size_t used = 0;
  if (have_snapshot) {
    for (std::size_t index = 0;
         index < snapshot.modes_read && used + 12 < sizeof(modes); ++index) {
      const int written = std::snprintf(
          modes + used, sizeof(modes) - used, "%s%s",
          index == 0 ? "" : ",",
          TagLabel(names, snapshot.modes[index]));
      if (written <= 0) {
        break;
      }
      used += static_cast<std::size_t>(written);
    }
  }
  if (used == 0) {
    std::snprintf(modes, sizeof(modes), "?");
  }

  char message[512]{};
  if (have_snapshot) {
    std::snprintf(
        message, sizeof(message),
        "viewmode call #%llu raw=%u site=0x%llX self=0x%llX tag=0x%llX(%s) "
        "count=%d cap=%d modes=[%s] limit=%d cool=%.3f grace=%.3f clock=%.3f "
        "gused=%u en=%d veh=%d active=%d ret=%u after=%s dec=%s tid=%lu",
        static_cast<unsigned long long>(call_index),
        static_cast<unsigned>(b_forward_raw),
        static_cast<unsigned long long>(site),
        static_cast<unsigned long long>(
            reinterpret_cast<std::uintptr_t>(self)),
        static_cast<unsigned long long>(snapshot.tag),
        TagLabel(names, snapshot.tag), static_cast<int>(snapshot.count),
        static_cast<int>(snapshot.capacity), modes,
        static_cast<int>(snapshot.limit),
        static_cast<double>(snapshot.cooldown),
        static_cast<double>(snapshot.grace), snapshot.cooldown_clock,
        static_cast<unsigned>(snapshot.grace_used), enabled ? 1 : 0, presence,
        view_active ? 1 : 0, static_cast<unsigned>(result),
        have_tag_after ? TagLabel(names, tag_after) : "?", decision,
        static_cast<unsigned long>(GetCurrentThreadId()));
  } else {
    std::snprintf(
        message, sizeof(message),
        "viewmode call #%llu raw=%u site=0x%llX self=0x%llX tag=<unreadable> "
        "en=%d veh=%d active=%d ret=%u dec=%s tid=%lu",
        static_cast<unsigned long long>(call_index),
        static_cast<unsigned>(b_forward_raw),
        static_cast<unsigned long long>(site),
        static_cast<unsigned long long>(
            reinterpret_cast<std::uintptr_t>(self)),
        enabled ? 1 : 0, presence, view_active ? 1 : 0,
        static_cast<unsigned>(result), decision,
        static_cast<unsigned long>(GetCurrentThreadId()));
  }
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
}

// The one read-and-print entry point the detour uses. Returns the byte to
// return to the game so the caller keeps a single exit path.
std::uint8_t RecordAndForward(Context &context, void *self,
                              const std::uint8_t b_forward_raw,
                              const std::uintptr_t site) noexcept {
  ViewModeState &state = context.view_mode;
  // Take the hook lease for the duration of the call: it is what keeps a
  // release from pulling the trampoline out from under us. The lease must be
  // closed on every path -- the host's stop barrier counts them, and one leaked
  // lease turns the next reload into "callback barrier timed out" and the
  // plugin into a quarantined generation that only a restart clears. The guard
  // below is what makes that impossible to forget.
  HookCallbackLease lease(context.hook);
  try {
    const std::uint64_t hook_id =
        state.hook_id.load(std::memory_order_acquire);
    if (hook_id != 0 && HookReady(context.hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, state.hook_generation.load(std::memory_order_acquire)};
      lease.Begin(handle);
    }
  } catch (...) {
    // A refused lease only costs the log line, never the game's behaviour.
  }
  const bool leased = lease.active();
  const std::uint64_t call_index =
      state.calls.fetch_add(1, std::memory_order_relaxed) + 1;
  ViewModeSnapshot snapshot{};
  const bool have_snapshot =
      ReadViewModeState(context, reinterpret_cast<std::uintptr_t>(self),
                        snapshot);
  if (!have_snapshot) {
    // The controller moved (map change, re-login) or the offsets are wrong.
    // Dropping the cached pointer makes the next tick resolve it again instead
    // of reading a freed block forever.
    state.pc.store(0, std::memory_order_release);
    state.published_tag.store(0, std::memory_order_relaxed);
    state.published_tag_known.store(false, std::memory_order_release);
  }
  if (have_snapshot && !state.shape_logged) {
    state.shape_logged = true;
    char message[256]{};
    const TagNames names = ReadNames(state);
    char modes[64]{};
    std::size_t used = 0;
    for (std::size_t index = 0;
         index < snapshot.modes_read && used + 12 < sizeof(modes); ++index) {
      const int written =
          std::snprintf(modes + used, sizeof(modes) - used, "%s%s",
                        index == 0 ? "" : ",",
                        TagLabel(names, snapshot.modes[index]));
      if (written <= 0) {
        break;
      }
      used += static_cast<std::size_t>(written);
    }
    std::snprintf(message, sizeof(message),
                  "viewmode state: pc=0x%llX count=%d cap=%d modes=[%s] "
                  "limit=%d cooldown=%.3f grace=%.3f",
                  static_cast<unsigned long long>(
                      state.pc.load(std::memory_order_relaxed)),
                  static_cast<int>(snapshot.count),
                  static_cast<int>(snapshot.capacity),
                  used == 0 ? "?" : modes, static_cast<int>(snapshot.limit),
                  static_cast<double>(snapshot.cooldown),
                  static_cast<double>(snapshot.grace));
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
  }

  std::uint8_t result = 0;
  ToggleViewModeFn original =
      g_toggle_view_mode_original.load(std::memory_order_acquire);
  const char *decision = "no-original";
  const bool manual = b_forward_raw == 0;
  const std::uint32_t tag_third =
      state.tag_third_person.load(std::memory_order_relaxed);
  const std::uint32_t tag_first =
      state.tag_first_person.load(std::memory_order_relaxed);
  // Only take over when every fact the plan SS2.2 requires is true: the plugin
  // is enabled, the presenter says a vehicle is being driven, the tag names are
  // resolved, the call is on the controller we resolved, and the trampoline
  // exists. Anything unknown is the game's business.
  const bool managed =
      original != nullptr && have_snapshot && tag_third != 0 && tag_first != 0 &&
      context.enabled.load(std::memory_order_acquire) &&
      state.in_vehicle.load(std::memory_order_acquire) &&
      reinterpret_cast<std::uintptr_t>(self) ==
          state.pc.load(std::memory_order_relaxed);
  if (original == nullptr) {
    decision = "no-original";
  } else if (!managed) {
    if (!have_snapshot) {
      decision = "unresolved";
    } else if (!context.enabled.load(std::memory_order_relaxed)) {
      decision = "disabled";
    } else if (!state.in_vehicle.load(std::memory_order_relaxed)) {
      decision = "on-foot";
    } else if (!manual) {
      decision = "script";
    } else {
      decision = "foreign-controller";
    }
    result = original(self, b_forward_raw);
    state.forwarded.fetch_add(1, std::memory_order_relaxed);
  } else if (!manual) {
    // Script / forced call: the boolean goes through untouched. TG-0 measured
    // every Z press as false, so `true` only ever comes from a script, and a
    // script has to be able to force a mode the plugin does not want.
    decision = "script";
    result = original(self, b_forward_raw);
    state.forwarded.fetch_add(1, std::memory_order_relaxed);
  } else if (snapshot.limit > 0) {
    // The vehicle input wrapper does not check +0x1E98 (TG-0 SS1); the on-foot
    // one does. Without this the plugin would switch modes inside a cutscene.
    decision = "blocked-limit";
    result = 0;
  } else {
    const std::uint32_t current = static_cast<std::uint32_t>(snapshot.tag);
    if (current == tag_third) {
      // The chase slot is `ThirdPerson + view_active`; plain third person is
      // `ThirdPerson + !view_active`. Both have the same game tag, so the plugin
      // flag is what separates them.
      state.chase_wanted = false;
      if (context.view_active.load(std::memory_order_acquire) ||
          context.last_view_active) {
        PublishInactive(context);
        decision = "chase-to-third";
        result = 1;
      } else {
        result = original(self, b_forward_raw);
        state.forwarded.fetch_add(1, std::memory_order_relaxed);
        decision = result != 0 ? "third-to-first" : "third-to-first-refused";
      }
    } else if (current == tag_first) {
      const bool was_chase =
          context.view_active.load(std::memory_order_acquire);
      if (was_chase) {
        // Inconsistent state (chase pose over a first-person tag): drop the
        // pose first, then let the game finish the switch.
        PublishInactive(context);
      }
      result = original(self, b_forward_raw);
      state.forwarded.fetch_add(1, std::memory_order_relaxed);
      if (result != 0 && !was_chase) {
        // The game has already put the game camera in third person; showing it
        // for one tick is the flash the plan forbids, so the chase pose goes up
        // inside the same call.
        state.chase_wanted = true;
        ActivateView(context,
                     context.fov_enabled.load(std::memory_order_acquire));
        decision = "first-to-chase";
      } else if (result != 0) {
        decision = "first-to-third";
      } else {
        // Refused (cooldown, or the bookkeeping path deciding there is nothing
        // to do). The state machine does not advance: the next Z tries again.
        decision = "first-to-third-refused";
      }
    } else {
      // Fixed / Seat / anything the game registers later: never fight it.
      if (context.view_active.load(std::memory_order_acquire)) {
        PublishInactive(context);
      }
      result = original(self, b_forward_raw);
      state.forwarded.fetch_add(1, std::memory_order_relaxed);
      decision = "external-pass";
    }
  }
  // The tag after the call is what pairs the return value with the effect: a
  // `ret=1` that did not move the tag would mean the game accepted a switch to
  // the mode it was already in.
  std::uint64_t tag_after = 0;
  bool have_tag_after = false;
  if (have_snapshot) {
    std::uintptr_t tag_address{};
    have_tag_after =
        AddAddress(reinterpret_cast<std::uintptr_t>(self),
                   kPcViewModeTagOffset, tag_address) &&
        Read(context.core, tag_address, tag_after);
  }

  const bool log_now =
      leased &&
      (call_index <= kToggleLogAllCalls ||
       (call_index % kToggleLogStrideAfter) == 0 ||
       site != state.last_site.load(std::memory_order_relaxed));
  if (log_now) {
    state.logged.fetch_add(1, std::memory_order_relaxed);
    state.last_site.store(site, std::memory_order_relaxed);
    LogCall(context, b_forward_raw, self, site, snapshot, have_snapshot,
            result, call_index, tag_after, have_tag_after, decision);
  }
  return result;
}

// Resolve an id back to the text the pool spells it with. This is the v1
// direction (`resolve_utf8`, an index lookup) and the one `anomaly-cli ue
// fname` uses; it works for ids registered at any point in the session.
bool ResolveNameText(Context &context, const std::uint32_t id, char *out,
                     const std::size_t capacity) noexcept {
  if (out == nullptr || capacity < 2 || id == 0 ||
      context.names == nullptr || context.names->resolve_utf8 == nullptr) {
    return false;
  }
  std::size_t size = capacity;
  out[0] = '\0';
  if (context.names->resolve_utf8(context.names->user, id, out, &size).code !=
          ANOMALY_STATUS_V1_OK ||
      size < 2 || size > capacity) {
    return false;
  }
  out[capacity - 1] = '\0';
  return out[0] != '\0';
}

// Resolve `CameraViewMode.ThirdPerson` / `.FirstPerson` to the ids this process
// assigned them.
//
// The obvious API is the names service's `find_utf8`, which searches the pool
// for a text. It comes back empty for these two on a live session: gameplay
// tags are registered early, and that search walks the pool from its newest
// entries. So the ids are taken from the controller's own instance array
// instead and their text is confirmed by resolving each id forwards. That is
// better evidence than a pool search in any case -- the array holds the ids the
// controller switches between, so the ids cannot disagree with the names -- and
// it doubles as the shape check the takeover needs: exactly two modes, spelling
// ThirdPerson and FirstPerson.
// The `ue5.GWorld` slot. The render-side probe resolves the same pattern under
// the profile-hash gate and publishes the slot; when that path has not run yet
// (or is idle because the camera is inactive) this resolves it locally, once
// per retry window, without writing to the probe's state.
bool ResolveWorldSlot(Context &context, std::uintptr_t &slot) noexcept {
  slot = context.view_mode.world_slot.load(std::memory_order_relaxed);
  if (slot != 0) {
    return true;
  }
  slot = context.render_x.gworld_slot;
  if (slot != 0) {
    context.view_mode.world_slot.store(slot, std::memory_order_relaxed);
    return true;
  }
  if (context.render_x.insurance_state != kAnchorInsuranceReady) {
    return false;
  }
  std::uintptr_t instruction{};
  if (ResolveSignatureStatus(context, kGWorldPattern, instruction).code !=
          ANOMALY_STATUS_V1_OK ||
      instruction == 0) {
    return false;
  }
  std::uintptr_t displacement_address{};
  std::int32_t displacement{};
  if (!AddAddress(instruction, kGWorldRipOffset, displacement_address) ||
      !Read(context.core, displacement_address, displacement)) {
    return false;
  }
  const std::intptr_t resolved =
      static_cast<std::intptr_t>(instruction) +
      static_cast<std::intptr_t>(kGWorldInstructionSize) +
      static_cast<std::intptr_t>(displacement);
  if (resolved <= 0) {
    return false;
  }
  slot = static_cast<std::uintptr_t>(resolved);
  context.view_mode.world_slot.store(slot, std::memory_order_relaxed);
  return true;
}

// `world -> gameInstance -> localPlayers.data[0] -> controller`. The same chain
// the host's `ReadCurrentPlayerLocation` and the render probe walk, stopped one
// hop earlier because the view-mode block lives on the controller.
bool ResolvePlayerController(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  if (state.pc.load(std::memory_order_relaxed) != 0) {
    return true;
  }
  std::uintptr_t slot{};
  std::uintptr_t world{};
  if (!ResolveWorldSlot(context, slot) ||
      !Read(context.core, slot, world) || world == 0) {
    return false;
  }
  std::uintptr_t game_instance{};
  std::uintptr_t players{};
  if (!ReadPointerAtOffset(context.core, world, kWorldGameInstanceOffset,
                           game_instance) ||
      !ReadPointerAtOffset(context.core, game_instance,
                           kGameInstanceLocalPlayersOffset, players)) {
    return false;
  }
  std::uintptr_t count_address{};
  std::int32_t players_count{};
  if (!AddAddress(game_instance,
                  kGameInstanceLocalPlayersOffset + sizeof(std::uintptr_t),
                  count_address) ||
      !Read(context.core, count_address, players_count) || players_count < 1 ||
      players_count > 64) {
    return false;
  }
  std::uintptr_t local_player{};
  std::uintptr_t controller{};
  if (!Read(context.core, players, local_player) || local_player == 0 ||
      !ReadPointerAtOffset(context.core, local_player,
                           kLocalPlayerControllerOffset, controller) ||
      controller == 0) {
    return false;
  }
  state.pc.store(controller, std::memory_order_release);
  // Until the watcher reads this controller the published tag belongs to the
  // previous one: "unknown" is the only safe answer, and unknown fails open.
  state.published_tag.store(0, std::memory_order_relaxed);
  state.published_tag_known.store(false, std::memory_order_release);
  char message[192]{};
  std::snprintf(message, sizeof(message),
                "viewmode: player controller resolved at 0x%llX (world "
                "slot=0x%llX)",
                static_cast<unsigned long long>(controller),
                static_cast<unsigned long long>(slot));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
  return true;
}

// The vtable is the second witness for the entry point: a pattern hit proves
// the bytes, the slot proves the game still dispatches through them. A
// mismatch latches, because nothing the plugin does can move the slot back.
bool VerifyToggleSlot(Context &context, const std::uintptr_t target) noexcept {
  ViewModeState &state = context.view_mode;
  if (state.vtable_mismatch) {
    return false;
  }
  const std::uintptr_t pc = state.pc.load(std::memory_order_acquire);
  std::uintptr_t vtable{};
  std::uintptr_t entry{};
  if (pc == 0 || !ReadPointerAtOffset(context.core, pc, 0, vtable) ||
      !ReadPointerAtOffset(context.core, vtable, kToggleVtableOffset, entry)) {
    // Not a mismatch, just not readable yet: the controller can be mid-swap.
    return false;
  }
  if (entry != target) {
    state.vtable_mismatch = true;
    if (!state.vtable_failure_logged) {
      state.vtable_failure_logged = true;
      char message[224]{};
      std::snprintf(
          message, sizeof(message),
          "viewmode: vtable slot +0x%X holds 0x%llX, pattern resolved 0x%llX; "
          "Z stays native",
          static_cast<unsigned>(kToggleVtableOffset),
          static_cast<unsigned long long>(entry),
          static_cast<unsigned long long>(target));
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, std::string_view(message));
    }
    return false;
  }
  return true;
}

}  // namespace

bool EnsureViewModeTags(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  if (state.names_ready.load(std::memory_order_acquire)) {
    return true;
  }
  if (context.elapsed < state.next_tags_retry) {
    return false;
  }
  state.next_tags_retry = context.elapsed + kHookRetrySeconds;
  const std::uintptr_t pc = state.pc.load(std::memory_order_relaxed);
  if (pc == 0) {
    return false;
  }
  ViewModeSnapshot snapshot{};
  if (!ReadViewModeState(context, pc, snapshot) ||
      snapshot.modes_read == 0) {
    return false;
  }
  std::uint32_t third = 0;
  std::uint32_t first = 0;
  std::uint32_t fixed = 0;
  std::uint32_t seat = 0;
  std::uint32_t driving = 0;
  char text[128]{};
  for (std::size_t index = 0; index < snapshot.modes_read; ++index) {
    const auto id = static_cast<std::uint32_t>(snapshot.modes[index]);
    if (!ResolveNameText(context, id, text, sizeof(text))) {
      continue;
    }
    if (std::strcmp(text, "CameraViewMode.ThirdPerson") == 0) {
      third = id;
    } else if (std::strcmp(text, "CameraViewMode.FirstPerson") == 0) {
      first = id;
    } else if (std::strcmp(text, "CameraViewMode.Fixed") == 0) {
      fixed = id;
    } else if (std::strcmp(text, "CameraViewMode.Seat") == 0) {
      seat = id;
    } else if (std::strcmp(text, "CameraViewMode.DrivingCamera") == 0) {
      driving = id;
    }
  }
  if (third == 0 || first == 0 || third == first) {
    if (!state.names_failure_logged) {
      state.names_failure_logged = true;
      char message[224]{};
      std::snprintf(message, sizeof(message),
                    "viewmode: controller array does not name the two view "
                    "modes yet (third=0x%X first=0x%X); retrying quietly",
                    third, first);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, std::string_view(message));
    }
    return false;
  }
  state.tag_third_person.store(third, std::memory_order_relaxed);
  state.tag_first_person.store(first, std::memory_order_relaxed);
  state.tag_fixed.store(fixed, std::memory_order_relaxed);
  state.tag_seat.store(seat, std::memory_order_relaxed);
  state.tag_driving.store(driving, std::memory_order_relaxed);
  state.names_ready.store(true, std::memory_order_release);
  char message[320]{};
  std::snprintf(message, sizeof(message),
                "viewmode: tags third=0x%X first=0x%X fixed=0x%X seat=0x%X "
                "driving=0x%X (from the controller array, count=%d)",
                third, first, fixed, seat, driving,
                static_cast<int>(snapshot.count));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
  return true;
}

// The game's own photo/cine camera. `PC+0x1760` holds a CineCameraComponent
// owned by the controller while the game shows a camera of its own; it is null
// during normal driving. The read failing means "not there": the plugin keeps
// the pre-fix behaviour instead of dropping the chase camera.
[[nodiscard]] static bool PcCineCameraPresent(Context &context) noexcept {
  const std::uintptr_t pc =
      context.view_mode.pc.load(std::memory_order_acquire);
  if (pc == 0) {
    return false;
  }
  std::uintptr_t address{};
  std::uintptr_t component = 0;
  return AddAddress(pc, kPcCineCameraOffset, address) &&
         Read(context.core, address, component) && component != 0;
}

// Watch the controller's view-mode block on every Game tick and report the
// moments the game changes it by itself. Two things in the first TG-0 session
// made this necessary:
//   * the tag moved FP -> TP -> FP without a single `Toggle` call, so something
//     else in the game switches the mode (a teleport, a transition);
//   * the manual-toggle limit stayed 0 for the whole session, so the only way to
//     see it engage is to be watching when it does.
// One 0x38-byte read per tick, and a log line only when a field moves.
void MonitorViewMode(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  const std::uintptr_t pc = state.pc.load(std::memory_order_relaxed);
  if (pc == 0) {
    return;
  }
  std::uintptr_t address{};
  std::uint8_t raw[kPcViewModeStateSpan]{};
  if (!AddAddress(pc, kPcViewModeTagOffset, address)) {
    return;
  }
  AnomalyMutableByteSpanV1 destination{raw, sizeof(raw)};
  if (context.core->read_memory == nullptr ||
      context.core->read_memory(context.core->user, address, destination)
              .code != ANOMALY_STATUS_V1_OK) {
    return;
  }
  std::uint64_t tag = 0;
  std::int32_t count = 0;
  std::int32_t limit = 0;
  std::memcpy(&tag, raw + 0x00, sizeof(tag));
  std::memcpy(&count, raw + 0x10, sizeof(count));
  std::memcpy(&limit, raw + 0x18, sizeof(limit));

  // Publish before the change filter: the presenter and the render-thread guard
  // read this copy, and it must be this tick's value, not the last edge's.
  state.published_tag.store(tag, std::memory_order_relaxed);
  state.published_tag_known.store(true, std::memory_order_release);

#if defined(NFS2015_VIEWMODE_TRACE)
  // Acceptance-only trace (plan TG-4 SS7: "record tag / view_active / view_mode_tag
  // for the frames around a transition"). Compiled in only with
  // -DNFS2015_VIEWMODE_TRACE=ON; the shipped package does not contain it, so it
  // adds neither a setting nor a runtime cost.
  {
    char line[224]{};
    std::snprintf(line, sizeof(line),
                  "viewmode trace: tag=0x%llX(%s) count=%d limit=%d veh=%d "
                  "wanted=%d active=%d pending=%d veto=%llu cine=%d",
                  static_cast<unsigned long long>(tag), TagLabel(ReadNames(state), tag),
                  static_cast<int>(count), static_cast<int>(limit),
                  context.vehicle_query_known
                      ? (context.vehicle_query_present ? 1 : 0)
                      : -1,
                  state.chase_wanted ? 1 : 0,
                  context.view_active.load(std::memory_order_relaxed) ? 1 : 0,
                  state.pending_entry ? 1 : 0,
                  static_cast<unsigned long long>(
                      context.diag.tag_veto_frames.load(std::memory_order_relaxed)),
                  PcCineCameraPresent(context) ? 1 : 0);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(line));
  }
#endif

  const bool first = !state.have_tag;
  if (!first && tag == state.last_tag && count == state.last_count &&
      limit == state.last_limit) {
    return;
  }
  const std::uint64_t previous = state.last_tag;
  const std::int32_t previous_limit = state.last_limit;
  const std::int32_t previous_count = state.last_count;
  state.last_tag = tag;
  state.last_count = count;
  state.last_limit = limit;
  state.have_tag = true;

  const TagNames names = ReadNames(state);
  char previous_text[64]{};
  if (first) {
    std::snprintf(previous_text, sizeof(previous_text), "?");
  } else {
    std::snprintf(previous_text, sizeof(previous_text), "0x%llX(%s)",
                  static_cast<unsigned long long>(previous),
                  TagLabel(names, previous));
  }
  char message[320]{};
  std::snprintf(
      message, sizeof(message),
      "viewmode watch: %s -> 0x%llX(%s) count=%d%s limit=%d%s veh=%d en=%d "
      "active=%d calls=%llu",
      previous_text, static_cast<unsigned long long>(tag), TagLabel(names, tag),
      static_cast<int>(count),
      (!first && count != previous_count) ? "*" : "",
      static_cast<int>(limit),
      (!first && limit != previous_limit) ? "*" : "",
      context.vehicle_query_known ? (context.vehicle_query_present ? 1 : 0)
                                  : -1,
      context.enabled.load(std::memory_order_relaxed) ? 1 : 0,
      context.view_active.load(std::memory_order_relaxed) ? 1 : 0,
      static_cast<unsigned long long>(
          state.calls.load(std::memory_order_relaxed)));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
}

namespace {

// A missing tag name or an unreadable tag is "unknown", and unknown must keep
// the pre-TG-1 behaviour: a host without `anomaly.ue5.names` would otherwise
// lose the chase camera entirely, which is exactly the escape hatch Q3=B wants.
[[nodiscard]] bool TagIsThirdPerson(const ViewModeState &state,
                                   const std::uint64_t tag,
                                   const bool known) noexcept {
  if (!known) {
    return true;
  }
  const std::uint32_t third =
      state.tag_third_person.load(std::memory_order_relaxed);
  if (third == 0) {
    return true;
  }
  // `FGameplayTag` is an `FName`: {ComparisonIndex, Number}. Number is always 0
  // for a gameplay tag, so the comparison index decides.
  return static_cast<std::uint32_t>(tag) == third;
}

}  // namespace

bool ReadLiveViewModeTag(Context &context, std::uint64_t &tag) noexcept {
  tag = 0;
  const std::uintptr_t pc =
      context.view_mode.pc.load(std::memory_order_acquire);
  if (pc == 0) {
    return false;
  }
  std::uintptr_t address{};
  return AddAddress(pc, kPcViewModeTagOffset, address) &&
         Read(context.core, address, tag);
}

const char *ViewModeTagLabel(Context &context,
                             const std::uint64_t tag) noexcept {
  return TagLabel(ReadNames(context.view_mode), tag);
}

bool ChaseSlotAllowed(Context &context) noexcept {
  const ViewModeState &state = context.view_mode;
  if (!state.chase_wanted) {
    return false;
  }
  if (PcCineCameraPresent(context)) {
    return false;
  }
  return TagIsThirdPerson(
      state, state.published_tag.load(std::memory_order_relaxed),
      state.published_tag_known.load(std::memory_order_acquire));
}

namespace {

// What the game's tag means to the entry logic. `kOther` is Fixed / Seat /
// anything else the plugin does not drive.
enum class LiveTagKind : std::uint8_t { kUnknown, kThird, kFirst, kOther };

[[nodiscard]] LiveTagKind ClassifyLiveTag(Context &context,
                                         std::uint64_t &tag) noexcept {
  const ViewModeState &state = context.view_mode;
  const std::uint32_t third =
      state.tag_third_person.load(std::memory_order_relaxed);
  const std::uint32_t first =
      state.tag_first_person.load(std::memory_order_relaxed);
  if (third == 0 || first == 0) {
    return LiveTagKind::kUnknown;
  }
  if (!ReadLiveViewModeTag(context, tag) || tag == 0) {
    return LiveTagKind::kUnknown;
  }
  if (static_cast<std::uint32_t>(tag) == third) {
    return LiveTagKind::kThird;
  }
  if (static_cast<std::uint32_t>(tag) == first) {
    return LiveTagKind::kFirst;
  }
  return LiveTagKind::kOther;
}

const char *TagKindName(const LiveTagKind kind) noexcept {
  switch (kind) {
    case LiveTagKind::kThird:
      return "TP";
    case LiveTagKind::kFirst:
      return "FP";
    case LiveTagKind::kOther:
      return "other";
    case LiveTagKind::kUnknown:
    default:
      return "unknown";
  }
}

}  // namespace

void ApplyEntryView(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  std::uint64_t tag = 0;
  const LiveTagKind kind = ClassifyLiveTag(context, tag);
  if (kind == LiveTagKind::kUnknown) {
    // Tags not resolved yet, or the controller tag is unreadable. Do not guess:
    // leave the request pending and try again next tick, but say so once.
    state.pending_entry = true;
    if (!state.entry_pending_logged) {
      state.entry_pending_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
          std::string_view("viewmode entry: the game tag is not readable yet, "
                           "waiting"));
    }
    return;
  }

  // Plan SS2.3.2, fixed rule - no setting, no third/first/keep variants:
  //   Third  -> hand the camera to the chase model, stay in ThirdPerson.
  //   First  -> leave it alone. The player was already using first person, so
  //             the first Z from here goes First -> chase (T7).
  //   Other  -> a cutscene owns the camera; retry when it ends.
  // Nothing here calls the game's toggle: the plugin only ever takes over a
  // tag the game is already in.
  state.chase_wanted = kind == LiveTagKind::kThird;
  state.pending_entry = kind == LiveTagKind::kOther;

  if (state.pending_entry) {
    if (state.entry_pending_logged) {
      return;
    }
    state.entry_pending_logged = true;
  } else {
    state.entry_pending_logged = false;
  }

  char message[192]{};
  std::snprintf(message, sizeof(message),
                "viewmode entry: tag=%s wanted=%d pending=%d",
                TagKindName(kind), state.chase_wanted ? 1 : 0,
                state.pending_entry ? 1 : 0);
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
}

void NoteVehicleExit(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  state.chase_wanted = false;
  state.pending_entry = false;
  state.entry_pending_logged = false;
  state.in_vehicle.store(false, std::memory_order_release);
}

bool ChaseCameraGuardAllows(Context &context) noexcept {
  const ViewModeState &state = context.view_mode;
  // The game's own camera (photo mode / cinematic) wins over everything below.
  // Same live read as the tag: the presenter's answer is a tick behind, and one
  // tick is a visible frame of the chase pose on top of the photo camera.
  if (PcCineCameraPresent(context)) {
    return false;
  }
  std::uint64_t tag = 0;
  // Live read first: the presenter's copy is up to one Game tick behind, and a
  // tick is a visible frame of the wrong camera.
  if (ReadLiveViewModeTag(context, tag)) {
    return TagIsThirdPerson(state, tag, true);
  }
  return TagIsThirdPerson(
      state, state.published_tag.load(std::memory_order_relaxed),
      state.published_tag_known.load(std::memory_order_acquire));
}

bool EnsureToggleViewModeHook(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  if (state.hook_id.load(std::memory_order_acquire) != 0) {
    return true;
  }
  if (state.pattern_unavailable || state.vtable_mismatch) {
    return false;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target = state.hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kToggleSwitchCameraViewModePattern, target)) {
      state.pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "viewmode: ToggleSwitchCameraViewMode pattern not found; Z stays "
          "native for this session");
      return false;
    }
    state.hook_target.store(target, std::memory_order_release);
  }
  if (!VerifyToggleSlot(context, target)) {
    return false;
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&ToggleViewModeDetour);
  request.label = StringView("nfs2015-chase-cam-toggle-view-mode");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_view_mode_context.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(context.hook->release(context.hook->user, handle));
    }
    g_view_mode_context.store(nullptr, std::memory_order_release);
    if (!state.hook_failure_logged) {
      state.hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "viewmode: toggle hook creation failed; Z stays native");
    }
    return false;
  }
  state.hook_generation.store(handle.generation, std::memory_order_release);
  g_toggle_view_mode_original.store(
      reinterpret_cast<ToggleViewModeFn>(original),
      std::memory_order_release);
  state.hook_id.store(handle.id, std::memory_order_release);
  char message[224]{};
  std::snprintf(message, sizeof(message),
                "viewmode: toggle hook installed (target=0x%llX, "
                "original=0x%llX, PC=0x%llX); Z cycles chase/third/first "
                "while driving, native otherwise",
                static_cast<unsigned long long>(target),
                static_cast<unsigned long long>(original),
                static_cast<unsigned long long>(
                    state.pc.load(std::memory_order_relaxed)));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, std::string_view(message));
  return true;
}

void ReleaseToggleViewModeHook(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  const std::uint64_t hook_id = state.hook_id.load(std::memory_order_acquire);
  bool released = false;
  if (hook_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        hook_id, state.hook_generation.load(std::memory_order_acquire)};
    released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  g_view_mode_context.store(nullptr, std::memory_order_release);
  if (released) {
    // Keep a failed trampoline alive: a late detour must still be able to
    // delegate instead of returning a fabricated result.
    g_toggle_view_mode_original.store(nullptr, std::memory_order_release);
  }
  state.hook_id.store(0, std::memory_order_release);
  state.hook_generation.store(0, std::memory_order_release);
  state.hook_target.store(0, std::memory_order_release);
  state.pattern_unavailable = false;
  state.vtable_mismatch = false;
  state.shape_logged = false;
  state.hook_failure_logged = false;
  state.vtable_failure_logged = false;
  state.next_retry = 0.0;
}

void ResetViewModeRecon(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  state.pc.store(0, std::memory_order_release);
  state.world_slot.store(0, std::memory_order_release);
  state.names_ready.store(false, std::memory_order_release);
  state.tag_third_person.store(0, std::memory_order_relaxed);
  state.tag_first_person.store(0, std::memory_order_relaxed);
  state.tag_fixed.store(0, std::memory_order_relaxed);
  state.tag_seat.store(0, std::memory_order_relaxed);
  state.tag_driving.store(0, std::memory_order_relaxed);
  state.calls.store(0, std::memory_order_relaxed);
  state.logged.store(0, std::memory_order_relaxed);
  state.last_site.store(0, std::memory_order_relaxed);
  state.forwarded.store(0, std::memory_order_relaxed);
  state.names_failure_logged = false;
  state.shape_logged = false;
}

void UpdateViewModeRecon(Context &context) noexcept {
  ViewModeState &state = context.view_mode;
  // Cheap, every tick: notices a mode change the game made on its own.
  MonitorViewMode(context);
  const bool installed = state.hook_id.load(std::memory_order_acquire) != 0;
  if (!installed && (state.pattern_unavailable || state.vtable_mismatch)) {
    return;
  }
  // Everything below either scans .text or walks the player chain, so it runs
  // on a slow cadence and never per tick. `PC` can stay unresolved for a while
  // after a map load; the retry is what eventually finds it.
  if (context.elapsed < state.next_retry) {
    return;
  }
  state.next_retry = context.elapsed + kHookRetrySeconds;
  if (!ResolvePlayerController(context)) {
    return;
  }
  // Tag resolution is independent of the hook: the log prints bare ids until
  // the array names them, which is still enough to match a call site.
  static_cast<void>(EnsureViewModeTags(context));
  if (installed) {
    return;
  }
  static_cast<void>(EnsureToggleViewModeHook(context));
}

std::uint8_t ANOMALY_CALL ToggleViewModeDetour(
    void *self, std::uint8_t b_forward_raw) noexcept {
  if (g_toggle_view_mode_original.load(std::memory_order_acquire) == nullptr) {
    // No trampoline, no safe answer: the game asked for a boolean and the only
    // honest one is "no change".
    return 0;
  }
  Context *context = g_view_mode_context.load(std::memory_order_acquire);
  if (context == nullptr ||
      !context->started.load(std::memory_order_acquire)) {
    return g_toggle_view_mode_original.load(std::memory_order_acquire)(
        self, b_forward_raw);
  }
  const std::uintptr_t site = NFS2015_RETURN_ADDRESS();
  try {
    return RecordAndForward(*context, self, b_forward_raw, site);
  } catch (...) {
    ToggleViewModeFn original =
        g_toggle_view_mode_original.load(std::memory_order_acquire);
    return original != nullptr ? original(self, b_forward_raw) : 0;
  }
}

}  // namespace nfs2015_chase_cam::detail
