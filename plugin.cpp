// NFS 2015 Chase Cam — third-party Anomaly plugin (ABI v1).
//
// This package is built against the installed Anomaly SDK only: it includes no
// host-internal headers and carries no JSON library of its own. Settings are
// persisted through `anomaly.config` and decoded through `anomaly.json`.
//
// Feeds the clean-room NFS 2015 chase-camera model in `chase_cam.hpp` from
// Anomaly's NTE snapshots, then replaces the view point produced by the game's
// camera manager. The game's own third-person camera is used as the car-yaw
// sensor; the player snapshot supplies the car position. Anomaly 2.7.0 removed
// `anomaly.nte.vehicle`, so vehicle presence is recovered by calling the same
// reflected getters that service used, through `ue5.objects` +
// `ue5.process-event`; the player capsule's AttachParent remains the anchor and
// the fallback. Speed comes from differentiating the player position and is
// signed against the resolved car yaw, with hysteresis for reverse. No GObjects
// scan.
//
// The `Probe` block below is the only code that reaches into the engine: it
// resolves three reflected functions, takes the Pawn object handle and calls
// `K2_GetActorRotation` from inside the once-per-frame AHUD callback. It is
// read-only, it never feeds the camera, and every result is shown in the
// diagnostics panel. See PLAN-camera-timing.md §8 P0.
//
// Thread domains:
//   on_update = Game. Reads NTE services, integrates the camera, publishes an
//               immutable view snapshot and persists settings.
//   detour    = whatever thread asks the camera manager for its view point.
//               It only calls the original, copies the published snapshot to
//               the two out-parameters, and writes the optional lens.
//   on_draw   = Render. Copies the display snapshot under a short lock and
//               calls UI outside it.

#include "localization.hpp"

#include "anomaly/sdk/cpp.hpp"
#include "anomaly/sdk/services/core.h"
#include "anomaly/sdk/services/interop.h"
#include "anomaly/sdk/services/json.h"
#include "anomaly/sdk/services/nte.h"
#include "anomaly/sdk/services/ui.h"
#include "anomaly/sdk/services/ui_resources.h"
#include "anomaly/sdk/services/ue5.h"

#include "chase_cam.hpp"
#include "wheel_steer.hpp"
#include "latency_window.hpp"
#include "car_sample.hpp"
#include "burnout_shot.hpp"
#include "view_blend.hpp"

#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <thread>
#include <system_error>

#include "src/constants.hpp"
#include "src/internal.hpp"
#include "src/sdk.hpp"
#include "src/state.hpp"

namespace nfs2015_chase_cam::detail {

// Q2 answer (PLAN-camera-timing.md): the game-camera pose the entity frame
// once exposed is already in the player ESP snapshot, which is a cached copy
// the plugin reads every tick anyway (`camera_position` / `camera_rotation`).
// There is no need to lease the entity service just to observe it.

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

// ---------------------------------------------------------------------------
// Render-thread car position probe (PLAN-render-thread-hook.md task group 0)
// ---------------------------------------------------------------------------
//
// Read-only reconnaissance for the hook route. It answers one question that the
// readings of the last months could not: does the render thread see a *newer*
// car transform than the Game-tick snapshot the camera is built from?
//
// The Game-tick half walks the host's own player chain (the offsets in
// `ReadCurrentPlayerLocation`) and compares it with the host snapshot. That is
// the control: if it does not reproduce the snapshot exactly, the hardcoded
// offsets are stale and the render-thread half is measuring the wrong field.
// The render half reads the same `root + 280` from the detour and subtracts the
// snapshot. Roughly one tick of travel of lead means the anchor exists (and the
// camera can use it); a near-zero difference means the renderer draws from a
// different field (`ComponentToWorld`) and task group 1 is next.
//
// No allocation, no lock of our own, no logging: stack buffers and atomics, so
// the render path stays as cheap as the rest of the detour.

struct PlayerChain final {
  std::uintptr_t pawn{};
  std::uintptr_t root{};
};

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

// `world -> gameInstance -> localPlayers.data[0] -> controller -> pawn -> root`.
// Exactly the host's chain, so a difference in the final read can only come
// from the moment of the read, not from a different path.
bool WalkPlayerChain(const AnomalyCoreServiceV1 *core,
                     const std::uintptr_t world,
                     PlayerChain &chain) noexcept {
  if (world == 0) {
    return false;
  }
  std::uintptr_t game_instance{};
  std::uintptr_t players{};
  if (!ReadPointerAtOffset(core, world, kWorldGameInstanceOffset,
                           game_instance) ||
      !ReadPointerAtOffset(core, game_instance,
                           kGameInstanceLocalPlayersOffset, players)) {
    return false;
  }
  std::uintptr_t count_address{};
  std::int32_t players_count{};
  if (!AddAddress(game_instance,
                  kGameInstanceLocalPlayersOffset + sizeof(std::uintptr_t),
                  count_address) ||
      !Read(core, count_address, players_count) || players_count < 1 ||
      players_count > 64) {
    return false;
  }
  std::uintptr_t local_player{};
  std::uintptr_t controller{};
  return Read(core, players, local_player) && local_player != 0 &&
         ReadPointerAtOffset(core, local_player, kLocalPlayerControllerOffset,
                             controller) &&
         ReadPointerAtOffset(core, controller, kControllerPawnOffset,
                             chain.pawn) &&
         ReadPointerAtOffset(core, chain.pawn, kActorRootComponentOffset,
                             chain.root);
}

bool ReadChainLocation(const AnomalyCoreServiceV1 *core,
                       const std::uintptr_t root, double cm[3]) noexcept {
  std::uintptr_t address{};
  if (core == nullptr || core->read_memory == nullptr ||
      !AddAddress(root, kSceneComponentLocationOffset, address)) {
    return false;
  }
  std::array<double, 3> values{};
  AnomalyMutableByteSpanV1 destination{
      reinterpret_cast<std::uint8_t *>(values.data()), sizeof(values)};
  if (core->read_memory(core->user, address, destination).code !=
      ANOMALY_STATUS_V1_OK) {
    return false;
  }
  if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
      !std::isfinite(values[2])) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    cm[axis] = values[axis];
  }
  return true;
}

// Read the player SceneComponent's AttachParent and its ComponentToWorld. While
// driving this is the vehicle's `VehicleMesh` SkeletalMesh. ComponentToWorld is
// the transform the renderer copies into the scene proxy; the proxy's own
// LocalToWorld is preferred because it is the transform the renderer actually
// draws with, so the camera and the car share one clock. The bounds origin is
// deliberately not used as the anchor: it is a derived centre that moves when
// a damaged door or the suspension changes the mesh bounds. `has_parent`
// separates "on foot / no parent" from "parent exists but its transform did
// not validate", so the panel can count only the latter. `proxy_used` reports
// which of the two transforms was published.
bool ReadVehicleParentAnchor(const AnomalyCoreServiceV1 *core,
                             const std::uintptr_t root,
                             const double root_cm[3], double parent_cm[3],
                             std::uintptr_t &parent_out, bool &has_parent,
                             bool &attach_read_ok, bool &proxy_used,
                             std::uintptr_t &proxy_out) noexcept {
  parent_out = 0;
  has_parent = false;
  attach_read_ok = false;
  proxy_used = false;
  proxy_out = 0;
  std::uintptr_t parent{};
  if (!ReadPointerAtOffset(core, root, kSceneComponentAttachParentOffset,
                           parent)) {
    return false;
  }
  attach_read_ok = true;
  if (parent == 0 || parent == root) {
    return false;
  }
  has_parent = true;
  std::array<double, 4> orientation{};
  std::array<double, 3> translation{};
  if (!Read(core, parent + kSceneComponentTransformOffset, orientation) ||
      !Read(core, parent + kSceneComponentTransformTranslationOffset,
            translation)) {
    return false;
  }
  const double norm2 =
      orientation[0] * orientation[0] + orientation[1] * orientation[1] +
      orientation[2] * orientation[2] + orientation[3] * orientation[3];
  if (!(norm2 > 0.81 && norm2 < 1.21) ||
      !std::isfinite(translation[0]) || !std::isfinite(translation[1]) ||
      !std::isfinite(translation[2])) {
    return false;
  }
  double parent_bounds_cm[3]{};
  if (!ReadChainLocation(core, parent, parent_bounds_cm)) {
    return false;
  }
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = parent_bounds_cm[axis] - root_cm[axis];
    squared += delta * delta;
  }
  if (!(squared <= kVehicleAnchorMaxOffsetCm * kVehicleAnchorMaxOffsetCm)) {
    return false;
  }

  // Prefer the renderer's copy. The chain is deliberately narrow: the proxy
  // must claim this component as its owner, and its translation must still be
  // close to ComponentToWorld. Either check failing falls back to the
  // component transform for this frame; it never disables the vehicle source.
  std::array<double, 3> proxy_translation{};
  std::uintptr_t proxy{};
  if (ReadPointerAtOffset(core, parent, kVehicleProxyOffset, proxy) &&
      proxy != 0 && proxy != parent) {
    std::uintptr_t owner{};
    if (ReadPointerAtOffset(core, proxy, kVehicleProxyOwnerOffset, owner) &&
        owner == parent &&
        Read(core, proxy + kVehicleProxyTranslationOffset, proxy_translation)) {
      const double dx = proxy_translation[0] - translation[0];
      const double dy = proxy_translation[1] - translation[1];
      const double dz = proxy_translation[2] - translation[2];
      proxy_used = std::isfinite(proxy_translation[0]) &&
                   std::isfinite(proxy_translation[1]) &&
                   std::isfinite(proxy_translation[2]) &&
                   dx * dx + dy * dy + dz * dz <=
                       kVehicleProxyMaxOffsetCm * kVehicleProxyMaxOffsetCm;
    }
  }
  // The renderer transform, not the derived bounds centre. The constant keeps
  // the old bounds-centre framing; the user's `originOffsetCm` still applies.
  for (std::size_t axis = 0; axis < 3; ++axis) {
    parent_cm[axis] =
        proxy_used ? proxy_translation[axis] : translation[axis];
  }
  proxy_out = proxy_used ? proxy : 0;
  parent_cm[2] += kVehicleAnchorCenterHeightCm;
  parent_out = parent;
  return true;
}

bool HexEqualsIgnoreCase(const char *actual,
                         const std::string_view expected) noexcept {
  if (actual == nullptr) {
    return false;
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    char value = actual[index];
    if (value == '\0') {
      return false;
    }
    if (value >= 'A' && value <= 'Z') {
      value = static_cast<char>(value - 'A' + 'a');
    }
    char wanted = expected[index];
    if (wanted >= 'A' && wanted <= 'Z') {
      wanted = static_cast<char>(wanted - 'A' + 'a');
    }
    if (value != wanted) {
      return false;
    }
  }
  return actual[expected.size()] == '\0';
}

bool AcceptedProfileHash(const char *actual) noexcept {
  for (const std::string_view accepted : kAcceptedProfileHashes) {
    if (HexEqualsIgnoreCase(actual, accepted)) {
      return true;
    }
  }
  return false;
}

void TryResolveRenderX(Context &context) noexcept {
  RenderXProbe &probe = context.render_x;
  if (probe.insurance_state != kAnchorInsuranceReady) {
    return;
  }
  std::uintptr_t instruction{};
  const AnomalyStatusV1 status =
      ResolveSignatureStatus(context, kGWorldPattern, instruction);
  if (status.code == ANOMALY_STATUS_V1_OK && instruction != 0) {
    std::uintptr_t displacement_address{};
    std::int32_t displacement{};
    std::intptr_t slot = 0;
    if (AddAddress(instruction, kGWorldRipOffset, displacement_address) &&
        Read(context.core, displacement_address, displacement)) {
      slot = static_cast<std::intptr_t>(instruction) +
             static_cast<std::intptr_t>(kGWorldInstructionSize) +
             static_cast<std::intptr_t>(displacement);
    }
    if (slot > 0) {
      probe.gworld_slot = static_cast<std::uintptr_t>(slot);
      probe.resolve_state = kRenderXReady;
      probe.resolve_code = ANOMALY_STATUS_V1_OK;
      probe.anchor_enabled.store(probe.chain_verified && !probe.chain_failed,
                                 std::memory_order_release);
      return;
    }
  }
  probe.resolve_state = kRenderXFailed;
  probe.resolve_code =
      status.code == ANOMALY_STATUS_V1_OK ? ANOMALY_STATUS_V1_FAILED
                                          : status.code;
  probe.anchor_enabled.store(false, std::memory_order_release);
}

// Read the host's build identity once, then close or open the X gate. The
// hardcoded offsets are only meaningful for the profile they were verified
// against; a mismatch is a version change, not a bug to retry.
void EvaluateAnchorInsurance(Context &context) noexcept {
  RenderXProbe &probe = context.render_x;
  std::uint8_t next = kAnchorInsuranceServiceMissing;
  std::array<char, 96> hash{};
  std::size_t hash_size = hash.size();
  if (context.build != nullptr && context.build->profile_hash != nullptr) {
    const AnomalyStatusV1 status = context.build->profile_hash(
        context.build->user, hash.data(), &hash_size);
    if (status.code == ANOMALY_STATUS_V1_OK && hash_size > 1 &&
        hash_size <= hash.size() && hash[0] != '\0') {
      next = AcceptedProfileHash(hash.data())
                 ? kAnchorInsuranceReady
                 : kAnchorInsuranceHashMismatch;
    } else {
      next = kAnchorInsuranceReadFailed;
    }
  }
  probe.insurance_state = next;
  probe.version_gate.store(next == kAnchorInsuranceReady,
                           std::memory_order_release);
  if (next != kAnchorInsuranceReady) {
    probe.anchor_enabled.store(false, std::memory_order_release);
  }
  if (probe.logged_insurance_state != next) {
    probe.logged_insurance_state = next;
    char message[320]{};
    if (next == kAnchorInsuranceReady) {
      std::snprintf(message, sizeof(message),
                    "anchor gate: ready (profile %.*s)",
                    static_cast<int>(hash_size > 0 ? hash_size - 1 : 0),
                    hash.data());
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    } else if (next == kAnchorInsuranceHashMismatch) {
      std::snprintf(message, sizeof(message),
                    "anchor gate: profile hash mismatch (got %.*s, want one of "
                    "%zu accepted 2.7.0 profiles); camera hooks disabled until "
                    "the plugin is updated",
                    static_cast<int>(hash_size > 0 ? hash_size - 1 : 0),
                    hash.data(),
                    std::size(kAcceptedProfileHashes));
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    } else if (next == kAnchorInsuranceServiceMissing) {
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "anchor gate: ue5 build service unavailable; route B disabled, "
          "legacy view hook only");
    } else {
      std::snprintf(message, sizeof(message),
                    "anchor gate: profile hash unreadable (size=%zu); "
                    "camera hooks disabled",
                    hash_size);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    }
  }
  if (next == kAnchorInsuranceReady) {
    TryResolveRenderX(context);
  }
}

// Game tick: refresh the world pointer and the pawn chain, then compare the
// same chain's location with the host snapshot. Caching here keeps the render
// thread down to the single 24-byte read that the phase question needs.
void UpdateRenderXProbe(Context &context, const double base_cm[3],
                        const bool player_valid) noexcept {
  RenderXProbe &probe = context.render_x;
  if (probe.insurance_state == kAnchorInsuranceWaiting &&
      context.elapsed >= probe.next_resolve_elapsed) {
    // The profile can still be settling on the first tick; a hash that was not
    // readable yet is worth one more look.
    probe.next_resolve_elapsed = context.elapsed + kRenderXResolveRetrySeconds;
    EvaluateAnchorInsurance(context);
  } else if (probe.insurance_state == kAnchorInsuranceReady &&
             probe.resolve_state != kRenderXReady &&
             context.elapsed >= probe.next_resolve_elapsed) {
    // The profile can still be settling on the first ticks, so a failed
    // pattern scan is retried - slowly, because a section scan is not free.
    probe.next_resolve_elapsed = context.elapsed + kRenderXResolveRetrySeconds;
    TryResolveRenderX(context);
  }
  if (probe.resolve_state == kRenderXReady) {
    std::uintptr_t world{};
    PlayerChain chain{};
    if (Read(context.core, probe.gworld_slot, world) && world != 0 &&
        WalkPlayerChain(context.core, world, chain)) {
      probe.root.store(chain.root, std::memory_order_relaxed);
    } else {
      probe.root.store(0, std::memory_order_relaxed);
    }
  }
  if (player_valid) {
    const std::uintptr_t root = probe.root.load(std::memory_order_relaxed);
    double chain_cm[3]{};
    if (root != 0 && ReadChainLocation(context.core, root, chain_cm)) {
      double squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double delta = chain_cm[axis] - base_cm[axis];
        squared += delta * delta;
      }
      const double distance = std::sqrt(squared);
      if (std::isfinite(distance)) {
        if (distance > probe.chain_max_cm) {
          probe.chain_max_cm = distance;
        }
        if (distance > kRenderXChainToleranceCm) {
          ++probe.chain_mismatches;
          if (probe.chain_fail_streak < kAnchorChainMismatchLimit) {
            ++probe.chain_fail_streak;
          }
          if (probe.chain_fail_streak >= kAnchorChainMismatchLimit) {
            probe.chain_failed = true;
            probe.anchor_enabled.store(false, std::memory_order_release);
            probe.anchor_valid.store(false, std::memory_order_release);
          }
        } else {
          probe.chain_fail_streak = 0;
          probe.chain_verified = true;
          if (probe.resolve_state == kRenderXReady && !probe.chain_failed) {
            probe.anchor_enabled.store(true, std::memory_order_release);
          }
        }
      }
    }
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.player_cm[axis].store(base_cm[axis], std::memory_order_relaxed);
  }
  // Speed and direction for the phase sample, from the probe's own successive
  // snapshots rather than the camera model: the probe then works with the
  // camera switched off, which is the natural way to bisect the twitch.
  double velocity_cmps[3]{};
  if (player_valid && probe.have_last_player &&
      context.elapsed > probe.last_player_time) {
    const double step_seconds = context.elapsed - probe.last_player_time;
    double squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double step = base_cm[axis] - probe.last_player_cm[axis];
      velocity_cmps[axis] = step / step_seconds;
      squared += step * step;
    }
    // A reset/teleport is not a direction: hold the previous answer instead of
    // projecting a phase sample onto it.
    if (!(squared <
          kViewHistorySnapDistanceCm * kViewHistorySnapDistanceCm)) {
      for (double &value : velocity_cmps) {
        value = 0.0;
      }
    }
  } else {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      velocity_cmps[axis] =
          probe.velocity_cmps[axis].load(std::memory_order_relaxed);
    }
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.velocity_cmps[axis].store(velocity_cmps[axis],
                                    std::memory_order_relaxed);
    probe.last_player_cm[axis] = base_cm[axis];
  }
  probe.last_player_time = context.elapsed;
  probe.have_last_player = player_valid;
  probe.game_thread_id.store(static_cast<std::uint32_t>(GetCurrentThreadId()),
                             std::memory_order_relaxed);
  probe.player_valid.store(player_valid, std::memory_order_release);
}

// Render thread: one read per frame of the cached field, compared with the
// Game-tick snapshot. Runs even when the plugin is not replacing the view, so
// the phase can be measured with the camera switched off.
void SampleRenderXPhase(Context &context) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!probe.anchor_enabled.load(std::memory_order_acquire)) {
    probe.vehicle_present.store(false, std::memory_order_release);
    probe.vehicle_present_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_sample_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_parent.store(0, std::memory_order_relaxed);
    return;
  }
  // The detour runs once per render pass; the passes of one frame are only a
  // fraction of a millisecond apart. One read per 8 ms across all detour
  // threads: the first pass reads, the rest consume the same snapshot. This is
  // also the only read on the camera path; no allocation, no lock, no log.
  const std::int64_t now_ns = NowNanoseconds();
  std::int64_t previous = probe.last_read_ns.load(std::memory_order_relaxed);
  if (previous != 0 && now_ns - previous < kPresenterSameFrameNs) {
    return;
  }
  if (!probe.last_read_ns.compare_exchange_strong(
          previous, now_ns, std::memory_order_relaxed)) {
    return;
  }
  const std::uintptr_t root = probe.root.load(std::memory_order_relaxed);
  if (root == 0) {
    probe.vehicle_present.store(false, std::memory_order_release);
    probe.vehicle_present_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_sample_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_parent.store(0, std::memory_order_relaxed);
    return;
  }
  double render_cm[3]{};
  if (!ReadChainLocation(context.core, root, render_cm)) {
    probe.read_failures.fetch_add(1, std::memory_order_relaxed);
    probe.anchor_read_failures.fetch_add(1, std::memory_order_relaxed);
    probe.anchor_valid.store(false, std::memory_order_release);
    probe.vehicle_anchor_valid.store(false, std::memory_order_release);
    probe.vehicle_render_used.store(false, std::memory_order_relaxed);
    probe.vehicle_present.store(false, std::memory_order_release);
    probe.vehicle_present_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_sample_ns.store(0, std::memory_order_relaxed);
    probe.vehicle_parent.store(0, std::memory_order_relaxed);
    probe.watched_proxy.store(0, std::memory_order_relaxed);
    return;
  }
  // Publish the same read the phase uses as the anchor snapshot. It is valid
  // at any speed; the phase statistics below still need a moving car.
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.anchor_cm[axis].store(render_cm[axis], std::memory_order_relaxed);
  }
  probe.anchor_ns.store(NowNanoseconds(), std::memory_order_relaxed);
  probe.anchor_valid.store(true, std::memory_order_release);
  // Same detour pass: follow the capsule's AttachParent to the vehicle mesh.
  // The parent's ComponentToWorld is the chassis transform; its bounds origin is
  // only the sanity distance here. When the character is on foot there is no
  // parent and the vehicle source falls back to X.
  {
    double parent_cm[3]{};
    std::uintptr_t parent{};
    bool has_parent = false;
    bool attach_read_ok = false;
    bool proxy_used = false;
    std::uintptr_t proxy{};
    if (ReadVehicleParentAnchor(context.core, root, render_cm, parent_cm,
                                parent, has_parent, attach_read_ok, proxy_used,
                                proxy)) {
      probe.vehicle_anchor_miss_streak.store(0, std::memory_order_relaxed);
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.vehicle_anchor_cm[axis].store(parent_cm[axis],
                                            std::memory_order_relaxed);
      }
      probe.vehicle_parent.store(parent, std::memory_order_relaxed);
      probe.vehicle_proxy_used.store(proxy_used, std::memory_order_relaxed);
      probe.watched_proxy.store(proxy, std::memory_order_relaxed);
      if (!proxy_used) {
        probe.vehicle_proxy_fallbacks.fetch_add(1,
                                                std::memory_order_relaxed);
      }
      probe.vehicle_anchor_ns.store(NowNanoseconds(),
                                    std::memory_order_relaxed);
      probe.vehicle_anchor_valid.store(true, std::memory_order_release);
      const std::int64_t presence_ns = NowNanoseconds();
      probe.vehicle_present_ns.store(presence_ns, std::memory_order_relaxed);
      probe.vehicle_sample_ns.store(presence_ns, std::memory_order_relaxed);
      probe.vehicle_present.store(true, std::memory_order_release);
      // Prefer the render thread's OnTransformChanged value when it belongs to this
      // exact proxy and is fresh. The Game-tick proxy read above and this
      // value are the same field, but the hook is written by the renderer at
      // the instant it copies the transform, so it is the clock the drawn car
      // actually uses. Fall back to the Game-tick read if the hook has not
      // fired yet or its snapshot is stale.
      bool render_used = false;
      if (proxy != 0 &&
          probe.transform_valid.load(std::memory_order_acquire)) {
        const std::uint64_t first =
            probe.transform_sequence.load(std::memory_order_acquire);
        if ((first & 1U) == 0) {
          const std::uintptr_t rendered_proxy =
              probe.transform_proxy.load(std::memory_order_relaxed);
          double rendered_cm[3]{};
          std::int64_t rendered_ns = 0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            rendered_cm[axis] =
                probe.transform_cm[axis].load(std::memory_order_relaxed);
          }
          rendered_ns =
              probe.transform_ns.load(std::memory_order_relaxed);
          const std::uint64_t second =
              probe.transform_sequence.load(std::memory_order_acquire);
          const std::int64_t age = NowNanoseconds() - rendered_ns;
          double squared = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double height =
                axis == 2 ? kVehicleAnchorCenterHeightCm : 0.0;
            const double delta =
                rendered_cm[axis] + height - parent_cm[axis];
            squared += delta * delta;
          }
          if (first == second && rendered_proxy == proxy &&
              std::isfinite(rendered_cm[0]) &&
              std::isfinite(rendered_cm[1]) &&
              std::isfinite(rendered_cm[2]) && age >= 0 &&
              age <= kVehicleRenderTransformMaxAgeNs &&
              squared <=
                  kVehicleProxyMaxOffsetCm * kVehicleProxyMaxOffsetCm) {
            {
              const auto age_us =
                  static_cast<std::uint64_t>(age / 1000);
              RaisePeak(probe.render_age_peak_us, age_us);
              probe.render_age_sum_us.fetch_add(age_us,
                                                std::memory_order_relaxed);
              probe.render_age_count.fetch_add(1, std::memory_order_relaxed);
            }
            // Same proxy, two clocks. `parent_cm` is the Game-tick read of the
            // proxy; `rendered_cm` is the renderer's notification. Their
            // difference is the phase the camera would inherit if it used the
            // hook. Wall distance and travel time are both reported because a
            // fixed mount offset becomes a speed-dependent time, while a real
            // phase step moves both.
            double phase_cm[3]{};
            double phase_squared = 0.0;
            for (std::size_t axis = 0; axis < 3; ++axis) {
              const double height =
                  axis == 2 ? kVehicleAnchorCenterHeightCm : 0.0;
              phase_cm[axis] = rendered_cm[axis] + height - parent_cm[axis];
              phase_squared += phase_cm[axis] * phase_cm[axis];
            }
            const double phase_distance = std::sqrt(phase_squared);
            if (std::isfinite(phase_distance) &&
                phase_distance < kViewHistorySnapDistanceCm) {
              const auto phase_dcm = static_cast<std::uint64_t>(
                  phase_distance * 10.0 + 0.5);
              RaisePeak(probe.render_proxy_peak_dcm, phase_dcm);
              probe.render_proxy_sum_dcm.fetch_add(
                  phase_dcm, std::memory_order_relaxed);
              probe.render_proxy_count.fetch_add(1,
                                                 std::memory_order_relaxed);
              double velocity_cmps[3]{};
              double speed_squared = 0.0;
              for (std::size_t axis = 0; axis < 3; ++axis) {
                velocity_cmps[axis] =
                    probe.velocity_cmps[axis].load(std::memory_order_relaxed);
                speed_squared += velocity_cmps[axis] * velocity_cmps[axis];
              }
              const double speed_cmps = std::sqrt(speed_squared);
              if (speed_cmps >= kRenderXMinimumSpeedCmps) {
                const double phase_ms = phase_distance / speed_cmps * 1000.0;
                if (std::isfinite(phase_ms) && phase_ms < 1000.0) {
                  const auto phase_dms =
                      static_cast<std::uint64_t>(phase_ms * 10.0 + 0.5);
                  RaisePeak(probe.render_proxy_peak_dms, phase_dms);
                  probe.render_proxy_sum_dms.fetch_add(
                      phase_dms, std::memory_order_relaxed);
                  probe.render_proxy_time_count.fetch_add(
                      1, std::memory_order_relaxed);
                }
                const double lead_cm =
                    (phase_cm[0] * velocity_cmps[0] +
                     phase_cm[1] * velocity_cmps[1] +
                     phase_cm[2] * velocity_cmps[2]) /
                    speed_cmps;
                if (std::isfinite(lead_cm) &&
                    std::fabs(lead_cm) < kViewHistorySnapDistanceCm) {
                  probe.render_proxy_lead_sum_dcm.fetch_add(
                      static_cast<std::int64_t>(lead_cm * 10.0),
                      std::memory_order_relaxed);
                  probe.render_proxy_lead_sum_dms.fetch_add(
                      static_cast<std::int64_t>(lead_cm / speed_cmps * 1000.0 *
                                                10.0),
                      std::memory_order_relaxed);
                  probe.render_proxy_lead_count.fetch_add(
                      1, std::memory_order_relaxed);
                }
              }
            }
            for (std::size_t axis = 0; axis < 3; ++axis) {
              const double height =
                  axis == 2 ? kVehicleAnchorCenterHeightCm : 0.0;
              probe.vehicle_anchor_cm[axis].store(
                  rendered_cm[axis] + height, std::memory_order_relaxed);
            }
            probe.vehicle_anchor_ns.store(rendered_ns,
                                          std::memory_order_relaxed);
            render_used = true;
          }
        }
      }
      probe.vehicle_render_used.store(render_used,
                                      std::memory_order_relaxed);
    } else {
      const bool definitely_on_foot = attach_read_ok && !has_parent;
      const std::uint32_t misses =
          probe.vehicle_anchor_miss_streak.fetch_add(
              1, std::memory_order_relaxed) +
          1;
      // A single torn read is not a vehicle change. Keep the last valid
      // proxy/anchor for at most three 20 Hz polls (about 24 ms), which covers
      // the render-thread writer without ever pinning a freed proxy for long.
      if (definitely_on_foot || misses >= 3) {
        probe.watched_proxy.store(0, std::memory_order_relaxed);
        probe.vehicle_anchor_valid.store(false, std::memory_order_release);
        probe.vehicle_proxy_used.store(false, std::memory_order_relaxed);
        probe.vehicle_render_used.store(false, std::memory_order_relaxed);
        probe.vehicle_present.store(false, std::memory_order_release);
        probe.vehicle_present_ns.store(0, std::memory_order_relaxed);
        probe.vehicle_sample_ns.store(
            definitely_on_foot ? NowNanoseconds() : 0,
            std::memory_order_relaxed);
        probe.vehicle_parent.store(0, std::memory_order_relaxed);
        probe.vehicle_anchor_miss_streak.store(0, std::memory_order_relaxed);
      } else {
        probe.vehicle_render_used.store(false, std::memory_order_relaxed);
        probe.vehicle_read_failures.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  if (!probe.player_valid.load(std::memory_order_acquire)) {
    return;
  }
  double player_cm[3]{};
  double velocity_cmps[3]{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    player_cm[axis] = probe.player_cm[axis].load(std::memory_order_relaxed);
    velocity_cmps[axis] =
        probe.velocity_cmps[axis].load(std::memory_order_relaxed);
  }
  const double speed_cmps =
      std::sqrt(velocity_cmps[0] * velocity_cmps[0] +
                velocity_cmps[1] * velocity_cmps[1]);
  if (!(speed_cmps >= kRenderXMinimumSpeedCmps)) {
    return;
  }
  double difference_cm[3]{};
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    difference_cm[axis] = render_cm[axis] - player_cm[axis];
    squared += difference_cm[axis] * difference_cm[axis];
  }
  const double distance_cm = std::sqrt(squared);
  // A teleport, a dead pointer or garbage: not a phase sample, and far outside
  // the range this question is about.
  if (!std::isfinite(distance_cm) ||
      distance_cm >= kViewHistorySnapDistanceCm) {
    return;
  }
  RaisePeak(probe.peak_dcm,
            static_cast<std::uint64_t>(distance_cm * 10.0));
  probe.sum_dcm.fetch_add(static_cast<std::uint64_t>(distance_cm * 10.0),
                          std::memory_order_relaxed);
  // The same distance as time. Speed varies inside the 0.5 s window, so the
  // centimetres alone cannot be compared with a tick interval; the phase this
  // probe is about is a time offset, and this is it.
  const double distance_ms = distance_cm / speed_cmps * 1000.0;
  if (std::isfinite(distance_ms) && distance_ms < 1000.0) {
    RaisePeak(probe.peak_dms,
              static_cast<std::uint64_t>(distance_ms * 10.0));
    probe.sum_dms.fetch_add(static_cast<std::uint64_t>(distance_ms * 10.0),
                            std::memory_order_relaxed);
  }
  probe.speed_sum_cmps.fetch_add(static_cast<std::uint64_t>(speed_cmps),
                                 std::memory_order_relaxed);
  probe.sample_count.fetch_add(1, std::memory_order_relaxed);
  if (static_cast<std::uint32_t>(GetCurrentThreadId()) ==
      probe.game_thread_id.load(std::memory_order_relaxed)) {
    probe.samples_on_game_thread.fetch_add(1, std::memory_order_relaxed);
  }
  const double lead_cm =
      (difference_cm[0] * velocity_cmps[0] +
       difference_cm[1] * velocity_cmps[1] +
       difference_cm[2] * velocity_cmps[2]) /
      speed_cmps;
  if (std::isfinite(lead_cm) &&
      std::fabs(lead_cm) < kViewHistorySnapDistanceCm) {
    probe.lead_sum_dcm.fetch_add(static_cast<std::int64_t>(lead_cm * 10.0),
                                 std::memory_order_relaxed);
    probe.lead_sum_dms.fetch_add(
        static_cast<std::int64_t>(lead_cm / speed_cmps * 1000.0 * 10.0),
        std::memory_order_relaxed);
    probe.lead_count.fetch_add(1, std::memory_order_relaxed);
  }
}

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept;

// Experimental: apply the fresh render proxy transform after the car's
// OnTransformChanged has updated LocalToWorld. This is deliberately a
// separate writer from ApplyRouteBEarly; it is used to test whether the
// transform notification is still before the visibility task data consumes
// the view.
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept;

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

// Route B's initializer, matching FMinimalInitializer in the 2.6.0 build.
// Every offset here is pinned by static_assert: the render-thread constructor
// reads these fields directly, so a layout drift must fail at compile time
// rather than move the camera from a misread matrix.
struct alignas(16) RouteBViewInit final {
  double view_rotation[4][4]{};
  double projection[4][4]{};
  double view_origin[3]{};
  std::int32_t constrained_rect[4]{};
  std::int32_t stereo_pass{};
  bool use_faux_ortho{};
};
static_assert(offsetof(RouteBViewInit, projection) == 0x80);
static_assert(offsetof(RouteBViewInit, view_origin) == 0x100);
static_assert(offsetof(RouteBViewInit, constrained_rect) == 0x118);
static_assert(offsetof(RouteBViewInit, stereo_pass) == 0x128);
static_assert(offsetof(RouteBViewInit, use_faux_ortho) == 0x12C);
static_assert(sizeof(RouteBViewInit) == 0x130);

void Multiply4x4(const double a[4][4], const double b[4][4],
                 double out[4][4]) noexcept {
  double result[4][4]{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      double sum = 0.0;
      for (std::size_t inner = 0; inner < 4; ++inner) {
        sum += a[row][inner] * b[inner][column];
      }
      result[row][column] = sum;
    }
  }
  std::memcpy(out, result, sizeof(result));
}

// FInverseRotationMatrix(ViewRotation) * ViewPlanesMatrix, exactly the
// composition FViewMatrices uses for a view rotation (degrees).
void MakeViewRotationMatrix(const double pitch_degrees,
                            const double yaw_degrees,
                            const double roll_degrees,
                            double out[4][4]) noexcept {
  const double yaw = yaw_degrees * kDegreesToRadians;
  const double pitch = pitch_degrees * kDegreesToRadians;
  const double roll = roll_degrees * kDegreesToRadians;
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);

  const double yaw_matrix[4][4] = {
      {cy, -sy, 0.0, 0.0},
      {sy, cy, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double pitch_matrix[4][4] = {
      {cp, 0.0, -sp, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {sp, 0.0, cp, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double roll_matrix[4][4] = {
      {1.0, 0.0, 0.0, 0.0},
      {0.0, cr, sr, 0.0},
      {0.0, -sr, cr, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double planes[4][4] = {
      {0.0, 0.0, 1.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  double yaw_pitch[4][4]{};
  double yaw_pitch_roll[4][4]{};
  Multiply4x4(yaw_matrix, pitch_matrix, yaw_pitch);
  Multiply4x4(yaw_pitch, roll_matrix, yaw_pitch_roll);
  Multiply4x4(yaw_pitch_roll, planes, out);
}

bool ReadRouteBSnapshot(const RenderXProbe &probe, double car_cm[3],
                        double forward[3], std::uintptr_t &proxy,
                        std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint64_t first =
        probe.transform_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0) {
      continue;
    }
    const std::uintptr_t watched =
        probe.watched_proxy.load(std::memory_order_relaxed);
    const std::uintptr_t current_proxy =
        probe.transform_proxy.load(std::memory_order_relaxed);
    const bool valid =
        probe.transform_valid.load(std::memory_order_relaxed);
    const bool orientation_valid =
        probe.transform_orientation_valid.load(std::memory_order_relaxed);
    double current_car[3]{};
    double current_forward[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      current_car[axis] =
          probe.transform_cm[axis].load(std::memory_order_relaxed);
      current_forward[axis] =
          probe.transform_forward[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t current_timestamp =
        probe.transform_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.transform_sequence.load(std::memory_order_acquire);
    if (first != second || !valid || !orientation_valid ||
        watched == 0 || current_proxy != watched ||
        current_timestamp == 0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = current_car[axis];
      forward[axis] = current_forward[axis];
    }
    proxy = current_proxy;
    timestamp_ns = current_timestamp;
    return std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
           std::isfinite(car_cm[2]) && std::isfinite(forward[0]) &&
           std::isfinite(forward[1]) && std::isfinite(forward[2]);
  }
  return false;
}

// The camera model's pitch source. Same seqlock and proxy checks as route B,
// but only the body pitch is needed. `atan2` handles both the old horizontally
// normalised vector and the new 3D one.
bool ReadRenderPitch(const RenderXProbe &probe, double &pitch,
                     std::uintptr_t &proxy,
                     std::int64_t &timestamp_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, snapshot_proxy,
                          snapshot_ns)) {
    return false;
  }
  if (!BodyPitchFromForward(forward, pitch)) {
    return false;
  }
  proxy = snapshot_proxy;
  timestamp_ns = snapshot_ns;
  return true;
}

// Update the cached body pitch from the render-thread snapshot and return the
// value for this Game tick. Fresh snapshots win; while moving, a short render
// gap holds the last good value and then falls back to flat. A parked car keeps
// the last orientation because the transform hook may not fire again until it
// moves. The snapshot must also be newer than the last reset/teleport floor, so
// an old slope cannot leak into the new frame.
double UpdateRenderPitch(Context &context) noexcept {
  double pitch = 0.0;
  std::uintptr_t proxy = 0;
  std::int64_t snapshot_ns = 0;
  const bool read = ReadRenderPitch(context.render_x, pitch, proxy,
                                    snapshot_ns);
  const std::uintptr_t watched =
      context.render_x.watched_proxy.load(std::memory_order_relaxed);
  // A different watched proxy is a different car. Drop the cached attitude
  // before trying the new one; otherwise a parked car switch could keep the
  // old car's slope indefinitely.
  if (context.render_pitch_valid && watched != 0 &&
      watched != context.render_pitch_proxy) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
  }
  const std::int64_t now_ns = NowNanoseconds();
  if (read && proxy == watched &&
      snapshot_ns >= context.render_pitch_floor_ns) {
    const std::int64_t age = now_ns - snapshot_ns;
    if (age >= 0 && age <= kVehicleRenderTransformMaxAgeNs) {
      context.render_pitch_valid = true;
      context.render_pitch_radians = pitch;
      context.render_pitch_ns = snapshot_ns;
      context.render_pitch_proxy = proxy;
    }
  }
  if (!context.render_pitch_valid) {
    return 0.0;
  }
  const std::int64_t held = now_ns - context.render_pitch_ns;
  const double speed_cmps =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const bool parked = !(speed_cmps >= kRenderPitchStationarySpeedCmps);
  if (held < 0 || (!parked && held > kRenderPitchHoldNs)) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
    return 0.0;
  }
  return context.render_pitch_radians;
}

// The route B fallback is the only path that applies body pitch outside the
// chase model. It must obey the same master switch and flat-pose diagnostic so
// disabling slope follow also disables the fallback tilt.
[[nodiscard]] bool RouteBFollowPitch(const Context &context) noexcept {
  return context.slope_follow.load(std::memory_order_acquire) &&
         !context.diagnostic_flat_pose.load(std::memory_order_acquire);
}

bool BuildRouteBPoseFromCar(const double car_cm[3],
                            const double forward[3],
                            const bool follow_pitch, double eye[3],
                            double rotation[3]) noexcept {
  // The fallback uses the same body basis as the chase model's position frame.
  // `forward` may be fully normalised or only horizontally normalised; the
  // helper handles both. The master switch and the flat-pose diagnostic keep
  // the fallback yaw-only too.
  double body_pitch = 0.0;
  if (!BodyPitchFromForward(forward, body_pitch)) {
    return false;
  }
  if (!follow_pitch) {
    body_pitch = 0.0;
  }
  const double car_yaw = std::atan2(forward[1], forward[0]);
  // FViewMatrices and FViewInfo store world positions in Unreal units
  // (centimetres), unlike the chase model's metres. Keep this conversion
  // explicit: a metre-valued ViewOrigin puts the camera near the world origin
  // and leaves only the skybox visible.
  const nfs2015_chase::Vec3 car{
      car_cm[0],
      car_cm[1],
      car_cm[2] + kVehicleAnchorCenterHeightCm,
  };
  const nfs2015_chase::Vec3 rest{kRouteBRestXM * 100.0, 0.0,
                                 kRouteBRestZM * 100.0};
  const nfs2015_chase::Vec3 look_offset{kRouteBLookAheadM * 100.0, 0.0,
                                        kRouteBLookHeightM * 100.0};
  // `rest` is in centimetres, matching `car_cm`; the basis is unitless, so
  // CarLocalToWorld keeps the centimetre scale.
  const nfs2015_chase::CarBasis basis =
      nfs2015_chase::MakeCarBasis(car_yaw, body_pitch);
  const nfs2015_chase::Vec3 eye_vec =
      nfs2015_chase::Add(car, nfs2015_chase::CarLocalToWorld(rest, basis));
  // The look target stays in the world-horizontal nose frame: the rig climbs
  // with the body, but the horizon must not tilt with the slope.
  const nfs2015_chase::Vec3 look_vec =
      nfs2015_chase::Add(car, nfs2015_chase::RotateYaw(car_yaw, look_offset));
  const nfs2015_chase::Vec3 delta =
      nfs2015_chase::Sub(look_vec, eye_vec);
  const double delta_horizontal =
      std::sqrt(delta.x * delta.x + delta.y * delta.y);
  if (!(delta_horizontal > 1e-4)) {
    return false;
  }
  eye[0] = eye_vec.x;
  eye[1] = eye_vec.y;
  eye[2] = eye_vec.z;
  rotation[0] = std::atan2(delta.z, delta_horizontal) * kRadiansToDegrees;
  rotation[1] = std::atan2(delta.y, delta.x) * kRadiansToDegrees;
  rotation[2] = 0.0;
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

bool ReadRouteBModelPose(const RenderXProbe &probe, double car_cm[3],
                         double eye_cm[3], double rotation[3],
                         std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t first =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0 || !probe.route_b_model_valid.load(
                                     std::memory_order_relaxed)) {
      continue;
    }
    double local_car[3]{};
    double local_eye[3]{};
    double local_rotation[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      local_car[axis] =
          probe.route_b_model_car_cm[axis].load(std::memory_order_relaxed);
      local_eye[axis] =
          probe.route_b_model_eye_cm[axis].load(std::memory_order_relaxed);
      local_rotation[axis] =
          probe.route_b_model_rotation[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t timestamp =
        probe.route_b_model_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if (first != second || timestamp == 0 || !std::isfinite(local_car[0]) ||
        !std::isfinite(local_car[1]) || !std::isfinite(local_car[2]) ||
        !std::isfinite(local_eye[0]) || !std::isfinite(local_eye[1]) ||
        !std::isfinite(local_eye[2]) || !std::isfinite(local_rotation[0]) ||
        !std::isfinite(local_rotation[1]) ||
        !std::isfinite(local_rotation[2])) {
      continue;
    }
    const double dx = local_eye[0] - local_car[0];
    const double dy = local_eye[1] - local_car[1];
    const double dz = local_eye[2] - local_car[2];
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
        dx * dx + dy * dy + dz * dz > 5000.0 * 5000.0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = local_car[axis];
      eye_cm[axis] = local_eye[axis];
      rotation[axis] = local_rotation[axis];
    }
    timestamp_ns = timestamp;
    return true;
  }
  return false;
}

// The NFS camera is still the model's camera: the render transform only
// supplies this frame's car translation. Moving the model pose by that delta
// keeps the NFS offset/attitude (drift swing, speed dolly, roll) while making
// the camera and the drawn car share one render frame. If the model has not
// produced a pose yet, the old rigid mount is the safe fallback.
bool BuildRouteBPoseFromModel(const RenderXProbe &probe,
                              const double car_cm[3],
                              const double forward[3], double eye[3],
                              double rotation[3]) noexcept {
  double model_car[3]{};
  double model_eye[3]{};
  double model_rotation[3]{};
  std::int64_t model_ns = 0;
  if (!ReadRouteBModelPose(probe, model_car, model_eye, model_rotation,
                           model_ns)) {
    return false;
  }
  const std::int64_t age = NowNanoseconds() - model_ns;
  if (age < 0 || age > 100'000'000) {
    return false;
  }
  // `model_eye` already contains the body-frame slope from the chase model.
  // Only its translation moves to this render frame; applying the pitch again
  // here would double the slope.
  const double fresh_car[3] = {
      car_cm[0],
      car_cm[1],
      car_cm[2] + kVehicleAnchorCenterHeightCm,
  };
  double delta[3]{};
  double distance_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    delta[axis] = fresh_car[axis] - model_car[axis];
    distance_squared += delta[axis] * delta[axis];
  }
  if (!std::isfinite(distance_squared) ||
      distance_squared > kVehicleProxyMaxOffsetCm * kVehicleProxyMaxOffsetCm) {
    return false;
  }
  const double scale = nfs2015_chase::Clamp(
      probe.route_b_model_camera_scale.load(std::memory_order_relaxed),
      kMinimumCameraScale, kMaximumCameraScale);
  const double origin_offset_cm =
      probe.route_b_model_origin_offset_cm.load(std::memory_order_relaxed);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double model_offset = model_eye[axis] - model_car[axis];
    eye[axis] = fresh_car[axis] + model_offset * scale;
    rotation[axis] = model_rotation[axis];
  }
  // `origin_offset_cm` is a user-facing vertical trim. The model already
  // contains the game-tick copy of it, so the render anchor above must not
  // apply it twice; adding it here makes the slider move route B.
  eye[2] += origin_offset_cm;
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]) &&
         std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
         std::isfinite(forward[2]);
}

bool BuildRouteBPose(const RenderXProbe &probe, const bool follow_pitch,
                     double eye[3], double rotation[3],
                     std::int64_t &age_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t proxy{};
  std::int64_t transform_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, proxy, transform_ns)) {
    return false;
  }
  const std::int64_t now = NowNanoseconds();
  age_ns = now - transform_ns;
  if (age_ns < 0 || age_ns > kVehicleRenderTransformMaxAgeNs) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

// The pre-visibility pass can run before the render transform notification
// publishes this frame's proxy matrix. The proxy itself, however, is already
// the object the renderer draws the car from. Read its current LocalToWorld
// directly (not as a prediction): if the matrix has already advanced, this
// keeps route B on the exact same car frame as the draw; if it has not, the
// snapshot fallback is used.
bool ReadPresenterPose(const Context &context, double location[3],
                       double rotation[3], float *lens,
                       bool *fov_enabled,
                       double car_cm[3] = nullptr) noexcept;
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept;

// The Game thread already published the exact NFS camera pose it is using
// (`CameraViewPointDetour`). Applying that pose before visibility avoids
// rebuilding a camera from a car transform that the render thread has not
// received yet: at this point the transform snapshot is often one frame old,
// while the presenter pose is the camera the game itself would draw with.
//
// The presenter pose is translated by the difference between the car it was
// built from and the car the renderer is about to draw. That is a same-frame
// anchor, not a speed/prediction compensation: no extrapolation, no latency
// estimate, only the transform read from the watched proxy on this frame.
bool BuildRouteBPoseFromPresenter(Context &context, const RenderXProbe &probe,
                                  double eye[3],
                                  double rotation[3]) noexcept {
  double presenter_car[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  if (!ReadPresenterPose(context, eye, rotation, &lens, &fov_enabled,
                         presenter_car)) {
    return false;
  }
  double fresh_car[3]{};
  double forward[3]{};
  if (ReadRouteBDirectCar(probe, fresh_car, forward)) {
    // Move the whole presenter pose onto this frame's VehicleMesh transform.
    // Aligning only X/Y left the camera on the Game tick's Z, so on a slope
    // the drawn car moved vertically under a camera that did not: visible
    // up/down bobbing. Z must come from the same ComponentToWorld as X/Y.
    for (std::size_t axis = 0; axis < 3; ++axis) {
      eye[axis] += fresh_car[axis] - presenter_car[axis];
    }
    // `presenter_car` already contains the configured origin offset, and
    // moving to the raw VehicleMesh cancels it. Re-apply the slider as a trim
    // relative to the default; comparing against the presenter's own offset
    // is always zero in steady state, which is why that version was inert.
    const double current_offset =
        context.origin_offset_cm.load(std::memory_order_acquire);
    eye[2] += current_offset - kDefaultOriginOffsetCm;
  }
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

// Read the car transform the renderer is using. The VehicleMesh component
// transform is the authoritative value at the Game camera point: the probe
// showed parent-final = 0.00 cm while the watched proxy was still one frame
// behind. Prefer that component; the proxy remains the fallback for frames
// where the parent pointer is not available. Returns the raw translation
// (without the anchor centre height).
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept {
  const std::uintptr_t parent =
      probe.vehicle_parent.load(std::memory_order_relaxed);
  if (parent != 0) {
    const auto *translation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformTranslationOffset);
    const auto *orientation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformOffset);
    const double qx = orientation[0];
    const double qy = orientation[1];
    const double qz = orientation[2];
    const double qw = orientation[3];
    const double norm2 = qx * qx + qy * qy + qz * qz + qw * qw;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = translation[axis];
    }
    if (std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
        std::isfinite(car_cm[2]) && norm2 > 0.81 && norm2 < 1.21) {
      const double inverse_norm = 1.0 / std::sqrt(norm2);
      const double x = qx * inverse_norm;
      const double y = qy * inverse_norm;
      const double z = qz * inverse_norm;
      const double w = qw * inverse_norm;
      // Rotate the local +X (nose) axis by the ComponentToWorld quaternion.
      const double tx = 0.0;
      const double ty = 2.0 * z;
      const double tz = -2.0 * y;
      forward[0] = 1.0 + w * tx + (y * tz - z * ty);
      forward[1] = w * ty + (z * tx - x * tz);
      forward[2] = w * tz + (x * ty - y * tx);
      const double length_squared = forward[0] * forward[0] +
                                    forward[1] * forward[1] +
                                    forward[2] * forward[2];
      if (std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
          std::isfinite(forward[2]) && length_squared > 1e-8) {
        const double inverse_length = 1.0 / std::sqrt(length_squared);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          forward[axis] *= inverse_length;
        }
        return true;
      }
    }
  }

  double snapshot_car[3]{};
  double snapshot_forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  const bool have_snapshot =
      ReadRouteBSnapshot(probe, snapshot_car, snapshot_forward,
                         snapshot_proxy, snapshot_ns);
  const std::uintptr_t watched =
      probe.watched_proxy.load(std::memory_order_relaxed);
  if (watched == 0) {
    return false;
  }
  const bool snapshot_matches = have_snapshot && snapshot_proxy == watched;

  const auto *matrix = reinterpret_cast<const double *>(watched + 0xE0);
  const auto *translation = reinterpret_cast<const double *>(
      watched + kVehicleProxyTranslationOffset);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    car_cm[axis] = translation[axis];
    forward[axis] = matrix[axis];
  }
  if (!std::isfinite(car_cm[0]) || !std::isfinite(car_cm[1]) ||
      !std::isfinite(car_cm[2]) || !std::isfinite(forward[0]) ||
      !std::isfinite(forward[1]) || !std::isfinite(forward[2])) {
    return false;
  }
  const double length_squared = forward[0] * forward[0] +
                                forward[1] * forward[1] +
                                forward[2] * forward[2];
  if (!(length_squared > 1e-8)) {
    return false;
  }
  const double inverse_length = 1.0 / std::sqrt(length_squared);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    forward[axis] *= inverse_length;
  }

  double reference_cm[3]{};
  bool have_reference = false;
  double limit_cm = 0.0;
  if (snapshot_matches) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] = snapshot_car[axis];
    }
    have_reference = true;
    limit_cm = kVehicleProxyMaxOffsetCm;
  } else if (probe.vehicle_anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.vehicle_anchor_cm[axis].load(std::memory_order_relaxed);
    }
    reference_cm[2] -= kVehicleAnchorCenterHeightCm;
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  } else if (probe.anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.anchor_cm[axis].load(std::memory_order_relaxed);
    }
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  }
  if (have_reference) {
    double distance_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = car_cm[axis] - reference_cm[axis];
      distance_squared += delta * delta;
    }
    if (!std::isfinite(distance_squared) ||
        distance_squared > limit_cm * limit_cm) {
      return false;
    }
  }
  return true;
}

bool BuildRouteBPoseDirect(const RenderXProbe &probe, const bool follow_pitch,
                           double eye[3],
                           double rotation[3]) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  if (!ReadRouteBDirectCar(probe, car_cm, forward)) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

bool IsRouteBFullScreenView(void *view) noexcept {
  if (view == nullptr ||
      reinterpret_cast<std::uintptr_t>(view) < 0x10000) {
    return false;
  }
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(
                  reinterpret_cast<std::uintptr_t>(view) +
                  kViewInfoRectOffset),
              sizeof(rect));
  return rect[2] >= kRouteBMinViewWidth &&
         rect[3] >= kRouteBMinViewHeight;
}

double Determinant4x4(const double matrix[16]) noexcept {
  return matrix[0] *
             (matrix[5] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) +
              matrix[7] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13])) -
         matrix[1] *
             (matrix[4] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12])) +
         matrix[2] *
             (matrix[4] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12])) -
         matrix[3] *
             (matrix[4] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12]) +
              matrix[6] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12]));
}

// A hash match proves the offsets were verified against this profile; it does
// not prove that a particular FViewInfo is still a live full-screen view. Check
// the fields route B is about to read and write before touching them, so a
// changed view layout fails closed instead of writing a camera pose into an
// unrelated object.
bool IsRouteBViewWritable(void *view) noexcept {
  if (!IsRouteBFullScreenView(view)) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);
  double location[3]{};
  double rotation[3]{};
  double projection[16]{};
  float fov = 0.0F;
  std::memcpy(location,
              reinterpret_cast<const void *>(address + kViewLocationOffset),
              sizeof(location));
  std::memcpy(rotation,
              reinterpret_cast<const void *>(address + kViewRotationOffset),
              sizeof(rotation));
  std::memcpy(projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(projection));
  std::memcpy(&fov, reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(fov));
  for (double value : location) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : rotation) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : projection) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (!(std::fabs(Determinant4x4(projection)) > 1e-12) ||
      !(fov > 1.0F) || !(fov < 179.0F)) {
    return false;
  }
  return true;
}

// Write the final route B pose into one view, before visibility and before
// PrepareViewStateForVisibility snapshots the temporal history. This is the
// only place the camera may be moved: a post-Prepare write would leave the
// culling frustum, motion vectors and TAA history describing a different
// camera, which is visible as a one-frame ghost/flash.
//
// The FViewMatrices constructor is used instead of scaling the stored
// projection in place. In-place scaling leaves InvProjectionMatrix,
// ProjectionNoAAMatrix and ScreenToClipMatrix describing the game's lens; TAA,
// SSR and the depth reconstruction read those on the same frame. The
// constructor refreshes every derived matrix from the projection and rotation
// the renderer actually uses.
bool ApplyRouteBPoseToView(void *view, const double eye[3],
                           const double rotation[3],
                           const RenderXProbe &probe) noexcept {
  if (view == nullptr || !IsRouteBFullScreenView(view) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  ViewMatricesCtorFn ctor =
      g_view_matrices_ctor.load(std::memory_order_acquire);
  if (ctor == nullptr) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);

  RouteBViewInit init{};
  std::memcpy(init.projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(init.projection));

  // The model lens replaces the game's lens for the same frame. Keep the
  // depth range and TAA jitter terms untouched; only the X/Y scale changes.
  const float desired =
      probe.route_b_model_lens.load(std::memory_order_relaxed);
  float current = 0.0F;
  std::memcpy(&current,
              reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(current));
  bool lens_changed = false;
  if (std::isfinite(current) && current > 1.0F && current < 179.0F &&
      desired > 1.0F && desired < 179.0F &&
      std::fabs(current - desired) >= 0.01F) {
    const double current_radians =
        static_cast<double>(current) * kDegreesToRadians;
    const double desired_radians =
        static_cast<double>(desired) * kDegreesToRadians;
    const double denominator = std::tan(desired_radians * 0.5);
    if (std::fabs(denominator) > 1e-6) {
      const double factor = std::tan(current_radians * 0.5) / denominator;
      if (std::isfinite(factor) && factor > 0.0) {
        init.projection[0][0] *= factor;
        init.projection[1][1] *= factor;
        lens_changed = true;
      }
    }
  }

  MakeViewRotationMatrix(rotation[0], rotation[1], rotation[2],
                         init.view_rotation);
  init.view_origin[0] = eye[0];
  init.view_origin[1] = eye[1];
  init.view_origin[2] = eye[2];
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(address + kViewInfoRectOffset),
              sizeof(rect));
  std::memcpy(init.constrained_rect, rect, sizeof(init.constrained_rect));
  init.stereo_pass = 0;
  init.use_faux_ortho = false;

  try {
    ctor(reinterpret_cast<void *>(address + kViewMatricesOffset), &init);
  } catch (...) {
    return false;
  }

  if (lens_changed) {
    std::memcpy(reinterpret_cast<void *>(address + kViewFovOffset), &desired,
                sizeof(desired));
    std::memcpy(reinterpret_cast<void *>(address + kViewDesiredFovOffset),
                &desired, sizeof(desired));
  }
  std::memcpy(reinterpret_cast<void *>(address + kViewLocationOffset), eye,
              sizeof(eye[0]) * 3);
  std::memcpy(reinterpret_cast<void *>(address + kViewRotationOffset),
              rotation, sizeof(rotation[0]) * 3);

  // Keep ShadowViewMatrices and ViewFrustum in sync; the constructor only
  // rebuilt the view matrices. SetupViewFrustum then refreshes both the culling
  // frustum and its origin for the non-stereo visibility path.
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update != nullptr) {
    update(view);
  }
  SceneViewSetupFrustumFn setup_frustum =
      g_scene_view_setup_frustum.load(std::memory_order_acquire);
  if (setup_frustum != nullptr) {
    setup_frustum(view);
  }
  return true;
}

// Apply route B before FSceneRenderer::PrepareViewStateForVisibility. At this
// point the view matrices are still the game's, but visibility tasks have not
// been launched and the temporal history has not been advanced. Rebuilding the
// matrices here makes the frustum and the TAA history describe the same chase
// camera the renderer will later use.
bool ApplyRouteBEarly(Context &context, void *renderer) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!probe.version_gate.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // A failed attempt this frame must not leave an older frame's view list
  // looking fresh to the diagnostics.
  probe.route_b_early_views_valid.store(false, std::memory_order_release);
  probe.route_b_early_view_count.store(0, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(0, std::memory_order_relaxed);
  for (auto &state : probe.route_b_early_view_states) {
    state.store(0, std::memory_order_relaxed);
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    probe.prepare_thread_mismatch.fetch_add(1, std::memory_order_relaxed);
  }
  if (renderer == nullptr) {
    return false;
  }
  if (!probe.anchor_enabled.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Only route B while the driving camera owns the view. After the player
  // exits the vehicle `PublishInactive` clears this flag, but the presenter
  // pose remains cached; without this gate the early hook kept writing that
  // last vehicle pose, freezing the on-foot camera at the parked car.
  if (!context.view_active.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update == nullptr) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  const auto renderer_address = reinterpret_cast<std::uintptr_t>(renderer);
  void *views = nullptr;
  std::int32_t count = 0;
  std::memcpy(&views,
              reinterpret_cast<const void *>(renderer_address +
                                             kSceneRendererAllViewsOffset),
              sizeof(views));
  std::memcpy(&count,
              reinterpret_cast<const void *>(
                  renderer_address + kSceneRendererAllViewCountOffset),
              sizeof(count));
  if (views == nullptr || count <= 0 || count > kRouteBMaxEarlyViews) {
    probe.prepare_no_views.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  double eye[3]{};
  double rotation[3]{};
  std::int64_t age_ns = 0;
  const std::int64_t now_ns = NowNanoseconds();
  const bool follow_pitch = RouteBFollowPitch(context);
  // The pre-visibility call can run one render frame before the transform
  // notification is published; the 2.7 log showed a 23 ms snapshot age here.
  // Using that snapshot makes the camera ride the previous car transform and
  // the car visibly shakes. The proxy itself is already the object the
  // renderer draws this frame, so read it directly first. The notification
  // snapshot is only a fallback when the proxy pointer is not available.
  // The presenter pose is the artifact-free path: it is the NFS camera the
  // Game thread already produced, and it is aligned horizontally here with
  // the fresh VehicleMesh transform. Its Z carries the model height and the
  // origin-offset slider. The model-pose path remains a fallback.
  bool have_pose =
      BuildRouteBPoseFromPresenter(context, probe, eye, rotation);
  if (have_pose) {
    probe.route_b_early_presenter.fetch_add(1, std::memory_order_relaxed);
  } else if (BuildRouteBPoseDirect(probe, follow_pitch, eye, rotation)) {
    probe.route_b_early_direct.fetch_add(1, std::memory_order_relaxed);
    have_pose = true;
  } else if (BuildRouteBPose(probe, follow_pitch, eye, rotation, age_ns)) {
    have_pose = true;
  }
  if (!have_pose) {
    const std::int64_t held_age = now_ns - probe.route_b_held_ns;
    if (probe.route_b_held_pose_valid && held_age >= 0 &&
        held_age <= kRouteBHeldPoseNs) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        eye[axis] = probe.route_b_held_eye_cm[axis];
        rotation[axis] = probe.route_b_held_rotation[axis];
      }
      probe.route_b_early_held.fetch_add(1, std::memory_order_relaxed);
      have_pose = true;
    }
  }
  if (!have_pose) {
    probe.prepare_pose_fail.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    if (context.presenter_burnout_blend.load(std::memory_order_relaxed) >
        0.001) {
      context.presenter_burnout_fallbacks.fetch_add(1,
                                                    std::memory_order_relaxed);
    }
    return false;
  }
  // Keep the last applied pose for the one-frame hold above. The pose is in
  // world centimetres, like the view fields it is copied into.
  probe.route_b_held_pose_valid = true;
  probe.route_b_held_ns = now_ns;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.route_b_held_eye_cm[axis] = eye[axis];
    probe.route_b_held_rotation[axis] = rotation[axis];
  }

  auto *view_array = static_cast<void *const *>(views);
  std::uint64_t applied = 0;
  for (std::int32_t index = 0; index < count; ++index) {
    void *view = view_array[index];
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    // One authoritative write, before visibility: pose, lens and every derived
    // matrix (including InvProjection) are rebuilt together.
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    if (applied <
        sizeof(probe.route_b_early_views) /
            sizeof(probe.route_b_early_views[0])) {
      probe.route_b_early_views[applied].store(
          reinterpret_cast<std::uintptr_t>(view), std::memory_order_relaxed);
      std::uintptr_t state = 0;
      std::memcpy(&state,
                  reinterpret_cast<const void *>(view_address +
                                                 kViewStateOffset),
                  sizeof(state));
      probe.route_b_early_view_states[applied].store(
          state, std::memory_order_relaxed);
    }
    ++applied;
  }

  probe.prepare_fullscreen_views.fetch_add(applied, std::memory_order_relaxed);
  if (applied == 0) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  probe.route_b_early_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_early_last_age_us.store(
      static_cast<std::uint64_t>(age_ns / 1000), std::memory_order_relaxed);
  const std::int64_t prepared_ns = NowNanoseconds();
  probe.route_b_early_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_early_view_count.store(
      static_cast<std::uint32_t>(applied), std::memory_order_release);
  probe.route_b_prepare_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(
      reinterpret_cast<std::uintptr_t>(renderer), std::memory_order_release);
  probe.route_b_prepare_generation.fetch_add(1, std::memory_order_acq_rel);
  probe.route_b_early_views_valid.store(true, std::memory_order_release);

  // TEMP read-only timing diagnostics: snapshot the car/camera sources at the
  // exact point where the early camera is written. No writes to game state.
  {
    probe.timing_frame.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_count.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    probe.timing_late_seen.store(false, std::memory_order_release);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.timing_early_view[axis].store(eye[axis],
                                          std::memory_order_relaxed);
    }
    double car_cm[3]{};
    double forward[3]{};
    if (ReadRouteBDirectCar(probe, car_cm, forward)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_early_car[axis].store(car_cm[axis],
                                           std::memory_order_relaxed);
      }
    }
    double presenter_car[3]{};
    double presenter_eye[3]{};
    double presenter_rotation[3]{};
    float presenter_lens = 0.0F;
    bool presenter_fov = false;
    if (ReadPresenterPose(context, presenter_eye, presenter_rotation,
                          &presenter_lens, &presenter_fov,
                          presenter_car)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_presenter_car[axis].store(
            presenter_car[axis], std::memory_order_relaxed);
      }
    }
  }
  return true;
}

// Experimental transform-hook writer. It is intentionally guarded by the
// view list captured by ApplyRouteBEarly and by the prepare generation, so it
// writes at most once per prepared frame. This is not a late uniform fallback.
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!context.started.load(std::memory_order_acquire) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return;
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    return;
  }
  if (!probe.route_b_early_views_valid.load(std::memory_order_acquire)) {
    return;
  }
  const std::uintptr_t prepared_renderer =
      probe.route_b_prepare_renderer.load(std::memory_order_relaxed);
  const std::uintptr_t active_renderer =
      probe.route_b_active_renderer.load(std::memory_order_acquire);
  if (prepared_renderer == 0 ||
      (active_renderer != 0 && active_renderer == prepared_renderer)) {
    return;
  }
  const std::uint64_t generation =
      probe.route_b_prepare_generation.load(std::memory_order_acquire);
  const std::uint64_t last_generation =
      probe.route_b_transform_generation.load(std::memory_order_acquire);
  if (generation == 0 || generation == last_generation) {
    return;
  }
  const std::int64_t prepared_ns =
      probe.route_b_prepare_ns.load(std::memory_order_relaxed);
  const std::int64_t now_ns = NowNanoseconds();
  if (prepared_ns == 0 || now_ns < prepared_ns ||
      now_ns - prepared_ns > 100'000'000) {
    return;
  }
  const std::uint32_t count =
      probe.route_b_early_view_count.load(std::memory_order_acquire);
  if (count == 0 ||
      count > sizeof(probe.route_b_early_views) /
                  sizeof(probe.route_b_early_views[0])) {
    return;
  }
  double eye[3]{};
  double rotation[3]{};
  const bool follow_pitch = RouteBFollowPitch(context);
  if (!BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) &&
      !BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation)) {
    return;
  }
  std::uint64_t applied = 0;
  for (std::uint32_t index = 0; index < count; ++index) {
    void *view = reinterpret_cast<void *>(
        probe.route_b_early_views[index].load(std::memory_order_relaxed));
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    std::uintptr_t state = 0;
    std::memcpy(&state,
                reinterpret_cast<const void *>(view_address +
                                               kViewStateOffset),
                sizeof(state));
    if (state == 0 ||
        state != probe.route_b_early_view_states[index].load(
                     std::memory_order_relaxed)) {
      continue;
    }
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    ++applied;
  }
  if (applied == 0) {
    return;
  }
  probe.route_b_render_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_transform_generation.store(generation,
                                            std::memory_order_release);
  probe.route_b_early_last_age_us.store(0, std::memory_order_relaxed);
  probe.route_b_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
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

// ---------------------------------------------------------------------------
// Camera view-point detour
// ---------------------------------------------------------------------------

bool AdvancePresenter(Context &context, std::int64_t now_ns,
                      const double *game_eye_cm, double location[3],
                      double rotation[3], float *lens, bool *fov_enabled,
                      bool *advanced) noexcept;

void RequestPresenterReset(Context &context) noexcept;

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  if (context != nullptr) {
    context->diag.detour_calls.fetch_add(1, std::memory_order_relaxed);
    context->diag.detour_calls_window.fetch_add(1,
                                               std::memory_order_relaxed);
    NoteDetourThread(*context);
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  CameraViewPointFn original =
      g_original.load(std::memory_order_acquire);
  try {
    const std::uint64_t hook_id =
        context == nullptr ? 0
                           : context->view_hook_id.load(std::memory_order_acquire);
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->view_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
      if (!leased) {
        context->diag.lease_failures.fetch_add(1,
                                               std::memory_order_relaxed);
      }
    }
  } catch (...) {
    leased = false;
  }

  bool original_called = false;
  try {
    if (original != nullptr) {
      original(object, location, rotation);
      original_called = true;
    }
  } catch (...) {
  }

  try {
    if (context != nullptr && location != nullptr && rotation != nullptr &&
        object != nullptr) {
      const std::uintptr_t manager = reinterpret_cast<std::uintptr_t>(object);
      context->observed_manager.store(manager, std::memory_order_release);
      // Task group 0 probe: the render thread's own read of the host's player
      // chain. Deliberately before the camera work, and independent of it.
      SampleRenderXPhase(*context);
      const bool pose_finite =
          std::isfinite(location[0]) && std::isfinite(location[1]) &&
          std::isfinite(location[2]) && std::isfinite(rotation[0]) &&
          std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
      if (pose_finite) {
        const std::uint64_t observed_sequence =
            context->observed_sequence.load(std::memory_order_relaxed);
        context->observed_sequence.store(observed_sequence + 1U,
                                         std::memory_order_release);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          context->observed_location[axis].store(
              location[axis], std::memory_order_relaxed);
          context->observed_rotation[axis].store(
              rotation[axis], std::memory_order_relaxed);
        }
        context->observed_valid.store(true, std::memory_order_relaxed);
        context->observed_sequence.store(observed_sequence + 2U,
                                         std::memory_order_release);
      }

      const std::uint64_t sequence =
          context->view_sequence.load(std::memory_order_acquire);
      const bool active = (sequence & 1U) == 0 &&
                          context->view_active.load(std::memory_order_relaxed);
      if (active && !leased) {
        // The game camera shows through for this call. Counted separately from
        // the torn read below: this one is the lease, not the snapshot.
        context->diag.detour_no_publish.fetch_add(1,
                                                 std::memory_order_relaxed);
      }
      if (leased && active) {
        context->view_output_frames.fetch_add(1, std::memory_order_relaxed);
        const std::int64_t render_now_ns = NowNanoseconds();
        double present_location[3]{};
        double present_rotation[3]{};
        float present_lens = 0.0F;
        bool present_fov_enabled = false;
        // The Game's own position is still in `location` here: the original
        // ran before us and we have not written yet.
        const double game_eye_cm[3] = {location[0], location[1],
                                       location[2]};
        bool advanced = false;
        if (AdvancePresenter(*context, render_now_ns, game_eye_cm,
                             present_location, present_rotation,
                             &present_lens, &present_fov_enabled, &advanced)) {
          // T3.4e: how fast the eye moves *relative to the car*, which is the
          // only thing that can look like twitching. The absolute eye speed is
          // dominated by the car's own travel, so it can never separate a
          // smooth camera from a juddering one. Sample only when this call
          // advanced the model: later passes of the same frame reuse the cached
          // pose, and dividing its step by a pass-length dt inflates the peak
          // by the whole frame motion (PLAN-camera-timing.md §4.1).
          if (advanced) {
            static thread_local bool relative_valid = false;
            static thread_local std::int64_t relative_time_ns = 0;
            static thread_local double relative_offset[3]{};
            static thread_local double relative_yaw_degrees = 0.0;
            double relative[3]{};
            const double anchor[3] = {
                context->presenter_anchor[0].load(std::memory_order_relaxed),
                context->presenter_anchor[1].load(std::memory_order_relaxed),
                context->presenter_anchor[2].load(std::memory_order_relaxed)};
            for (std::size_t axis = 0; axis < 3; ++axis) {
              relative[axis] = present_location[axis] - anchor[axis];
            }
            const double yaw_degrees = present_rotation[1];
            if (relative_valid && render_now_ns > relative_time_ns) {
              const double call_seconds =
                  static_cast<double>(render_now_ns - relative_time_ns) * 1e-9;
              if (call_seconds > 2e-4 && call_seconds < 0.2) {
                double step_squared = 0.0;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                  const double step = relative[axis] - relative_offset[axis];
                  step_squared += step * step;
                }
                const double speed_cmps = std::sqrt(step_squared) / call_seconds;
                const double yaw_step = nfs2015_chase::WrapPi(
                    (yaw_degrees - relative_yaw_degrees) *
                    nfs2015_chase::kRadiansPerDegree);
                const double turn_dps =
                    std::fabs(yaw_step * nfs2015_chase::kDegreesPerRadian) /
                    call_seconds;
                RaisePeak(context->diag.render_speed_peak_mmps,
                          static_cast<std::uint64_t>(speed_cmps * 10.0));
                LowerFloor(context->diag.render_speed_floor_mmps,
                           static_cast<std::uint64_t>(speed_cmps * 10.0));
                RaisePeak(context->diag.render_turn_peak_cdps,
                          static_cast<std::uint64_t>(turn_dps * 100.0));
                LowerFloor(context->diag.render_turn_floor_cdps,
                           static_cast<std::uint64_t>(turn_dps * 100.0));
              }
            }
            for (std::size_t axis = 0; axis < 3; ++axis) {
              relative_offset[axis] = relative[axis];
            }
            relative_yaw_degrees = yaw_degrees;
            relative_time_ns = render_now_ns;
            relative_valid = true;
          }
          {
            const bool skip_position =
                context->diagnostic_skip_position.load(
                    std::memory_order_relaxed);
            const bool skip_rotation =
                context->diagnostic_skip_rotation.load(
                    std::memory_order_relaxed);
            // `AdvancePresenter` applies the Game-camera borrow inside its
            // lock while producing the pose; when it returned a cached pose
            // (same frame, or another thread won the lock) the borrow is
            // already reflected there.
            for (std::size_t axis = 0; axis < 3; ++axis) {
              if (!skip_position) {
                location[axis] = static_cast<float>(present_location[axis]);
              }
              if (!skip_rotation) {
                rotation[axis] = static_cast<float>(present_rotation[axis]);
              }
            }
          }

          const std::uintptr_t pov =
              context->camera_pov_manager.load(std::memory_order_acquire) ==
                      manager
                  ? context->camera_pov.load(std::memory_order_acquire)
                  : 0;
          if (pov != 0 && present_fov_enabled) {
            if (std::isfinite(present_lens) && present_lens > 1.0F &&
                present_lens < 179.0F) {
              std::uintptr_t address{};
              if (AddAddress(pov, kCameraPovFovOffset, address)) {
                *reinterpret_cast<float *>(address) = present_lens;
              }
            }
          } else if (context->fov_restore.exchange(
                         false, std::memory_order_acq_rel)) {
            const std::uintptr_t restore_pov =
                context->camera_pov_manager.load(std::memory_order_acquire) ==
                        manager
                    ? context->camera_pov.load(std::memory_order_acquire)
                    : 0;
            const float game_lens =
                context->game_fov.load(std::memory_order_acquire);
            std::uintptr_t address{};
            if (restore_pov != 0 && std::isfinite(game_lens) &&
                game_lens > 1.0F && game_lens < 179.0F &&
                AddAddress(restore_pov, kCameraPovFovOffset, address)) {
              *reinterpret_cast<float *>(address) = game_lens;
            }
          }
        }
      } else if (context->fov_restore.exchange(false,
                                               std::memory_order_acq_rel)) {
        const std::uintptr_t pov =
            context->camera_pov_manager.load(std::memory_order_acquire) ==
                    manager
                ? context->camera_pov.load(std::memory_order_acquire)
                : 0;
        const float game_lens =
            context->game_fov.load(std::memory_order_acquire);
        std::uintptr_t address{};
        if (pov != 0 && std::isfinite(game_lens) && game_lens > 1.0F &&
            game_lens < 179.0F &&
            AddAddress(pov, kCameraPovFovOffset, address)) {
          *reinterpret_cast<float *>(address) = game_lens;
        }
      }
    }
  } catch (...) {
  }

  // Same rule as the transform detour: release a lease that was actually
  // acquired, even while the host is already stopping the hook service.
  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

// ---------------------------------------------------------------------------
// Game tick
// ---------------------------------------------------------------------------

bool QueryPlayer(Context &context, double origin_cm[3], double base_cm[3],
                 std::uint64_t &sequence,
                 AnomalyGenerationHandleV1 &handle) noexcept {
  sequence = 0;
  handle = {};
  context.camera_yaw_valid = false;
  context.camera_yaw_radians = 0.0;
  context.probe_bounds_dx_cm = 0.0;
  context.probe_bounds_dy_cm = 0.0;
  if (!PlayerReady(context.player)) {
    return false;
  }
  AnomalyNtePlayerSnapshotV1 player{sizeof(player)};
  if (context.player->snapshot(context.player->user, &player).code !=
          ANOMALY_STATUS_V1_OK ||
      !SnapshotValid(player.flags)) {
    return false;
  }
  sequence = player.sequence;
  handle = player.handle;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    base_cm[axis] = player.position[axis];
  }
  double extent_z = kFallbackPlayerCenterHeightCm;
  AnomalyNtePlayerEspSnapshotV1 esp{sizeof(esp)};
  const bool has_esp =
      HasField<AnomalyNtePlayerServiceV1,
               decltype(AnomalyNtePlayerServiceV1::esp_snapshot)>(
          context.player,
          offsetof(AnomalyNtePlayerServiceV1, esp_snapshot)) &&
      context.player->esp_snapshot != nullptr;
  if (has_esp &&
      context.player->esp_snapshot(context.player->user, &esp).code ==
          ANOMALY_STATUS_V1_OK) {
    // PARTIAL means the optional ESP tail is missing. Its bounds fields are
    // not a trustworthy origin, even when they happen to be finite. Also
    // reject a stale ESP sample that does not describe the current pawn.
    const bool complete =
        SnapshotValid(esp.flags) &&
        (esp.flags & ANOMALY_NTE_SNAPSHOT_V1_PARTIAL) == 0;
    const bool finite =
        std::isfinite(esp.bounds_center[0]) &&
        std::isfinite(esp.bounds_center[1]) &&
        std::isfinite(esp.bounds_center[2]) &&
        std::isfinite(esp.bounds_extent[2]);
    const double dx = esp.bounds_center[0] - player.position[0];
    const double dy = esp.bounds_center[1] - player.position[1];
    const double dz = esp.bounds_center[2] - player.position[2];
    // Probe input: the horizontal lever arm of the bounds centre in the car's
    // own frame. Kept even when the bounds are not plausible enough to use as
    // an origin, because the probe is asking whether the direction of this
    // vector is stable, not whether its length is right.
    if (complete && finite) {
      context.probe_bounds_dx_cm = dx;
      context.probe_bounds_dy_cm = dy;
    }
    const bool plausible =
        complete && finite && dx * dx + dy * dy < 500.0 * 500.0 &&
        std::fabs(dz) < 500.0 && esp.bounds_extent[2] > 0.5F &&
        esp.bounds_extent[2] < 1000.0F;
    if (plausible) {
      extent_z = esp.bounds_extent[2];
      // The probe measures how much this radius moves tick to tick. It is the
      // only mesh-bound quantity that still reaches the eye.
      context.probe_extent_z_cm = static_cast<double>(extent_z);
    }
    // T3.1: the game camera's yaw, sampled by the host on the same Game tick
    // as the car position and delivered as a cached copy. The detour's
    // observation is the same rotation read on the render thread at ~1400 Hz,
    // which the 50 Hz Game tick then samples, so it aliases. Independent of
    // the bounds sanity check above: the camera tail is usable even when the
    // bounds jump.
    if (complete && std::isfinite(esp.camera_rotation[1])) {
      context.camera_yaw_radians =
          esp.camera_rotation[1] * kDegreesToRadians;
      context.camera_yaw_valid = true;
    }
  }
  const double origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
  // The camera model is anchored to the rigid Pawn root. The mesh bounds
  // centre and extent move with body roll and pitch; anchoring the camera to
  // them made the eye inherit that motion 1:1, which reads as twitch. Keep the
  // bounds fields only for the probe, never for the model origin.
  origin_cm[0] = base_cm[0];
  origin_cm[1] = base_cm[1];
  origin_cm[2] = base_cm[2] + origin_offset_cm;
  // The frame callback projects this. The Pawn root, so the measurement is of
  // what the game actually moved, with no mesh bound in it.
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.probe_car_cm[axis].store(player.position[axis],
                                     std::memory_order_relaxed);
  }
  context.probe_car_valid.store(std::isfinite(player.position[0]) &&
                                    std::isfinite(player.position[1]) &&
                                    std::isfinite(player.position[2]),
                                std::memory_order_relaxed);
  return std::isfinite(origin_cm[0]) && std::isfinite(origin_cm[1]) &&
         std::isfinite(origin_cm[2]) && std::isfinite(base_cm[0]) &&
         std::isfinite(base_cm[1]) && std::isfinite(base_cm[2]);
}

double WrappedAngle(const double angle) noexcept {
  return nfs2015_chase::WrapPi(angle);
}

// A world change, teleport, or model reset must not carry the old slope's
// pitch into the new frame. The floor also rejects a still-fresh snapshot that
// was published before the reset.
void ResetRenderPitch(Context &context) noexcept {
  context.render_pitch_valid = false;
  context.render_pitch_radians = 0.0;
  context.render_pitch_ns = 0;
  context.render_pitch_floor_ns = NowNanoseconds();
  context.render_pitch_proxy = 0;
}

bool WorldChanged(Context &context) noexcept {
  if (context.session == nullptr || context.session->snapshot == nullptr) {
    return false;
  }
  AnomalyNteSessionSnapshotV1 snapshot{sizeof(snapshot)};
  if (context.session->snapshot(context.session->user, &snapshot).code !=
          ANOMALY_STATUS_V1_OK ||
      snapshot.state != ANOMALY_NTE_SESSION_V1_WORLD_READY ||
      snapshot.world.id == 0 || snapshot.world.generation == 0) {
    return false;
  }
  if (context.world.id == 0) {
    context.world = snapshot.world;
    return true;
  }
  if (context.world.id != snapshot.world.id ||
      context.world.generation != snapshot.world.generation) {
    context.world = snapshot.world;
    return true;
  }
  return false;
}

void ResetMotion(Context &context) noexcept {
  NoteViewSnap(context, kSnapReasonMotion);
  context.yaw_source = kYawSourceNone;
  context.have_last_position = false;
  context.have_last_yaw = false;
  context.reversing = false;
  context.speed_sign_known = true;
  context.last_vehicle_parent = 0;
  context.last_vehicle_speed_mps = -1.0;
  context.last_vehicle_speed_time = 0.0;
  context.last_speed_exact = false;
  context.brake_deceleration_mps2 = 0.0;
  context.brake_input = 0.0;
  context.brake_hold_until = 0.0;
  context.vehicle_acceleration_mps2 = 0.0;
  context.last_player_sequence = 0;
  context.last_player_time = context.elapsed;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.velocity_cmps[axis] = 0.0;
    context.acceleration_cmps2[axis] = 0.0;
  }
  context.yaw_rate = 0.0;
  RequestPresenterReset(context);
}

void UpdateBrakeInput(Context &context, const bool speed_valid,
                      const bool speed_fresh, const bool derived,
                      const double speed_kmh, const double dt) noexcept {
  const double input_smoothing =
      derived ? kDerivedBrakingInputSmoothing : kBrakingInputSmoothing;
  const double deceleration_smoothing =
      derived ? kDerivedBrakingDecelerationSmoothing
              : kBrakingDecelerationSmoothing;
  if (!speed_valid) {
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
        context.vehicle_acceleration_mps2, 0.0, deceleration_smoothing, dt);
    return;
  }
  if (!speed_fresh) {
    // The motion sample is held between host snapshots, so there is no new
    // speed delta to differentiate. Differentiating it at tick rate would read
    // as zero acceleration on the held ticks and a spike on the sampled one,
    // i.e. a sawtooth in the braking pitch.
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input,
        context.elapsed <= context.brake_hold_until
            ? context.brake_last_target
            : 0.0,
        input_smoothing, dt);
    return;
  }
  // Braking pitch follows speed magnitude: reversing decelerates too, and the
  // camera model already receives the signed speed separately.
  const double speed_mps = std::fabs(speed_kmh) / 3.6;
  if (context.last_vehicle_speed_mps < 0.0) {
    context.last_vehicle_speed_mps = speed_mps;
    context.last_vehicle_speed_time = context.elapsed;
    return;
  }
  const double elapsed = context.elapsed - context.last_vehicle_speed_time;
  const double previous_mps = context.last_vehicle_speed_mps;
  context.last_vehicle_speed_mps = speed_mps;
  context.last_vehicle_speed_time = context.elapsed;
  if (elapsed <= 1e-4) {
    return;
  }
  if (derived && elapsed > kBrakingMaxSampleIntervalSeconds) {
    // Only the differentiated position is invalid across a hitch; the exact
    // speed getter is a direct sample and can span a longer poll gap.
    context.brake_deceleration_mps2 = 0.0;
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    return;
  }
  const double raw_acceleration = (speed_mps - previous_mps) / elapsed;
  context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
      context.vehicle_acceleration_mps2,
      nfs2015_chase::Clamp(raw_acceleration, -30.0, 30.0),
      deceleration_smoothing, elapsed);
  const double deceleration =
      nfs2015_chase::Clamp(-raw_acceleration, 0.0, 30.0);
  context.brake_deceleration_mps2 = nfs2015_chase::Damp(
      context.brake_deceleration_mps2, deceleration,
      deceleration_smoothing, elapsed);
  const double deadzone =
      derived ? kDerivedBrakingPitchDeadzoneMps2
              : kBrakingPitchDeadzoneMps2;
  const double excess =
      (std::max)(0.0, context.brake_deceleration_mps2 - deadzone);
  const double target = nfs2015_chase::Clamp(
      excess / (kBrakingPitchFullMps2 - deadzone), 0.0, 1.0);
  if (target > 0.05) {
    context.brake_hold_until = context.elapsed + kBrakingHoldSeconds;
  }
  const double gated =
      context.elapsed <= context.brake_hold_until ? target : 0.0;
  context.brake_last_target = gated;
  context.brake_input = nfs2015_chase::Damp(
      context.brake_input, gated, input_smoothing, dt);
}

void UpdateMotion(Context &context, const double origin_cm[3],
                  const std::uint64_t sequence) noexcept {
  if (!context.have_last_position || sequence != context.last_player_sequence) {
    // Q5: does the player sequence advance once per tick? A jump means the
    // position derivative below is divided by the wrong elapsed time.
    if (context.last_player_sequence != 0 &&
        sequence != context.last_player_sequence &&
        sequence != context.last_player_sequence + 1U) {
      context.diag.player_sequence_jumps.fetch_add(1,
                                                  std::memory_order_relaxed);
    }
    if (context.have_last_position && sequence != context.last_player_sequence) {
      const double elapsed = context.elapsed - context.last_player_time;
      if (elapsed > 1e-4) {
        std::array<double, 3> raw{};
        double distance_squared = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          raw[axis] = (origin_cm[axis] - context.last_position_cm[axis]) /
                      elapsed;
          distance_squared +=
              (origin_cm[axis] - context.last_position_cm[axis]) *
              (origin_cm[axis] - context.last_position_cm[axis]);
        }
        const double raw_speed = std::hypot(raw[0], raw[1]);
        if (distance_squared < kMaximumTeleportStepCm * kMaximumTeleportStepCm &&
            std::isfinite(raw_speed) &&
            raw_speed < kMaximumPlausibleSpeedCmps) {
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double previous = context.velocity_cmps[axis];
            context.velocity_cmps[axis] = nfs2015_chase::Damp(
                context.velocity_cmps[axis], raw[axis], kVelocitySmoothing,
                elapsed);
            context.acceleration_cmps2[axis] = nfs2015_chase::Damp(
                context.acceleration_cmps2[axis],
                (context.velocity_cmps[axis] - previous) / elapsed,
                kAccelerationSmoothing, elapsed);
          }
        } else {
          NoteViewSnap(context, kSnapReasonTeleport);
          ResetRenderPitch(context);
          // A teleport is not a brake. Clear the derived-speed history so the
          // next sample cannot turn the jump to zero into a full pitch input.
          context.last_vehicle_speed_mps = -1.0;
          context.last_vehicle_speed_time = context.elapsed;
          context.brake_deceleration_mps2 = 0.0;
          context.brake_input = 0.0;
          context.brake_hold_until = 0.0;
          context.brake_last_target = 0.0;
          context.vehicle_acceleration_mps2 = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            context.velocity_cmps[axis] = 0.0;
            context.acceleration_cmps2[axis] = 0.0;
          }
        }
      }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      context.last_position_cm[axis] = origin_cm[axis];
    }
    context.last_player_sequence = sequence;
    context.last_player_time = context.elapsed;
    context.have_last_position = true;
  }

  const double acceleration_length =
      std::sqrt(context.acceleration_cmps2[0] *
                    context.acceleration_cmps2[0] +
                context.acceleration_cmps2[1] *
                    context.acceleration_cmps2[1] +
                context.acceleration_cmps2[2] *
                    context.acceleration_cmps2[2]);
  if (acceleration_length > kMaximumPlausibleAccelerationCmps2) {
    const double scale = kMaximumPlausibleAccelerationCmps2 / acceleration_length;
    for (double &value : context.acceleration_cmps2) {
      value *= scale;
    }
  }
}

struct ObservedPose final {
  std::array<double, 3> location{};
  std::array<double, 3> rotation{};
};

// Try-read the detour's copy of the game camera pose. The detour writes it on
// another thread; a torn or in-progress write is reported as unavailable
// instead of spinning the Game thread.
[[nodiscard]] bool ReadObservedPose(Context &context,
                                    ObservedPose &pose) noexcept {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t before =
        context.observed_sequence.load(std::memory_order_acquire);
    if ((before & 1U) != 0) {
      continue;
    }
    if (!context.observed_valid.load(std::memory_order_relaxed)) {
      return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      pose.location[axis] =
          context.observed_location[axis].load(std::memory_order_relaxed);
      pose.rotation[axis] =
          context.observed_rotation[axis].load(std::memory_order_relaxed);
    }
    const std::uint64_t after =
        context.observed_sequence.load(std::memory_order_acquire);
    if (before == after) {
      return true;
    }
  }
  context.diag.observed_read_failures.fetch_add(1,
                                                std::memory_order_relaxed);
  return false;
}

double SelectCarYaw(Context &context, const double velocity_yaw,
                    const double velocity_mps, const bool have_velocity,
                    const bool exact_speed,
                    const double signed_speed_kmh) noexcept {
  // Prefer the Game-tick camera yaw: same clock as the car position, so the
  // reference cannot alias against a faster writer. The detour's render-thread
  // observation is only a fallback for profiles without the ESP tail.
  double camera_yaw = 0.0;
  bool have_camera_yaw = false;
  if (context.camera_yaw_valid) {
    camera_yaw = context.camera_yaw_radians;
    have_camera_yaw = true;
  } else {
    ObservedPose observed{};
    if (ReadObservedPose(context, observed)) {
      const double raw = observed.rotation[1];
      if (std::isfinite(raw)) {
        camera_yaw = raw * kDegreesToRadians;
        have_camera_yaw = true;
      }
    }
    context.diag.camera_yaw_esp_misses.fetch_add(1,
                                                std::memory_order_relaxed);
  }
  context.speed_sign_known = exact_speed || have_camera_yaw;
  if (exact_speed) {
    // GetForwardSpeedKmH is signed, so reverse is stateless again exactly as
    // it was in 2.6. The derived-speed hysteresis is only the fallback.
    context.reversing = signed_speed_kmh < -kReverseSpeedKmh;
  }
  if (!have_camera_yaw) {
    NoteYawSource(context, kYawSourceNoObservation);
    if (!have_velocity) {
      return context.have_last_yaw ? context.last_car_yaw : 0.0;
    }
    // Without a camera reference the sign is unknown. Reuse the last known
    // reverse state rather than flipping the camera to the nose in one tick.
    return context.reversing
               ? WrappedAngle(velocity_yaw + nfs2015_chase::kPi)
               : velocity_yaw;
  }
  if (!have_velocity) {
    // Too slow for the travel direction to mean anything: the camera is the
    // only continuous reference left.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  // Reversing travels along the nose plus 180 degrees, so the camera stays
  // aligned with the nose instead of ending up in front of the car. When the
  // exact signed speed is unavailable, infer the sign from the angle between
  // travel and the game-camera yaw. The two thresholds form a hysteresis band
  // so a drift or a camera swing cannot chatter the state; below walking speed
  // the last decision is held.
  const double alignment =
      std::cos(WrappedAngle(velocity_yaw - camera_yaw));
  if (!exact_speed && velocity_mps >= kReverseDecisionMinSpeedMps) {
    if (context.reversing) {
      if (alignment > kReverseExitCosine) {
        context.reversing = false;
      }
    } else if (alignment < kReverseEnterCosine) {
      context.reversing = true;
    }
  }
  const bool reversing = context.reversing;
  if (!reversing && velocity_mps < kReverseDecisionMinSpeedMps &&
      alignment < kReverseEnterCosine) {
    // Below the reverse decision speed the travel direction is too noisy to
    // commit to a gear. The camera still points along the nose, so keep it
    // instead of flipping the rig to the noisy velocity vector.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  const double travel_yaw =
      reversing ? WrappedAngle(velocity_yaw + nfs2015_chase::kPi)
                : velocity_yaw;
  if (std::fabs(WrappedAngle(camera_yaw - travel_yaw)) >
      kMaximumCameraYawDifference) {
    NoteYawSource(context, kYawSourceVelocity);
    return travel_yaw;
  }
  NoteYawSource(context, kYawSourceObserved);
  return camera_yaw;
}

void PublishInactive(Context &context) noexcept {
  const std::uint64_t sequence =
      context.view_sequence.load(std::memory_order_relaxed);
  context.view_sequence.store(sequence + 1U, std::memory_order_release);
  context.view_active.store(false, std::memory_order_release);
  context.view_sequence.store(sequence + 2U, std::memory_order_release);
  // Re-entering the camera must not blend from the view that was active
  // before the player stopped driving.
  NoteViewSnap(context, kSnapReasonInactive);
  if (context.last_view_active && context.last_fov_enabled) {
    context.fov_restore.store(true, std::memory_order_release);
  }
  context.last_view_active = false;
  context.last_fov_enabled = false;
}

[[nodiscard]] bool ViewHistoryJumped(
    const std::array<double, 3> &old_location,
    const std::array<double, 3> &old_rotation,
    const std::array<double, 3> &new_location,
    const std::array<double, 3> &new_rotation) noexcept {
  const double dx = new_location[0] - old_location[0];
  const double dy = new_location[1] - old_location[1];
  const double dz = new_location[2] - old_location[2];
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      dx * dx + dy * dy + dz * dz >
          kViewHistorySnapDistanceCm * kViewHistorySnapDistanceCm) {
    return true;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(old_rotation[axis]) ||
        !std::isfinite(new_rotation[axis])) {
      return true;
    }
    const double delta = WrappedAngle(
        (new_rotation[axis] - old_rotation[axis]) * kDegreesToRadians);
    if (std::fabs(delta) >
        kViewHistorySnapAngleDegrees * kDegreesToRadians) {
      return true;
    }
  }
  return false;
}

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
    // The detour gates on this: without it the game camera shows through.
    context.last_view_active = true;
    context.last_fov_enabled = fov_enabled;
    context.view_active.store(true, std::memory_order_release);
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

void UpdateDisplay(Context &context) noexcept {
  std::scoped_lock lock(context.display_mutex);
  context.display.started = context.started.load(std::memory_order_acquire);
  context.display.enabled = context.enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_enabled =
      context.wheel_steer_enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_active = context.wheel_steer.active;
  context.display.wheel_steer_engaged = context.wheel_steer.engaged;
  context.display.wheel_steer_wheels =
      static_cast<std::uint32_t>(context.wheel_steer.wheel_count);
  context.display.wheel_steer_angle_degrees =
      context.wheel_steer.applied_angle * kRadiansToDegrees;
  context.display.wheel_steer_target_degrees =
      context.wheel_steer.target_angle * kRadiansToDegrees;
  context.display.wheel_steer_entry_speed_kmh =
      context.wheel_steer_entry_speed_kmh.load(std::memory_order_acquire);
  context.display.wheel_steer_max_degrees =
      context.wheel_steer_max_degrees.load(std::memory_order_acquire);
  context.display.wheel_steer_gain =
      context.wheel_steer_gain.load(std::memory_order_acquire);
  context.display.wheel_steer_full_slip_degrees =
      context.wheel_steer_full_slip_degrees.load(
          std::memory_order_acquire);
  context.display.wheel_steer_rate_degrees_per_second =
      context.wheel_steer_rate_degrees_per_second.load(
          std::memory_order_acquire);
  context.display.speed_distance_scale =
      context.speed_distance_scale.load(std::memory_order_acquire);
  context.display.hook_ready =
      context.hook_ready.load(std::memory_order_acquire);
  context.display.active =
      context.view_active.load(std::memory_order_acquire) &&
      context.hook_ready.load(std::memory_order_acquire);
  context.display.update_hz = context.update_hz;
  context.display.view_output_hz = context.view_output_hz;
  context.display.interpolation_ratio = context.interpolation_ratio;
  context.display.detour_hz = context.detour_hz;
  context.display.tick_ms_last = context.camera_ms_last;
  context.display.tick_ms_avg = context.camera_cost.Average();
  context.display.tick_ms_p99 = context.camera_cost.Percentile(0.99);
  context.display.tick_ms_max = context.camera_cost.Maximum();
  const Diagnostics &diag = context.diag;
  std::uint64_t snap_total = 0;
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    const std::uint64_t count =
        diag.snap_by_reason[index].load(std::memory_order_relaxed);
    context.display.snap_by_reason[index] = count;
    snap_total += count;
  }
  context.display.snap_total = snap_total;
  context.display.yaw_source_switches =
      diag.yaw_source_switches.load(std::memory_order_relaxed);
  context.display.observed_read_failures =
      diag.observed_read_failures.load(std::memory_order_relaxed);
  context.display.lease_failures =
      diag.lease_failures.load(std::memory_order_relaxed);
  context.display.view_read_torn =
      diag.view_read_torn.load(std::memory_order_relaxed);
  context.display.detour_calls =
      diag.detour_calls.load(std::memory_order_relaxed);
  context.display.detour_no_publish =
      diag.detour_no_publish.load(std::memory_order_relaxed);
  context.display.detour_thread_count =
      diag.detour_thread_count.load(std::memory_order_relaxed);
  context.display.player_sequence_jumps =
      diag.player_sequence_jumps.load(std::memory_order_relaxed);
  context.display.camera_yaw_esp_misses =
      diag.camera_yaw_esp_misses.load(std::memory_order_relaxed);
  context.display.slip_from_position_degrees =
      diag.slip_from_position_degrees.load(std::memory_order_relaxed);
  context.display.slip_sign_changes =
      diag.slip_sign_changes.load(std::memory_order_relaxed);
  context.display.slip_max_step_degrees =
      static_cast<double>(
          diag.slip_max_step_ddeg.load(std::memory_order_relaxed)) /
      10.0;
  const std::uint64_t speed_floor_raw =
      diag.render_speed_floor_mmps.load(std::memory_order_relaxed);
  // The floor starts at UINT64_MAX so the first sample sets it; an untouched
  // window must not print that sentinel.
  context.display.render_speed_floor_mps =
      speed_floor_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(speed_floor_raw) / 1000.0;
  context.display.render_speed_valid = speed_floor_raw != UINT64_MAX;
  context.display.render_speed_peak_mps =
      static_cast<double>(
          diag.render_speed_peak_mmps.load(std::memory_order_relaxed)) /
      1000.0;
  const std::uint64_t turn_floor_raw =
      diag.render_turn_floor_cdps.load(std::memory_order_relaxed);
  context.display.render_turn_floor_dps =
      turn_floor_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(turn_floor_raw) / 100.0;
  context.display.render_turn_valid = turn_floor_raw != UINT64_MAX;
  context.display.render_turn_peak_dps =
      static_cast<double>(
          diag.render_turn_peak_cdps.load(std::memory_order_relaxed)) /
      100.0;
  context.display.car_step_cm =
      static_cast<double>(diag.car_step_dcm.load(std::memory_order_relaxed)) /
      10.0;
  const Probe &probe = context.probe;
  context.display.probe_ahud_service =
      context.ahud != nullptr && ProbeAhudReady(context.ahud);
  context.display.probe_subscribe_state = probe.subscribe_state;
  context.display.probe_subscribe_code = probe.subscribe_code;
  context.display.probe_frame_hz = probe.frame_hz;
  context.display.probe_frame_calls = probe.frame_calls;
  context.display.probe_camera_yaw_step_max_degrees =
      probe.camera_yaw_step_max_ddeg;
  context.display.probe_camera_yaw_step_avg_degrees =
      probe.camera_yaw_step_count == 0
          ? 0.0
          : probe.camera_yaw_step_sum_ddeg /
                static_cast<double>(probe.camera_yaw_step_count);
  context.display.probe_camera_yaw_valid = probe.camera_yaw_valid;
  context.display.probe_camera_yaw_degrees = probe.camera_yaw_degrees;
  context.display.probe_bounds_offset_cm = probe.bounds_offset_cm;
  context.display.probe_bounds_yaw_valid = probe.bounds_yaw_valid;

  context.display.probe_camera_minus_bounds_degrees =
      probe.camera_minus_bounds_valid ? probe.camera_minus_bounds_degrees
                                      : 0.0;
  context.display.resets_in_window =
      diag.reset_window.load(std::memory_order_relaxed);
  {
    const std::uint64_t flow_count =
        diag.flow_count.load(std::memory_order_relaxed);
    const std::uint64_t flow_sum =
        diag.flow_sum_ccm.load(std::memory_order_relaxed);
    context.display.flow_valid = flow_count != 0;
    context.display.flow_peak_cm =
        static_cast<double>(
            diag.flow_peak_ccm.load(std::memory_order_relaxed)) /
        100.0;
    context.display.flow_avg_cm =
        flow_count == 0 ? 0.0
                        : static_cast<double>(flow_sum) /
                              static_cast<double>(flow_count) / 100.0;
    const std::uint64_t frame_count =
        diag.frame_count.load(std::memory_order_relaxed);
    const std::uint64_t frame_sum =
        diag.frame_sum_us.load(std::memory_order_relaxed);
    context.display.frame_valid = frame_count != 0;
    context.display.frame_peak_ms =
        static_cast<double>(
            diag.frame_peak_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.frame_mean_ms =
        frame_count == 0 ? 0.0
                         : static_cast<double>(frame_sum) /
                               static_cast<double>(frame_count) / 1000.0;
  }
  const std::uint64_t judder_count =
      diag.judder_count.load(std::memory_order_relaxed);
  const std::uint64_t judder_sum =
      diag.judder_sum_dpx.load(std::memory_order_relaxed);
  context.display.judder_valid = judder_count != 0;
  context.display.judder_peak_px =
      static_cast<double>(
          diag.judder_peak_dpx.load(std::memory_order_relaxed)) /
      10.0;
  context.display.judder_avg_px =
      judder_count == 0
          ? 0.0
          : static_cast<double>(judder_sum) /
                static_cast<double>(judder_count) / 10.0;
  {
    const std::uint64_t pitch_count =
        diag.pitch_count.load(std::memory_order_relaxed);
    const std::uint64_t pitch_sum =
        diag.pitch_sum_cdeg.load(std::memory_order_relaxed);
    const std::uint64_t roll_count =
        diag.roll_count.load(std::memory_order_relaxed);
    const std::uint64_t roll_sum =
        diag.roll_sum_cdeg.load(std::memory_order_relaxed);
    const std::uint64_t lens_count =
        diag.lens_count.load(std::memory_order_relaxed);
    const std::uint64_t lens_sum =
        diag.lens_sum_cdeg.load(std::memory_order_relaxed);
    context.display.rotation_valid = pitch_count != 0 && roll_count != 0;
    context.display.pitch_peak_degrees =
        static_cast<double>(
            diag.pitch_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.pitch_avg_degrees =
        pitch_count == 0 ? 0.0
                         : static_cast<double>(pitch_sum) /
                               static_cast<double>(pitch_count) / 100.0;
    context.display.roll_peak_degrees =
        static_cast<double>(
            diag.roll_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.roll_avg_degrees =
        roll_count == 0 ? 0.0
                        : static_cast<double>(roll_sum) /
                              static_cast<double>(roll_count) / 100.0;
    context.display.lens_valid = lens_count != 0;
    context.display.lens_peak_degrees =
        static_cast<double>(
            diag.lens_peak_cdeg.load(std::memory_order_relaxed)) /
        100.0;
    context.display.lens_avg_degrees =
        lens_count == 0 ? 0.0
                        : static_cast<double>(lens_sum) /
                              static_cast<double>(lens_count) / 100.0;
  }
  const std::uint64_t judder_car_count =
      diag.judder_car_count.load(std::memory_order_relaxed);
  const std::uint64_t judder_car_sum =
      diag.judder_car_sum_dpx.load(std::memory_order_relaxed);
  context.display.judder_car_valid = judder_car_count != 0;
  context.display.judder_car_peak_px =
      static_cast<double>(
          diag.judder_car_peak_dpx.load(std::memory_order_relaxed)) /
      10.0;
  context.display.judder_car_avg_px =
      judder_car_count == 0
          ? 0.0
          : static_cast<double>(judder_car_sum) /
                static_cast<double>(judder_car_count) / 10.0;
  context.display.probe_height_step_cm = probe.height_step_cm;
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint64_t samples =
        render_x.sample_count.load(std::memory_order_relaxed);
    const std::uint64_t sum_dcm =
        render_x.sum_dcm.load(std::memory_order_relaxed);
    const std::uint64_t sum_dms =
        render_x.sum_dms.load(std::memory_order_relaxed);
    const std::int64_t lead_count =
        render_x.lead_count.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dcm =
        render_x.lead_sum_dcm.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dms =
        render_x.lead_sum_dms.load(std::memory_order_relaxed);
    const std::uint64_t speed_sum_cmps =
        render_x.speed_sum_cmps.load(std::memory_order_relaxed);
    context.display.render_x_state = render_x.resolve_state;
    context.display.render_x_code = render_x.resolve_code;
    context.display.render_x_failures =
        render_x.read_failures.load(std::memory_order_relaxed);
    context.display.render_x_samples = samples;
    context.display.render_x_valid = samples != 0;
    context.display.render_x_peak_cm =
        static_cast<double>(
            render_x.peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_cm =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dcm) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_peak_ms =
        static_cast<double>(
            render_x.peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_ms =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dms) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_speed_kmh =
        samples == 0 ? 0.0
                     : static_cast<double>(speed_sum_cmps) /
                           static_cast<double>(samples) * 0.036;
    context.display.render_x_lead_cm =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dcm) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_lead_ms =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dms) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_chain_cm = render_x.chain_max_cm;
    context.display.render_x_chain_mismatches = render_x.chain_mismatches;
    context.display.render_x_gt_samples =
        render_x.samples_on_game_thread.load(std::memory_order_relaxed);
    const std::uint64_t proxy_count =
        render_x.render_proxy_count.load(std::memory_order_relaxed);
    const std::uint64_t proxy_time_count =
        render_x.render_proxy_time_count.load(std::memory_order_relaxed);
    const std::int64_t proxy_lead_count =
        render_x.render_proxy_lead_count.load(std::memory_order_relaxed);
    const std::uint64_t age_count =
        render_x.render_age_count.load(std::memory_order_relaxed);
    context.display.render_proxy_valid = proxy_count != 0;
    context.display.render_proxy_peak_cm =
        static_cast<double>(
            render_x.render_proxy_peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_cm =
        proxy_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dcm.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_count) / 10.0;
    context.display.render_proxy_peak_ms =
        static_cast<double>(
            render_x.render_proxy_peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_ms =
        proxy_time_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dms.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_time_count) / 10.0;
    context.display.render_proxy_lead_cm =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dcm.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_proxy_lead_ms =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dms.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_age_valid = age_count != 0;
    context.display.render_age_peak_ms =
        static_cast<double>(
            render_x.render_age_peak_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.render_age_avg_ms =
        age_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_age_sum_us.load(std::memory_order_relaxed)) /
                  static_cast<double>(age_count) / 1000.0;
  }
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint32_t source =
        context.anchor_source.load(std::memory_order_acquire);
    const std::uint8_t insurance = render_x.insurance_state;
    const std::uint8_t last_used =
        render_x.last_used.load(std::memory_order_relaxed);
    context.display.anchor_source = source;
    context.display.anchor_fallbacks =
        render_x.anchor_fallbacks.load(std::memory_order_relaxed);
    context.display.anchor_read_failures =
        render_x.anchor_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_anchor_valid =
        render_x.vehicle_anchor_valid.load(std::memory_order_relaxed);
    context.display.vehicle_present =
        context.vehicle_query_known
            ? context.vehicle_query_present
            : render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_presence_known =
        context.vehicle_query_known ||
        render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_proxy_used =
        render_x.vehicle_proxy_used.load(std::memory_order_relaxed);
    context.display.vehicle_render_used =
        render_x.vehicle_render_used.load(std::memory_order_relaxed);
    context.display.transform_hook_valid =
        context.transform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.transform_updates =
        render_x.transform_updates.load(std::memory_order_relaxed);
    context.display.view_uniform_hook_valid =
        context.view_uniform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.view_uniform_calls =
        render_x.view_uniform_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_render_calls =
        render_x.view_uniform_render_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_game_calls =
        render_x.view_uniform_game_calls.load(std::memory_order_relaxed);
    context.display.render_thread_id =
        render_x.render_thread_id.load(std::memory_order_relaxed);
    const std::uint64_t view_age_count =
        render_x.view_uniform_age_count.load(std::memory_order_relaxed);
    context.display.view_uniform_age_valid = view_age_count != 0;
    context.display.view_uniform_age_avg_ms =
        view_age_count == 0
            ? 0.0
            : static_cast<double>(render_x.view_uniform_age_sum_us.load(
                  std::memory_order_relaxed)) /
                  static_cast<double>(view_age_count) / 1000.0;
    context.display.view_uniform_age_peak_ms =
        static_cast<double>(render_x.view_uniform_age_peak_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_applied =
        render_x.route_b_applied.load(std::memory_order_relaxed);
    context.display.route_b_fallbacks =
        render_x.route_b_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_witness_failures =
        render_x.route_b_witness_failures.load(std::memory_order_relaxed);
    context.display.route_b_early_applied =
        render_x.route_b_early_applied.load(std::memory_order_relaxed);
    context.display.route_b_early_fallbacks =
        render_x.route_b_early_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_last_age_ms =
        static_cast<double>(
            render_x.route_b_last_age_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_early_last_age_ms =
        static_cast<double>(render_x.route_b_early_last_age_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.prepare_calls =
        render_x.prepare_calls.load(std::memory_order_relaxed);
    context.display.prepare_thread_mismatch =
        render_x.prepare_thread_mismatch.load(std::memory_order_relaxed);
    context.display.prepare_anchor_off =
        render_x.prepare_anchor_off.load(std::memory_order_relaxed);
    context.display.prepare_no_views =
        render_x.prepare_no_views.load(std::memory_order_relaxed);
    context.display.prepare_pose_fail =
        render_x.prepare_pose_fail.load(std::memory_order_relaxed);
    context.display.prepare_fullscreen_views =
        render_x.prepare_fullscreen_views.load(std::memory_order_relaxed);
    context.display.route_b_early_direct =
        render_x.route_b_early_direct.load(std::memory_order_relaxed);
    context.display.route_b_early_held =
        render_x.route_b_early_held.load(std::memory_order_relaxed);
    context.display.vehicle_fallbacks =
        render_x.vehicle_fallbacks.load(std::memory_order_relaxed);
    context.display.vehicle_read_failures =
        render_x.vehicle_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_proxy_fallbacks =
        render_x.vehicle_proxy_fallbacks.load(std::memory_order_relaxed);
    std::uint8_t gate = kAnchorGateOff;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        gate = kAnchorGateHash;
      } else if (insurance == kAnchorInsuranceServiceMissing) {
        gate = kAnchorGateService;
      } else if (insurance == kAnchorInsuranceReadFailed) {
        gate = kAnchorGateRead;
      } else if (render_x.chain_failed) {
        gate = kAnchorGateOffset;
      } else if (render_x.resolve_state == kRenderXFailed) {
        gate = kAnchorGateResolve;
      } else if (insurance == kAnchorInsuranceReady &&
                 render_x.resolve_state == kRenderXReady) {
        gate = kAnchorGateOk;
      }
    }
    context.display.anchor_gate = gate;
    std::uint8_t effective = kAnchorStateDisabled;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        effective = kAnchorStateHashMismatch;
      } else if (render_x.resolve_state == kRenderXFailed) {
        effective = kAnchorStateResolveFailed;
      } else {
        effective = last_used;
      }
    } else if (source == kAnchorSourceBorrow) {
      effective = kAnchorStateBorrow;
    }
    context.display.anchor_effective = effective;
  }
  const std::uint64_t interval_min_raw =
      diag.tick_interval_min_ns.load(std::memory_order_relaxed);
  context.display.tick_interval_valid = interval_min_raw != UINT64_MAX;
  context.display.tick_interval_min_ms =
      interval_min_raw == UINT64_MAX
          ? 0.0
          : static_cast<double>(interval_min_raw) * 1e-6;
  context.display.tick_interval_max_ms =
      static_cast<double>(
          diag.tick_interval_max_ns.load(std::memory_order_relaxed)) *
      1e-6;
  context.display.preset = context.preset.load(std::memory_order_acquire);
  context.display.fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  context.display.camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
  context.display.position_spring =
      context.position_spring.load(std::memory_order_acquire);
  context.display.yaw_spring =
      context.yaw_spring.load(std::memory_order_acquire);
  context.display.drift_yaw_spring =
      context.drift_yaw_spring.load(std::memory_order_acquire);
  context.display.look_yaw_spring =
      context.look_yaw_spring.load(std::memory_order_acquire);
  context.display.yaw_return_spring =
      context.yaw_return_spring.load(std::memory_order_acquire);
  context.display.pitch_spring =
      context.pitch_spring.load(std::memory_order_acquire);
  context.display.roll_spring =
      context.roll_spring.load(std::memory_order_acquire);
  context.display.origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
  context.display.aspect_ratio =
      context.aspect_ratio.load(std::memory_order_acquire);
  context.display.speed_distance_scale =
      context.speed_distance_scale.load(std::memory_order_acquire);
  context.display.roll_scale =
      context.roll_scale.load(std::memory_order_acquire);
  context.display.pitch_scale =
      context.pitch_scale.load(std::memory_order_acquire);
  context.display.shake_scale =
      context.shake_scale.load(std::memory_order_acquire);
  context.display.burnout_enabled =
      context.burnout_enabled.load(std::memory_order_acquire);
  context.display.burnout_entry_delay =
      context.burnout_entry_delay.load(std::memory_order_acquire);
  context.display.burnout_entry_time =
      context.burnout_entry_time.load(std::memory_order_acquire);
  context.display.burnout_exit_delay =
      context.burnout_exit_delay.load(std::memory_order_acquire);
  context.display.burnout_exit_time =
      context.burnout_exit_time.load(std::memory_order_acquire);
  context.display.burnout_launch_exit_time =
      context.burnout_launch_exit_time.load(std::memory_order_acquire);
  context.display.burnout_fov =
      context.burnout_fov.load(std::memory_order_acquire);
  context.display.burnout_shake =
      context.burnout_shake.load(std::memory_order_acquire);
  context.display.burnout_side_mode =
      context.burnout_side_mode.load(std::memory_order_acquire);
  context.display.burnout_entry_speed =
      context.burnout_entry_speed.load(std::memory_order_acquire);
  context.display.burnout_hold_speed =
      context.burnout_hold_speed.load(std::memory_order_acquire);
  context.display.burnout_shot_blend =
      context.presenter_burnout_blend.load(std::memory_order_relaxed);
  context.display.burnout_shot_side =
      context.presenter_burnout_side.load(std::memory_order_relaxed);
  context.display.burnout_shot_fallbacks =
      context.presenter_burnout_fallbacks.load(std::memory_order_relaxed);
  context.display.slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  context.display.slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  context.display.roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  context.display.diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  context.display.diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  context.display.borrow_strength =
      context.borrow_strength.load(std::memory_order_acquire);
  context.display.diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  context.display.diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
}

void RetryHookIfNeeded(Context &context) noexcept {
  const bool view_point_ready =
      context.view_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_point_pattern_unavailable;
  const bool transform_ready =
      context.transform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.transform_pattern_unavailable;
  const bool view_uniform_ready =
      context.view_uniform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_uniform_pattern_unavailable;
  const bool early_route_b_ready =
      context.prepare_view_state_hook_id.load(std::memory_order_acquire) != 0 ||
      context.early_route_b_unavailable;
  if (view_point_ready && transform_ready && view_uniform_ready &&
      early_route_b_ready) {
    return;
  }
  if (context.hook_retry_requested.exchange(false, std::memory_order_acq_rel)) {
    context.next_hook_retry = 0.0;
  }
  if (context.elapsed < context.next_hook_retry) {
    return;
  }
  context.next_hook_retry = context.elapsed + kHookRetrySeconds;
  // The profile hash is the layout witness for the route B hooks. Patterns
  // prove the entry point exists; they cannot prove that FViewInfo or
  // FSceneRenderer still have the fields we write. If the gate is closed,
  // stay out of the game instead of guessing. The legacy view-point hook is
  // kept only when the host genuinely has no build service (2.5).
  const bool version_gate =
      context.render_x.version_gate.load(std::memory_order_acquire);
  const bool legacy_without_build =
      context.render_x.insurance_state == kAnchorInsuranceServiceMissing;
  if (!version_gate && !legacy_without_build) {
    return;
  }
  if (version_gate) {
    static_cast<void>(EnsurePrepareViewStateHook(context));
    static_cast<void>(EnsureTransformHook(context));
    static_cast<void>(EnsureViewUniformHook(context));
  }
  static_cast<void>(EnsureHook(context));
}

void LogViewUniformHookDiagnostics(Context &context) noexcept {
  if (context.view_uniform_diag_logged) {
    return;
  }
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t calls =
      probe.view_uniform_calls.load(std::memory_order_relaxed);
  const std::uint64_t render_calls =
      probe.view_uniform_render_calls.load(std::memory_order_relaxed);
  if (render_calls < 10 && calls < 200) {
    return;
  }
  const std::uint64_t age_count =
      probe.view_uniform_age_count.load(std::memory_order_relaxed);
  const double age_avg_ms =
      age_count == 0
          ? 0.0
          : static_cast<double>(
                probe.view_uniform_age_sum_us.load(std::memory_order_relaxed)) /
                static_cast<double>(age_count) / 1000.0;
  const double age_peak_ms =
      static_cast<double>(
          probe.view_uniform_age_peak_us.load(std::memory_order_relaxed)) /
      1000.0;
  char message[192]{};
  std::snprintf(
      message, sizeof(message),
      "nfs2015 chase cam: view hook calls=%llu render=%llu game=%llu "
      "transform_thread=%u age avg/peak=%.2f/%.2f ms samples=%llu",
      static_cast<unsigned long long>(calls),
      static_cast<unsigned long long>(render_calls),
      static_cast<unsigned long long>(
          probe.view_uniform_game_calls.load(std::memory_order_relaxed)),
      render_calls == 0
          ? 0U
          : probe.render_thread_id.load(std::memory_order_relaxed),
      age_avg_ms, age_peak_ms,
      static_cast<unsigned long long>(age_count));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.view_uniform_diag_logged = true;
}

void LogRouteBDiagnostics(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t applied =
      probe.route_b_applied.load(std::memory_order_relaxed);
  const std::uint64_t early_applied =
      probe.route_b_early_applied.load(std::memory_order_relaxed);
  const std::uint64_t prepare_calls =
      probe.prepare_calls.load(std::memory_order_relaxed);
  if (!context.route_b_early_failure_logged && early_applied == 0 &&
      applied == 0 && prepare_calls >= 120) {
    context.route_b_early_failure_logged = true;
    char warning[320]{};
    std::snprintf(
        warning, sizeof(warning),
        "nfs2015 chase cam: route B early path has not applied yet: "
        "prep=%llu off=%llu nv=%llu pf=%llu fs=%llu dir=%llu mm=%llu fb=%llu",
        static_cast<unsigned long long>(prepare_calls),
        static_cast<unsigned long long>(
            probe.prepare_anchor_off.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_no_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_pose_fail.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_fullscreen_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_direct.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_thread_mismatch.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_fallbacks.load(std::memory_order_relaxed)));
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, warning);
  }
  if (context.route_b_diag_logged) {
    return;
  }
  if (applied + early_applied < 10) {
    return;
  }
  char message[256]{};
  std::snprintf(message, sizeof(message),
                "nfs2015 chase cam: route B early=%llu late=%llu "
                "fallbacks=%llu witness=%llu early_age=%.2f ms "
                "late_age=%.2f ms presenter=%llu direct=%llu held=%llu",
                static_cast<unsigned long long>(early_applied),
                static_cast<unsigned long long>(applied),
                static_cast<unsigned long long>(
                    probe.route_b_early_fallbacks.load(
                        std::memory_order_relaxed) +
                    probe.route_b_fallbacks.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_witness_failures.load(
                        std::memory_order_relaxed)),
                static_cast<double>(probe.route_b_early_last_age_us.load(
                    std::memory_order_relaxed)) /
                    1000.0,
                static_cast<double>(
                    probe.route_b_last_age_us.load(std::memory_order_relaxed)) /
                    1000.0,
                static_cast<unsigned long long>(
                    probe.route_b_early_presenter.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_direct.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_held.load(
                        std::memory_order_relaxed)));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.route_b_diag_logged = true;
}

// TEMP read-only timing report. It only logs values already captured by the
// early/transform/late callbacks; it does not change any camera state.
void LogRouteBTiming(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const auto early = probe.timing_early_count.load(std::memory_order_relaxed);
  if (early == 0 || context.route_b_timing_windows >= 15) {
    return;
  }
  const auto now = GetTickCount64();
  if (context.route_b_timing_last_log_ms != 0 &&
      now - context.route_b_timing_last_log_ms < 2000) {
    return;
  }
  context.route_b_timing_last_log_ms = now;
  ++context.route_b_timing_windows;

  const auto late = probe.timing_late_count.load(std::memory_order_relaxed);
  const auto car_count =
      probe.timing_car_delta_count.load(std::memory_order_relaxed);
  const auto view_count =
      probe.timing_view_delta_count.load(std::memory_order_relaxed);
  const double car_avg =
      car_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_car_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(car_count) / 10.0;
  const double car_peak =
      static_cast<double>(probe.timing_car_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;
  const double view_avg =
      view_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_view_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(view_count) / 10.0;
  const double view_peak =
      static_cast<double>(probe.timing_view_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;

  char line[384]{};
  std::snprintf(
      line, sizeof(line),
      "nfs2015 chase cam: timing early=%llu late=%llu "
      "transform before/between/after=%llu/%llu/%llu "
      "car_delta avg/peak=%.2f/%.2f cm view_delta avg/peak=%.2f/%.2f cm",
      static_cast<unsigned long long>(early),
      static_cast<unsigned long long>(late),
      static_cast<unsigned long long>(
          probe.timing_transform_before.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_between.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_after.load(std::memory_order_relaxed)),
      car_avg, car_peak, view_avg, view_peak);
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, line);

  char sample[384]{};
  std::snprintf(
      sample, sizeof(sample),
      "nfs2015 chase cam: timing sample early_car=(%.2f,%.2f,%.2f) "
      "late_car=(%.2f,%.2f,%.2f) presenter_car=(%.2f,%.2f,%.2f) "
      "early_view=(%.2f,%.2f,%.2f) late_view=(%.2f,%.2f,%.2f)",
      probe.timing_early_car[0].load(std::memory_order_relaxed),
      probe.timing_early_car[1].load(std::memory_order_relaxed),
      probe.timing_early_car[2].load(std::memory_order_relaxed),
      probe.timing_late_car[0].load(std::memory_order_relaxed),
      probe.timing_late_car[1].load(std::memory_order_relaxed),
      probe.timing_late_car[2].load(std::memory_order_relaxed),
      probe.timing_presenter_car[0].load(std::memory_order_relaxed),
      probe.timing_presenter_car[1].load(std::memory_order_relaxed),
      probe.timing_presenter_car[2].load(std::memory_order_relaxed),
      probe.timing_early_view[0].load(std::memory_order_relaxed),
      probe.timing_early_view[1].load(std::memory_order_relaxed),
      probe.timing_early_view[2].load(std::memory_order_relaxed),
      probe.timing_late_view[0].load(std::memory_order_relaxed),
      probe.timing_late_view[1].load(std::memory_order_relaxed),
      probe.timing_late_view[2].load(std::memory_order_relaxed));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, sample);
}

// ---------------------------------------------------------------------------
// Plugin lifecycle
// ---------------------------------------------------------------------------

AnomalyStatusV1 ANOMALY_CALL Load(const AnomalyHostApiV1 *host,
                                  void **plugin_context) {
  if (host == nullptr || plugin_context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  *plugin_context = nullptr;
  auto *context = new (std::nothrow) Context();
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_FAILED);
  }
  context->localizer = nfs2015_chase_cam::Localizer(host);
  context->core = Query<AnomalyCoreServiceV1>(
      host, ANOMALY_CORE_SERVICE_V1_ID, ANOMALY_CORE_SERVICE_V1_VERSION);
  context->config = Query<AnomalyConfigServiceV1>(
      host, ANOMALY_CONFIG_SERVICE_V1_ID, ANOMALY_CONFIG_SERVICE_V1_VERSION);
  context->json = Query<AnomalyJsonServiceV1>(
      host, ANOMALY_JSON_SERVICE_V1_ID, ANOMALY_JSON_SERVICE_V1_VERSION);
  context->input = Query<AnomalyInputServiceV1>(
      host, ANOMALY_INPUT_SERVICE_V1_ID, ANOMALY_INPUT_SERVICE_V1_VERSION);
  context->ui = Query<AnomalyUiServiceV1>(
      host, ANOMALY_UI_SERVICE_V1_ID, ANOMALY_UI_SERVICE_V1_VERSION);
  // 2.6.0 managed windows. Optional: 2.5.0 has no `anomaly.window`, in which
  // case Draw falls back to the direct `anomaly.ui` begin/end pair.
  context->window = Query<AnomalyWindowServiceV1>(
      host, ANOMALY_WINDOW_SERVICE_V1_ID, ANOMALY_WINDOW_SERVICE_V1_VERSION);
  context->signature = Query<AnomalySignatureServiceV1>(
      host, ANOMALY_SIGNATURE_SERVICE_V1_ID,
      ANOMALY_SIGNATURE_SERVICE_V1_VERSION);
  context->hook = Query<AnomalyHookServiceV1>(
      host, ANOMALY_HOOK_SERVICE_V1_ID, ANOMALY_HOOK_SERVICE_V1_VERSION);
  context->session = Query<AnomalyNteSessionServiceV1>(
      host, ANOMALY_NTE_SESSION_SERVICE_V1_ID,
      ANOMALY_NTE_SESSION_SERVICE_V1_VERSION);
  context->player = Query<AnomalyNtePlayerServiceV1>(
      host, ANOMALY_NTE_PLAYER_SERVICE_V1_ID,
      ANOMALY_NTE_PLAYER_SERVICE_V1_VERSION);
  // 2.6's anomaly.nte.vehicle service is gone. The getters it called are still
  // ordinary reflected UFunctions, so resolve them through the generic UE5
  // services instead of guessing from motion.
  context->objects = Query<AnomalyUe5ObjectsServiceV1>(
      host, ANOMALY_UE5_OBJECTS_SERVICE_V1_ID,
      ANOMALY_UE5_OBJECTS_SERVICE_V1_VERSION);
  context->events = Query<AnomalyUe5ProcessEventServiceV1>(
      host, ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_ID,
      ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_VERSION);
  // Optional engine access for the probe. Declared optional in the manifest, so
  // a host without `ue5.ahud` simply leaves this null and the probe reports
  // "off" instead of failing the load.
  context->ahud = Query<AnomalyUe5AhudServiceV1>(
      host, ANOMALY_UE5_AHUD_SERVICE_V1_ID, ANOMALY_UE5_AHUD_SERVICE_V1_VERSION);
  // Version insurance for the X anchor. Optional: a host without the build
  // service leaves the X path disabled and the borrow estimate in charge.
  context->build = Query<AnomalyUe5BuildServiceV1>(
      host, ANOMALY_UE5_BUILD_SERVICE_V1_ID,
      ANOMALY_UE5_BUILD_SERVICE_V1_VERSION);
  // The entities/actors services are deliberately not leased. Every one of
  // their reflection calls (`component_bounds`, `frame`, `page`) runs a
  // name-keyed walk inside the host on every call, and the camera needs none
  // of them: the Pawn root is the vehicle actor while driving, and its
  // position and bounds arrive through the player service's cached snapshots.
  if (!CoreReady(context->core) || !ConfigReady(context->config) ||
      !UiReady(context->ui) || !SignatureReady(context->signature) ||
      !HookReady(context->hook)) {
    delete context;
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE,
                  "required camera services are unavailable");
  }
  const auto schema_status = context->config->register_schema(
      context->config->user, StringView(kSettingsSchemaId),
      kSettingsSchemaVersion, Bytes(kSettingsSchema), &context->settings_schema);
  if (schema_status.code != ANOMALY_STATUS_V1_OK ||
      context->settings_schema.id == 0 || !LoadSettings(*context)) {
    ReleaseSettingsSchema(*context);
    delete context;
    return Status(ANOMALY_STATUS_V1_FAILED, "camera settings are invalid");
  }
  *plugin_context = context;
  return anomaly::sdk::Ok();
}

void RegisterSettingsWindow(Context &context) noexcept {
  if (!WindowReady(context.window) || context.window_handle.id != 0) {
    return;
  }
  try {
    AnomalyWindowSpecV1 spec{};
    spec.struct_size = sizeof(spec);
    spec.flags = 0;
    spec.id = StringView(kSettingsWindowId);
    const std::string title =
        context.localizer.Text("window.title", "NFS 2015 Chase Cam");
    spec.title = StringView(title);
    spec.initial_width = 430.0F;
    spec.initial_height = 720.0F;
    spec.minimum_width = 320.0F;
    spec.minimum_height = 240.0F;
    spec.maximum_width = 900.0F;
    spec.maximum_height = 1200.0F;
    spec.default_open = 1;
    const AnomalyStatusV1 status = context.window->register_window(
        context.window->user, &spec, &context.window_handle);
    if (status.code != ANOMALY_STATUS_V1_OK ||
        context.window_handle.id == 0) {
      context.window_handle = {};
    }
  } catch (...) {
    context.window_handle = {};
  }
}

void ReleaseSettingsWindow(Context &context) noexcept {
  if (context.window_handle.id != 0 && WindowReady(context.window)) {
    static_cast<void>(context.window->release_window(
        context.window->user, context.window_handle));
  }
  context.window_handle = {};
}

AnomalyStatusV1 ANOMALY_CALL Start(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->elapsed = 0.0;
  context->update_rate_window = 0.0;
  context->update_rate_frames = 0;
  context->update_hz = 0.0;
  context->view_output_hz = 0.0;
  context->interpolation_ratio = 0.0;
  context->view_output_frames.store(0, std::memory_order_relaxed);
  context->configured_preset =
      context->preset.load(std::memory_order_acquire);
  context->chase.SetConfig(context->configured_preset == 0
                               ? nfs2015_chase::GamePreset()
                               : nfs2015_chase::CinematicPreset());
  context->reset_requested.store(true, std::memory_order_release);
  // Signature learning is a Lifecycle-domain operation; cache the target here
  // so the Game tick only has to create the hook.
  std::uintptr_t view_point_target = 0;
  if (ResolveSignature(*context, kCameraViewPointPattern, view_point_target)) {
    context->hook_target.store(view_point_target, std::memory_order_release);
  }
  ResetMotion(*context);
  context->wheel_steer = WheelSteerState{};
  context->vehicle_present_last = false;
  context->last_vehicle_present_time = 0.0;
  context->vehicle_getter_handles = {};
  context->next_vehicle_getter_retry = {};
  context->next_vehicle_query_time = 0.0;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->observed_valid.store(false, std::memory_order_release);
  PublishInactive(*context);
  // T0: a new session starts from a clean counter baseline so the recorded
  // numbers describe this run only.
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    context->diag.snap_by_reason[index].store(0, std::memory_order_relaxed);
  }
  context->diag.yaw_source_switches.store(0, std::memory_order_relaxed);
  context->diag.observed_read_failures.store(0, std::memory_order_relaxed);
  context->diag.lease_failures.store(0, std::memory_order_relaxed);
  context->diag.view_read_torn.store(0, std::memory_order_relaxed);
  context->diag.detour_calls.store(0, std::memory_order_relaxed);
  context->diag.detour_calls_window.store(0, std::memory_order_relaxed);
  context->diag.detour_no_publish.store(0, std::memory_order_relaxed);
  context->diag.player_sequence_jumps.store(0, std::memory_order_relaxed);
  context->diag.camera_yaw_esp_misses.store(0, std::memory_order_relaxed);
  context->diag.slip_from_position_degrees.store(0.0,
                                                 std::memory_order_relaxed);
  context->diag.slip_sign_changes.store(0, std::memory_order_relaxed);
  context->diag.slip_last_sign.store(0, std::memory_order_relaxed);
  context->diag.slip_max_step_ddeg.store(0, std::memory_order_relaxed);
  context->last_slip_degrees = 0.0;
  context->have_last_slip = false;
  context->diag.render_speed_floor_mmps.store(UINT64_MAX,
                                              std::memory_order_relaxed);
  context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
  context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                             std::memory_order_relaxed);
  context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
  context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
  context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                           std::memory_order_relaxed);
  context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
  context->camera_cost.Reset();
  context->camera_ms_last = 0.0;
  // A fresh session re-runs the P0 probe from scratch. Stop already dropped the
  // old subscription, so nothing can be in flight while this resets.
  context->probe = Probe{};
  // Task group 1: check the profile hash, then resolve `ue5.GWorld` from the
  // Lifecycle domain where signature learning is allowed. The Game tick
  // retries a failed pattern scan, but never bypasses the hash gate.
  context->render_x.Reset();
  EvaluateAnchorInsurance(*context);
  context->detour_hz = 0.0;
  context->brake_last_target = 0.0;
  static_cast<void>(RegisterToggleHotkey(*context));
  RegisterSettingsWindow(*context);
  // Install the hook from the Game tick, not from this lifecycle callback: the
  // target function is called on the Game thread, so create/enable cannot
  // re-enter the detour before the original pointer is published.
  context->next_hook_retry = 0.0;
  context->started.store(true, std::memory_order_release);
  return anomaly::sdk::Ok();
}

AnomalyStatusV1 ANOMALY_CALL Stop(void *plugin_context,
                                  std::uint32_t /*deadline_milliseconds*/) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->started.store(false, std::memory_order_release);
  // Put the front wheels back to their stock relative rotation before the
  // vehicle getter caches are dropped. If the game is already tearing down the
  // car the writes fail harmlessly; a normal stop restores the visual pose.
  ReleaseWheelSteer(*context);
  // Both callbacks check `started` or `g_active` first, so dropping these two
  // before any release means the revocation below has nothing to wait for.
  // Draining a callback that is still running the model is what turns a stop
  // into a timeout, and a timeout into a quarantined plugin that the Host will
  // not load again until the process restarts.
  g_active.store(nullptr, std::memory_order_release);
  context->chase_initialized = false;
  ++context->sample_epoch;
  context->sample_mean_interval_ns.store(0.0, std::memory_order_relaxed);
  context->game_offset_valid = false;
  context->game_offset_last_ns = 0;
  context->car_ring.Clear();
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
  ReleaseProbeSubscription(*context);
  ReleaseSettingsWindow(*context);
  ReleaseHook(*context);
  if (context->settings_revision.load(std::memory_order_acquire) !=
      context->persisted_settings_revision.load(std::memory_order_acquire)) {
    static_cast<void>(PersistSettings(*context));
  }
  std::scoped_lock lock(context->display_mutex);
  context->display.started = false;
  context->display.hook_ready = false;
  context->display.active = false;
  context->display.driving = false;
  context->display.reversing = false;
  context->display.speed_source = kSpeedSourceUnavailable;
  context->display.speed_sign_known = false;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->next_vehicle_query_time = 0.0;
  return anomaly::sdk::Ok();
}

void ANOMALY_CALL Unload(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  static_cast<void>(Stop(context, 0));
  ReleaseSettingsSchema(*context);
  delete context;
}

void ANOMALY_CALL Update(void *plugin_context, const double delta_seconds) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  try {
    if (!context->started.load(std::memory_order_acquire)) {
      UpdateDisplay(*context);
      return;
    }
    const double dt =
        std::isfinite(delta_seconds)
            ? nfs2015_chase::Clamp(delta_seconds, 0.0, 0.25)
            : 0.0;
    context->elapsed += dt;
    if (dt > 0.0) {
      context->update_rate_window += dt;
      ++context->update_rate_frames;
      if (context->update_rate_window >= kUpdateRateWindowSeconds) {
        context->update_hz =
            static_cast<double>(context->update_rate_frames) /
            context->update_rate_window;
        const std::uint64_t outputs =
            context->view_output_frames.exchange(0, std::memory_order_relaxed);
        // Every detour call that produced a pose is now a render-clock model
        // step, so the "interpolated" share is simply the presented share.
        const std::uint64_t interpolated = outputs;
        context->view_output_hz =
            static_cast<double>(outputs) / context->update_rate_window;
        context->interpolation_ratio =
            outputs != 0
                ? static_cast<double>(interpolated) /
                      static_cast<double>(outputs)
                : 0.0;
        // Q1/Q4: how often the presenter is actually asked for a pose, next
        // to the Game tick rate. These two should track the render rate once
        // T2 moves the model off the Game tick.
        const std::uint64_t detour_calls =
            context->diag.detour_calls_window.exchange(
                0, std::memory_order_relaxed);
        context->detour_hz =
            static_cast<double>(detour_calls) / context->update_rate_window;
        context->diag.render_speed_floor_mmps.store(
            UINT64_MAX, std::memory_order_relaxed);
        context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
        context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                                   std::memory_order_relaxed);
        context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
        context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
        context->probe.camera_yaw_step_max_ddeg = 0.0;
        context->probe.camera_yaw_step_sum_ddeg = 0.0;
        context->probe.camera_yaw_step_count = 0;
        context->probe.height_step_cm = 0.0;
        context->probe.track_anchor.count = 0;
        context->probe.eye_count = 0;
        context->probe.track_car.count = 0;
        context->diag.judder_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_count.store(0, std::memory_order_relaxed);
        context->diag.judder_car_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_count.store(0, std::memory_order_relaxed);
        context->diag.reset_window.store(0, std::memory_order_relaxed);
        ResetResidual(context->diag.flow_peak_ccm,
                      context->diag.flow_sum_ccm,
                      context->diag.flow_count);
        ResetResidual(context->diag.frame_peak_us,
                      context->diag.frame_sum_us,
                      context->diag.frame_count);
        ResetResidual(context->diag.pitch_peak_cdeg,
                      context->diag.pitch_sum_cdeg,
                      context->diag.pitch_count);
        ResetResidual(context->diag.roll_peak_cdeg,
                      context->diag.roll_sum_cdeg,
                      context->diag.roll_count);
        ResetResidual(context->diag.lens_peak_cdeg,
                      context->diag.lens_sum_cdeg,
                      context->diag.lens_count);
        context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                                 std::memory_order_relaxed);
        context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
        context->render_x.ResetWindow();
        context->diag.slip_max_step_ddeg.store(0,
                                               std::memory_order_relaxed);
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    RetryHookIfNeeded(*context);
    LogViewUniformHookDiagnostics(*context);
    LogRouteBDiagnostics(*context);
    LogRouteBTiming(*context);
    static_cast<void>(ResolveCameraPov(*context));
    // T0: time the Game-tick work that the camera itself owns. The host
    // callback as a whole also persists settings and resolves the POV, so only
    // UpdateCamera is bracketed here.
    const std::int64_t camera_begin_ns = NowNanoseconds();
    UpdateCamera(*context, dt);
    context->camera_ms_last =
        static_cast<double>(NowNanoseconds() - camera_begin_ns) * 1e-6;
    context->camera_cost.Add(context->camera_ms_last);
    // Probe: read-only reconnaissance, one Game tick at a time. It runs after
    // UpdateCamera so the yaw it compares is this tick's sample.
    RunProbe(*context, dt);
    UpdateDisplay(*context);
    const auto revision =
        context->settings_revision.load(std::memory_order_acquire);
    if (revision !=
        context->persisted_settings_revision.load(std::memory_order_acquire)) {
      const auto now = GetTickCount64();
      const auto changed_at =
          context->settings_changed_at.load(std::memory_order_acquire);
      if (now - changed_at >= kSettingsSaveDelayMilliseconds) {
        if (!PersistSettings(*context)) {
          // Back off instead of retrying a failing config write every tick.
          context->settings_changed_at.store(now, std::memory_order_release);
        }
      }
    }
  } catch (...) {
    // Host callback boundary: plugin exceptions must not escape.
  }
}

}  // namespace nfs2015_chase_cam::detail

ANOMALY_SDK_EXPORT AnomalyStatusV1 ANOMALY_CALL AnomalyPluginEntryV1(
    AnomalyPluginDescriptorV1 *descriptor) {
  if (descriptor == nullptr ||
      descriptor->struct_size < sizeof(*descriptor)) {
    return {ANOMALY_STATUS_V1_INVALID_ARGUMENT, 0, {}};
  }
  using namespace nfs2015_chase_cam::detail;
  *descriptor = {
      sizeof(*descriptor),
      ANOMALY_PLUGIN_API_V1_MAJOR,
      ANOMALY_PLUGIN_API_V1_MINOR,
      StringView(kPluginId),
      StringView("NFS 2015 Chase Cam"),
      StringView("RobinEatCorn"),
      StringView("2.0.3"),
      Load,
      Start,
      Stop,
      Unload,
      Update,
      Draw};
  return anomaly::sdk::Ok();
}
