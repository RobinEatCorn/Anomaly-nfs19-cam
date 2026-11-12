// Render-thread car position probe: the read-only reconnaissance that answers
// whether the render thread sees a newer car transform than the Game-tick
// snapshot the camera is built from. No allocation, no lock of our own, no
// logging: stack buffers and atomics, so the render path stays cheap.
#include "internal.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace nfs2015_chase_cam::detail {

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
                    "%zu accepted runtime profiles); camera hooks disabled until "
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

}  // namespace nfs2015_chase_cam::detail
