// Game-thread vehicle runtime. It resolves the profile gate and `ue5.GWorld`,
// walks the player pawn chain, and reads the vehicle mesh's ComponentToWorld.
// The camera consumes the live transform at the scene-view phase; this module
// also publishes the presence and body-yaw sample the Game tick needs.
#include "internal.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace nfs2015_chase_cam::detail {

namespace {

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

enum class ParentRead { kNoParent, kFailed, kValid };

// Follow the player SceneComponent's AttachParent and read its
// ComponentToWorld. While driving this is the vehicle's `VehicleMesh`
// SkeletalMesh: translation is the chassis pivot at the ground, and the
// quaternion's local +X is the body nose. A missing parent is a definite "on
// foot" answer; a failed transform read is not.
ParentRead ReadParentTransform(const AnomalyCoreServiceV1 *core,
                               const std::uintptr_t root,
                               const double root_cm[3],
                               std::uintptr_t &parent_out,
                               double translation[3],
                               double forward[3]) noexcept {
  parent_out = 0;
  std::uintptr_t parent{};
  if (!ReadPointerAtOffset(core, root, kSceneComponentAttachParentOffset,
                           parent)) {
    return ParentRead::kFailed;
  }
  if (parent == 0 || parent == root) {
    return ParentRead::kNoParent;
  }
  std::array<double, 4> orientation{};
  std::array<double, 3> local_translation{};
  if (!Read(core, parent + kSceneComponentTransformOffset, orientation) ||
      !Read(core, parent + kSceneComponentTransformTranslationOffset,
            local_translation)) {
    return ParentRead::kFailed;
  }
  const double norm2 =
      orientation[0] * orientation[0] + orientation[1] * orientation[1] +
      orientation[2] * orientation[2] + orientation[3] * orientation[3];
  if (!(norm2 > 0.81 && norm2 < 1.21) ||
      !std::isfinite(local_translation[0]) ||
      !std::isfinite(local_translation[1]) ||
      !std::isfinite(local_translation[2])) {
    return ParentRead::kFailed;
  }
  // The capsule sits inside the vehicle; a parent transform far from the pawn
  // root is a stale/recycled pointer, not the car.
  double parent_location[3]{};
  if (!ReadChainLocation(core, parent, parent_location)) {
    return ParentRead::kFailed;
  }
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = parent_location[axis] - root_cm[axis];
    squared += delta * delta;
  }
  if (!std::isfinite(squared) ||
      squared > kVehicleAnchorMaxOffsetCm * kVehicleAnchorMaxOffsetCm) {
    return ParentRead::kFailed;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm2);
  const double x = orientation[0] * inverse_norm;
  const double y = orientation[1] * inverse_norm;
  const double z = orientation[2] * inverse_norm;
  const double w = orientation[3] * inverse_norm;
  // Rotate the local +X (nose) axis by the ComponentToWorld quaternion.
  const double tx = 0.0;
  const double ty = 2.0 * z;
  const double tz = -2.0 * y;
  forward[0] = 1.0 + w * tx + (y * tz - z * ty);
  forward[1] = w * ty + (z * tx - x * tz);
  forward[2] = w * tz + (x * ty - y * tx);
  const double length_squared =
      forward[0] * forward[0] + forward[1] * forward[1] +
      forward[2] * forward[2];
  if (!std::isfinite(forward[0]) || !std::isfinite(forward[1]) ||
      !std::isfinite(forward[2]) || !(length_squared > 1e-8)) {
    return ParentRead::kFailed;
  }
  const double inverse_length = 1.0 / std::sqrt(length_squared);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    forward[axis] *= inverse_length;
    translation[axis] = local_translation[axis];
  }
  parent_out = parent;
  return ParentRead::kValid;
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

}  // namespace

void EvaluateProfileGate(Context &context) noexcept {
  VehicleRuntime &runtime = context.vehicle_runtime;
  std::uint8_t next = kProfileGateServiceMissing;
  std::array<char, 96> hash{};
  std::size_t hash_size = hash.size();
  if (context.build != nullptr && context.build->profile_hash != nullptr) {
    const AnomalyStatusV1 status = context.build->profile_hash(
        context.build->user, hash.data(), &hash_size);
    if (status.code == ANOMALY_STATUS_V1_OK && hash_size > 1 &&
        hash_size <= hash.size() && hash[0] != '\0') {
      next = AcceptedProfileHash(hash.data()) ? kProfileGateReady
                                              : kProfileGateHashMismatch;
    } else {
      next = kProfileGateReadFailed;
    }
  }
  runtime.profile_state = next;
  runtime.version_gate.store(next == kProfileGateReady,
                             std::memory_order_release);
  if (next != kProfileGateReady) {
    runtime.root.store(0, std::memory_order_release);
  }
  if (runtime.logged_profile_state != next) {
    runtime.logged_profile_state = next;
    char message[320]{};
    if (next == kProfileGateReady) {
      std::snprintf(message, sizeof(message),
                    "profile gate: ready (%.*s)",
                    static_cast<int>(hash_size > 0 ? hash_size - 1 : 0),
                    hash.data());
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    } else if (next == kProfileGateHashMismatch) {
      std::snprintf(message, sizeof(message),
                    "profile gate: hash mismatch (got %.*s, want one of %zu "
                    "accepted runtime profiles); camera hooks disabled until "
                    "the plugin is updated",
                    static_cast<int>(hash_size > 0 ? hash_size - 1 : 0),
                    hash.data(), std::size(kAcceptedProfileHashes));
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    } else if (next == kProfileGateServiceMissing) {
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "profile gate: ue5 build service unavailable; game-thread camera "
          "disabled");
    } else {
      std::snprintf(message, sizeof(message),
                    "profile gate: hash unreadable (size=%zu); camera hooks "
                    "disabled",
                    hash_size);
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    }
  }
  if (next == kProfileGateReady) {
    TryResolveVehicleRuntime(context);
  }
}

void TryResolveVehicleRuntime(Context &context) noexcept {
  VehicleRuntime &runtime = context.vehicle_runtime;
  if (runtime.profile_state != kProfileGateReady) {
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
      runtime.gworld_slot = static_cast<std::uintptr_t>(slot);
      runtime.resolve_state = kVehicleRuntimeReady;
      runtime.resolve_code = ANOMALY_STATUS_V1_OK;
      return;
    }
  }
  runtime.resolve_state = kVehicleRuntimeFailed;
  runtime.resolve_code =
      status.code == ANOMALY_STATUS_V1_OK ? ANOMALY_STATUS_V1_FAILED
                                          : status.code;
}

void UpdateVehicleRuntime(Context &context, const double base_cm[3],
                          const bool player_valid) noexcept {
  VehicleRuntime &runtime = context.vehicle_runtime;
  if (runtime.profile_state == kProfileGateWaiting &&
      context.elapsed >= runtime.next_resolve_elapsed) {
    // The profile can still be settling on the first tick; a hash that was not
    // readable yet is worth one more look.
    runtime.next_resolve_elapsed =
        context.elapsed + kVehicleRuntimeResolveRetrySeconds;
    EvaluateProfileGate(context);
  } else if (runtime.profile_state == kProfileGateReady &&
             runtime.resolve_state != kVehicleRuntimeReady &&
             context.elapsed >= runtime.next_resolve_elapsed) {
    runtime.next_resolve_elapsed =
        context.elapsed + kVehicleRuntimeResolveRetrySeconds;
    TryResolveVehicleRuntime(context);
  }
  if (runtime.profile_state != kProfileGateReady ||
      runtime.resolve_state != kVehicleRuntimeReady) {
    runtime.root.store(0, std::memory_order_relaxed);
    runtime.vehicle_present.store(false, std::memory_order_release);
    runtime.vehicle_parent.store(0, std::memory_order_relaxed);
    runtime.body_yaw_valid.store(false, std::memory_order_release);
    return;
  }

  std::uintptr_t world{};
  PlayerChain chain{};
  if (Read(context.core, runtime.gworld_slot, world) && world != 0 &&
      WalkPlayerChain(context.core, world, chain)) {
    runtime.root.store(chain.root, std::memory_order_relaxed);
  } else {
    runtime.root.store(0, std::memory_order_relaxed);
  }

  if (!player_valid) {
    return;
  }
  const std::uintptr_t root = runtime.root.load(std::memory_order_relaxed);
  double root_cm[3]{};
  if (root == 0 || !ReadChainLocation(context.core, root, root_cm)) {
    runtime.vehicle_present.store(false, std::memory_order_release);
    runtime.vehicle_parent.store(0, std::memory_order_relaxed);
    runtime.vehicle_sample_ns.store(NowNanoseconds(),
                                    std::memory_order_relaxed);
    runtime.body_yaw_valid.store(false, std::memory_order_release);
    return;
  }

  // The same-chain cross-check. A stale offset table is caught here before the
  // camera reads a transform through it.
  double chain_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = root_cm[axis] - base_cm[axis];
    chain_squared += delta * delta;
  }
  const double chain_distance = std::sqrt(chain_squared);
  if (std::isfinite(chain_distance)) {
    if (chain_distance > runtime.chain_max_cm) {
      runtime.chain_max_cm = chain_distance;
    }
    if (chain_distance > kVehicleChainToleranceCm) {
      ++runtime.chain_mismatches;
      if (runtime.chain_fail_streak < kVehicleChainMismatchLimit) {
        ++runtime.chain_fail_streak;
      }
      if (runtime.chain_fail_streak >= kVehicleChainMismatchLimit) {
        runtime.chain_failed = true;
      }
    } else {
      runtime.chain_fail_streak = 0;
      runtime.chain_verified = true;
    }
  }

  std::uintptr_t parent{};
  double translation[3]{};
  double forward[3]{};
  const ParentRead parent_read =
      ReadParentTransform(context.core, root, root_cm, parent, translation,
                          forward);
  const std::int64_t now_ns = NowNanoseconds();
  switch (parent_read) {
    case ParentRead::kValid: {
      runtime.vehicle_miss_streak = 0;
      runtime.vehicle_parent.store(parent, std::memory_order_relaxed);
      runtime.vehicle_sample_ns.store(now_ns, std::memory_order_relaxed);
      runtime.vehicle_present_ns.store(now_ns, std::memory_order_relaxed);
      runtime.vehicle_present.store(true, std::memory_order_release);
      const double yaw = std::atan2(forward[1], forward[0]);
      if (std::isfinite(yaw)) {
        runtime.body_yaw.store(yaw, std::memory_order_relaxed);
        runtime.body_yaw_ns.store(now_ns, std::memory_order_relaxed);
        runtime.body_yaw_valid.store(true, std::memory_order_release);
      } else {
        runtime.body_yaw_valid.store(false, std::memory_order_release);
      }
      break;
    }
    case ParentRead::kNoParent:
      // A definite "on foot" answer. No grace period: the vehicle query is
      // authoritative when it is available, and keeping a stale parent here
      // would keep the chase camera alive after the player left the car.
      runtime.vehicle_miss_streak = 0;
      runtime.vehicle_parent.store(0, std::memory_order_relaxed);
      runtime.vehicle_sample_ns.store(now_ns, std::memory_order_relaxed);
      runtime.vehicle_present.store(false, std::memory_order_release);
      runtime.body_yaw_valid.store(false, std::memory_order_release);
      break;
    case ParentRead::kFailed:
      runtime.vehicle_read_failures.fetch_add(1, std::memory_order_relaxed);
      ++runtime.vehicle_miss_streak;
      if (runtime.vehicle_miss_streak >= 3) {
        runtime.vehicle_parent.store(0, std::memory_order_relaxed);
        runtime.vehicle_sample_ns.store(now_ns, std::memory_order_relaxed);
        runtime.vehicle_present.store(false, std::memory_order_release);
        runtime.body_yaw_valid.store(false, std::memory_order_release);
      }
      break;
  }
}

bool ReadVehicleComponentTransform(Context &context, double car_cm[3],
                                   double forward[3]) noexcept {
  VehicleRuntime &runtime = context.vehicle_runtime;
  if (!runtime.version_gate.load(std::memory_order_acquire) ||
      runtime.chain_failed) {
    return false;
  }
  const std::uintptr_t root = runtime.root.load(std::memory_order_relaxed);
  if (root == 0) {
    return false;
  }
  double root_cm[3]{};
  if (!ReadChainLocation(context.core, root, root_cm)) {
    return false;
  }
  std::uintptr_t parent{};
  double translation[3]{};
  double local_forward[3]{};
  const ParentRead read = ReadParentTransform(
      context.core, root, root_cm, parent, translation, local_forward);
  if (read != ParentRead::kValid) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    car_cm[axis] = translation[axis];
    forward[axis] = local_forward[axis];
  }
  return true;
}

bool ReadVehicleBodyYaw(const Context &context, double &yaw,
                        std::int64_t &timestamp_ns) noexcept {
  const VehicleRuntime &runtime = context.vehicle_runtime;
  if (!runtime.version_gate.load(std::memory_order_acquire) ||
      runtime.chain_failed ||
      !runtime.body_yaw_valid.load(std::memory_order_acquire)) {
    return false;
  }
  yaw = runtime.body_yaw.load(std::memory_order_relaxed);
  timestamp_ns = runtime.body_yaw_ns.load(std::memory_order_relaxed);
  return std::isfinite(yaw) && timestamp_ns > 0;
}

}  // namespace nfs2015_chase_cam::detail
