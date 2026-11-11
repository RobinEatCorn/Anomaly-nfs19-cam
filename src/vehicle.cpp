// Vehicle presence and speed: the 20 Hz poll of anomaly.nte.vehicle plus the
// cached object handles every other module reads through the Context.
#include "internal.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace nfs2015_chase_cam::detail {

// The 2.6 Host's anomaly.nte.vehicle snapshot called these three reflected
// getters; the service is gone in 2.7 but the game functions are unchanged.
// A successful call that returns null is an authoritative "not driving"; a
// failed call only means "unknown" and leaves the speed fallback available.
constexpr double kVehicleQueryIntervalSeconds = 0.05;
std::string_view constexpr kVehicleGetterPaths[3] = {
    "/Script/HTGame.HTPlayerController:BP_GetCurrentDriveVehicle",
    // The typo is the game's own function name, not ours.
    "/Script/HTGame.HTPlayerCharacter:BP_GetCurrentDirvingVehicle",
    "/Script/HTGame.HTPlayerCharacter:GetBoardWaterVehicle"};

// Turn a live UObject pointer into the registry handle ProcessEvent needs. The
// object's internal index is the same field the host's ObjectHandleLocked
// reads; snapshot_at returns the registered slot's serial and generation.
bool ObjectHandleFromPointer(Context &context, const std::uintptr_t object,
                             AnomalyGenerationHandleV1 &handle) noexcept {
  handle = {};
  if (object == 0 || !Ue5ObjectSnapshotReady(context.objects)) {
    return false;
  }
  std::int32_t raw_index = -1;
  if (!Read(context.core, object + kObjectInternalIndexOffset, raw_index) ||
      raw_index < 0) {
    return false;
  }
  AnomalyUe5ObjectSnapshotV1 snapshot{sizeof(snapshot)};
  const auto status = context.objects->snapshot_at(
      context.objects->user, static_cast<std::uint32_t>(raw_index), &snapshot);
  if (status.code != ANOMALY_STATUS_V1_OK || snapshot.handle.id == 0) {
    return false;
  }
  // snapshot_at is indexed, not pointer-keyed. Compare the FName id we read
  // from the raw object with the registry slot's name so a stale vehicle
  // pointer cannot silently turn into a different object's handle.
  std::uint32_t name_id = 0;
  if (snapshot.name_id == 0 ||
      !Read(context.core, object + kObjectNameOffset, name_id) ||
      name_id != snapshot.name_id) {
    return false;
  }
  handle = snapshot.handle;
  return true;
}

// GetForwardSpeedKmH belongs to HTVehicleMovementComponent, i.e. wheeled
// vehicles. Water vehicles have a different movement type and getter, so do
// not call it on them. Walk the class chain with the two offsets the 2.6
// profile validated and compare registry handles; this needs no extra service.
bool VehicleIsWheeled(Context &context, const std::uintptr_t vehicle) noexcept {
  if (vehicle == 0 || !Ue5ObjectSnapshotReady(context.objects)) {
    return false;
  }
  if (context.vehicle_wheeled_class.id == 0 &&
      context.elapsed >= context.next_vehicle_wheeled_class_retry) {
    context.next_vehicle_wheeled_class_retry =
        context.elapsed + kVehicleGetterRetrySeconds;
    AnomalyGenerationHandleV1 wheeled_class{};
    if (context.objects
                ->find_exact(context.objects->user,
                             StringView("/Script/HTGame.HTWheeledVehicleBase"),
                             &wheeled_class)
                .code == ANOMALY_STATUS_V1_OK &&
        wheeled_class.id != 0) {
      context.vehicle_wheeled_class = wheeled_class;
    }
  }
  if (context.vehicle_wheeled_class.id == 0) {
    return false;
  }
  std::uintptr_t cls = 0;
  if (!Read(context.core, vehicle + kObjectClassOffset, cls) || cls == 0) {
    return false;
  }
  for (std::uint32_t depth = 0; depth < 64 && cls != 0; ++depth) {
    AnomalyGenerationHandleV1 handle{};
    if (!ObjectHandleFromPointer(context, cls, handle)) {
      return false;
    }
    if (handle.id == context.vehicle_wheeled_class.id &&
        handle.generation == context.vehicle_wheeled_class.generation) {
      return true;
    }
    std::uintptr_t parent = 0;
    if (!Read(context.core, cls + kUstructSuperStructOffset, parent) ||
        parent == 0) {
      break;
    }
    cls = parent;
  }
  return false;
}

// The old vehicle service returned the signed GetForwardSpeedKmH value. Read
// the same function from the vehicle's movement component so the camera can
// use exact speed instead of differentiating the player position. Resolution
// failures retry on a 2 s backoff; this is an object lookup, not a .text scan,
// and a transient CONFLICT must not disable exact speed for the session.
void QueryVehicleSpeed(Context &context, const std::uintptr_t vehicle,
                       VehiclePresence &presence) noexcept {
  if (vehicle == 0 || !Ue5ObjectSnapshotReady(context.objects) ||
      !Ue5ProcessEventInvokeReady(context.events)) {
    return;
  }
  if (context.vehicle_speed_object != vehicle) {
    context.vehicle_speed_object = vehicle;
    context.vehicle_movement_handle = {};
    context.next_vehicle_wheeled_class_retry = 0.0;
  }
  if (!context.vehicle_speed_wheeled &&
      context.elapsed >= context.next_vehicle_wheeled_class_retry) {
    context.vehicle_speed_wheeled = VehicleIsWheeled(context, vehicle);
    // A water vehicle stays false, but class/snapshot lookups can fail
    // transiently. Retry on the same backoff instead of disabling exact speed
    // for the whole session.
    context.next_vehicle_wheeled_class_retry =
        context.elapsed + kVehicleGetterRetrySeconds;
  }
  if (!context.vehicle_speed_wheeled) {
    return;
  }
  if (context.vehicle_speed_function.id == 0 &&
      context.elapsed >= context.next_vehicle_speed_function_retry) {
    context.next_vehicle_speed_function_retry =
        context.elapsed + kVehicleGetterRetrySeconds;
    for (const std::string_view path : kVehicleSpeedFunctionPaths) {
      AnomalyGenerationHandleV1 function{};
      if (context.objects
                  ->find_exact(context.objects->user, StringView(path),
                               &function)
                  .code == ANOMALY_STATUS_V1_OK &&
          function.id != 0) {
        context.vehicle_speed_function = function;
        break;
      }
    }
  }
  if (context.vehicle_speed_function.id == 0) {
    if (!context.vehicle_speed_diag_logged) {
      context.vehicle_speed_diag_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
          "nfs2015 chase cam: GetForwardSpeedKmH unavailable; speed stays "
          "position-derived");
    }
    return;
  }
  std::uintptr_t movement = 0;
  if (!Read(context.core, vehicle + kVehicleMovementComponentOffset,
            movement) ||
      movement == 0) {
    return;
  }
  if (context.vehicle_movement_handle.id == 0 &&
      !ObjectHandleFromPointer(context, movement,
                               context.vehicle_movement_handle)) {
    return;
  }
  alignas(8) std::array<std::uint8_t, 32> output{};
  const auto status = context.events->invoke(
      context.events->user, context.vehicle_movement_handle,
      context.vehicle_speed_function, output.data(), output.size());
  if (status.code != ANOMALY_STATUS_V1_OK) {
    // Re-resolve after a backoff; a transient CONFLICT or a stale receiver
    // handle should not disable exact speed for the rest of the session.
    context.vehicle_movement_handle = {};
    context.vehicle_speed_function = {};
    context.next_vehicle_speed_function_retry =
        context.elapsed + kVehicleGetterRetrySeconds;
    return;
  }
  float speed = 0.0F;
  std::memcpy(&speed, output.data(), sizeof(speed));
  if (!std::isfinite(speed)) {
    return;
  }
  presence.speed_known = true;
  presence.speed_fresh = true;
  presence.speed_kmh = static_cast<double>(speed);
  if (!context.vehicle_speed_active_logged) {
    context.vehicle_speed_active_logged = true;
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
        "nfs2015 chase cam: exact GetForwardSpeedKmH active");
  }
}

std::uint64_t ObjectRegistryGeneration(Context &context) noexcept {
  if (context.objects == nullptr || context.objects->generation == nullptr) {
    return 0;
  }
  return context.objects->generation(context.objects->user);
}

// ue5.objects bumps its generation when the registry is rebuilt (world change,
// streaming, or a slot replacement). Every cached handle belongs to the old
// registry and must not be handed to invoke() afterwards.
void ResetVehicleObjectCaches(Context &context) noexcept {
  context.burnout_probe = BurnoutProbeState{};
  context.burnout_signals = BurnoutSignals{};
  context.burnout_detection = BurnoutDetectionState{};
  // Restore the stock relative rotation while the old component pointers are
  // still the best available, then drop them. If the registry change already
  // destroyed the car the writes fail harmlessly and are simply forgotten.
  ReleaseWheelSteer(context);
  context.vehicle_getter_handles = {};
  context.next_vehicle_getter_retry = {};
  context.vehicle_query_known = false;
  context.vehicle_query_present = false;
  context.vehicle_query_object = 0;
  context.vehicle_query_speed_known = false;
  context.vehicle_query_speed_fresh = false;
  context.vehicle_query_speed_kmh = 0.0;
  // Keep the 20 Hz poll deadline and the camera's presence identity: a
  // ue5.objects generation change invalidates handles, not the vehicle or the
  // camera history. Resetting them here made route B/identity churn while
  // streaming.
  context.vehicle_speed_object = 0;
  context.vehicle_speed_function = {};
  context.vehicle_movement_handle = {};
  context.vehicle_wheeled_class = {};
  context.vehicle_speed_wheeled = false;
  context.next_vehicle_wheeled_class_retry = 0.0;
  context.next_vehicle_speed_function_retry = 0.0;
  context.vehicle_speed_diag_logged = false;
  context.vehicle_speed_active_logged = false;
}

bool RefreshVehicleObjectGeneration(Context &context) noexcept {
  const std::uint64_t generation = ObjectRegistryGeneration(context);
  if (generation == 0) {
    // Service temporarily unpublished: do not reset the 20 Hz poll deadline,
    // or presence would start calling get_object_handles every tick.
    return false;
  }
  if (generation == context.vehicle_objects_generation) {
    return true;
  }
  ResetVehicleObjectCaches(context);
  context.vehicle_objects_generation = generation;
  return true;
}

VehiclePresence QueryVehiclePresence(Context &context) noexcept {
  VehiclePresence result{};
  if (!PlayerHandlesReady(context.player) ||
      !Ue5ObjectsReady(context.objects) ||
      !Ue5ProcessEventInvokeReady(context.events)) {
    return result;
  }
  if (context.elapsed < context.next_vehicle_query_time) {
    return {context.vehicle_query_known, context.vehicle_query_present,
            context.vehicle_query_object, context.vehicle_query_speed_known,
            false, context.vehicle_query_speed_kmh};
  }
  context.next_vehicle_query_time =
      context.elapsed + kVehicleQueryIntervalSeconds;
  context.vehicle_query_speed_fresh = false;

  AnomalyGenerationHandleV1 pawn{};
  AnomalyGenerationHandleV1 controller{};
  if (context.player->get_object_handles(context.player->user, &pawn,
                                         &controller).code !=
          ANOMALY_STATUS_V1_OK ||
      (pawn.id == 0 && controller.id == 0)) {
    context.vehicle_query_known = false;
    context.vehicle_query_present = false;
    context.vehicle_query_object = 0;
    context.vehicle_query_speed_known = false;
    context.vehicle_query_speed_fresh = false;
    context.vehicle_query_speed_kmh = 0.0;
    return result;
  }

  if (!RefreshVehicleObjectGeneration(context)) {
    return result;
  }

  const std::array<AnomalyGenerationHandleV1, 3> receivers{
      {controller, pawn, pawn}};
  bool executed = false;
  bool result_uncertain = false;
  for (std::size_t index = 0; index < receivers.size(); ++index) {
    if (receivers[index].id == 0) {
      continue;
    }
    if (context.vehicle_getter_handles[index].id == 0) {
      if (context.elapsed < context.next_vehicle_getter_retry[index]) {
        continue;
      }
      AnomalyGenerationHandleV1 function{};
      if (context.objects
                  ->find_exact(context.objects->user,
                               StringView(kVehicleGetterPaths[index]),
                               &function)
                  .code != ANOMALY_STATUS_V1_OK ||
          function.id == 0) {
        context.next_vehicle_getter_retry[index] =
            context.elapsed + kVehicleGetterRetrySeconds;
        continue;
      }
      context.vehicle_getter_handles[index] = function;
    }

    alignas(8) std::array<std::uint8_t, 8> output{};
    const auto status = context.events->invoke(
        context.events->user, receivers[index],
        context.vehicle_getter_handles[index], output.data(), output.size());
    if (status.code != ANOMALY_STATUS_V1_OK) {
      if (status.code == ANOMALY_STATUS_V1_NOT_FOUND) {
        context.vehicle_getter_handles[index] = {};
        context.next_vehicle_getter_retry[index] =
            context.elapsed + kVehicleGetterRetrySeconds;
      }
      continue;
    }
    executed = true;
    std::uintptr_t object = 0;
    std::memcpy(&object, output.data(), sizeof(object));
    if (object == 0) {
      continue;
    }
    std::uintptr_t readable = 0;
    if (!Read(context.core, object, readable)) {
      // The call dispatched but its result is not a live UObject. Do not turn
      // that into a confident "not driving".
      result_uncertain = true;
      continue;
    }
    result.known = true;
    result.present = true;
    result.object = object;
    break;
  }
  if (!result.present) {
    result.known = executed && !result_uncertain;
  }
  if (result.present) {
    QueryVehicleSpeed(context, result.object, result);
  }
  if (!result.present || !result.known) {
    result.speed_known = false;
    result.speed_fresh = false;
    result.speed_kmh = 0.0;
  }
  context.vehicle_query_known = result.known;
  context.vehicle_query_present = result.present;
  context.vehicle_query_object = result.object;
  context.vehicle_query_speed_known = result.speed_known;
  context.vehicle_query_speed_fresh = result.speed_fresh;
  context.vehicle_query_speed_kmh = result.speed_kmh;
  return result;
}

}  // namespace nfs2015_chase_cam::detail
