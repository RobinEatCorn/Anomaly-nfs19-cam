// The burnout probe and the burnout signal policy: invoke the reflected burnout
// getters through the object registry, read their outputs, and turn the latest
// facts into one CarSample::burnout on the Game tick.
#include "internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>

namespace nfs2015_chase_cam::detail {

constexpr double kBurnoutProbeDirectIntervalSeconds = 0.05;
constexpr double kBurnoutProbeWheelIntervalSeconds = 0.25;
constexpr double kBurnoutProbeRuntimeIntervalSeconds = 0.25;
constexpr double kBurnoutProbeRuntimeSignalIntervalSeconds = 0.10;

bool ReadBurnoutBool(const BurnoutProbeResult &result,
                     const std::size_t offset, bool &value) noexcept {
  if (!result.has_output || offset >= result.output_size) {
    return false;
  }
  const std::uint8_t raw = result.output[offset];
  if (raw > 1U) {
    return false;
  }
  value = raw != 0U;
  return true;
}

bool ReadBurnoutFloat(const BurnoutProbeResult &result,
                      const std::size_t offset, double &value) noexcept {
  if (!result.has_output || offset + sizeof(float) > result.output_size) {
    return false;
  }
  float raw = 0.0F;
  std::memcpy(&raw, result.output + offset, sizeof(raw));
  if (!std::isfinite(raw) || std::fabs(raw) > 1.0e6F) {
    return false;
  }
  value = static_cast<double>(raw);
  return true;
}

bool ReadBurnoutInt(const BurnoutProbeResult &result,
                    const std::size_t offset, std::int32_t &value) noexcept {
  if (!result.has_output || offset + sizeof(std::int32_t) > result.output_size) {
    return false;
  }
  std::int32_t raw = 0;
  std::memcpy(&raw, result.output + offset, sizeof(raw));
  value = raw;
  return true;
}

bool ReadBurnoutPointer(const BurnoutProbeResult &result,
                        const std::size_t offset,
                        std::uintptr_t &value) noexcept {
  if (!result.has_output || offset + sizeof(std::uintptr_t) > result.output_size) {
    return false;
  }
  std::uintptr_t raw = 0;
  std::memcpy(&raw, result.output + offset, sizeof(raw));
  value = raw;
  return raw != 0;
}

// GetWheelIsBurnout/Rep return at offset 4 because the reflected parameter
// layout puts the one-byte wheel index first. Fall back to offset 0 so the
// probe still reports a value if a future build reorders the parameters.
bool ReadBurnoutWheelBool(const BurnoutProbeResult &result,
                          bool &value) noexcept {
  if (ReadBurnoutBool(result, 4, value)) {
    return true;
  }
  return ReadBurnoutBool(result, 0, value);
}

bool IsDirectBurnoutProbe(const BurnoutProbeIndex index) noexcept {
  switch (index) {
    case kBurnoutProbeIsBurnout:
    case kBurnoutProbeThrottle:
    case kBurnoutProbeBrake:
    case kBurnoutProbeSteering:
    case kBurnoutProbeSkidAngle:
    case kBurnoutProbePlayerThrottle:
      return true;
    default:
      return false;
  }
}

bool IsWheelBurnoutProbe(const BurnoutProbeIndex index) noexcept {
  return index == kBurnoutProbeWheelBurnout ||
         index == kBurnoutProbeWheelBurnoutRep ||
         index == kBurnoutProbeWheelAngularSpeed;
}

bool ResolveBurnoutFunction(Context &context, const BurnoutProbeIndex index,
                            BurnoutProbeResult &result) noexcept {
  if (result.function.id != 0) {
    return true;
  }
  if (context.elapsed < result.next_retry) {
    return false;
  }
  result.next_retry = context.elapsed + kVehicleGetterRetrySeconds;
  const auto &spec = kBurnoutProbeSpecs[static_cast<std::size_t>(index)];
  AnomalyGenerationHandleV1 function{};
  const auto status = context.objects->find_exact(
      context.objects->user, StringView(spec.path), &function);
  if (status.code == ANOMALY_STATUS_V1_OK && function.id != 0) {
    result.function = function;
    result.status = kBurnoutProbeResolved;
    char message[256]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: burnout probe %s resolved: %s",
                  spec.label, spec.path);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    return true;
  }
  if (!result.logged) {
    result.logged = true;
    char message[256]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: burnout probe %s unresolved (%u): %s",
                  spec.label, status.code, spec.path);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
  }
  return false;
}

bool InvokeBurnoutFunction(Context &context, const BurnoutProbeIndex index,
                           const AnomalyGenerationHandleV1 receiver,
                           const int wheel_index,
                           BurnoutProbeResult &result) noexcept {
  if (receiver.id == 0 || result.function.id == 0) {
    return false;
  }
  const auto &spec = kBurnoutProbeSpecs[static_cast<std::size_t>(index)];
  alignas(16) std::array<std::uint8_t, 32> buffer{};
  if (spec.input_size == 1) {
    buffer[0] = static_cast<std::uint8_t>(wheel_index);
  } else if (spec.input_size == 4) {
    const std::int32_t value = static_cast<std::int32_t>(wheel_index);
    std::memcpy(buffer.data(), &value, sizeof(value));
  }
  const auto status = context.events->invoke(
      context.events->user, receiver, result.function, buffer.data(),
      buffer.size());
  result.last_code = status.code;
  if (status.code != ANOMALY_STATUS_V1_OK) {
    ++result.failures;
    if (status.code == ANOMALY_STATUS_V1_BUFFER_TOO_SMALL) {
      result.status = kBurnoutProbeTooSmall;
    } else if (status.code == ANOMALY_STATUS_V1_NOT_FOUND ||
               status.code == ANOMALY_STATUS_V1_FAILED) {
      result.function = {};
      result.status = kBurnoutProbeUnresolved;
      result.next_retry = context.elapsed + kVehicleGetterRetrySeconds;
    }
    if (result.failures == 1U) {
      char message[256]{};
      std::snprintf(message, sizeof(message),
                    "nfs2015 chase cam: burnout probe %s invoke failed "
                    "code=%u receiver=%llx function=%llx",
                    spec.label, status.code,
                    static_cast<unsigned long long>(receiver.id),
                    static_cast<unsigned long long>(result.function.id));
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, message);
    }
    return false;
  }
  ++result.calls;
  result.status = kBurnoutProbeOk;
  result.has_output = true;
  result.output_size = static_cast<std::uint16_t>(buffer.size());
  std::memcpy(result.output, buffer.data(),
              (std::min)(buffer.size(), sizeof(result.output)));
  return true;
}

AnomalyGenerationHandleV1 BurnoutReceiverHandle(
    Context &context, const BurnoutProbeReceiver receiver) noexcept {
  BurnoutProbeState &probe = context.burnout_probe;
  switch (receiver) {
    case kBurnoutReceiverVehicle:
      return probe.vehicle_handle;
    case kBurnoutReceiverMovement:
      return probe.movement_handle;
    case kBurnoutReceiverRuntimeInfo:
      return probe.runtime_info_handle;
    default:
      return {};
  }
}

void UpdateBurnoutProbe(Context &context, const VehiclePresence &vehicle,
                        const double speed_mps,
                        const double forward_speed) noexcept {
  BurnoutProbeState &probe = context.burnout_probe;
  if (!vehicle.present || vehicle.object == 0 ||
      !Ue5ObjectSnapshotReady(context.objects) ||
      !Ue5ProcessEventInvokeReady(context.events)) {
    probe = BurnoutProbeState{};
    context.burnout_signals = BurnoutSignals{};
    context.burnout_detection = BurnoutDetectionState{};
    return;
  }
  if (!RefreshVehicleObjectGeneration(context)) {
    return;
  }
  if (probe.vehicle_object != vehicle.object) {
    probe = BurnoutProbeState{};
    probe.vehicle_object = vehicle.object;
    context.burnout_signals = BurnoutSignals{};
    context.burnout_detection = BurnoutDetectionState{};
  }
  if (probe.summary_due <= 0.0) {
    probe.summary_due = context.elapsed + 2.0;
  }
  if (probe.vehicle_handle.id == 0 &&
      !ObjectHandleFromPointer(context, vehicle.object,
                               probe.vehicle_handle)) {
    return;
  }

  if (probe.movement_object == 0) {
    std::uintptr_t movement = 0;
    if (Read(context.core, vehicle.object + kVehicleMovementComponentOffset,
             movement) &&
        movement != 0) {
      probe.movement_object = movement;
    }
  }
  if (probe.movement_handle.id == 0 && probe.movement_object != 0 &&
      !ObjectHandleFromPointer(context, probe.movement_object,
                               probe.movement_handle)) {
    probe.movement_object = 0;
  }

  // GetCurrentRuntimeInfo returns the UObject that owns GetInputInfo and
  // GetWheelCount. Resolve it through the interface function; the runtime
  // object is not the vehicle actor.
  if (context.elapsed >= probe.next_runtime) {
    probe.next_runtime =
        context.elapsed + kBurnoutProbeRuntimeIntervalSeconds;
    auto &runtime = probe.results[kBurnoutProbeRuntimeInfo];
    if (probe.vehicle_handle.id != 0 &&
        ResolveBurnoutFunction(context, kBurnoutProbeRuntimeInfo, runtime) &&
        InvokeBurnoutFunction(context, kBurnoutProbeRuntimeInfo,
                              probe.vehicle_handle, 0, runtime)) {
      std::uintptr_t runtime_object = 0;
      if (ReadBurnoutPointer(runtime, 0, runtime_object)) {
        if (probe.runtime_info_object != runtime_object) {
          probe.runtime_info_object = runtime_object;
          probe.runtime_info_handle = {};
        }
        if (probe.runtime_info_handle.id == 0) {
          static_cast<void>(ObjectHandleFromPointer(
              context, runtime_object, probe.runtime_info_handle));
        }
      }
    }
  }

  // Direct vehicle/movement getters at 20 Hz.
  if (context.elapsed >= probe.next_direct) {
    probe.next_direct =
        context.elapsed + kBurnoutProbeDirectIntervalSeconds;
    for (std::size_t raw_index = 0; raw_index < kBurnoutProbeCount;
         ++raw_index) {
      const auto index = static_cast<BurnoutProbeIndex>(raw_index);
      if (!IsDirectBurnoutProbe(index)) {
        continue;
      }
      auto &result = probe.results[raw_index];
      if (!ResolveBurnoutFunction(context, index, result)) {
        continue;
      }
      const auto receiver = BurnoutReceiverHandle(
          context, kBurnoutProbeSpecs[raw_index].receiver);
      if (receiver.id == 0) {
        continue;
      }
      static_cast<void>(
          InvokeBurnoutFunction(context, index, receiver, 0, result));
    }
  }

  // Runtime-owned getters at 10 Hz.
  if (probe.runtime_info_handle.id != 0 &&
      context.elapsed >= probe.next_runtime_signals) {
    probe.next_runtime_signals =
        context.elapsed + kBurnoutProbeRuntimeSignalIntervalSeconds;
    constexpr BurnoutProbeIndex kRuntimeSignals[] = {
        kBurnoutProbeWheelCount, kBurnoutProbeInputInfo};
    for (const BurnoutProbeIndex index : kRuntimeSignals) {
      const auto slot = static_cast<std::size_t>(index);
      auto &result = probe.results[slot];
      if (!ResolveBurnoutFunction(context, index, result)) {
        continue;
      }
      static_cast<void>(InvokeBurnoutFunction(
          context, index, probe.runtime_info_handle, 0, result));
    }
  }

  // Per-wheel getters at 4 Hz. Each wheel call is a separate output buffer, so
  // interpret it immediately and keep the canonical result only for the
  // cached function handle and counters.
  if (context.elapsed >= probe.next_wheel) {
    probe.next_wheel = context.elapsed + kBurnoutProbeWheelIntervalSeconds;
    const std::int32_t count = probe.wheel_count_known ? probe.wheel_count : 4;
    const std::size_t wheel_limit = static_cast<std::size_t>(
        std::clamp<std::int32_t>(count, 1, kBurnoutProbeMaxWheels));
    for (std::size_t raw_index = 0; raw_index < kBurnoutProbeCount;
         ++raw_index) {
      const auto index = static_cast<BurnoutProbeIndex>(raw_index);
      if (!IsWheelBurnoutProbe(index)) {
        continue;
      }
      auto &canonical = probe.results[raw_index];
      if (!ResolveBurnoutFunction(context, index, canonical)) {
        continue;
      }
      const auto receiver = BurnoutReceiverHandle(
          context, kBurnoutProbeSpecs[raw_index].receiver);
      if (receiver.id == 0) {
        continue;
      }
      for (std::size_t wheel = 0; wheel < wheel_limit; ++wheel) {
        BurnoutProbeResult call = canonical;
        if (!InvokeBurnoutFunction(context, index, receiver,
                                   static_cast<int>(wheel), call)) {
          continue;
        }
        if (index == kBurnoutProbeWheelBurnout) {
          bool value = false;
          if (ReadBurnoutWheelBool(call, value)) {
            probe.wheel_burnout_known[wheel] = true;
            probe.wheel_burnout[wheel] = value;
          }
        } else if (index == kBurnoutProbeWheelBurnoutRep) {
          bool value = false;
          if (ReadBurnoutWheelBool(call, value)) {
            probe.wheel_rep_known[wheel] = true;
            probe.wheel_rep[wheel] = value;
          }
        } else if (index == kBurnoutProbeWheelAngularSpeed) {
          double value = 0.0;
          if (ReadBurnoutFloat(call, 4, value) ||
              ReadBurnoutFloat(call, 0, value)) {
            probe.wheel_angular_known[wheel] = true;
            probe.wheel_angular[wheel] = value;
          }
        }
        canonical = call;
      }
    }
  }

  // Interpret the direct results. A transient invoke failure keeps the last
  // successful output; the diagnostic panel shows the failure counter.
  bool value = false;
  double numeric = 0.0;
  if (ReadBurnoutBool(probe.results[kBurnoutProbeIsBurnout], 0, value)) {
    probe.is_burnout_known = true;
    probe.is_burnout = value;
  } else {
    probe.is_burnout_known = false;
  }
  if (ReadBurnoutFloat(probe.results[kBurnoutProbeThrottle], 0, numeric)) {
    probe.throttle_known = true;
    probe.throttle = numeric;
  } else {
    probe.throttle_known = false;
  }
  if (ReadBurnoutFloat(probe.results[kBurnoutProbeBrake], 0, numeric)) {
    probe.brake_known = true;
    probe.brake = numeric;
  } else {
    probe.brake_known = false;
  }
  if (ReadBurnoutFloat(probe.results[kBurnoutProbeSteering], 0, numeric)) {
    probe.steering_known = true;
    probe.steering = numeric;
  } else {
    probe.steering_known = false;
  }
  if (ReadBurnoutFloat(probe.results[kBurnoutProbeSkidAngle], 0, numeric)) {
    probe.skid_known = true;
    probe.skid_angle = numeric;
  } else {
    probe.skid_known = false;
  }
  if (ReadBurnoutFloat(probe.results[kBurnoutProbePlayerThrottle], 0,
                       numeric)) {
    probe.player_throttle_known = true;
    probe.player_throttle = numeric;
  } else {
    probe.player_throttle_known = false;
  }
  if (ReadBurnoutInt(probe.results[kBurnoutProbeWheelCount], 0,
                     probe.wheel_count)) {
    probe.wheel_count_known = true;
  } else {
    probe.wheel_count_known = false;
  }

  auto &input = probe.results[kBurnoutProbeInputInfo];
  if (input.has_output && input.output_size != 0) {
    probe.input_raw_size =
        (std::min)(static_cast<std::size_t>(input.output_size),
                   sizeof(probe.input_raw));
    std::memcpy(probe.input_raw, input.output, probe.input_raw_size);
    // Reflection confirms the 16-byte layout: ThrottleInput float @0,
    // SteeringInput float @4, BrakeInput float @8, bHandbrakeInput bool @12.
    // Byte 0 is therefore the low byte of a float, not a handbrake candidate;
    // keep it visible as a legacy cross-check, but resolve the handbrake from
    // byte 12 only. The old byte-0 fallback is intentionally gone.
    probe.handbrake12_known =
        ReadBurnoutBool(input, 12, probe.handbrake12);
    probe.handbrake0_known = ReadBurnoutBool(input, 0, probe.handbrake0);
    if (probe.handbrake12_known) {
      probe.handbrake_known = true;
      probe.handbrake = probe.handbrake12;
    } else {
      probe.handbrake_known = false;
    }
    const bool changed =
        probe.input_raw_size != probe.last_input_raw_size ||
        std::memcmp(probe.input_raw, probe.last_input_raw,
                    probe.input_raw_size) != 0;
    if (changed && context.elapsed - probe.last_input_log >= 0.5) {
      probe.last_input_log = context.elapsed;
      char message[256]{};
      int used = std::snprintf(message, sizeof(message),
                               "nfs2015 chase cam: GetInputInfo raw=");
      for (std::size_t byte = 0;
           byte < probe.input_raw_size && used > 0 &&
           static_cast<std::size_t>(used) + 4 < sizeof(message);
           ++byte) {
        used += std::snprintf(message + used,
                              sizeof(message) - static_cast<std::size_t>(used),
                              "%02x ", probe.input_raw[byte]);
      }
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
    }
    std::memcpy(probe.last_input_raw, probe.input_raw,
                probe.input_raw_size);
    probe.last_input_raw_size = probe.input_raw_size;
  } else {
    probe.handbrake_known = false;
    probe.handbrake12_known = false;
    probe.handbrake0_known = false;
    probe.input_raw_size = 0;
  }

  std::uint32_t wheel_mask = 0;
  std::uint32_t rep_mask = 0;
  for (std::size_t wheel = 0; wheel < kBurnoutProbeMaxWheels; ++wheel) {
    if (probe.wheel_burnout_known[wheel] && probe.wheel_burnout[wheel]) {
      wheel_mask |= 1U << wheel;
    }
    if (probe.wheel_rep_known[wheel] && probe.wheel_rep[wheel]) {
      rep_mask |= 1U << wheel;
    }
  }
  const bool burnout_state = probe.is_burnout_known && probe.is_burnout;
  const bool handbrake_state = probe.handbrake_known && probe.handbrake;
  const bool handbrake12_state =
      probe.handbrake12_known && probe.handbrake12;
  const bool handbrake0_state = probe.handbrake0_known && probe.handbrake0;
  if (!probe.have_last_state) {
    probe.have_last_state = true;
  } else if (burnout_state != probe.last_burnout ||
             handbrake_state != probe.last_handbrake ||
             handbrake12_state != probe.last_handbrake12 ||
             handbrake0_state != probe.last_handbrake0 ||
             wheel_mask != probe.last_wheel_mask) {
    char message[320]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: burnout state=%d HB12=%d B0=%d "
                  "resolvedHB=%d throttle=%.3f brake=%.3f steer=%+.3f "
                  "wheels=%02x speed=%.2f fwd=%.2f",
                  burnout_state ? 1 : 0, handbrake12_state ? 1 : 0,
                  handbrake0_state ? 1 : 0, handbrake_state ? 1 : 0,
                  probe.throttle_known ? probe.throttle : -1.0,
                  probe.brake_known ? probe.brake : -1.0,
                  probe.steering_known ? probe.steering : 0.0,
                  wheel_mask, speed_mps, forward_speed);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }
  probe.last_burnout = burnout_state;
  probe.last_handbrake = handbrake_state;
  probe.last_handbrake12 = handbrake12_state;
  probe.last_handbrake0 = handbrake0_state;
  probe.last_wheel_mask = wheel_mask;

  BurnoutSignals signals;
  signals.direct_known = probe.is_burnout_known;
  signals.direct = probe.is_burnout;
  signals.handbrake_known = probe.handbrake_known;
  signals.handbrake = probe.handbrake;
  signals.throttle_known = probe.throttle_known;
  signals.throttle = probe.throttle;
  signals.player_throttle_known = probe.player_throttle_known;
  signals.player_throttle = probe.player_throttle;
  signals.brake_known = probe.brake_known;
  signals.brake = probe.brake;
  signals.steering_known = probe.steering_known;
  signals.steering = probe.steering;
  // Wheel indices 2 and 3 are the rear pair on this car (Phase 0). The exact
  // left/right mapping is intentionally not asserted here: both rear wheels
  // report burnout together, and the shot falls back to steering when the two
  // flags agree. A future GetWheelInfo::SideType query can make this exact.
  signals.left_known = probe.wheel_burnout_known[2];
  signals.left = probe.wheel_burnout[2];
  signals.right_known = probe.wheel_burnout_known[3];
  signals.right = probe.wheel_burnout[3];
  signals.known = signals.direct_known || signals.handbrake_known ||
                  signals.throttle_known || signals.player_throttle_known ||
                  signals.steering_known || signals.left_known ||
                  signals.right_known;
  context.burnout_signals = signals;

  // While the burnout is active, sample angular speed and speed every 0.5 s.
  // This is the data Phase 1 needs for shake/intensity, and it is cheap enough
  // to keep alongside the 4 Hz wheel probe.
  if (!burnout_state) {
    probe.next_active_log = 0.0;
  } else if (context.elapsed >= probe.next_active_log) {
    probe.next_active_log = context.elapsed + 0.5;
    char active[512]{};
    std::snprintf(
        active, sizeof(active),
        "nfs2015 chase cam: burnout active throttle=%.3f brake=%.3f "
        "steer=%+.3f playerThr=%.3f skid=%.1f HB12=%d wheels=%02x "
        "rep=%02x angular=%.1f/%.1f/%.1f/%.1f speed=%.2f fwd=%.2f",
        probe.throttle_known ? probe.throttle : -1.0,
        probe.brake_known ? probe.brake : -1.0,
        probe.steering_known ? probe.steering : 0.0,
        probe.player_throttle_known ? probe.player_throttle : -1.0,
        probe.skid_known ? probe.skid_angle : 0.0,
        handbrake12_state ? 1 : 0, wheel_mask, rep_mask,
        probe.wheel_angular_known[0] ? probe.wheel_angular[0] : 0.0,
        probe.wheel_angular_known[1] ? probe.wheel_angular[1] : 0.0,
        probe.wheel_angular_known[2] ? probe.wheel_angular[2] : 0.0,
        probe.wheel_angular_known[3] ? probe.wheel_angular[3] : 0.0,
        speed_mps, forward_speed);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, active);
  }

  if (!probe.summary_logged && context.elapsed >= probe.summary_due) {
    probe.summary_logged = true;
    std::uint32_t resolved = 0;
    std::uint32_t ok = 0;
    std::uint32_t failed = 0;
    for (const BurnoutProbeResult &result : probe.results) {
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
    char message[640]{};
    std::snprintf(message, sizeof(message),
                  "nfs2015 chase cam: burnout probe resolved=%u/%zu ok=%u "
                  "failed=%u isBurnout=%d HB12=%d B0=%d throttle=%.3f "
                  "brake=%.3f steer=%+.3f playerThr=%.3f skid=%.1f "
                  "wheels=%d burnoutMask=%02x repMask=%02x "
                  "angular=%.1f/%.1f/%.1f/%.1f",
                  resolved, kBurnoutProbeCount, ok, failed,
                  burnout_state ? 1 : 0, handbrake12_state ? 1 : 0,
                  handbrake0_state ? 1 : 0,
                  probe.throttle_known ? probe.throttle : -1.0,
                  probe.brake_known ? probe.brake : -1.0,
                  probe.steering_known ? probe.steering : 0.0,
                  probe.player_throttle_known ? probe.player_throttle : -1.0,
                  probe.skid_known ? probe.skid_angle : 0.0,
                  probe.wheel_count_known ? probe.wheel_count : -1,
                  wheel_mask, rep_mask,
                  probe.wheel_angular_known[0] ? probe.wheel_angular[0] : 0.0,
                  probe.wheel_angular_known[1] ? probe.wheel_angular[1] : 0.0,
                  probe.wheel_angular_known[2] ? probe.wheel_angular[2] : 0.0,
                  probe.wheel_angular_known[3] ? probe.wheel_angular[3] : 0.0);
    std::string summary(message);
    summary.append(" input=");
    for (std::size_t byte = 0; byte < probe.input_raw_size; ++byte) {
      char hex[4]{};
      std::snprintf(hex, sizeof(hex), "%02x", probe.input_raw[byte]);
      summary.append(hex);
      if (byte + 1 < probe.input_raw_size) {
        summary.push_back(' ');
      }
    }
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, summary);
    std::string details;
    details.reserve(512);
    for (std::size_t index = 0; index < kBurnoutProbeCount; ++index) {
      const auto &result = probe.results[index];
      if (!details.empty()) {
        details.push_back(' ');
      }
      details.append(kBurnoutProbeSpecs[index].label);
      details.push_back('=');
      details.append(std::to_string(result.status));
      details.push_back('/');
      details.append(std::to_string(result.last_code));
    }
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
        std::string_view(details));
  }
}

constexpr double kBurnoutEntrySpeedMps = 1.0;
constexpr double kBurnoutHoldSpeedMps = 3.0;
constexpr double kBurnoutHoldSeconds = 0.28;
constexpr double kBurnoutIntensityAttack = 12.0;
constexpr double kBurnoutIntensityRelease = 8.0;
constexpr double kBurnoutFallbackThrottle = 0.15;

// Turn the latest reflected facts into one CarSample::burnout. This is the
// only place the Game tick applies policy: the ground/speed gates, the short
// hold that bridges the 20 Hz poll and the intensity ramp. The render-side
// shot state machine still owns the entry/exit delays and the side latch.
[[nodiscard]] nfs2015_chase::BurnoutSample UpdateBurnoutDetection(
    Context &context, const bool on_ground, const double speed_mps,
    const double forward_speed, const double dt) noexcept {
  BurnoutDetectionState &state = context.burnout_detection;
  const BurnoutSignals &signals = context.burnout_signals;

  const bool direct = signals.direct_known && signals.direct;
  const bool throttle_known =
      signals.throttle_known || signals.player_throttle_known;
  const double throttle =
      (std::max)(signals.throttle_known ? signals.throttle : 0.0,
                 signals.player_throttle_known ? signals.player_throttle : 0.0);
  const bool fallback = signals.handbrake_known && signals.handbrake &&
                        throttle_known && throttle > kBurnoutFallbackThrottle;
  const bool direction_ok = std::isfinite(forward_speed) &&
                            forward_speed > -0.5;
  const bool safe = on_ground && direction_ok && std::isfinite(speed_mps);
  const double absolute_speed =
      safe ? std::fabs(speed_mps) : std::numeric_limits<double>::infinity();
  const bool condition = safe && (direct || fallback);
  const bool raw_active = condition && absolute_speed < kBurnoutEntrySpeedMps;
  const bool continue_active =
      condition && state.active && absolute_speed < kBurnoutHoldSpeedMps;
  const bool hold_active =
      state.active && safe && absolute_speed < kBurnoutHoldSpeedMps &&
      context.elapsed < state.hold_until;
  const bool force_exit =
      !safe || absolute_speed >= kBurnoutHoldSpeedMps;
  const bool active =
      !force_exit && (raw_active || continue_active || hold_active);

  if (raw_active || continue_active) {
    state.hold_until = context.elapsed + kBurnoutHoldSeconds;
  }
  if (force_exit) {
    state.hold_until = 0.0;
  }
  state.raw_active = raw_active;
  state.direct = direct;
  state.fallback = fallback;
  state.active = active;

  const double target =
      active && condition
          ? (throttle_known ? nfs2015_chase::Clamp(throttle, 0.0, 1.0)
                            : (direct ? 1.0 : 0.0))
          : 0.0;
  const double rate = target > state.intensity ? kBurnoutIntensityAttack
                                               : kBurnoutIntensityRelease;
  state.intensity =
      nfs2015_chase::Clamp(nfs2015_chase::Damp(state.intensity, target, rate,
                                               dt),
                           0.0, 1.0);

  nfs2015_chase::BurnoutSample sample;
  sample.active = active;
  sample.left = signals.left_known && signals.left;
  sample.right = signals.right_known && signals.right;
  sample.intensity = state.intensity;
  sample.throttle = throttle_known ? throttle : 0.0;
  sample.handbrake = signals.handbrake_known && signals.handbrake ? 1.0 : 0.0;
  sample.steering = signals.steering_known
                        ? nfs2015_chase::Clamp(signals.steering, -1.0, 1.0)
                        : 0.0;
  sample.speed_mps = std::isfinite(forward_speed) ? forward_speed : 0.0;

  if (!state.have_last_log || state.last_log_active != sample.active ||
      state.last_log_direct != direct ||
      state.last_log_fallback != fallback) {
    state.have_last_log = true;
    state.last_log_active = sample.active;
    state.last_log_direct = direct;
    state.last_log_fallback = fallback;
    char message[320]{};
    std::snprintf(
        message, sizeof(message),
        "nfs2015 chase cam: burnout sample active=%d direct=%d fallback=%d "
        "left=%d right=%d intensity=%.2f throttle=%.2f HB=%.0f steer=%+.2f "
        "speed=%+.2f",
        sample.active ? 1 : 0, direct ? 1 : 0, fallback ? 1 : 0,
        sample.left ? 1 : 0, sample.right ? 1 : 0, sample.intensity,
        sample.throttle, sample.handbrake, sample.steering, sample.speed_mps);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  }
  return sample;
}

}  // namespace nfs2015_chase_cam::detail
