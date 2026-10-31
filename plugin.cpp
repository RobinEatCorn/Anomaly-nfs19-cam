// NFS 2015 Chase Cam — third-party Anomaly plugin (ABI v1).
//
// This package is built against the installed Anomaly SDK only: it includes no
// host-internal headers and carries no JSON library of its own. Settings are
// persisted through `anomaly.config` and decoded through `anomaly.json`.
//
// Feeds the clean-room NFS 2015 chase-camera model in `chase_cam.hpp` from
// Anomaly's NTE snapshots, then replaces the view point produced by the game's
// camera manager. The game's own third-person camera is used as the car-yaw
// sensor; the player snapshot supplies the car position and the vehicle
// snapshot supplies the speed. No GObjects scan, no ProcessEvent call.
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

#include "chase_cam.hpp"
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
#include <limits>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <system_error>

namespace {

using anomaly::sdk::StringView;

constexpr std::string_view kPluginId =
    "io.github.robineatcorn.nfs2015-chase-cam";
constexpr std::string_view kSettingsSchemaId = "nfs2015-chase-cam-settings-v1";
constexpr std::uint32_t kSettingsSchemaVersion = 1;
constexpr std::size_t kMaximumSettingsBytes = 2048;
constexpr std::uint64_t kSettingsSaveDelayMilliseconds = 500;
constexpr std::uint32_t kToggleKey = VK_F7;

constexpr double kDegreesToRadians =
    3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees =
    180.0 / 3.14159265358979323846;
constexpr double kDefaultCameraScale = 1.0;
constexpr double kMinimumCameraScale = 0.6;
constexpr double kMaximumCameraScale = 1.6;
constexpr double kDefaultOriginOffsetCm = 0.0;
constexpr double kMinimumOriginOffsetCm = -150.0;
constexpr double kMaximumOriginOffsetCm = 150.0;
constexpr double kDefaultAspectRatio = 16.0 / 9.0;
constexpr double kMinimumAspectRatio = 1.0;
constexpr double kMaximumAspectRatio = 3.5;
constexpr double kDefaultSpeedDistanceScale = 1.0;
constexpr double kMinimumSpeedDistanceScale = 0.0;
constexpr double kMaximumSpeedDistanceScale = 2.0;
constexpr double kDefaultRollScale = 1.0;
constexpr double kMinimumRollScale = 0.0;
constexpr double kMaximumRollScale = 2.0;
constexpr double kDefaultPitchScale = 1.0;
constexpr double kMinimumPitchScale = 0.0;
constexpr double kMaximumPitchScale = 2.0;
constexpr double kDefaultShakeScale = 1.0;
constexpr double kMinimumShakeScale = 0.0;
constexpr double kMaximumShakeScale = 2.0;
// Braking pitch needs a speed-based signal, not the position-derived
// acceleration: coasting drag, curbs, and sample noise all look like braking
// in the latter. A dead zone plus a short hold rejects those single-tick dips.
constexpr double kBrakingPitchDeadzoneMps2 = 1.2;
constexpr double kBrakingPitchFullMps2 = 5.0;
constexpr double kBrakingDecelerationSmoothing = 8.0;
constexpr double kBrakingInputSmoothing = 12.0;
constexpr double kBrakingHoldSeconds = 0.12;
constexpr std::uint64_t kResetFeedbackMilliseconds = 1500;
constexpr double kFallbackPlayerCenterHeightCm = 90.0;
constexpr double kMinimumVehicleSpeedMps = 6.0;
constexpr double kMaximumPlausibleSpeedCmps = 30000.0;
constexpr double kMaximumPlausibleAccelerationCmps2 = 5000.0;
constexpr double kMaximumTeleportStepCm = 5000.0;
constexpr double kVelocitySmoothing = 18.0;
constexpr double kAccelerationSmoothing = 10.0;
constexpr double kYawRateSmoothing = 12.0;
constexpr double kMaximumYawRate = 3.0;
// The game camera can swing when its collision probe hits city geometry. That
// is not the car yaw, so cap how far the resolved body yaw may move per tick.
constexpr double kMaximumBodyYawRate = 6.0;  // rad/s
constexpr double kUpdateRateWindowSeconds = 0.5;
// The view is rendered from the last two Game-tick snapshots. Delay the pair
// by at most one tick so a 30 Hz city tick can still feed a 60 Hz renderer;
// the cap keeps a long hitch from making the camera trail too far behind.
constexpr double kMaximumViewInterpolationDelaySeconds = 0.25;
// A longer gap is a hitch/teleport, not a low update rate. Snap the history
// instead of blending across it (which would both trail and step at the end).
constexpr double kMaximumViewInterpolationGapSeconds = 0.25;
// The camera-to-car distance is the direct screen-size driver. Low-pass it so
// Game-tick sampling/spring phase noise cannot make the car breathe in frame.
constexpr double kViewDistanceSmoothing = 4.0;
constexpr double kDrivingGraceSeconds = 0.35;
// A reset, teleport, or retry can move the camera arbitrarily far in one tick.
// Do not interpolate across such a discontinuity. Keep this close to the
// teleport detector: at 20-30 Game ticks a fast car legitimately moves several
// metres per tick, and treating that as a teleport would disable interpolation
// exactly when it is most useful.
constexpr double kViewHistorySnapDistanceCm = 5000.0;
constexpr double kViewHistorySnapAngleDegrees = 60.0;
constexpr double kMaximumCameraYawDifference = 1.4;  // ~80 degrees
constexpr double kHookRetrySeconds = 2.0;
// The detour runs one engine tick after the model published. Shift the
// published eye by the game camera's movement since that publish so the car
// and the camera are rendered in the same frame. A larger step is a
// teleport/hitch, not car motion.
constexpr double kMaximumGameCameraSyncCm = 1000.0;

// The view-point getter of the game's CameraManager. The same signature the
// built-in Camera Tools plugin resolves; only one plugin can own the hook.
constexpr std::string_view kCameraViewPointPattern =
    "48 89 5C 24 08 57 48 83 EC 20 48 8B 01 49 8B F8 48 8B DA "
    "FF 90 A0 07 00 00 0F 10 00 0F 11 03 F2 0F 10 48 10 F2 0F "
    "11 4B 10 0F 10 40 18 48 8B 5C 24 30 0F 11 07 F2 0F 10 48 "
    "28 F2 0F 11 4F 10 48 83 C4 20 5F C3";
constexpr std::uint32_t kCameraPovAccessorVtableOffset = 0x7A0;
constexpr std::uint32_t kCameraPovFovOffset = 0x30;

constexpr std::string_view kSettingsSchema = R"json(
{
  "type":"object",
  "additionalProperties":false,
  "required":["enabled","preset","cameraScale","originOffsetCm","aspectRatio","fovEnabled"],
  "properties":{
    "enabled":{"type":"boolean"},
    "preset":{"type":"integer","minimum":0,"maximum":1},
    "cameraScale":{"type":"number","minimum":0.6,"maximum":1.6},
    "originOffsetCm":{"type":"number","minimum":-150.0,"maximum":150.0},
    "aspectRatio":{"type":"number","minimum":1.0,"maximum":3.5},
    "fovEnabled":{"type":"boolean"},
    "speedDistanceScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "rollScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "pitchScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "shakeScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "rollOnlyInDrift":{"type":"boolean"}
  }
}
)json";

using CameraViewPointFn = void(ANOMALY_CALL *)(void *, double *, double *);

template <typename Struct, typename Field>
bool HasField(const Struct *value, const std::size_t offset) noexcept {
  return value != nullptr && value->struct_size >= offset + sizeof(Field);
}

AnomalyStatusV1 Status(const std::uint32_t code,
                       const std::string_view message = {}) noexcept {
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

bool InputReady(const AnomalyInputServiceV1 *service) noexcept {
  return HasField<AnomalyInputServiceV1,
                  decltype(AnomalyInputServiceV1::release_hotkey)>(
             service, offsetof(AnomalyInputServiceV1, release_hotkey)) &&
         service->register_hotkey != nullptr &&
         service->release_hotkey != nullptr;
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

bool PlayerReady(const AnomalyNtePlayerServiceV1 *service) noexcept {
  return service != nullptr && service->snapshot != nullptr;
}

bool VehicleReady(const AnomalyNteVehicleServiceV1 *service) noexcept {
  return service != nullptr && service->snapshot != nullptr;
}

template <typename Service>
bool ComponentBoundsReady(const Service *service) noexcept {
  return service != nullptr && service->component_bounds != nullptr;
}

bool SnapshotValid(const std::uint32_t flags) noexcept {
  // STALE only means the sampler has not refreshed this tick (the interval is
  // configurable); the last sample is still the best known player/vehicle
  // state. PARTIAL means the optional ESP/camera tail is missing, and the
  // fallback origin handles that. Neither makes the position unusable.
  return (flags & ANOMALY_NTE_SNAPSHOT_V1_VALID) != 0;
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

struct DisplayState final {
  bool started{};
  bool enabled{};
  bool hook_ready{};
  bool active{};
  bool driving{};
  bool player_valid{};
  bool vehicle_service_available{};
  std::uint32_t camera_base_source{};
  bool fov_enabled{};
  std::uint32_t preset{};
  double speed_kmh{};
  double slip_degrees{};
  double drift{};
  double fov_horizontal{};
  double camera_distance_m{};
  double camera_distance_raw_m{};
  double update_hz{};
  double view_output_hz{};
  double interpolation_ratio{};
  double camera_scale{kDefaultCameraScale};
  double origin_offset_cm{kDefaultOriginOffsetCm};
  double aspect_ratio{kDefaultAspectRatio};
  double speed_distance_scale{kDefaultSpeedDistanceScale};
  double roll_scale{kDefaultRollScale};
  double pitch_scale{kDefaultPitchScale};
  double shake_scale{kDefaultShakeScale};
  bool roll_only_in_drift{};
};

struct Context final {
  nfs2015_chase_cam::Localizer localizer;
  const AnomalyCoreServiceV1 *core{};
  const AnomalyConfigServiceV1 *config{};
  const AnomalyJsonServiceV1 *json{};
  const AnomalyInputServiceV1 *input{};
  const AnomalyUiServiceV1 *ui{};
  const AnomalySignatureServiceV1 *signature{};
  const AnomalyHookServiceV1 *hook{};
  const AnomalyNteSessionServiceV1 *session{};
  const AnomalyNtePlayerServiceV1 *player{};
  const AnomalyNteVehicleServiceV1 *vehicle{};
  const AnomalyNteEntitiesServiceV1 *entities{};
  const AnomalyNteActorsServiceV1 *actors{};

  AnomalyGenerationHandleV1 settings_schema{};
  AnomalyGenerationHandleV1 toggle_hotkey{};
  std::atomic<std::uint64_t> view_hook_id{};
  std::atomic<std::uint64_t> view_hook_generation{};

  // Settings: Render/hotkey write, Game reads.
  std::atomic_bool enabled{true};
  std::atomic<std::uint32_t> preset{0};
  std::atomic<double> camera_scale{kDefaultCameraScale};
  std::atomic<double> origin_offset_cm{kDefaultOriginOffsetCm};
  std::atomic<double> aspect_ratio{kDefaultAspectRatio};
  std::atomic<double> speed_distance_scale{kDefaultSpeedDistanceScale};
  std::atomic<double> roll_scale{kDefaultRollScale};
  std::atomic<double> pitch_scale{kDefaultPitchScale};
  std::atomic<double> shake_scale{kDefaultShakeScale};
  std::atomic_bool roll_only_in_drift{};
  std::atomic_bool fov_enabled{true};
  std::atomic<std::uint64_t> settings_revision{};
  std::atomic<std::uint64_t> persisted_settings_revision{};
  std::atomic<std::uint64_t> settings_changed_at{};

  // Hook and manager/POV state. `observed_manager` is written by the detour;
  // `resolved_manager` and `camera_pov` are written by the Game tick.
  std::atomic<std::uintptr_t> hook_target{};
  std::atomic_bool hook_ready{};
  std::atomic<std::uintptr_t> observed_manager{};
  std::uintptr_t resolved_manager{};
  std::atomic<std::uintptr_t> camera_pov{};
  std::atomic<std::uintptr_t> camera_pov_manager{};
  std::atomic<float> game_fov{};
  std::atomic_bool fov_restore{};
  std::atomic<double> observed_location[3]{};
  std::atomic<double> observed_rotation[3]{};
  std::atomic<std::uint64_t> observed_sequence{};
  std::atomic_bool observed_valid{};
  double next_hook_retry{};
  bool hook_failure_logged{};

  // Published view snapshot. The location fields are offsets from the car,
  // not absolute world positions: the detour adds the latest car position so
  // a Game-tick car transform step cannot change the camera-to-car distance.
  // Written under a seqlock: odd while a write is in progress, even when the
  // snapshot is complete.
  std::atomic<std::uint64_t> view_sequence{};
  std::atomic_bool view_active{};
  std::atomic_bool view_has_previous{};
  std::atomic<bool> view_fov_enabled{true};
  // `view_car_location` is the transform the car body is rendered at: the
  // VehicleMesh bounds center when entity reflection is available, otherwise
  // the pawn root. ESP bounds are only a model-height reference.
  std::atomic<double> view_car_location[3]{};
  // Game camera location at the time the published view was computed. The
  // detour adds the movement since then so the eye and car share a frame.
  std::atomic<double> view_game_camera_location[3]{};
  std::atomic_bool view_game_camera_valid{};
  std::atomic<double> view_update_hz{};
  std::atomic<double> view_location[3]{};
  std::atomic<double> view_rotation[3]{};
  std::atomic<float> view_fov{};
  std::atomic<std::int64_t> view_previous_time_ns{};
  std::atomic<std::int64_t> view_current_time_ns{};
  std::atomic<double> view_distance_cm{};
  std::atomic<double> view_previous_distance_cm{};
  std::atomic<double> view_previous_location[3]{};
  std::atomic<double> view_previous_rotation[3]{};
  std::atomic<float> view_previous_fov{};
  std::atomic<bool> view_previous_fov_enabled{};

  // Vehicle entity resolution cache. The vehicle service handle is an object
  // handle, not necessarily an entity-cache handle; look the vehicle up by its
  // entity id and reuse the frame index while the cache generation is stable.
  std::uint64_t vehicle_entity_id{};
  std::uint64_t vehicle_entity_frame_generation{};
  std::uint32_t vehicle_entity_index{};
  AnomalyGenerationHandleV1 vehicle_entity{};
  // Fallback scan result and its rate limit. The scan is only a last resort
  // when the vehicle service and the local player entity cannot name the car.
  AnomalyGenerationHandleV1 scanned_vehicle_entity{};
  double next_vehicle_scan_elapsed{};
  bool base_diagnostic_logged{};

  // Game-domain state.
  std::atomic_bool started{};
  double elapsed{};
  double update_rate_window{};
  std::uint32_t update_rate_frames{};
  double update_hz{};
  double view_output_hz{};
  double interpolation_ratio{};
  std::atomic<std::uint64_t> view_output_frames{};
  std::atomic<std::uint64_t> view_interpolated_frames{};
  nfs2015_chase::ChaseCam chase{nfs2015_chase::GamePreset()};
  std::uint32_t configured_preset{0};
  bool chase_initialized{};
  std::atomic_bool reset_requested{};
  std::atomic<std::uint64_t> reset_feedback_until{};
  std::atomic_bool hook_retry_requested{};
  AnomalyGenerationHandleV1 world{};
  AnomalyGenerationHandleV1 player_handle{};
  AnomalyGenerationHandleV1 vehicle_handle{};
  bool have_last_position{};
  bool have_last_yaw{};
  double last_vehicle_valid_elapsed{-1.0};
  double last_vehicle_speed_mps{-1.0};
  double last_vehicle_speed_time{};
  double brake_deceleration_mps2{};
  double brake_input{};
  double brake_hold_until{};
  double vehicle_acceleration_mps2{};
  double smoothed_view_distance_cm{};
  double raw_view_distance_cm{};
  bool have_view_distance{};
  std::uint64_t last_player_sequence{};
  double last_player_time{};
  double last_position_cm[3]{};
  double velocity_cmps[3]{};
  double acceleration_cmps2[3]{};
  double last_car_yaw{};
  double yaw_rate{};
  bool last_view_active{};
  bool last_fov_enabled{};
  std::uint32_t camera_base_source{};
  bool view_snap{true};

  // Game writes, Render copies.
  mutable std::mutex display_mutex;
  DisplayState display{};
};

std::atomic<Context *> g_active{};
std::atomic<CameraViewPointFn> g_original{};

void Log(const Context &context, const std::uint32_t level,
         const std::string_view message) noexcept {
  if (context.core == nullptr || context.core->log == nullptr) {
    return;
  }
  context.core->log(context.core->user, level, StringView(message));
}

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

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept;

bool EnsureHook(Context &context) noexcept {
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
    g_active.store(nullptr, std::memory_order_release);
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

void MarkSettingsDirty(Context &context) noexcept {
  context.settings_changed_at.store(GetTickCount64(),
                                    std::memory_order_release);
  context.settings_revision.fetch_add(1, std::memory_order_acq_rel);
}

bool SettingsValid(const std::uint32_t preset, const double camera_scale,
                   const double origin_offset_cm, const double aspect_ratio,
                   const double speed_distance_scale,
                   const double roll_scale,
                   const double pitch_scale,
                   const double shake_scale) noexcept {
  return std::isfinite(camera_scale) &&
         camera_scale >= kMinimumCameraScale &&
         camera_scale <= kMaximumCameraScale &&
         std::isfinite(origin_offset_cm) &&
         origin_offset_cm >= kMinimumOriginOffsetCm &&
         origin_offset_cm <= kMaximumOriginOffsetCm &&
         std::isfinite(aspect_ratio) && aspect_ratio >= kMinimumAspectRatio &&
         aspect_ratio <= kMaximumAspectRatio &&
         std::isfinite(speed_distance_scale) &&
         speed_distance_scale >= kMinimumSpeedDistanceScale &&
         speed_distance_scale <= kMaximumSpeedDistanceScale &&
         std::isfinite(roll_scale) && roll_scale >= kMinimumRollScale &&
         roll_scale <= kMaximumRollScale &&
         std::isfinite(pitch_scale) && pitch_scale >= kMinimumPitchScale &&
         pitch_scale <= kMaximumPitchScale &&
         std::isfinite(shake_scale) && shake_scale >= kMinimumShakeScale &&
         shake_scale <= kMaximumShakeScale && preset <= 1U;
}

bool AppendText(char *&cursor, char *const end,
                const std::string_view text) noexcept {
  if (text.size() > static_cast<std::size_t>(end - cursor)) {
    return false;
  }
  if (!text.empty()) {
    std::memcpy(cursor, text.data(), text.size());
    cursor += text.size();
  }
  return true;
}

bool AppendNumber(char *&cursor, char *const end,
                  const double value) noexcept {
#if defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 11
  // libstdc++ before GCC 11 has no floating-point std::to_chars, and MinGW-w64
  // toolchains shipped by LTS distributions are still on GCC 10. %.17g is the
  // shortest form that round-trips a double.
  const auto capacity = static_cast<std::size_t>(end - cursor);
  const int written = std::snprintf(cursor, capacity, "%.17g", value);
  if (written < 0 || static_cast<std::size_t>(written) >= capacity) {
    return false;
  }
  cursor += written;
  return true;
#else
  const auto result = std::to_chars(cursor, end, value);
  if (result.ec != std::errc{}) {
    return false;
  }
  cursor = result.ptr;
  return true;
#endif
}

bool PersistSettings(Context &context) noexcept {
  const auto revision =
      context.settings_revision.load(std::memory_order_acquire);
  const bool enabled = context.enabled.load(std::memory_order_acquire);
  const auto preset = context.preset.load(std::memory_order_acquire);
  const double camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
  const double origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
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
  const bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  const bool fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  if (!ConfigReady(context.config) ||
      !SettingsValid(preset, camera_scale, origin_offset_cm, aspect_ratio,
                     speed_distance_scale, roll_scale, pitch_scale,
                     shake_scale) ||
      context.settings_revision.load(std::memory_order_acquire) != revision) {
    // A UI write landed while the fields were being read; keep the revision
    // unpersisted so the next tick writes a coherent document.
    return false;
  }
  std::array<char, kMaximumSettingsBytes> document{};
  char *cursor = document.data();
  char *const end = document.data() + document.size();
  // `std::to_chars` renders numbers independently of the process locale, so a
  // game that switches the C locale to a decimal-comma one cannot corrupt the
  // stored JSON.
  const bool built =
      AppendText(cursor, end, "{\"enabled\":") &&
      AppendText(cursor, end, enabled ? "true" : "false") &&
      AppendText(cursor, end, ",\"preset\":") &&
      AppendNumber(cursor, end, static_cast<double>(preset)) &&
      AppendText(cursor, end, ",\"cameraScale\":") &&
      AppendNumber(cursor, end, camera_scale) &&
      AppendText(cursor, end, ",\"originOffsetCm\":") &&
      AppendNumber(cursor, end, origin_offset_cm) &&
      AppendText(cursor, end, ",\"aspectRatio\":") &&
      AppendNumber(cursor, end, aspect_ratio) &&
      AppendText(cursor, end, ",\"fovEnabled\":") &&
      AppendText(cursor, end, fov_enabled ? "true" : "false") &&
      AppendText(cursor, end, ",\"speedDistanceScale\":") &&
      AppendNumber(cursor, end, speed_distance_scale) &&
      AppendText(cursor, end, ",\"rollScale\":") &&
      AppendNumber(cursor, end, roll_scale) &&
      AppendText(cursor, end, ",\"pitchScale\":") &&
      AppendNumber(cursor, end, pitch_scale) &&
      AppendText(cursor, end, ",\"shakeScale\":") &&
      AppendNumber(cursor, end, shake_scale) &&
      AppendText(cursor, end, ",\"rollOnlyInDrift\":") &&
      AppendText(cursor, end, roll_only_in_drift ? "true" : "false") &&
      AppendText(cursor, end, "}");
  if (!built) {
    return false;
  }
  const auto status = context.config->write_atomic(
      context.config->user, StringView(kSettingsSchemaId),
      kSettingsSchemaVersion,
      Bytes(std::string_view(
          document.data(), static_cast<std::size_t>(cursor - document.data()))));
  if (status.code != ANOMALY_STATUS_V1_OK) {
    return false;
  }
  context.persisted_settings_revision.store(revision,
                                            std::memory_order_release);
  return true;
}

void SetDefaults(Context &context) noexcept {
  context.enabled.store(true, std::memory_order_release);
  context.preset.store(0, std::memory_order_release);
  context.camera_scale.store(kDefaultCameraScale, std::memory_order_release);
  context.origin_offset_cm.store(kDefaultOriginOffsetCm,
                                 std::memory_order_release);
  context.aspect_ratio.store(kDefaultAspectRatio, std::memory_order_release);
  context.speed_distance_scale.store(kDefaultSpeedDistanceScale,
                                     std::memory_order_release);
  context.roll_scale.store(kDefaultRollScale, std::memory_order_release);
  context.pitch_scale.store(kDefaultPitchScale, std::memory_order_release);
  context.shake_scale.store(kDefaultShakeScale, std::memory_order_release);
  context.roll_only_in_drift.store(false, std::memory_order_release);
  context.fov_enabled.store(true, std::memory_order_release);
}

// Settings are decoded through the host JSON service, so the package needs no
// JSON library of its own. Every handle is plugin-scoped and released as soon
// as the value has been copied out.
class JsonHandle final {
public:
  JsonHandle() noexcept = default;
  JsonHandle(const AnomalyJsonServiceV1 *service,
             const AnomalyGenerationHandleV1 handle) noexcept
      : service_(service), handle_(handle) {}
  ~JsonHandle() {
    if (service_ != nullptr && handle_.id != 0) {
      static_cast<void>(service_->release(service_->user, handle_));
    }
  }
  JsonHandle(const JsonHandle &) = delete;
  JsonHandle &operator=(const JsonHandle &) = delete;

private:
  const AnomalyJsonServiceV1 *service_{};
  AnomalyGenerationHandleV1 handle_{};
};

bool JsonObjectNumber(const AnomalyJsonServiceV1 &json,
                      const AnomalyGenerationHandleV1 object,
                      const std::string_view key, double &value) noexcept {
  AnomalyGenerationHandleV1 child{};
  if (json.object_find(json.user, object, StringView(key), &child)
              .code != ANOMALY_STATUS_V1_OK) {
    return false;
  }
  const JsonHandle scope(&json, child);
  return json.number_value(json.user, child, &value).code ==
         ANOMALY_STATUS_V1_OK;
}

bool JsonObjectBoolean(const AnomalyJsonServiceV1 &json,
                       const AnomalyGenerationHandleV1 object,
                       const std::string_view key, bool &value) noexcept {
  AnomalyGenerationHandleV1 child{};
  if (json.object_find(json.user, object, StringView(key), &child)
              .code != ANOMALY_STATUS_V1_OK) {
    return false;
  }
  const JsonHandle scope(&json, child);
  std::int32_t raw{};
  if (json.boolean_value(json.user, child, &raw).code !=
      ANOMALY_STATUS_V1_OK) {
    return false;
  }
  value = raw != 0;
  return true;
}

bool LoadSettings(Context &context) noexcept {
  std::uint32_t version{};
  std::size_t size{};
  const auto status = context.config->read(
      context.config->user, StringView(kSettingsSchemaId), &version,
      {nullptr, 0}, &size);
  if (status.code == ANOMALY_STATUS_V1_NOT_FOUND) {
    SetDefaults(context);
    static_cast<void>(PersistSettings(context));
    return true;
  }
  if (status.code != ANOMALY_STATUS_V1_OK ||
      version != kSettingsSchemaVersion || size == 0 ||
      size > kMaximumSettingsBytes) {
    return false;
  }
  // Bounded stack buffer: the stored document never exceeds the size the read
  // above reported, so decoding needs no allocation and no exception guard.
  std::array<char, kMaximumSettingsBytes> document{};
  std::size_t copied = size;
  if (context.config
              ->read(context.config->user, StringView(kSettingsSchemaId),
                     &version,
                     {reinterpret_cast<std::uint8_t *>(document.data()),
                      size},
                     &copied)
              .code != ANOMALY_STATUS_V1_OK ||
      copied == 0 || copied > size) {
    return false;
  }
  // Without the host JSON service the stored document cannot be decoded. Keep
  // the camera usable with defaults instead of refusing to load.
  if (!JsonReady(context.json)) {
    SetDefaults(context);
    return true;
  }
  AnomalyGenerationHandleV1 root{};
  if (context.json
          ->parse(context.json->user,
                  StringView(std::string_view(document.data(), copied)),
                  &root)
          .code != ANOMALY_STATUS_V1_OK ||
      root.id == 0) {
    return false;
  }
  const JsonHandle root_scope(context.json, root);
  const AnomalyJsonServiceV1 &json = *context.json;
  std::uint32_t kind{};
  std::size_t field_count{};
  if (json.kind(json.user, root, &kind).code != ANOMALY_STATUS_V1_OK ||
      kind != ANOMALY_JSON_V1_OBJECT ||
      json.object_size(json.user, root, &field_count).code !=
          ANOMALY_STATUS_V1_OK ||
      field_count < 6 || field_count > 12) {
    return false;
  }
  bool enabled{};
  bool fov_enabled{};
  double preset_number{};
  double camera_scale{};
  double origin_offset_cm{};
  double aspect_ratio{};
  if (!JsonObjectBoolean(json, root, "enabled", enabled) ||
      !JsonObjectNumber(json, root, "preset", preset_number) ||
      !JsonObjectNumber(json, root, "cameraScale", camera_scale) ||
      !JsonObjectNumber(json, root, "originOffsetCm", origin_offset_cm) ||
      !JsonObjectNumber(json, root, "aspectRatio", aspect_ratio) ||
      !JsonObjectBoolean(json, root, "fovEnabled", fov_enabled)) {
    return false;
  }
  if (!std::isfinite(preset_number) || preset_number < 0.0 ||
      preset_number > 1.0 || std::floor(preset_number) != preset_number) {
    return false;
  }
  const auto preset = static_cast<std::uint32_t>(preset_number);
  double speed_distance_scale = kDefaultSpeedDistanceScale;
  double roll_scale = kDefaultRollScale;
  double pitch_scale = kDefaultPitchScale;
  double shake_scale = kDefaultShakeScale;
  bool roll_only_in_drift = false;
  static_cast<void>(JsonObjectNumber(json, root, "speedDistanceScale",
                                     speed_distance_scale));
  static_cast<void>(JsonObjectNumber(json, root, "rollScale", roll_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "pitchScale", pitch_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "shakeScale", shake_scale));
  static_cast<void>(JsonObjectBoolean(json, root, "rollOnlyInDrift",
                                      roll_only_in_drift));
  if (!SettingsValid(preset, camera_scale, origin_offset_cm, aspect_ratio,
                     speed_distance_scale, roll_scale, pitch_scale,
                     shake_scale)) {
    return false;
  }
  context.enabled.store(enabled, std::memory_order_release);
  context.preset.store(preset, std::memory_order_release);
  context.camera_scale.store(camera_scale, std::memory_order_release);
  context.origin_offset_cm.store(origin_offset_cm,
                                 std::memory_order_release);
  context.aspect_ratio.store(aspect_ratio, std::memory_order_release);
  context.speed_distance_scale.store(speed_distance_scale,
                                     std::memory_order_release);
  context.roll_scale.store(roll_scale, std::memory_order_release);
  context.pitch_scale.store(pitch_scale, std::memory_order_release);
  context.shake_scale.store(shake_scale, std::memory_order_release);
  context.roll_only_in_drift.store(roll_only_in_drift,
                                   std::memory_order_release);
  context.fov_enabled.store(fov_enabled, std::memory_order_release);
  return true;
}

void ReleaseSettingsSchema(Context &context) noexcept {
  if (context.settings_schema.id != 0 && ConfigReady(context.config)) {
    static_cast<void>(context.config->unregister_schema(
        context.config->user, context.settings_schema));
  }
  context.settings_schema = {};
}

void ANOMALY_CALL ToggleHotkey(void *user, AnomalyGenerationHandleV1,
                               const AnomalyInputSnapshotV1 *) noexcept {
  auto *context = static_cast<Context *>(user);
  if (context == nullptr) {
    return;
  }
  const bool enabled = !context->enabled.load(std::memory_order_acquire);
  context->enabled.store(enabled, std::memory_order_release);
  MarkSettingsDirty(*context);
}

bool RegisterToggleHotkey(Context &context) noexcept {
  if (!InputReady(context.input)) {
    return false;
  }
  try {
    AnomalyHotkeySpecV1 spec{sizeof(spec)};
    spec.virtual_key = kToggleKey;
    spec.flags = ANOMALY_HOTKEY_V1_ALLOW_EXTRA_MODIFIERS |
                 ANOMALY_HOTKEY_V1_ALLOW_WHILE_UI_CAPTURED;
    spec.id = StringView("nfs2015-chase-cam-toggle");
    AnomalyGenerationHandleV1 handle{};
    const auto status = context.input->register_hotkey(
        context.input->user, &spec, ToggleHotkey, &context, &handle);
    if (status.code == ANOMALY_STATUS_V1_OK && handle.id != 0) {
      context.toggle_hotkey = handle;
      return true;
    }
  } catch (...) {
  }
  return false;
}

void ReleaseToggleHotkey(Context &context) noexcept {
  if (context.toggle_hotkey.id != 0 && InputReady(context.input)) {
    static_cast<void>(
        context.input->release_hotkey(context.input->user,
                                      context.toggle_hotkey));
  }
  context.toggle_hotkey = {};
}

// ---------------------------------------------------------------------------
// Camera view-point detour
// ---------------------------------------------------------------------------

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
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

  std::array<double, 3> game_camera_location{};
  bool game_camera_valid = false;
  if (original_called && location != nullptr) {
    game_camera_valid = std::isfinite(location[0]) &&
                        std::isfinite(location[1]) &&
                        std::isfinite(location[2]);
    if (game_camera_valid) {
      game_camera_location = {location[0], location[1], location[2]};
    }
  }

  try {
    if (context != nullptr && location != nullptr && rotation != nullptr &&
        object != nullptr) {
      const std::uintptr_t manager = reinterpret_cast<std::uintptr_t>(object);
      context->observed_manager.store(manager, std::memory_order_release);
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
      if (leased && active) {
        context->view_output_frames.fetch_add(1, std::memory_order_relaxed);
        std::array<double, 3> current_car_location{};
        std::array<double, 3> current_game_camera_location{};
        std::array<double, 3> current_location{};
        std::array<double, 3> current_rotation{};
        std::array<double, 3> previous_location{};
        std::array<double, 3> previous_rotation{};
        float current_lens{};
        float previous_lens{};
        double current_distance{};
        double previous_distance{};
        bool current_fov_enabled{};
        bool current_game_camera_valid{};
        bool previous_fov_enabled{};
        bool has_previous{};
        std::int64_t previous_time_ns{};
        std::int64_t current_time_ns{};
        double update_hz{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
          current_car_location[axis] =
              context->view_car_location[axis].load(
                  std::memory_order_relaxed);
          current_game_camera_location[axis] =
              context->view_game_camera_location[axis].load(
                  std::memory_order_relaxed);
          current_location[axis] =
              context->view_location[axis].load(std::memory_order_relaxed);
          current_rotation[axis] =
              context->view_rotation[axis].load(std::memory_order_relaxed);
          previous_location[axis] =
              context->view_previous_location[axis].load(
                  std::memory_order_relaxed);
          previous_rotation[axis] =
              context->view_previous_rotation[axis].load(
                  std::memory_order_relaxed);
        }
        current_lens = context->view_fov.load(std::memory_order_relaxed);
        previous_lens =
            context->view_previous_fov.load(std::memory_order_relaxed);
        current_distance =
            context->view_distance_cm.load(std::memory_order_relaxed);
        previous_distance =
            context->view_previous_distance_cm.load(std::memory_order_relaxed);
        current_fov_enabled =
            context->view_fov_enabled.load(std::memory_order_relaxed);
        previous_fov_enabled =
            context->view_previous_fov_enabled.load(std::memory_order_relaxed);
        has_previous =
            context->view_has_previous.load(std::memory_order_relaxed);
        previous_time_ns =
            context->view_previous_time_ns.load(std::memory_order_relaxed);
        current_time_ns =
            context->view_current_time_ns.load(std::memory_order_relaxed);
        update_hz = context->view_update_hz.load(std::memory_order_relaxed);
        current_game_camera_valid =
            context->view_game_camera_valid.load(std::memory_order_relaxed);
        const std::uint64_t after =
            context->view_sequence.load(std::memory_order_acquire);
        const bool finite =
            std::isfinite(current_car_location[0]) &&
            std::isfinite(current_car_location[1]) &&
            std::isfinite(current_car_location[2]) &&
            std::isfinite(current_location[0]) &&
            std::isfinite(current_location[1]) &&
            std::isfinite(current_location[2]) &&
            std::isfinite(current_rotation[0]) &&
            std::isfinite(current_rotation[1]) &&
            std::isfinite(current_rotation[2]);
        if (sequence == after && finite) {
          const std::int64_t render_now_ns = NowNanoseconds();
          std::array<double, 3> render_offset = current_location;
          std::array<double, 3> render_rotation = current_rotation;
          float render_lens = current_lens;
          double render_distance = current_distance;

          // Present the previous/current pair one Game tick behind the writer.
          // At a 30 Hz city tick this gives the renderer a fresh blend for the
          // frames between two model updates; at 60 Hz it adds at most one
          // frame of latency and avoids stepping.
          const bool previous_finite =
              has_previous && previous_time_ns > 0 &&
              current_time_ns > previous_time_ns &&
              std::isfinite(previous_location[0]) &&
              std::isfinite(previous_location[1]) &&
              std::isfinite(previous_location[2]) &&
              std::isfinite(previous_rotation[0]) &&
              std::isfinite(previous_rotation[1]) &&
              std::isfinite(previous_rotation[2]);
          if (previous_finite) {
            context->view_interpolated_frames.fetch_add(
                1, std::memory_order_relaxed);
            // The historical pair is presented one tick behind. At 60 Hz
            // that is one frame, but below 60 Hz it lets the car body move a
            // full camera tick ahead of the camera. Use the newest snapshot
            // directly when the tick is already slow.
            const double interpolation_delay =
                update_hz > 0.0 && update_hz < 57.0
                    ? 0.0
                    : kMaximumViewInterpolationDelaySeconds;
            const double alpha = nfs2015_chase::ViewInterpolationAlpha(
                previous_time_ns, current_time_ns, render_now_ns,
                interpolation_delay);
            nfs2015_chase::ViewPose from;
            from.position = {previous_location[0], previous_location[1],
                             previous_location[2]};
            from.pitch_degrees = previous_rotation[0];
            from.yaw_degrees = previous_rotation[1];
            from.roll_degrees = previous_rotation[2];
            from.fov_degrees = previous_lens;
            nfs2015_chase::ViewPose to;
            to.position = {current_location[0], current_location[1],
                           current_location[2]};
            to.pitch_degrees = current_rotation[0];
            to.yaw_degrees = current_rotation[1];
            to.roll_degrees = current_rotation[2];
            to.fov_degrees = current_lens;
            const auto blended =
                nfs2015_chase::BlendViewPose(from, to, alpha);
            render_offset = {blended.position.x, blended.position.y,
                             blended.position.z};
            if (std::isfinite(previous_distance) &&
                std::isfinite(current_distance) && current_distance > 0.0) {
              render_distance = nfs2015_chase::Lerp(
                  previous_distance, current_distance, alpha);
            }
            render_rotation = {blended.pitch_degrees, blended.yaw_degrees,
                               blended.roll_degrees};
            const bool blend_fov =
                previous_fov_enabled && current_fov_enabled &&
                std::isfinite(previous_lens) && previous_lens > 1.0F &&
                previous_lens < 179.0F && std::isfinite(current_lens) &&
                current_lens > 1.0F && current_lens < 179.0F;
            if (blend_fov) {
              render_lens = static_cast<float>(blended.fov_degrees);
            }
          }

          // The model is published one engine tick after the world updated,
          // so the detour's original game camera pose is one frame ahead of
          // the published base. Move the eye by that camera delta so the car
          // and the camera are in the same frame. This is measured motion,
          // not a velocity extrapolation.
          double sync_delta[3]{};
          bool sync = game_camera_valid && current_game_camera_valid;
          if (sync) {
            double sync_distance2 = 0.0;
            for (std::size_t axis = 0; axis < 3; ++axis) {
              sync_delta[axis] = game_camera_location[axis] -
                                 current_game_camera_location[axis];
              if (!std::isfinite(sync_delta[axis])) {
                sync = false;
                break;
              }
              sync_distance2 += sync_delta[axis] * sync_delta[axis];
            }
            const double limit = kMaximumGameCameraSyncCm;
            if (sync && sync_distance2 > limit * limit) {
              sync = false;
            }
          }

          if (std::isfinite(render_distance) && render_distance > 0.0) {
            const nfs2015_chase::Vec3 direction =
                nfs2015_chase::Normalize({render_offset[0], render_offset[1],
                                          render_offset[2]});
            const nfs2015_chase::Vec3 sized =
                nfs2015_chase::Mul(direction, render_distance);
            render_offset = {sized.x, sized.y, sized.z};
          }
          for (std::size_t axis = 0; axis < 3; ++axis) {
            location[axis] = current_car_location[axis] + render_offset[axis] +
                             (sync ? sync_delta[axis] : 0.0);
            rotation[axis] = render_rotation[axis];
          }
          const std::uintptr_t pov =
              context->camera_pov_manager.load(std::memory_order_acquire) ==
                      manager
                  ? context->camera_pov.load(std::memory_order_acquire)
                  : 0;
          if (pov != 0 && current_fov_enabled) {
            if (std::isfinite(render_lens) && render_lens > 1.0F &&
                render_lens < 179.0F) {
              std::uintptr_t address{};
              if (AddAddress(pov, kCameraPovFovOffset, address)) {
                *reinterpret_cast<float *>(address) = render_lens;
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

  if (leased && context != nullptr && HookReady(context->hook)) {
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
  double center[3] = {player.position[0], player.position[1],
                      player.position[2]};
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
    const bool plausible =
        complete && finite && dx * dx + dy * dy < 500.0 * 500.0 &&
        std::fabs(dz) < 500.0 && esp.bounds_extent[2] > 0.5F &&
        esp.bounds_extent[2] < 1000.0F;
    if (plausible) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        center[axis] = esp.bounds_center[axis];
      }
      extent_z = esp.bounds_extent[2];
    }
  }
  const double origin_offset_cm =
      context.origin_offset_cm.load(std::memory_order_acquire);
  origin_cm[0] = center[0];
  origin_cm[1] = center[1];
  origin_cm[2] = center[2] - extent_z + origin_offset_cm;
  return std::isfinite(origin_cm[0]) && std::isfinite(origin_cm[1]) &&
         std::isfinite(origin_cm[2]) && std::isfinite(base_cm[0]) &&
         std::isfinite(base_cm[1]) && std::isfinite(base_cm[2]);
}

bool QueryVehicle(Context &context, bool &valid, bool &speed_valid,
                  double &speed_kmh, AnomalyGenerationHandleV1 &handle,
                  std::uint64_t &entity_id,
                  char class_name[ANOMALY_NTE_VEHICLE_CLASS_NAME_MAX_UTF8_BYTES +
                                  1]) noexcept {
  valid = false;
  speed_valid = false;
  speed_kmh = 0.0;
  handle = {};
  entity_id = 0;
  class_name[0] = '\0';
  if (!VehicleReady(context.vehicle)) {
    return false;
  }
  AnomalyNteVehicleSnapshotV1 vehicle{sizeof(vehicle)};
  const auto result =
      context.vehicle->snapshot(context.vehicle->user, &vehicle);
  if (result.code != ANOMALY_STATUS_V1_OK ||
      (vehicle.flags & ANOMALY_NTE_VEHICLE_V1_VALID) == 0) {
    return true;
  }
  valid = true;
  handle = vehicle.vehicle;
  entity_id = vehicle.vehicle_entity_id;
  std::memcpy(class_name, vehicle.vehicle_class_name,
              sizeof(vehicle.vehicle_class_name));
  class_name[sizeof(vehicle.vehicle_class_name) - 1] = '\0';
  if ((vehicle.flags & ANOMALY_NTE_VEHICLE_V1_HAS_SPEED) != 0 &&
      std::isfinite(vehicle.speed_kmh)) {
    speed_valid = true;
    speed_kmh = std::fabs(vehicle.speed_kmh);
  }
  return true;
}

bool FindVehicleEntity(Context &context, std::uint64_t entity_id,
                       AnomalyNteEntitySnapshotV1 &snapshot,
                       std::uint32_t &index) noexcept;

// The player Pawn's root component is the vehicle actor root, not the
// simulated body mesh. VehicleMesh is the rendered body; its bounds center is
// the transform the car is actually drawn at. Reading it prevents a root-vs-
// mesh offset from turning into camera shake.
enum class CameraBaseSource : std::uint32_t {
  pawn_root = 0,
  vehicle_root = 1,
  vehicle_mesh = 2,
};

bool ReadComponentTransform(const auto *service,
                            const AnomalyGenerationHandleV1 &handle,
                            const std::string_view property,
                            double center_cm[3],
                            double extent_cm[3]) noexcept {
  AnomalyNteEntityComponentBoundsV1 bounds{sizeof(bounds)};
  if (service->component_bounds(service->user, handle, StringView(property),
                                &bounds)
              .code != ANOMALY_STATUS_V1_OK ||
      !SnapshotValid(bounds.flags) ||
      bounds.entity.id != handle.id ||
      bounds.entity.generation != handle.generation ||
      !std::isfinite(bounds.bounds_center[0]) ||
      !std::isfinite(bounds.bounds_center[1]) ||
      !std::isfinite(bounds.bounds_center[2]) ||
      !std::isfinite(bounds.bounds_extent[2]) ||
      bounds.bounds_extent[2] <= 0.5 ||
      bounds.bounds_extent[2] >= 1000.0) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    center_cm[axis] = bounds.bounds_center[axis];
  }
  extent_cm[0] = bounds.bounds_extent[0];
  extent_cm[1] = bounds.bounds_extent[1];
  extent_cm[2] = bounds.bounds_extent[2];
  return true;
}

// The reflected property names are game-specific. NFS 2015 exposes the
// rendered body as VehicleMesh; Mesh/SkeletalMesh cover the stock UE names.
[[nodiscard]] bool ReadEntityMesh(
    Context &context, const AnomalyGenerationHandleV1 &handle,
    double center_cm[3], double extent_cm[3],
    bool include_generic_names) noexcept {
  if (handle.id == 0 || handle.generation == 0) {
    return false;
  }
  static constexpr std::string_view kMeshProperties[] = {
      "VehicleMesh", "Mesh", "SkeletalMesh"};
  for (const std::string_view property : kMeshProperties) {
    if (!include_generic_names && property != "VehicleMesh") {
      continue;
    }
    if (ComponentBoundsReady(context.entities) &&
        ReadComponentTransform(context.entities, handle, property, center_cm,
                               extent_cm)) {
      return true;
    }
    if (ComponentBoundsReady(context.actors) &&
        ReadComponentTransform(context.actors, handle, property, center_cm,
                               extent_cm)) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] bool ReadVehicleRoot(
    Context &context, const AnomalyGenerationHandleV1 &handle,
    double center_cm[3], double extent_cm[3]) noexcept {
  if (handle.id == 0 || handle.generation == 0) {
    return false;
  }
  if (ComponentBoundsReady(context.entities) &&
      ReadComponentTransform(context.entities, handle, "RootComponent",
                             center_cm, extent_cm)) {
    return true;
  }
  return ComponentBoundsReady(context.actors) &&
         ReadComponentTransform(context.actors, handle, "RootComponent",
                                center_cm, extent_cm);
}

[[nodiscard]] bool ReadVehicleBase(
    Context &context, const AnomalyGenerationHandleV1 &handle,
    double center_cm[3], double extent_cm[3],
    CameraBaseSource &source) noexcept {
  if (ReadEntityMesh(context, handle, center_cm, extent_cm, true)) {
    source = CameraBaseSource::vehicle_mesh;
    return true;
  }
  if (ReadVehicleRoot(context, handle, center_cm, extent_cm)) {
    source = CameraBaseSource::vehicle_root;
    return true;
  }
  return false;
}

[[nodiscard]] bool CopyEntityBounds(
    const AnomalyNteEntitySnapshotV1 &snapshot, double center_cm[3],
    double extent_cm[3]) noexcept {
  if (!std::isfinite(snapshot.bounds_center[0]) ||
      !std::isfinite(snapshot.bounds_center[1]) ||
      !std::isfinite(snapshot.bounds_center[2]) ||
      !std::isfinite(snapshot.bounds_extent[0]) ||
      !std::isfinite(snapshot.bounds_extent[1]) ||
      !std::isfinite(snapshot.bounds_extent[2]) ||
      snapshot.bounds_extent[2] <= 0.5 ||
      snapshot.bounds_extent[2] >= 1000.0) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    center_cm[axis] = snapshot.bounds_center[axis];
    extent_cm[axis] = snapshot.bounds_extent[axis];
  }
  return true;
}

// The vehicle service's identity tail is optional. On runtimes that do not
// fill vehicle_entity_id, the local player entity is still in the cache; while
// driving its rendered body is the vehicle mesh.
[[nodiscard]] bool FindLocalPlayerEntity(
    Context &context, AnomalyNteEntitySnapshotV1 &snapshot) noexcept {
  const auto find = [&](const auto *service) -> bool {
    if (service == nullptr || service->page == nullptr) {
      return false;
    }
    constexpr std::uint32_t kPageCapacity = 8;
    std::array<AnomalyNteEntitySnapshotV1, kPageCapacity> page{};
    for (auto &entry : page) {
      entry = AnomalyNteEntitySnapshotV1{sizeof(entry)};
    }
    AnomalyNteEntityPageRequestV1 request{sizeof(request)};
    request.capacity = kPageCapacity;
    request.required_flags = ANOMALY_NTE_ENTITY_V1_LOCAL_PLAYER;
    AnomalyNteEntityPageResultV1 result{sizeof(result)};
    if (service->page(service->user, &request, page.data(), &result).code !=
            ANOMALY_STATUS_V1_OK ||
        result.generation == 0 || result.returned == 0 ||
        result.returned > kPageCapacity) {
      return false;
    }
    snapshot = page[0];
    return snapshot.handle.id != 0 && snapshot.handle.generation != 0;
  };
  return find(context.entities) || find(context.actors);
}

[[nodiscard]] bool PlausibleVehicleBounds(
    const double center[3], const double extent[3]) noexcept {
  if (!std::isfinite(center[0]) || !std::isfinite(center[1]) ||
      !std::isfinite(center[2]) || !std::isfinite(extent[0]) ||
      !std::isfinite(extent[1]) || !std::isfinite(extent[2])) {
    return false;
  }
  // Bounds are UE centimetres. A car is roughly 4 m long, 2 m wide and 1.5 m
  // tall, i.e. half extents near 200x100x75 cm. Characters and props are
  // either too narrow or too small.
  const double max_xy = (std::max)(extent[0], extent[1]);
  const double min_xy = (std::min)(extent[0], extent[1]);
  return max_xy >= 100.0 && min_xy >= 30.0 && extent[2] >= 20.0 &&
         extent[2] <= 500.0;
}

// Last high-level fallback when neither the vehicle service nor the local
// player entity can name the car: scan the entity/actor cache for the closest
// vehicle-shaped bounds. This only runs when the cached handles fail, at most
// every two seconds, and is capped so a large city cache cannot stall a tick.
[[nodiscard]] bool ScanVehicleEntity(
    Context &context, const double player_position_cm[3],
    AnomalyNteEntitySnapshotV1 &best) noexcept {
  bool found = false;
  double best_distance_sq = (std::numeric_limits<double>::max)();
  const auto scan = [&](const auto *service) {
    if (service == nullptr || service->page == nullptr) {
      return;
    }
    constexpr std::uint32_t kPageCapacity = 128;
    constexpr std::uint32_t kMaximumScanEntries = 4096;
    std::array<AnomalyNteEntitySnapshotV1, kPageCapacity> page{};
    std::uint64_t generation = 0;
    std::uint32_t offset = 0;
    std::uint32_t total = 0;
    std::uint32_t scanned = 0;
    while (scanned < kMaximumScanEntries) {
      AnomalyNteEntityPageRequestV1 request{sizeof(request)};
      request.generation = generation;
      request.offset = offset;
      request.capacity = kPageCapacity;
      for (auto &entry : page) {
        entry = AnomalyNteEntitySnapshotV1{sizeof(entry)};
      }
      AnomalyNteEntityPageResultV1 result{sizeof(result)};
      if (service->page(service->user, &request, page.data(), &result).code !=
              ANOMALY_STATUS_V1_OK ||
          result.generation == 0 || result.returned > kPageCapacity) {
        return;
      }
      generation = result.generation;
      total = result.total_matches;
      for (std::uint32_t i = 0; i < result.returned; ++i) {
        const AnomalyNteEntitySnapshotV1 &candidate = page[i];
        if (candidate.entity_id == 0 ||
            (candidate.flags & ANOMALY_NTE_ENTITY_V1_LOCAL_PLAYER) != 0 ||
            !PlausibleVehicleBounds(candidate.bounds_center,
                                    candidate.bounds_extent)) {
          continue;
        }
        const double dx =
            candidate.bounds_center[0] - player_position_cm[0];
        const double dy =
            candidate.bounds_center[1] - player_position_cm[1];
        const double dz =
            candidate.bounds_center[2] - player_position_cm[2];
        const double distance_sq = dx * dx + dy * dy + dz * dz;
        if (distance_sq < best_distance_sq) {
          best_distance_sq = distance_sq;
          best = candidate;
          found = true;
        }
      }
      scanned += result.returned;
      if (result.returned == 0 || result.next_offset <= offset ||
          result.next_offset >= total) {
        return;
      }
      offset = result.next_offset;
    }
  };
  scan(context.entities);
  scan(context.actors);
  constexpr double kMaximumDistanceCm = 1500.0;
  return found &&
         best_distance_sq <= kMaximumDistanceCm * kMaximumDistanceCm;
}

// The player service's bounds come from the Pawn root component, while the
// rendered car body is the vehicle actor's root/VehicleMesh. Prefer the
// vehicle actor transform so a Pawn that stays a character while driving (or
// a root-vs-body offset) cannot move the car relative to the camera.
CameraBaseSource QueryVehicleBase(
    Context &context, const AnomalyGenerationHandleV1 &vehicle_handle,
    const AnomalyGenerationHandleV1 &player_handle,
    const std::uint64_t vehicle_entity_id,
    const double player_position_cm[3], const bool allow_fallback,
    double center_cm[3], double extent_cm[3]) noexcept {
  AnomalyNteEntitySnapshotV1 vehicle_entity{};
  std::uint32_t vehicle_entity_index = 0;
  if (FindVehicleEntity(context, vehicle_entity_id, vehicle_entity,
                        vehicle_entity_index)) {
    CameraBaseSource source = CameraBaseSource::pawn_root;
    if (ReadVehicleBase(context, vehicle_entity.handle, center_cm, extent_cm,
                        source)) {
      return source;
    }
    if (CopyEntityBounds(vehicle_entity, center_cm, extent_cm)) {
      return CameraBaseSource::vehicle_root;
    }
  }

  if (!allow_fallback) {
    return CameraBaseSource::pawn_root;
  }

  AnomalyNteEntitySnapshotV1 player_entity{};
  // A generic Mesh on the player entity could be the character, so only the
  // vehicle-specific property is accepted here.
  if (FindLocalPlayerEntity(context, player_entity) &&
      ReadEntityMesh(context, player_entity.handle, center_cm, extent_cm,
                     false)) {
    return CameraBaseSource::vehicle_mesh;
  }

  if (context.scanned_vehicle_entity.id != 0) {
    CameraBaseSource source = CameraBaseSource::pawn_root;
    if (ReadVehicleBase(context, context.scanned_vehicle_entity, center_cm,
                        extent_cm, source)) {
      return source;
    }
    context.scanned_vehicle_entity = {};
  }

  if (context.elapsed >= context.next_vehicle_scan_elapsed) {
    context.next_vehicle_scan_elapsed = context.elapsed + 2.0;
    AnomalyNteEntitySnapshotV1 scanned{};
    if (ScanVehicleEntity(context, player_position_cm, scanned)) {
      CameraBaseSource source = CameraBaseSource::pawn_root;
      if (ReadVehicleBase(context, scanned.handle, center_cm, extent_cm,
                          source)) {
        context.scanned_vehicle_entity = scanned.handle;
        return source;
      }
      if (CopyEntityBounds(scanned, center_cm, extent_cm)) {
        context.scanned_vehicle_entity = scanned.handle;
        return CameraBaseSource::vehicle_root;
      }
    }
  }

  struct Candidate {
    AnomalyGenerationHandleV1 handle;
    CameraBaseSource source;
  };
  const Candidate candidates[] = {
      {vehicle_handle, CameraBaseSource::vehicle_mesh},
      {player_handle, CameraBaseSource::vehicle_mesh},
      {vehicle_handle, CameraBaseSource::vehicle_root},
      {player_handle, CameraBaseSource::vehicle_root},
  };
  for (const Candidate &candidate : candidates) {
    if (candidate.handle.id == 0 || candidate.handle.generation == 0) {
      continue;
    }
    if (!allow_fallback && candidate.handle.id == player_handle.id &&
        candidate.handle.generation == player_handle.generation) {
      continue;
    }
    if (candidate.source == CameraBaseSource::vehicle_mesh) {
      const bool generic_names =
          candidate.handle.id != player_handle.id ||
          candidate.handle.generation != player_handle.generation;
      if (ReadEntityMesh(context, candidate.handle, center_cm, extent_cm,
                         generic_names)) {
        return CameraBaseSource::vehicle_mesh;
      }
    } else if (ReadVehicleRoot(context, candidate.handle, center_cm,
                              extent_cm)) {
      return CameraBaseSource::vehicle_root;
    }
  }
  return CameraBaseSource::pawn_root;
}

// The vehicle service exposes an object handle plus the entity-cache id. Find
// the matching entity snapshot so component_bounds can read the live vehicle
// transform; the object handle alone is not guaranteed to be in that cache.
bool FindVehicleEntity(Context &context, const std::uint64_t entity_id,
                       AnomalyNteEntitySnapshotV1 &snapshot,
                       std::uint32_t &index) noexcept {
  if (!ComponentBoundsReady(context.entities) || entity_id == 0 ||
      context.entities->frame == nullptr ||
      context.entities->snapshot_at == nullptr) {
    return false;
  }
  AnomalyNteEntityFrameV1 frame{sizeof(frame)};
  if (context.entities->frame(context.entities->user, &frame).code !=
          ANOMALY_STATUS_V1_OK ||
      frame.generation == 0) {
    return false;
  }
  if (context.vehicle_entity_frame_generation == frame.generation &&
      context.vehicle_entity_index < frame.entity_count &&
      context.vehicle_entity.id != 0) {
    AnomalyNteEntitySnapshotV1 cached{sizeof(cached)};
    if (context.entities
                ->snapshot_at(context.entities->user, frame.generation,
                              context.vehicle_entity_index, &cached)
                .code == ANOMALY_STATUS_V1_OK &&
        cached.entity_id == entity_id &&
        cached.handle.id != 0 && cached.handle.generation != 0) {
      snapshot = cached;
      index = context.vehicle_entity_index;
      context.vehicle_entity = cached.handle;
      return true;
    }
  }

  constexpr std::uint32_t kPageCapacity = 128;
  std::array<AnomalyNteEntitySnapshotV1, kPageCapacity> page{};
  std::uint32_t offset = 0;
  while (offset < frame.entity_count) {
    AnomalyNteEntityPageRequestV1 request{sizeof(request)};
    request.generation = frame.generation;
    request.offset = offset;
    request.capacity = kPageCapacity;
    for (auto &entry : page) {
      entry = AnomalyNteEntitySnapshotV1{sizeof(entry)};
    }
    AnomalyNteEntityPageResultV1 result{sizeof(result)};
    if (context.entities
                ->page(context.entities->user, &request, page.data(), &result)
                .code != ANOMALY_STATUS_V1_OK ||
        result.generation != frame.generation || result.returned == 0 ||
        result.returned > kPageCapacity || result.next_offset <= offset) {
      break;
    }
    for (std::uint32_t i = 0; i < result.returned; ++i) {
      if (page[i].entity_id == entity_id && page[i].handle.id != 0 &&
          page[i].handle.generation != 0) {
        snapshot = page[i];
        index = offset + i;
        context.vehicle_entity_id = entity_id;
        context.vehicle_entity_frame_generation = frame.generation;
        context.vehicle_entity_index = index;
        context.vehicle_entity = page[i].handle;
        return true;
      }
    }
    offset = result.next_offset;
  }
  return false;
}

double WrappedAngle(const double angle) noexcept {
  return nfs2015_chase::WrapPi(angle);
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
  context.view_snap = true;
  context.have_last_position = false;
  context.have_last_yaw = false;
  context.last_vehicle_valid_elapsed = -1.0;
  context.last_vehicle_speed_mps = -1.0;
  context.last_vehicle_speed_time = 0.0;
  context.brake_deceleration_mps2 = 0.0;
  context.brake_input = 0.0;
  context.brake_hold_until = 0.0;
  context.vehicle_acceleration_mps2 = 0.0;
  context.smoothed_view_distance_cm = 0.0;
  context.raw_view_distance_cm = 0.0;
  context.have_view_distance = false;
  context.last_player_sequence = 0;
  context.last_player_time = context.elapsed;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.velocity_cmps[axis] = 0.0;
    context.acceleration_cmps2[axis] = 0.0;
  }
  context.yaw_rate = 0.0;
  context.chase_initialized = false;
}

void UpdateBrakeInput(Context &context, const bool vehicle_valid,
                      const bool speed_valid, const double speed_kmh,
                      const double dt) noexcept {
  if (!vehicle_valid || !speed_valid) {
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, kBrakingInputSmoothing, dt);
    context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
        context.vehicle_acceleration_mps2, 0.0,
        kBrakingDecelerationSmoothing, dt);
    return;
  }
  const double speed_mps = speed_kmh / 3.6;
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
  const double raw_acceleration = (speed_mps - previous_mps) / elapsed;
  context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
      context.vehicle_acceleration_mps2,
      nfs2015_chase::Clamp(raw_acceleration, -30.0, 30.0),
      kBrakingDecelerationSmoothing, elapsed);
  const double deceleration =
      nfs2015_chase::Clamp(-raw_acceleration, 0.0, 30.0);
  context.brake_deceleration_mps2 = nfs2015_chase::Damp(
      context.brake_deceleration_mps2, deceleration,
      kBrakingDecelerationSmoothing, elapsed);
  const double excess =
      (std::max)(0.0, context.brake_deceleration_mps2 -
                          kBrakingPitchDeadzoneMps2);
  const double target = nfs2015_chase::Clamp(
      excess / (kBrakingPitchFullMps2 - kBrakingPitchDeadzoneMps2), 0.0, 1.0);
  if (target > 0.05) {
    context.brake_hold_until = context.elapsed + kBrakingHoldSeconds;
  }
  const double gated =
      context.elapsed <= context.brake_hold_until ? target : 0.0;
  context.brake_input = nfs2015_chase::Damp(
      context.brake_input, gated, kBrakingInputSmoothing, dt);
}

void UpdateMotion(Context &context, const double origin_cm[3],
                  const std::uint64_t sequence, const double dt) noexcept {
  if (!context.have_last_position || sequence != context.last_player_sequence) {
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
          context.view_snap = true;
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
  (void)dt;
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
  return false;
}

double SelectCarYaw(Context &context, const double velocity_yaw,
                    const bool have_velocity) noexcept {
  ObservedPose observed{};
  if (!ReadObservedPose(context, observed)) {
    return have_velocity ? velocity_yaw : 0.0;
  }
  const double raw = observed.rotation[1];
  if (!std::isfinite(raw)) {
    return have_velocity ? velocity_yaw : 0.0;
  }
  const double camera_yaw = raw * kDegreesToRadians;
  if (!have_velocity) {
    return camera_yaw;
  }
  const double difference = WrappedAngle(camera_yaw - velocity_yaw);
  if (std::fabs(difference) > kMaximumCameraYawDifference) {
    return velocity_yaw;
  }
  return camera_yaw;
}

void PublishInactive(Context &context) noexcept {
  const std::uint64_t sequence =
      context.view_sequence.load(std::memory_order_relaxed);
  context.view_sequence.store(sequence + 1U, std::memory_order_release);
  context.view_active.store(false, std::memory_order_release);
  context.view_has_previous.store(false, std::memory_order_release);
  context.view_game_camera_valid.store(false, std::memory_order_relaxed);
  context.view_sequence.store(sequence + 2U, std::memory_order_release);
  // Re-entering the camera must not blend from the view that was active
  // before the player stopped driving.
  context.view_snap = true;
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

void PublishView(Context &context, const nfs2015_chase::CameraState &view,
                 const double origin_cm[3], const double base_cm[3],
                 const double update_hz, const double camera_scale,
                 const double aspect_ratio, const bool fov_enabled,
                 const double dt) noexcept {
  const double scale = nfs2015_chase::Clamp(camera_scale, kMinimumCameraScale,
                                            kMaximumCameraScale);
  const nfs2015_chase::Vec3 origin{origin_cm[0], origin_cm[1], origin_cm[2]};
  const nfs2015_chase::Vec3 base{base_cm[0], base_cm[1], base_cm[2]};
  // Scale the camera around the NFS model origin (bounds bottom + offset),
  // not around the pawn root: the two differ by the vehicle bounds height.
  const nfs2015_chase::Vec3 scaled_position = nfs2015_chase::Add(
      origin, nfs2015_chase::Mul(
                  nfs2015_chase::Sub(
                      nfs2015_chase::Mul(view.position, 100.0), origin),
                  scale));
  // Publish the eye offset from the pawn root used by the renderer. ESP bounds
  // are only a model-height reference and can jitter with bounds/LOD changes;
  // they must not drive the camera-car distance.
  const nfs2015_chase::Vec3 offset =
      nfs2015_chase::Sub(scaled_position, base);
  std::array<double, 3> next_location{offset.x, offset.y, offset.z};
  const double raw_distance_cm = nfs2015_chase::Length(offset);
  context.raw_view_distance_cm = raw_distance_cm;
  const std::array<double, 3> next_rotation{view.pitch_degrees,
                                            view.yaw_degrees,
                                            view.roll_degrees};
  const float next_fov =
      fov_enabled
          ? static_cast<float>(view.HorizontalFovDegrees(aspect_ratio))
          : 0.0F;
  const std::int64_t now_ns = NowNanoseconds();

  // The Game thread is the only writer. Read the old current offset/rotation
  // before the odd sequence so it can become the interpolation start point.
  const bool had_current =
      context.view_active.load(std::memory_order_acquire);
  std::array<double, 3> old_base_location{};
  std::array<double, 3> old_location{};
  std::array<double, 3> old_rotation{};
  std::int64_t old_time_ns = 0;
  float old_fov = 0.0F;
  bool old_fov_enabled = false;
  if (had_current) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      old_base_location[axis] =
          context.view_car_location[axis].load(std::memory_order_relaxed);
      old_location[axis] =
          context.view_location[axis].load(std::memory_order_relaxed);
      old_rotation[axis] =
          context.view_rotation[axis].load(std::memory_order_relaxed);
    }
    old_time_ns =
        context.view_current_time_ns.load(std::memory_order_relaxed);
    old_fov = context.view_fov.load(std::memory_order_relaxed);
    old_fov_enabled =
        context.view_fov_enabled.load(std::memory_order_relaxed);
  }
  const bool force_snap = context.view_snap;
  context.view_snap = false;
  const double gap_seconds =
      had_current && old_time_ns > 0
          ? static_cast<double>(now_ns - old_time_ns) * 1e-9
          : 0.0;
  const bool car_jumped =
      had_current &&
      nfs2015_chase::Length(nfs2015_chase::Sub(
          {base_cm[0], base_cm[1], base_cm[2]},
          {old_base_location[0], old_base_location[1],
           old_base_location[2]})) > kViewHistorySnapDistanceCm;
  const bool previous_valid =
      had_current && old_time_ns > 0 && !force_snap && !car_jumped &&
      gap_seconds <= kMaximumViewInterpolationGapSeconds &&
      !ViewHistoryJumped(old_location, old_rotation, next_location,
                         next_rotation);

  double old_distance_cm = 0.0;
  if (had_current) {
    old_distance_cm =
        context.view_distance_cm.load(std::memory_order_relaxed);
  }
  if (!context.have_view_distance || !previous_valid) {
    context.smoothed_view_distance_cm = raw_distance_cm;
    context.have_view_distance = true;
  } else {
    context.smoothed_view_distance_cm = nfs2015_chase::Damp(
        context.smoothed_view_distance_cm, raw_distance_cm,
        kViewDistanceSmoothing, dt);
  }
  if (raw_distance_cm > 1e-6) {
    const double factor =
        context.smoothed_view_distance_cm / raw_distance_cm;
    for (double &component : next_location) {
      component *= factor;
    }
  }

  ObservedPose observed{};
  const bool observed_valid = ReadObservedPose(context, observed);
  const std::uint64_t sequence =
      context.view_sequence.load(std::memory_order_relaxed);
  context.view_sequence.store(sequence + 1U, std::memory_order_release);
  if (previous_valid) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      context.view_previous_location[axis].store(
          old_location[axis], std::memory_order_relaxed);
      context.view_previous_rotation[axis].store(
          old_rotation[axis], std::memory_order_relaxed);
    }
    context.view_previous_time_ns.store(old_time_ns,
                                        std::memory_order_relaxed);
    context.view_previous_fov.store(old_fov, std::memory_order_relaxed);
    context.view_previous_fov_enabled.store(
        old_fov_enabled, std::memory_order_relaxed);
    context.view_previous_distance_cm.store(old_distance_cm,
                                            std::memory_order_relaxed);
  }
  context.view_has_previous.store(previous_valid, std::memory_order_relaxed);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.view_car_location[axis].store(base_cm[axis],
                                          std::memory_order_relaxed);
    context.view_game_camera_location[axis].store(
        observed_valid ? observed.location[axis] : 0.0,
        std::memory_order_relaxed);
    context.view_location[axis].store(next_location[axis],
                                      std::memory_order_relaxed);
    context.view_rotation[axis].store(next_rotation[axis],
                                      std::memory_order_relaxed);
  }
  context.view_current_time_ns.store(now_ns, std::memory_order_relaxed);
  context.view_game_camera_valid.store(observed_valid,
                                       std::memory_order_relaxed);
  context.view_update_hz.store(update_hz, std::memory_order_relaxed);
  context.view_fov.store(next_fov, std::memory_order_relaxed);
  context.view_fov_enabled.store(fov_enabled, std::memory_order_relaxed);
  context.view_active.store(true, std::memory_order_release);
  context.view_sequence.store(sequence + 2U, std::memory_order_release);
  if (fov_enabled) {
    // This frame writes the NFS lens, so no game-lens restore is pending.
    context.fov_restore.store(false, std::memory_order_release);
  } else if (context.last_view_active && context.last_fov_enabled) {
    context.fov_restore.store(true, std::memory_order_release);
  }
  // When disabled, keep an already pending restore (e.g. after PublishInactive)
  // so the next detour can put the game lens back.
  context.last_view_active = true;
  context.last_fov_enabled = fov_enabled;
}

void UpdateCamera(Context &context, const double dt) noexcept {
  const bool enabled = context.enabled.load(std::memory_order_acquire);
  const std::uint32_t preset = context.preset.load(std::memory_order_acquire);
  const bool fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  const double camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
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
  const bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  if (preset != context.configured_preset) {
    context.configured_preset = preset;
    context.chase.SetConfig(preset == 0 ? nfs2015_chase::GamePreset()
                                        : nfs2015_chase::CinematicPreset());
    context.chase_initialized = false;
    context.view_snap = true;
  }
  // These are cheap setters that do not reset the springs.
  context.chase.SetSpeedDistanceScale(speed_distance_scale);
  context.chase.SetRollScale(roll_scale);
  context.chase.SetPitchScale(pitch_scale);
  context.chase.SetShakeScale(shake_scale);
  context.chase.SetRollOnlyInDrift(roll_only_in_drift);
  if (context.reset_requested.exchange(false, std::memory_order_acq_rel)) {
    context.chase_initialized = false;
    context.have_last_yaw = false;
    context.view_snap = true;
  }
  if (WorldChanged(context)) {
    ResetMotion(context);
    context.scanned_vehicle_entity = {};
    context.next_vehicle_scan_elapsed = 0.0;
  }

  double origin_cm[3]{};
  double base_cm[3]{};
  std::uint64_t player_sequence = 0;
  AnomalyGenerationHandleV1 player_handle{};
  const bool player_valid = QueryPlayer(context, origin_cm, base_cm,
                                        player_sequence, player_handle);
  bool vehicle_valid = false;
  bool vehicle_speed_valid = false;
  double vehicle_speed_kmh = 0.0;
  AnomalyGenerationHandleV1 vehicle_handle{};
  std::uint64_t vehicle_entity_id = 0;
  char vehicle_class_name[ANOMALY_NTE_VEHICLE_CLASS_NAME_MAX_UTF8_BYTES + 1]{};
  const bool vehicle_service =
      QueryVehicle(context, vehicle_valid, vehicle_speed_valid,
                   vehicle_speed_kmh, vehicle_handle, vehicle_entity_id,
                   vehicle_class_name);
  std::uint32_t camera_base_source = 0;
  if (vehicle_valid || player_valid) {
    double mesh_center[3]{};
    double mesh_extent[3]{};
    const bool allow_base_fallback = vehicle_valid || !vehicle_service;
    const CameraBaseSource source = QueryVehicleBase(
        context, vehicle_handle, player_handle, vehicle_entity_id, base_cm,
        allow_base_fallback, mesh_center, mesh_extent);
    if (source != CameraBaseSource::pawn_root) {
      camera_base_source = static_cast<std::uint32_t>(source);
      const double origin_offset_cm =
          context.origin_offset_cm.load(std::memory_order_acquire);
      for (std::size_t axis = 0; axis < 3; ++axis) {
        base_cm[axis] = mesh_center[axis];
      }
      origin_cm[0] = mesh_center[0];
      origin_cm[1] = mesh_center[1];
      origin_cm[2] = mesh_center[2] - mesh_extent[2] + origin_offset_cm;
    }
  }
  if (camera_base_source != context.camera_base_source) {
    context.camera_base_source = camera_base_source;
    context.view_snap = true;
  }
  if (!context.base_diagnostic_logged && vehicle_valid) {
    context.base_diagnostic_logged = true;
    char detail[320]{};
    std::snprintf(
        detail, sizeof(detail),
        "camera base source=%u vehicle_id=%llu class=%s entities=%d "
        "actors=%d vehicle_service_version=%u",
        camera_base_source,
        static_cast<unsigned long long>(vehicle_entity_id),
        vehicle_class_name[0] != '\0' ? vehicle_class_name : "-",
        context.entities != nullptr ? 1 : 0,
        context.actors != nullptr ? 1 : 0,
        context.vehicle != nullptr ? context.vehicle->service_version : 0);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, detail);
  }

  bool identity_changed = false;
  if (player_valid && player_handle.id != 0 &&
      (player_handle.id != context.player_handle.id ||
       player_handle.generation != context.player_handle.generation)) {
    context.player_handle = player_handle;
    identity_changed = true;
  }
  if (vehicle_valid && vehicle_handle.id != 0 &&
      (vehicle_handle.id != context.vehicle_handle.id ||
       vehicle_handle.generation != context.vehicle_handle.generation)) {
    context.vehicle_handle = vehicle_handle;
    identity_changed = true;
  }
  if (identity_changed) {
    ResetMotion(context);
    context.chase_initialized = false;
    context.scanned_vehicle_entity = {};
    context.next_vehicle_scan_elapsed = 0.0;
  }

  if (vehicle_valid) {
    context.last_vehicle_valid_elapsed = context.elapsed;
  }

  if (!enabled || !player_valid) {
    PublishInactive(context);
    ResetMotion(context);
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.player_valid = player_valid;
    context.display.vehicle_service_available = vehicle_service;
    context.display.camera_base_source = 0;
    return;
  }

  UpdateMotion(context, origin_cm, player_sequence, dt);

  const double velocity_length =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const double velocity_mps = velocity_length / 100.0;
  const bool speed_plausible =
      velocity_mps >= kMinimumVehicleSpeedMps && velocity_mps <
          kMaximumPlausibleSpeedCmps / 100.0;
  // The vehicle snapshot can miss a tick while the city streams. Keep the
  // camera running through a short gap instead of tearing down its history and
  // snapping back to the game camera for one frame.
  const bool vehicle_recent =
      vehicle_valid ||
      (context.last_vehicle_valid_elapsed >= 0.0 &&
       context.elapsed - context.last_vehicle_valid_elapsed <=
           kDrivingGraceSeconds);
  const bool driving =
      vehicle_recent || (!vehicle_service && speed_plausible);
  if (!driving) {
    PublishInactive(context);
    ResetMotion(context);
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.player_valid = true;
    context.display.vehicle_service_available = vehicle_service;
    context.display.camera_base_source = 0;
    context.display.speed_kmh = vehicle_speed_kmh;
    return;
  }

  // The position sample follows the pawn attached to the vehicle. The vehicle
  // service's forward-speed value is more stable than a position derivative,
  // so use it for magnitude while keeping the sampled direction for slip.
  if (vehicle_valid && vehicle_speed_valid && vehicle_speed_kmh > 0.5) {
    const double target_cmps = vehicle_speed_kmh * (100000.0 / 3600.0);
    if (velocity_length > 50.0) {
      const double scale = target_cmps / velocity_length;
      if (scale > 0.5 && scale < 2.0) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          context.velocity_cmps[axis] *= scale;
        }
      }
    }
  }

  const double velocity_yaw =
      velocity_length > 1.0
          ? std::atan2(context.velocity_cmps[1], context.velocity_cmps[0])
          : (context.have_last_yaw ? context.last_car_yaw : 0.0);
  double car_yaw =
      SelectCarYaw(context, velocity_yaw, velocity_length > 50.0);
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

  if (vehicle_valid) {
    UpdateBrakeInput(context, vehicle_valid, vehicle_speed_valid,
                     vehicle_speed_kmh, dt);
  }

  nfs2015_chase::CarState car;
  car.position = {origin_cm[0] / 100.0, origin_cm[1] / 100.0,
                  origin_cm[2] / 100.0};
  car.yaw = car_yaw;
  car.velocity = world_velocity;
  car.yaw_rate = context.yaw_rate;
  nfs2015_chase::Vec3 acceleration{
      context.acceleration_cmps2[0] / 100.0,
      context.acceleration_cmps2[1] / 100.0,
      context.acceleration_cmps2[2] / 100.0};
  if (vehicle_valid && vehicle_speed_valid) {
    // Longitudinal dolly from the raw position derivative is noisy in city
    // traffic. Keep the lateral component (it drives roll) but replace the
    // forward component with the smoothed vehicle-speed derivative.
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

  nfs2015_chase::CameraState view;
  if (!context.chase_initialized || !context.chase.initialized()) {
    context.chase.Reset(car);
    context.chase_initialized = true;
  }
  view = context.chase.Update(dt, car);
  PublishView(context, view, origin_cm, base_cm, context.update_hz,
              camera_scale, aspect_ratio, fov_enabled, dt);

  {
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = true;
    context.display.player_valid = true;
    context.display.vehicle_service_available = vehicle_service;
    context.display.camera_base_source = camera_base_source;
    context.display.speed_kmh = vehicle_speed_kmh > 0.5
                                    ? vehicle_speed_kmh
                                    : speed_mps * 3.6;
    context.display.slip_degrees = slip_angle * kRadiansToDegrees;
    context.display.drift = context.chase.drift();
    context.display.fov_horizontal =
        fov_enabled ? view.HorizontalFovDegrees(aspect_ratio) : 0.0;
    context.display.camera_distance_m =
        context.smoothed_view_distance_cm / 100.0;
    context.display.camera_distance_raw_m =
        context.raw_view_distance_cm / 100.0;
  }
}

void UpdateDisplay(Context &context) noexcept {
  std::scoped_lock lock(context.display_mutex);
  context.display.started = context.started.load(std::memory_order_acquire);
  context.display.enabled = context.enabled.load(std::memory_order_acquire);
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
  context.display.preset = context.preset.load(std::memory_order_acquire);
  context.display.fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  context.display.camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
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
  context.display.roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
}

void RetryHookIfNeeded(Context &context) noexcept {
  if (context.hook_ready.load(std::memory_order_acquire) &&
      context.view_hook_id.load(std::memory_order_acquire) != 0) {
    return;
  }
  if (context.hook_retry_requested.exchange(false, std::memory_order_acq_rel)) {
    context.next_hook_retry = 0.0;
  }
  if (context.elapsed < context.next_hook_retry) {
    return;
  }
  context.next_hook_retry = context.elapsed + kHookRetrySeconds;
  static_cast<void>(EnsureHook(context));
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
  context->vehicle = Query<AnomalyNteVehicleServiceV1>(
      host, ANOMALY_NTE_VEHICLE_SERVICE_V1_ID, 1U);
  context->entities = Query<AnomalyNteEntitiesServiceV1>(
      host, ANOMALY_NTE_ENTITIES_SERVICE_V1_ID,
      ANOMALY_NTE_ENTITIES_SERVICE_V1_VERSION);
  context->actors = Query<AnomalyNteActorsServiceV1>(
      host, ANOMALY_NTE_ACTORS_SERVICE_V1_ID,
      ANOMALY_NTE_ACTORS_SERVICE_V1_VERSION);
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
  context->view_interpolated_frames.store(0, std::memory_order_relaxed);
  context->configured_preset =
      context->preset.load(std::memory_order_acquire);
  context->chase.SetConfig(context->configured_preset == 0
                               ? nfs2015_chase::GamePreset()
                               : nfs2015_chase::CinematicPreset());
  context->chase_initialized = false;
  context->reset_requested.store(true, std::memory_order_release);
  // Signature learning is a Lifecycle-domain operation; cache the target here
  // so the Game tick only has to create the hook.
  std::uintptr_t view_point_target = 0;
  if (ResolveSignature(*context, kCameraViewPointPattern, view_point_target)) {
    context->hook_target.store(view_point_target, std::memory_order_release);
  }
  ResetMotion(*context);
  context->observed_valid.store(false, std::memory_order_release);
  PublishInactive(*context);
  static_cast<void>(RegisterToggleHotkey(*context));
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
  context->chase_initialized = false;
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
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
        const std::uint64_t interpolated =
            context->view_interpolated_frames.exchange(
                0, std::memory_order_relaxed);
        context->view_output_hz =
            static_cast<double>(outputs) / context->update_rate_window;
        context->interpolation_ratio =
            outputs != 0
                ? static_cast<double>(interpolated) /
                      static_cast<double>(outputs)
                : 0.0;
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    RetryHookIfNeeded(*context);
    static_cast<void>(ResolveCameraPov(*context));
    UpdateCamera(*context, dt);
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

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

std::string PresetName(const nfs2015_chase_cam::Localizer &localizer,
                       const std::uint32_t preset) {
  return preset == 0
             ? localizer.Text("preset.game", "NFS 2015 (retail data)")
             : localizer.Text("preset.cinematic", "Cinematic reconstruction");
}

void DrawStatusLine(const AnomalyUiServiceV1 *ui,
                    const nfs2015_chase_cam::Localizer &localizer,
                    const char *key, const char *fallback,
                    const std::string_view value) {
  const std::string label = localizer.Text(key, fallback);
  std::string line;
  line.reserve(label.size() + value.size() + 3U);
  line.append(label).append(": ").append(value);
  ui->text(ui->user, StringView(line));
}

void ANOMALY_CALL Draw(void *plugin_context, const AnomalyUiServiceV1 *ui_v1) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  const AnomalyUiServiceV1 *ui = ui_v1 != nullptr ? ui_v1 : context->ui;
  if (!UiReady(ui)) {
    return;
  }
  bool window_begun = false;
  try {
    DisplayState display;
    {
      std::scoped_lock lock(context->display_mutex);
      display = context->display;
    }

    const std::string title =
        context->localizer.Text("window.title", "NFS 2015 Chase Cam");
    ui->set_next_window_size(ui->user, 430.0F, 0.0F, 4U);
    int open = 1;
    const int visible =
        ui->begin_window(ui->user, StringView(title), &open, 0);
    window_begun = true;
    if (visible != 0) {
      const std::string enabled_label = context->localizer.Label(
          "setting.enabled", "Enable camera", "chase-enabled");
      int enabled = display.enabled ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(enabled_label), &enabled) != 0) {
        context->enabled.store(enabled != 0, std::memory_order_release);
        context->reset_requested.store(true, std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string preset_label =
          context->localizer.Text("setting.preset", "Preset");
      ui->text(ui->user, StringView(preset_label));
      if (ui->button(ui->user, StringView(PresetName(context->localizer, 0)),
                     0.0F, 0.0F) != 0) {
        context->preset.store(0, std::memory_order_release);
        context->reset_requested.store(true, std::memory_order_release);
        MarkSettingsDirty(*context);
      }
      if (ui->button(ui->user, StringView(PresetName(context->localizer, 1)),
                     0.0F, 0.0F) != 0) {
        context->preset.store(1, std::memory_order_release);
        context->reset_requested.store(true, std::memory_order_release);
        MarkSettingsDirty(*context);
      }
      std::string selected = context->localizer.Text("status.preset", "Preset");
      selected.append(": ").append(PresetName(context->localizer, display.preset));
      ui->text(ui->user, StringView(selected));

      const std::string fov_label = context->localizer.Label(
          "setting.fov", "Apply NFS lens (FOV)", "chase-fov");
      int fov_enabled = display.fov_enabled ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(fov_label), &fov_enabled) != 0) {
        context->fov_enabled.store(fov_enabled != 0, std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string scale_label = context->localizer.Label(
          "setting.camera_scale", "Camera distance / height scale",
          "chase-scale");
      float scale = static_cast<float>(display.camera_scale);
      if (ui->slider_float(ui->user, StringView(scale_label), &scale,
                           static_cast<float>(kMinimumCameraScale),
                           static_cast<float>(kMaximumCameraScale)) != 0) {
        context->camera_scale.store(
            nfs2015_chase::Clamp(static_cast<double>(scale),
                                 kMinimumCameraScale, kMaximumCameraScale),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string distance_label = context->localizer.Label(
          "setting.speed_distance", "Speed / accel distance scale",
          "chase-speed-distance");
      float distance_scale =
          static_cast<float>(display.speed_distance_scale);
      if (ui->slider_float(
              ui->user, StringView(distance_label), &distance_scale,
              static_cast<float>(kMinimumSpeedDistanceScale),
              static_cast<float>(kMaximumSpeedDistanceScale)) != 0) {
        context->speed_distance_scale.store(
            nfs2015_chase::Clamp(static_cast<double>(distance_scale),
                                 kMinimumSpeedDistanceScale,
                                 kMaximumSpeedDistanceScale),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string origin_label = context->localizer.Label(
          "setting.origin_offset", "Car origin offset (cm)",
          "chase-origin-offset");
      float origin = static_cast<float>(display.origin_offset_cm);
      if (ui->slider_float(ui->user, StringView(origin_label), &origin,
                           static_cast<float>(kMinimumOriginOffsetCm),
                           static_cast<float>(kMaximumOriginOffsetCm)) != 0) {
        context->origin_offset_cm.store(
            nfs2015_chase::Clamp(static_cast<double>(origin),
                                 kMinimumOriginOffsetCm,
                                 kMaximumOriginOffsetCm),
            std::memory_order_release);
        context->reset_requested.store(true, std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string roll_label = context->localizer.Label(
          "setting.roll_scale", "Roll strength", "chase-roll-scale");
      float roll_scale = static_cast<float>(display.roll_scale);
      if (ui->slider_float(ui->user, StringView(roll_label), &roll_scale,
                           static_cast<float>(kMinimumRollScale),
                           static_cast<float>(kMaximumRollScale)) != 0) {
        context->roll_scale.store(
            nfs2015_chase::Clamp(static_cast<double>(roll_scale),
                                 kMinimumRollScale, kMaximumRollScale),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string pitch_label = context->localizer.Label(
          "setting.pitch_scale", "Braking pitch strength",
          "chase-pitch-scale");
      float pitch_scale = static_cast<float>(display.pitch_scale);
      if (ui->slider_float(ui->user, StringView(pitch_label), &pitch_scale,
                           static_cast<float>(kMinimumPitchScale),
                           static_cast<float>(kMaximumPitchScale)) != 0) {
        context->pitch_scale.store(
            nfs2015_chase::Clamp(static_cast<double>(pitch_scale),
                                 kMinimumPitchScale, kMaximumPitchScale),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string shake_label = context->localizer.Label(
          "setting.shake_scale", "Camera shake strength",
          "chase-shake-scale");
      float shake_scale = static_cast<float>(display.shake_scale);
      if (ui->slider_float(ui->user, StringView(shake_label), &shake_scale,
                           static_cast<float>(kMinimumShakeScale),
                           static_cast<float>(kMaximumShakeScale)) != 0) {
        context->shake_scale.store(
            nfs2015_chase::Clamp(static_cast<double>(shake_scale),
                                 kMinimumShakeScale, kMaximumShakeScale),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string roll_drift_label = context->localizer.Label(
          "setting.roll_only_drift", "Roll only while drifting",
          "chase-roll-only-drift");
      int roll_only_drift = display.roll_only_in_drift ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(roll_drift_label),
                       &roll_only_drift) != 0) {
        context->roll_only_in_drift.store(roll_only_drift != 0,
                                          std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string aspect_label = context->localizer.Label(
          "setting.aspect_ratio", "Viewport aspect ratio", "chase-aspect");
      float aspect = static_cast<float>(display.aspect_ratio);
      if (ui->slider_float(ui->user, StringView(aspect_label), &aspect,
                           static_cast<float>(kMinimumAspectRatio),
                           static_cast<float>(kMaximumAspectRatio)) != 0) {
        context->aspect_ratio.store(
            nfs2015_chase::Clamp(static_cast<double>(aspect),
                                 kMinimumAspectRatio, kMaximumAspectRatio),
            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      ui->separator(ui->user);
      std::string active_value;
      if (!display.started) {
        active_value = context->localizer.Text("status.stopped", "Stopped");
      } else if (!display.enabled) {
        active_value = context->localizer.Text("status.disabled", "Disabled");
      } else if (display.active) {
        active_value = context->localizer.Text("status.active", "Active");
      } else if (display.driving) {
        active_value = context->localizer.Text("status.starting", "Starting");
      } else {
        active_value =
            context->localizer.Text("status.waiting_vehicle", "Waiting for a vehicle");
      }
      DrawStatusLine(ui, context->localizer, "status.camera", "Camera",
                     active_value);
      DrawStatusLine(
          ui, context->localizer, "status.hook", "View-point hook",
          display.hook_ready
              ? context->localizer.Text("status.ready", "Ready")
              : context->localizer.Text(
                    display.started ? "status.unavailable" : "status.stopped",
                    display.started ? "Unavailable" : "Stopped"));
      if (display.started) {
        char update_line[64]{};
        std::snprintf(update_line, sizeof(update_line), "%.1f Hz",
                      display.update_hz);
        DrawStatusLine(ui, context->localizer, "status.update_rate",
                       "Camera update rate", update_line);
        char output_line[64]{};
        std::snprintf(output_line, sizeof(output_line), "%.1f Hz / %.0f%%",
                      display.view_output_hz,
                      display.interpolation_ratio * 100.0);
        DrawStatusLine(ui, context->localizer, "status.view_output",
                       "View output / interp", output_line);
      }
      if (display.started && !display.hook_ready) {
        ui->text(ui->user,
                 StringView(context->localizer.Text(
                     "hint.hook_conflict",
                     "Another camera plugin may own the view-point getter. "
                     "Stop Camera Tools or close BetterPose's camera hook, "
                     "then retry.")));
        if (ui->button(ui->user,
                       StringView(context->localizer.Text("action.retry_hook",
                                                          "Retry hook")),
                       0.0F, 0.0F) != 0) {
          context->hook_retry_requested.store(true, std::memory_order_release);
        }
      }
      if (display.driving) {
        char line[192]{};
        std::snprintf(line, sizeof(line),
                      "%s: %.1f km/h   %s: %.1f°   %s: %.1f°   %s: %.2f",
                      context->localizer.Text("status.speed", "Speed").c_str(),
                      display.speed_kmh,
                      context->localizer.Text("status.slip", "Slip").c_str(),
                      display.slip_degrees,
                      context->localizer.Text("status.fov", "FOV").c_str(),
                      display.fov_horizontal,
                      context->localizer.Text("status.drift", "Drift").c_str(),
                      display.drift);
        char distance_line[32]{};
        std::snprintf(distance_line, sizeof(distance_line), "%.2f / %.2f m",
                      display.camera_distance_m,
                      display.camera_distance_raw_m);
        DrawStatusLine(ui, context->localizer, "status.distance",
                       "Camera-car distance", distance_line);
        DrawStatusLine(
            ui, context->localizer, "status.car_base", "Camera base",
            display.camera_base_source == 2
                ? context->localizer.Text("base.vehicle_mesh", "VehicleMesh")
                : display.camera_base_source == 1
                      ? context->localizer.Text("base.vehicle_root",
                                                "Vehicle root")
                      : context->localizer.Text("base.pawn_root",
                                                "Pawn root"));
        char speed_distance_line[32]{};
        std::snprintf(speed_distance_line, sizeof(speed_distance_line),
                      "%.2f", display.speed_distance_scale);
        DrawStatusLine(ui, context->localizer, "status.speed_distance",
                       "Speed distance scale", speed_distance_line);
        ui->text(ui->user, StringView(line));
      }
      if (!display.player_valid) {
        ui->text(ui->user,
                 StringView(context->localizer.Text(
                     "hint.player_unavailable",
                     "Player snapshot is unavailable; the camera stays idle.")));
      }
      if (!display.vehicle_service_available) {
        ui->text(ui->user,
                 StringView(context->localizer.Text(
                     "hint.vehicle_service",
                     "Vehicle service is unavailable; speed-gated fallback is in use.")));
      }
      if (ui->button(ui->user,
                     StringView(context->localizer.Text("action.reset",
                                                        "Reset camera")),
                     0.0F, 0.0F) != 0) {
        context->reset_requested.store(true, std::memory_order_release);
        context->reset_feedback_until.store(
            GetTickCount64() + kResetFeedbackMilliseconds,
            std::memory_order_release);
      }
      if (GetTickCount64() <
          context->reset_feedback_until.load(std::memory_order_acquire)) {
        ui->text(ui->user,
                 StringView(context->localizer.Text(
                     "hint.reset_done",
                     "Camera re-centered on the car.")));
      }
      ui->separator(ui->user);
      ui->text(ui->user,
               StringView(context->localizer.Text(
                   "hint.toggle",
                   "F7 toggles this camera. The plugin only drives the view "
                   "while a vehicle is active.")));
    }
  } catch (...) {
  }
  if (window_begun) {
    ui->end_window(ui->user);
  }
}

}  // namespace

ANOMALY_SDK_EXPORT AnomalyStatusV1 ANOMALY_CALL AnomalyPluginEntryV1(
    AnomalyPluginDescriptorV1 *descriptor) {
  if (descriptor == nullptr ||
      descriptor->struct_size < sizeof(*descriptor)) {
    return {ANOMALY_STATUS_V1_INVALID_ARGUMENT, 0, {}};
  }
  *descriptor = {
      sizeof(*descriptor),
      ANOMALY_PLUGIN_API_V1_MAJOR,
      ANOMALY_PLUGIN_API_V1_MINOR,
      StringView(kPluginId),
      StringView("NFS 2015 Chase Cam"),
      StringView("RobinEatCorn"),
      StringView("1.0.0"),
      Load,
      Start,
      Stop,
      Unload,
      Update,
      Draw};
  return anomaly::sdk::Ok();
}
