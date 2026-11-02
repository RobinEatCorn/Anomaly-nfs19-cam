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
// snapshot supplies the speed. No GObjects scan.
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
#include "latency_window.hpp"
#include "car_sample.hpp"
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
#include <thread>
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
// A longer gap is a hitch/teleport, not a low update rate. Snap the history
// instead of blending across it (which would both trail and step at the end).
// T2 presenter: the largest step the springs are allowed to take in one frame.
// How far behind the newest sample the model is fed is one and a half mean tick
// intervals, decided where the ring is sampled.
constexpr double kMaximumPresenterStepSeconds = 0.1;
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
// The camera's yaw is the nose reference until it strays this far from the
// direction of travel: past it the camera is describing something other than
// the car (a wall, a spin, a scripted shot), and the travel direction is the
// safer answer.
constexpr double kMaximumCameraYawDifference = 1.4;  // ~80 degrees
// The host reports a signed forward speed, so reverse needs no hysteresis: at
// walking pace and below the travel direction is not used at all.
constexpr double kReverseSpeedKmh = 1.0;
constexpr double kHookRetrySeconds = 2.0;
// The detour runs one engine tick after the model published. Shift the
// published eye by the game camera's movement since that publish so the car
// and the camera are rendered in the same frame. A larger step is a
// teleport/hitch, not car motion.

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

// ---------------------------------------------------------------------------
// Diagnostics (PLAN-camera-timing.md T0)
// ---------------------------------------------------------------------------
//
// Every silent degradation the camera used to take without telling anyone is
// counted here. The point of T0 is to rank the suspected causes from one
// recorded session instead of inferring them from the source, so the counters
// are always on and cheap: one relaxed atomic increment on paths that already
// call into the host far more expensively.
enum SnapReason : std::uint32_t {
  kSnapReasonMotion = 1U << 0U,      // ResetMotion: world/entity/driving change
  kSnapReasonTeleport = 1U << 1U,    // implausible position step
  kSnapReasonInactive = 1U << 2U,    // camera went idle and came back
  kSnapReasonPreset = 1U << 3U,      // preset switch
  kSnapReasonUserReset = 1U << 4U,   // "Reset camera" button
  kSnapReasonCount = 5
};

// Which measurement `SelectCarYaw` returned. A switch between two of these
// inside one drift is exactly the car-yaw step that shows up as a camera jump,
// so this count is the direct measure of the suspected jump source.
enum YawSource : std::uint32_t {
  kYawSourceNone = 0,
  kYawSourceObserved = 1,
  kYawSourceVelocity = 2,
  kYawSourceNoObservation = 3,
};

constexpr std::size_t kDiagnosticThreadSlots = 8;

struct Diagnostics final {
  std::atomic<std::uint64_t> snap_by_reason[kSnapReasonCount]{};
  std::atomic<std::uint64_t> yaw_source_switches{};
  std::atomic<std::uint64_t> observed_read_failures{};
  std::atomic<std::uint64_t> lease_failures{};
  std::atomic<std::uint64_t> view_read_torn{};
  std::atomic<std::uint64_t> detour_calls{};
  std::atomic<std::uint64_t> detour_calls_window{};
  // Active view but no published pose: the frame the game camera shows through.
  std::atomic<std::uint64_t> detour_no_publish{};
  // Q1: which threads call the hooked getter.
  std::atomic<std::uint32_t> detour_threads[kDiagnosticThreadSlots]{};
  std::atomic<std::uint32_t> detour_thread_count{};
  // Q6: ticks the camera ran in a vehicle on a stale snapshot (the grace path).
  std::atomic<std::uint64_t> vehicle_gap_ticks{};
  std::atomic<std::uint64_t> player_sequence_jumps{};
  // Vehicle service polls, i.e. how often the expensive host call ran.
  std::atomic<std::uint64_t> vehicle_polls{};
  // T3.1: ticks that fell back to the render-thread camera observation because
  // the Game-tick ESP camera tail was unavailable. Should stay 0.
  std::atomic<std::uint64_t> camera_yaw_esp_misses{};
  // Experiment (PLAN-camera-timing.md T3.4). Two independent ways to measure
  // the slip angle, side by side, plus the presenter's per-frame correction.
  // Nothing here changes behaviour; the numbers decide which fix is worth
  // building.
  std::atomic<double> slip_signed_speed_kmh{};
  std::atomic<double> slip_from_position_degrees{};
  std::atomic<double> slip_from_speed_degrees{};
  std::atomic<std::uint64_t> slip_sign_changes{};
  std::atomic<int> slip_last_sign{};
  // Tenths of a degree: the largest single-tick change of the slip angle in the
  // current window. A real drift entry is bounded by the yaw rate (~1-2 deg per
  // tick at 50 Hz); anything much larger is the estimator, not the car.
  std::atomic<std::uint64_t> slip_max_step_ddeg{};
  // Tenths of a centimetre, so the max can use an integer CAS loop.
  // Eye speed on the render clock, in millimetres per second, floor and peak.
  // A smooth camera moves at a nearly constant speed, so the two converge on
  // the car's own speed; a camera that freezes and jumps once per tick shows a
  // floor near zero against a peak near twice the average. This is immune to
  // the call rate, because each sample is divided by that call's own elapsed
  // time - the earlier "second difference" metric was dominated by call timing.
  std::atomic<std::uint64_t> render_speed_floor_mmps{};
  std::atomic<std::uint64_t> render_speed_peak_mmps{};
  // Turn rate on the render clock, in hundredths of a degree per second.
  std::atomic<std::uint64_t> render_turn_floor_cdps{};
  std::atomic<std::uint64_t> render_turn_peak_cdps{};
  // How far the anchor moves in one Game tick, in tenths of a centimetre. The
  // scale the two jitter numbers must be compared against.
  std::atomic<std::uint64_t> car_step_dcm{};
  // Judder instrument. Every frame the game's own projection puts the car at a
  // screen position; the second difference of that position is how far the car
  // departs from a straight continuation of its previous two frames, in pixels.
  // That is the quantity the eye sees, measured with the same transform the
  // game uses to draw, so it needs no interpretation. Tenths of a pixel.
  std::atomic<std::uint64_t> judder_peak_dpx{};
  std::atomic<std::uint64_t> judder_sum_dpx{};
  std::atomic<std::uint64_t> judder_count{};
  // Model restarts inside the current window. A restart re-seeds the springs
  // from the car, so the eye jumps and springs back: a far bigger visible event
  // than any amount of interpolation error, and one that no smoothness
  // measurement of the running model can see, because it only measures the
  // times when the model is running.
  std::atomic<std::uint64_t> reset_window{};
  // The camera's own speed, and how much that speed jumps between frames, in
  // hundredths of a centimetre. This is the world's flow rate: the car travels
  // with the camera and stays put in frame while the whole picture jolts, which
  // is why a measurement of the car cannot see it.
  std::atomic<std::uint64_t> flow_peak_ccm{};
  std::atomic<std::uint64_t> flow_sum_ccm{};
  std::atomic<std::uint64_t> flow_count{};
  // Frame pacing, in microseconds: the longest frame of the window and the mean.
  std::atomic<std::uint64_t> frame_peak_us{};
  std::atomic<std::uint64_t> frame_sum_us{};
  std::atomic<std::uint64_t> frame_count{};
  std::atomic<std::uint64_t> judder_car_peak_dpx{};
  std::atomic<std::uint64_t> judder_car_sum_dpx{};
  std::atomic<std::uint64_t> judder_car_count{};
  // Rotation and field of view carry the same kind of residual, in hundredths
  // of a degree. A point at the centre of the frame cannot see either: rotating
  // the camera about itself barely moves it, and a lens change scales about it.
  // These are the channels that a car-centred projection is blind to.
  std::atomic<std::uint64_t> pitch_peak_cdeg{};
  std::atomic<std::uint64_t> pitch_sum_cdeg{};
  std::atomic<std::uint64_t> pitch_count{};
  std::atomic<std::uint64_t> roll_peak_cdeg{};
  std::atomic<std::uint64_t> roll_sum_cdeg{};
  std::atomic<std::uint64_t> roll_count{};
  std::atomic<std::uint64_t> lens_peak_cdeg{};
  std::atomic<std::uint64_t> lens_sum_cdeg{};
  std::atomic<std::uint64_t> lens_count{};
  // Game-tick interval in nanoseconds, floor and peak over the window. The host
  // ticks once per frame on this build, so this is also the frame interval.
  std::atomic<std::uint64_t> tick_interval_min_ns{};
  std::atomic<std::uint64_t> tick_interval_max_ns{};
};

// ---------------------------------------------------------------------------
// Probe (PLAN-camera-timing.md §8.1). Read-only reconnaissance, not camera
// code. It answers two questions that decide what to build next:
//   1. does the host's once-per-frame callback really fire once per frame, and
//      what is the frame-rate to Game-tick ratio (the size of the quantisation
//      error a fixed 1/120 step leaves behind);
//   2. is there a car-yaw source that does not come from the game camera, and
//      how noisy is the game camera's yaw anyway.
// Nothing here feeds the chase model, so a failed probe cannot change what the
// player sees; it only prints numbers.
//
// This version targets Anomaly v2.5.0. That ABI has `anomaly.ue5.ahud` but
// neither `nte.player.get_object_handles` nor `ProcessEvent.invoke`, so asking
// the engine for the car's own rotation is not available; see §8.2. Everything
// below is built from data v2.5.0 already publishes.
// ---------------------------------------------------------------------------
constexpr std::uint8_t kProbeUnset = 0;
constexpr std::uint8_t kProbeOk = 1;
constexpr std::uint8_t kProbeMissing = 2;
// Half a metre of headroom over any real car: a bounds offset longer than this
// is not a body-frame lever arm, it is a broken sample.
constexpr double kProbeMaximumBoundsOffsetCm = 300.0;
// Below this the direction of the bounds offset is dominated by suspension and
// quantisation noise, so the candidate angle is not reported at all.
constexpr double kProbeMinimumBoundsOffsetCm = 20.0;

struct Probe final {
  // Once-per-frame subscription. `anomaly.ue5.ahud` exists in v2.5.0.
  std::uint32_t subscribe_attempts{};
  double next_subscribe_elapsed{};
  std::uint8_t subscribe_state{};
  std::uint32_t subscribe_code{};
  bool subscribed{};
  AnomalyGenerationHandleV1 subscription{};
  // Per-frame window. The host serialises the callback on the Game thread, so
  // the callback and the Game tick never touch these at the same time.
  std::uint64_t frame_calls{};
  std::uint64_t frame_window_calls{};
  double frame_window_elapsed{};
  double frame_hz{};

  // Game-camera yaw noise, in tenths of a degree per Game tick. If this is not
  // far below the slip-angle step the model already shows, the camera's own
  // jitter is entering the model as "car yaw".
  double camera_yaw_step_max_ddeg{};
  double camera_yaw_step_sum_ddeg{};
  std::uint32_t camera_yaw_step_count{};
  double camera_yaw_step_avg_ddeg{};
  double last_camera_yaw_degrees{};
  bool have_last_camera_yaw{};

  // Body-yaw candidate taken from `bounds_center - player.position`, i.e. a
  // fixed vector in the car's own frame that therefore rotates with the body
  // and has nothing to do with the camera. It only means anything when the
  // offset is long enough to survive noise, hence the magnitude is reported.
  double bounds_offset_cm{};
  double bounds_yaw_degrees{};
  bool bounds_yaw_valid{};
  double camera_yaw_degrees{};
  bool camera_yaw_valid{};
  double camera_minus_bounds_degrees{};
  bool camera_minus_bounds_valid{};
  // Largest one-tick change of the mesh bound radius the plugin subtracts to
  // build its "ground point". That subtraction is the only place a mesh bound
  // still reaches the eye on this game, and only on the vertical axis.
  double height_step_cm{};
  // Judder: where the game draws the car on screen, in pixels, for the last two
  // frames. Filled from the frame callback, which is the only place the game's
  // projection is available.
  // Where the game draws a world point this frame, in pixels, for the last two
  // frames. [0] is the newer. Timestamps are kept because frames are not
  // equally long, and a residual that ignored that would report frame jitter as
  // judder.
  //
  // Two tracks, because there are two different questions. `anchor` is the car
  // position this camera was actually built from: its residual is how smooth
  // our own pipeline is. `car` is the raw newest Game-tick position, which is a
  // staircase by construction because the Game only moves the pawn once a tick:
  // its residual is how far that staircase is from what we draw. If only `car`
  // is rough the picture is fine and the number was an artefact of the
  // measuring point; if `anchor` is rough the camera itself is moving in steps.
  struct ScreenTrack final {
    bool valid{};
    unsigned count{};
    double px[2][2]{};
    std::int64_t ns[2]{};
  };
  ScreenTrack track_anchor{};
  ScreenTrack track_car{};

  // The same residual for a single scalar, used for pitch, roll and the lens.
  struct ScalarTrack final {
    unsigned count{};
    double value[2]{};
    std::int64_t ns[2]{};
  };
  // The camera's own published position, for the flow measurement.
  double eye_cm[2][3]{};
  std::int64_t eye_ns[2]{};
  unsigned eye_count{};
  // Frame pacing.
  std::int64_t frame_last_ns{};
  ScalarTrack track_pitch{};
  ScalarTrack track_roll{};
  ScalarTrack track_lens{};
};

struct DisplayState final {
  bool started{};
  bool enabled{};
  bool hook_ready{};
  bool active{};
  bool driving{};
  bool player_valid{};
  bool vehicle_service_available{};
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
  bool diagnostic_flat_pose{};
  bool diagnostic_zero_lag{};
  bool diagnostic_skip_position{};
  bool diagnostic_skip_rotation{};

  // Diagnostics. Copied from Context::diag on the Game tick so Draw reads a
  // plain snapshot on the render thread.
  double detour_hz{};
  double tick_ms_last{};
  double tick_ms_avg{};
  double tick_ms_p99{};
  double tick_ms_max{};
  std::uint64_t snap_total{};
  std::uint64_t snap_by_reason[kSnapReasonCount]{};
  std::uint64_t yaw_source_switches{};
  std::uint64_t observed_read_failures{};
  std::uint64_t lease_failures{};
  std::uint64_t view_read_torn{};
  std::uint64_t detour_calls{};
  std::uint64_t detour_no_publish{};
  std::uint64_t detour_thread_count{};
  std::uint64_t vehicle_gap_ticks{};
  std::uint64_t player_sequence_jumps{};
  std::uint64_t vehicle_polls{};
  std::uint64_t camera_yaw_esp_misses{};
  double slip_signed_speed_kmh{};
  double slip_from_position_degrees{};
  double slip_from_speed_degrees{};
  std::uint64_t slip_sign_changes{};
  double slip_max_step_degrees{};
  bool render_speed_valid{};
  bool render_turn_valid{};
  double render_speed_floor_mps{};
  double render_speed_peak_mps{};
  double render_turn_floor_dps{};
  double render_turn_peak_dps{};
  double car_step_cm{};

  // Probe. Copied from Context::probe on the Game tick.
  bool probe_ahud_service{};
  std::uint8_t probe_subscribe_state{};
  std::uint32_t probe_subscribe_code{};
  double probe_frame_hz{};
  std::uint64_t probe_frame_calls{};
  double probe_camera_yaw_step_max_degrees{};
  double probe_camera_yaw_step_avg_degrees{};
  bool probe_camera_yaw_valid{};
  double probe_camera_yaw_degrees{};
  double probe_bounds_offset_cm{};
  bool probe_bounds_yaw_valid{};
  double probe_camera_minus_bounds_degrees{};
  bool judder_valid{};
  double judder_peak_px{};
  double judder_avg_px{};
  bool judder_car_valid{};
  double judder_car_peak_px{};
  double judder_car_avg_px{};
  bool rotation_valid{};
  double pitch_peak_degrees{};
  double pitch_avg_degrees{};
  double roll_peak_degrees{};
  double roll_avg_degrees{};
  bool lens_valid{};
  double lens_peak_degrees{};
  double lens_avg_degrees{};
  std::uint64_t resets_in_window{};
  bool flow_valid{};
  double flow_peak_cm{};
  double flow_avg_cm{};
  bool frame_valid{};
  double frame_peak_ms{};
  double frame_mean_ms{};
  double probe_height_step_cm{};
  bool tick_interval_valid{};
  double tick_interval_min_ms{};
  double tick_interval_max_ms{};
};

// Keep a window peak without a lock. The Game thread resets these between
// windows, so a plain CAS loop is enough.
template <typename T>
void LowerFloor(std::atomic<T> &floor, T value) noexcept {
  T previous = floor.load(std::memory_order_relaxed);
  while (value < previous &&
         !floor.compare_exchange_weak(previous, value,
                                     std::memory_order_relaxed)) {
  }
}

template <typename T>
void RaisePeak(std::atomic<T> &peak, T value) noexcept {
  T previous = peak.load(std::memory_order_relaxed);
  while (value > previous &&
         !peak.compare_exchange_weak(previous, value,
                                     std::memory_order_relaxed)) {
  }
}

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
  // Optional engine access. Only the probe uses this; it may be null on a host
  // without the `ue5.ahud` feature, and every use is guarded.
  const AnomalyUe5AhudServiceV1 *ahud{};

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
  // Bisect switch: hold every attitude term at zero and stop writing the lens,
  // so only position and yaw survive. One click decides whether the twitch is
  // inside the attitude model at all, without reading a single number.
  std::atomic_bool diagnostic_flat_pose{};
  // Second bisect switch: stop delaying at all and use the newest Game-tick
  // position. If the Game draws the car from that same value, the car sits
  // still in frame and only the background steps; if it draws an interpolated
  // position, the car itself will start stepping instead. Either answer says
  // which of the two we have been chasing.
  std::atomic_bool diagnostic_zero_lag{};
  // Third pair of bisect switches: what we write. Every build so far has
  // overwritten the Game's location and rotation together, so the write itself
  // has never been varied, only the arithmetic behind it. Turning one of these
  // off leaves the Game's own value in place for that half.
  std::atomic_bool diagnostic_skip_position{};
  std::atomic_bool diagnostic_skip_rotation{};
  // High-frequency borrow from the Game's own camera position. The Game solves
  // where the car is drawn and puts its camera there; we only have 50 Hz samples
  // of it and cannot reproduce the instant. Keep the slow part of the Game
  // camera's offset from the car, which is its framing, and take the fast part,
  // which is the car-locked truth our samples are missing.
  bool borrow_valid{};
  double borrow_lowpass_cm[3]{};
  std::int64_t borrow_last_ns{};
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
  std::atomic<bool> view_fov_enabled{true};
  // `view_car_location` is the published camera anchor. The detour computes
  // `anchor + offset`, and the offset is built from the same anchor, so the
  // anchor cancels out of the world position: it only has to be stable. The
  // Pawn root location is the cheapest stable choice (one cached snapshot
  // field per tick), and it is the actor transform the car is drawn at.
  std::atomic<double> view_car_location[3]{};
  // Game camera location at the time the published view was computed. The
  // detour adds the movement since then so the eye and car share a frame.

  // Vehicle service poll cache. The host's vehicle snapshot runs three
  // ProcessEvent dispatches plus a class-chain name walk on every call, so it
  // is polled on an interval and the last sample is held in between. Speed is
  // a slow variable and every consumer of it is damped already.
  double next_vehicle_query_elapsed{};
  bool vehicle_cached_valid{};
  bool vehicle_cached_speed_valid{};
  double vehicle_cached_speed_kmh{};
  // True only on the tick that actually polled the host, so the speed
  // derivative is not read from a held sample.
  bool vehicle_cached_fresh{};
  double brake_last_target{};
  AnomalyGenerationHandleV1 vehicle_cached_handle{};
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
  nfs2015_chase::ChaseCam chase{nfs2015_chase::GamePreset()};
  std::uint32_t configured_preset{0};
  bool chase_initialized{};
  // T2: the chase model runs on the render clock, so the Game tick only writes
  // car samples and the render side advances the springs. The model state is
  // shared, hence the try-lock: a render call that loses the race uses the last
  // produced pose instead of blocking a frame.
  nfs2015_chase::SampleRing car_ring;
  std::atomic_flag presenter_lock = ATOMIC_FLAG_INIT;
  std::int64_t presenter_last_ns{};
  std::uint64_t presenter_epoch{};
  bool presenter_initialized{};
  std::int64_t presenter_pose_ns{};
  std::atomic<std::uint64_t> presenter_pose_sequence{};
  std::atomic<double> presenter_pose_location[3]{};
  std::atomic<double> presenter_pose_rotation[3]{};
  std::atomic<float> presenter_pose_lens{};
  std::atomic<bool> presenter_pose_fov{};
  std::atomic<bool> presenter_pose_valid{};
  std::atomic<double> presenter_anchor[3]{};
  std::atomic<double> presenter_drift{};
  std::atomic<double> presenter_distance_cm{};
  std::atomic<std::int64_t> sample_interval_ns{};
  // Running mean of the Game tick interval. The presenter looks back this far
  // times kSampleDelayFactor, which is more than one interval, so the sample
  // point always sits inside a bracket instead of clamping at its edge. An
  // offline harness (tests/judder_repro.cpp) scores the alternatives on a 45 Hz
  // tick against 59 Hz frames with hitches, as degrees of car-on-screen wobble
  // per frame: last interval raw 0.354, this mean 0.124, this mean out to one
  // and a half intervals with continuation 0.006.
  std::atomic<double> sample_mean_interval_ns{};
  std::int64_t sample_last_ns{};
  // Bumped by the Game tick when the model must restart (world, preset, reset).
  std::uint64_t sample_epoch{};
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
  std::uint64_t last_player_sequence{};
  double last_player_time{};
  double last_position_cm[3]{};
  double velocity_cmps[3]{};
  double acceleration_cmps2[3]{};
  double last_car_yaw{};
  double yaw_rate{};
  double last_slip_degrees{};
  bool have_last_slip{};
  // Travel direction is measured from the nose or from its opposite end (T3.2).
  // Latest raw host forward speed, still signed: negative means reversing.
  double vehicle_speed_signed_kmh{};
  bool last_view_active{};
  bool last_fov_enabled{};
  // T3.1: game-camera yaw from the player ESP snapshot, i.e. sampled by the
  // host on this Game tick instead of observed on the render thread.
  double camera_yaw_radians{};
  bool camera_yaw_valid{};
  bool view_snap{true};

  // Diagnostics (T0). Game thread owns the plain fields; the detour only
  // touches Context::diag atomics.
  Diagnostics diag{};
  Probe probe{};
  // Probe input: the world XY offset of the bounds centre from the Pawn root,
  // captured by QueryPlayer on this Game tick. A fixed vector in the car's own
  // frame, so its direction is a body-yaw candidate that never involves the
  // camera.
  double probe_bounds_dx_cm{};
  double probe_bounds_dy_cm{};
  // Probe input: the mesh bound radius that `origin_cm.z = center.z - extent_z`
  // subtracts, captured by QueryPlayer. Its per-tick change is the vertical bob
  // the eye inherits from the mesh.
  // The car's world position, in the centimetres the game uses, for the frame
  // callback to project. The Pawn root, not the mesh bounds: the root is what
  // the game moves between ticks.
  std::atomic<double> probe_car_cm[3]{};
  std::atomic<bool> probe_car_valid{false};
  double probe_extent_z_cm{};
  double probe_last_extent_z_cm{};
  bool probe_have_extent{};
  std::uint32_t view_snap_reasons{};
  std::uint32_t yaw_source{kYawSourceNone};
  nfs2015_chase_cam::LatencyWindow camera_cost{};
  double camera_ms_last{};
  double detour_hz{};
  bool diagnostics_open{};

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
  const bool diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  const bool diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  const bool diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  const bool diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
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
      AppendText(cursor, end, ",\"diagnosticFlatPose\":") &&
      AppendText(cursor, end, diagnostic_flat_pose ? "true" : "false") &&
      AppendText(cursor, end, ",\"diagnosticZeroLag\":") &&
      AppendText(cursor, end, diagnostic_zero_lag ? "true" : "false") &&
      AppendText(cursor, end, ",\"diagnosticSkipPosition\":") &&
      AppendText(cursor, end, diagnostic_skip_position ? "true" : "false") &&
      AppendText(cursor, end, ",\"diagnosticSkipRotation\":") &&
      AppendText(cursor, end, diagnostic_skip_rotation ? "true" : "false") &&
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
  context.diagnostic_flat_pose.store(false, std::memory_order_release);
  context.diagnostic_zero_lag.store(false, std::memory_order_release);
  context.diagnostic_skip_position.store(false, std::memory_order_release);
  context.diagnostic_skip_rotation.store(false, std::memory_order_release);
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
  bool diagnostic_flat_pose = false;
  bool diagnostic_zero_lag = false;
  bool diagnostic_skip_position = false;
  bool diagnostic_skip_rotation = false;
  static_cast<void>(JsonObjectNumber(json, root, "speedDistanceScale",
                                     speed_distance_scale));
  static_cast<void>(JsonObjectNumber(json, root, "rollScale", roll_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "pitchScale", pitch_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "shakeScale", shake_scale));
  static_cast<void>(JsonObjectBoolean(json, root, "rollOnlyInDrift",
                                      roll_only_in_drift));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticFlatPose",
                                      diagnostic_flat_pose));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticZeroLag",
                                      diagnostic_zero_lag));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticSkipPosition",
                                      diagnostic_skip_position));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticSkipRotation",
                                      diagnostic_skip_rotation));
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
  context.diagnostic_flat_pose.store(diagnostic_flat_pose,
                                     std::memory_order_release);
  context.diagnostic_zero_lag.store(diagnostic_zero_lag,
                                    std::memory_order_release);
  context.diagnostic_skip_position.store(diagnostic_skip_position,
                                         std::memory_order_release);
  context.diagnostic_skip_rotation.store(diagnostic_skip_rotation,
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

bool AdvancePresenter(Context &context, std::int64_t now_ns,
                      double location[3], double rotation[3], float *lens,
                      bool *fov_enabled) noexcept;

void RequestPresenterReset(Context &context) noexcept;

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
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
        if (AdvancePresenter(*context, render_now_ns, present_location,
                             present_rotation, &present_lens,
                             &present_fov_enabled)) {
          // T3.4e: how fast the eye moves *relative to the car*, which is the
          // only thing that can look like twitching. The absolute eye speed is
          // dominated by the car's own travel, so it can never separate a
          // smooth camera from a juddering one.
          {
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
            // The Game's own position is still in `location` here: the
            // original ran before us and we have not written yet.
            double game_eye_cm[3]{};
            for (std::size_t axis = 0; axis < 3; ++axis) {
              game_eye_cm[axis] = location[axis];
            }
            const double borrow_dt =
                context->borrow_last_ns == 0
                    ? 0.0
                    : static_cast<double>(render_now_ns -
                                          context->borrow_last_ns) *
                          1e-9;
            context->borrow_last_ns = render_now_ns;
            for (std::size_t axis = 0; axis < 3; ++axis) {
              const double target = game_eye_cm[axis] - present_location[axis];
              if (!context->borrow_valid) {
                context->borrow_lowpass_cm[axis] = target;
              } else if (borrow_dt > 1e-5 && borrow_dt < 0.2) {
                // 0.3 s low-pass: slow enough that the Game's own framing is
                // removed, fast enough that a drift's push-in still follows.
                constexpr double kBorrowRate = 3.3;
                const double blend = 1.0 - std::exp(-kBorrowRate * borrow_dt);
                context->borrow_lowpass_cm[axis] +=
                    (target - context->borrow_lowpass_cm[axis]) * blend;
                present_location[axis] +=
                    target - context->borrow_lowpass_cm[axis];
              }
            }
            context->borrow_valid = true;
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
      for (std::size_t axis = 0; axis < 3; ++axis) {
        center[axis] = esp.bounds_center[axis];
      }
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
  origin_cm[0] = center[0];
  origin_cm[1] = center[1];
  origin_cm[2] = center[2] - extent_z + origin_offset_cm;
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

// Every call re-runs `BP_GetCurrentDriveVehicle` and `GetForwardSpeedKmH`
// through ProcessEvent and walks the object's class chain by name, because the
// host caches none of it. That is a few hundred microseconds of Game tick per
// call, spent mostly to learn a speed that changes slowly.
constexpr double kVehicleQueryIntervalSeconds = 0.05;

bool PollVehicle(Context &context, bool &valid, bool &speed_valid,
                 double &speed_kmh, AnomalyGenerationHandleV1 &handle,
                 char class_name[ANOMALY_NTE_VEHICLE_CLASS_NAME_MAX_UTF8_BYTES +
                                 1]) noexcept {
  valid = false;
  speed_valid = false;
  speed_kmh = 0.0;
  handle = {};
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
  std::memcpy(class_name, vehicle.vehicle_class_name,
              sizeof(vehicle.vehicle_class_name));
  class_name[sizeof(vehicle.vehicle_class_name) - 1] = '\0';
  if ((vehicle.flags & ANOMALY_NTE_VEHICLE_V1_HAS_SPEED) != 0 &&
      std::isfinite(vehicle.speed_kmh)) {
    speed_valid = true;
    speed_kmh = std::fabs(vehicle.speed_kmh);
    // `GetForwardSpeedKmH` is the forward component of velocity in the car's
    // own frame, and it is signed: the sign separates forward from reverse
    // without any state, and the magnitude is the numerator of an exact slip
    // angle that needs no differentiation.
    context.vehicle_speed_signed_kmh = static_cast<double>(vehicle.speed_kmh);
    context.diag.slip_signed_speed_kmh.store(
        context.vehicle_speed_signed_kmh, std::memory_order_relaxed);
  }
  return true;
}

// Applies the poll interval; the Game tick reads the cache instead of the host.
void RefreshVehicleSample(Context &context) noexcept {
  context.vehicle_cached_fresh = false;
  if (!VehicleReady(context.vehicle)) {
    context.vehicle_cached_valid = false;
    context.vehicle_cached_speed_valid = false;
    context.vehicle_cached_speed_kmh = 0.0;
    context.vehicle_cached_handle = {};
    context.vehicle_speed_signed_kmh = 0.0;
    return;
  }
  if (context.elapsed < context.next_vehicle_query_elapsed) {
    return;
  }
  context.next_vehicle_query_elapsed =
      context.elapsed + kVehicleQueryIntervalSeconds;
  context.diag.vehicle_polls.fetch_add(1, std::memory_order_relaxed);
  bool valid = false;
  bool speed_valid = false;
  double speed_kmh = 0.0;
  AnomalyGenerationHandleV1 handle{};
  char class_name[ANOMALY_NTE_VEHICLE_CLASS_NAME_MAX_UTF8_BYTES + 1]{};
  static_cast<void>(PollVehicle(context, valid, speed_valid, speed_kmh, handle,
                               class_name));
  context.vehicle_cached_valid = valid;
  context.vehicle_cached_speed_valid = speed_valid;
  if (!speed_valid) {
    context.vehicle_speed_signed_kmh = 0.0;
  }
  context.vehicle_cached_speed_kmh = speed_kmh;
  context.vehicle_cached_handle = handle;
  context.vehicle_cached_fresh = true;
  if (!context.base_diagnostic_logged && valid) {
    context.base_diagnostic_logged = true;
    char detail[192]{};
    std::snprintf(detail, sizeof(detail),
                  "vehicle snapshot available: class=%s service_version=%u "
                  "poll_interval=%.0f ms",
                  class_name[0] != '\0' ? class_name : "-",
                  context.vehicle->service_version,
                  kVehicleQueryIntervalSeconds * 1000.0);
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, detail);
  }
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
  NoteViewSnap(context, kSnapReasonMotion);
  context.yaw_source = kYawSourceNone;
  context.have_last_position = false;
  context.have_last_yaw = false;
  context.last_vehicle_valid_elapsed = -1.0;
  context.last_vehicle_speed_mps = -1.0;
  context.last_vehicle_speed_time = 0.0;
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

void UpdateBrakeInput(Context &context, const bool vehicle_valid,
                      const bool speed_valid, const bool speed_fresh,
                      const double speed_kmh, const double dt) noexcept {
  if (!vehicle_valid || !speed_valid) {
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, kBrakingInputSmoothing, dt);
    context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
        context.vehicle_acceleration_mps2, 0.0,
        kBrakingDecelerationSmoothing, dt);
    return;
  }
  if (!speed_fresh) {
    // The sample is held between polls, so there is no new speed delta to
    // differentiate. Differentiating it at tick rate would read as zero
    // acceleration on the held ticks and a spike on the sampled one, i.e. a
    // sawtooth in the braking pitch.
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input,
        context.elapsed <= context.brake_hold_until
            ? context.brake_last_target
            : 0.0,
        kBrakingInputSmoothing, dt);
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
  context.brake_last_target = gated;
  context.brake_input = nfs2015_chase::Damp(
      context.brake_input, gated, kBrakingInputSmoothing, dt);
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
                    const double velocity_mps,
                    const bool have_velocity) noexcept {
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
  if (!have_camera_yaw) {
    NoteYawSource(context, kYawSourceNoObservation);
    return have_velocity ? velocity_yaw : 0.0;
  }
  if (!have_velocity) {
    // Too slow for the travel direction to mean anything: the camera is the
    // only continuous reference left.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  // Reversing travels along the nose plus 180 degrees, so the camera stays
  // aligned with the nose instead of ending up in front of the car. The host's
  // forward speed is signed, which makes this stateless: an earlier version
  // derived it from the previous tick's own output and latched into reverse,
  // parking the camera in front of the car for the rest of the session.
  const bool reversing =
      context.vehicle_speed_signed_kmh < -kReverseSpeedKmh;
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
  context.chase_initialized = false;
  ++context.sample_epoch;
}

// The model owns its state; other render threads must read a consistent pose.
void PublishPresenterPose(Context &context, const double location[3],
                          const double rotation[3], const float lens,
                          const bool fov_enabled) noexcept {
  const std::uint64_t sequence =
      context.presenter_pose_sequence.load(std::memory_order_relaxed);
  context.presenter_pose_sequence.store(sequence + 1U,
                                        std::memory_order_release);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.presenter_pose_location[axis].store(location[axis],
                                                std::memory_order_relaxed);
    context.presenter_pose_rotation[axis].store(rotation[axis],
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
                                     bool *fov_enabled) noexcept {
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

// Advance the model to `now_ns` and produce the pose for this render call.
// Returns false when there is nothing to present (no sample yet, or another
// thread is advancing and no cached pose exists).
bool AdvancePresenter(Context &context, const std::int64_t now_ns,
                      double location[3], double rotation[3], float *lens,
                      bool *fov_enabled) noexcept {
  if (!TryLockPresenter(context)) {
    return ReadPresenterPose(context, location, rotation, lens, fov_enabled);
  }
  struct Unlock final {
    Context &context;
    ~Unlock() { UnlockPresenter(context); }
  } unlock{context};

  if (context.presenter_pose_valid.load(std::memory_order_relaxed) &&
      now_ns == context.presenter_pose_ns) {
    // A second pass of the same frame: the model must not advance twice.
    return ReadPresenterPose(context, location, rotation, lens, fov_enabled);
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
    return false;
  }

  if (!context.presenter_initialized || context.presenter_epoch != sample.epoch ||
      !context.chase.initialized()) {
    context.chase.Reset(sample.car);
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
  const nfs2015_chase::CameraState view = context.chase.Update(elapsed, sample.car);

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
  PublishPresenterPose(context, location, rotation, *lens, *fov_enabled);
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

  return true;
}


void UpdateCamera(Context &context, const double dt) noexcept {
  const bool enabled = context.enabled.load(std::memory_order_acquire);
  const std::uint32_t preset = context.preset.load(std::memory_order_acquire);
  bool fov_enabled = context.fov_enabled.load(std::memory_order_acquire);
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
  bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  // The bisect switch: no attitude, no lens. Position and yaw only.
  const bool diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  const double effective_roll_scale = diagnostic_flat_pose ? 0.0 : roll_scale;
  const double effective_pitch_scale = diagnostic_flat_pose ? 0.0 : pitch_scale;
  const double effective_shake_scale = diagnostic_flat_pose ? 0.0 : shake_scale;
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
  // These are cheap setters that do not reset the springs.
  context.chase.SetSpeedDistanceScale(speed_distance_scale);
  context.chase.SetRollScale(effective_roll_scale);
  context.chase.SetPitchScale(effective_pitch_scale);
  context.chase.SetShakeScale(effective_shake_scale);
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
  // The camera anchor is the Pawn root location that `QueryPlayer` already
  // fills. `base_cm` cancels out of the published world position (the detour
  // adds it back), so it only has to be stable and cheap. Resolving the
  // rendered body through `component_bounds` used to cost ~2 ms of the Game
  // tick: the host walks the class's whole reflected property list by name on
  // every call, decoding an FName twice per property and reading it through
  // ReadProcessMemory. Nothing about the camera needs that.
  RefreshVehicleSample(context);
  const bool vehicle_service = VehicleReady(context.vehicle);
  const bool vehicle_valid = context.vehicle_cached_valid;
  const bool vehicle_speed_valid = context.vehicle_cached_speed_valid;
  const bool vehicle_speed_fresh = context.vehicle_cached_fresh;
  const double vehicle_speed_kmh = context.vehicle_cached_speed_kmh;
  const AnomalyGenerationHandleV1 vehicle_handle =
      context.vehicle_cached_handle;

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
    RequestPresenterReset(context);
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
    return;
  }

  UpdateMotion(context, origin_cm, player_sequence);

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
  // Q6: ticks the camera kept driving on a stale vehicle snapshot. The grace
  // path absorbs a streaming gap, so counting it separates "the drop is
  // invisible" from "the drop tears the view down".
  if (!vehicle_valid && vehicle_service && vehicle_recent) {
    context.diag.vehicle_gap_ticks.fetch_add(1, std::memory_order_relaxed);
  }
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
  double car_yaw = SelectCarYaw(context, velocity_yaw, velocity_length / 100.0,
                                velocity_length > 50.0);
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
  // Experiment (T3.4): the slip angle twice, on the same tick. `position` is
  // what the model uses today, differentiated from the Pawn root's bounds
  // origin. `speed` comes straight from the host's forward speed over the
  // travel speed, so it needs no differentiation at all. If the second one is
  // steady while the first flickers, the derivative is the twitch.
  const double slip_position_degrees =
      slip_angle * kRadiansToDegrees;
  double slip_speed_degrees = 0.0;
  if (vehicle_speed_valid && speed_mps > 0.5) {
    const double forward_mps = std::fabs(
        context.diag.slip_signed_speed_kmh.load(std::memory_order_relaxed) /
        3.6);
    const double ratio =
        nfs2015_chase::Clamp(forward_mps / speed_mps, 0.0, 1.0);
    slip_speed_degrees = std::acos(ratio) * kRadiansToDegrees;
  }
  context.diag.slip_from_position_degrees.store(
      slip_position_degrees, std::memory_order_relaxed);
  context.diag.slip_from_speed_degrees.store(slip_speed_degrees,
                                             std::memory_order_relaxed);
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

  if (vehicle_valid) {
    UpdateBrakeInput(context, vehicle_valid, vehicle_speed_valid,
                     vehicle_speed_fresh, vehicle_speed_kmh, dt);
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

  // T2: the tick no longer advances the model - it publishes a car sample and
  // the render thread integrates the springs on its own clock.
  {
    nfs2015_chase::CarSample sample;
    sample.time_ns = NowNanoseconds();
    sample.car = car;
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
  // The tick no longer advances the model, so `delta_seconds` is unused here;
  // keep taking it because the ABI defines the signature.
  (void)dt;

  {
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = true;
    context.display.player_valid = true;
    context.display.vehicle_service_available = vehicle_service;
    context.display.speed_kmh = vehicle_speed_kmh > 0.5
                                    ? vehicle_speed_kmh
                                    : speed_mps * 3.6;
    context.display.slip_degrees = slip_angle * kRadiansToDegrees;
    context.display.drift =
        context.presenter_drift.load(std::memory_order_relaxed);
    context.display.fov_horizontal = 0.0;
    context.display.camera_distance_m =
        context.presenter_distance_cm.load(std::memory_order_relaxed) / 100.0;
    context.display.camera_distance_raw_m =
        context.display.camera_distance_m;
  }
}

// ---------------------------------------------------------------------------
// Probe (PLAN-camera-timing.md §8.1). See the `Probe` declaration above for
// what each number is for. Every entry point here is a no-op until
// `anomaly.ue5.ahud` is published, so this cannot break a working install.
// ---------------------------------------------------------------------------

bool ProbeAhudReady(const AnomalyUe5AhudServiceV1 *service) noexcept {
  return HasField<AnomalyUe5AhudServiceV1,
                  decltype(AnomalyUe5AhudServiceV1::unsubscribe)>(
             service, offsetof(AnomalyUe5AhudServiceV1, unsubscribe)) &&
         service->subscribe != nullptr && service->unsubscribe != nullptr;
}

// Runs on the Game thread inside the host's AHUD frame dispatch, i.e. once per
// rendered frame. It only counts, so it measures the real frame rate without
// touching the engine. That rate is what decides how much of the fixed 1/120
// step's error remains visible: at 59 fps one substep is half a frame, at 30 fps
// it is a quarter.
// One projected world point. The residual is this frame's departure from
// carrying on at the previous screen velocity, converted back to pixels by
// multiplying by the frame time: it says how far the point moved differently,
// and it does not care how long the frame was.
struct ScreenPointProjector final {
  const AnomalyUe5AhudFrameV1 *frame;

  [[nodiscard]] bool Project(const double world[3], float screen[2]) const noexcept {
    if (frame->project(frame->user, world, screen, nullptr) == 0) {
      return false;
    }
    return std::isfinite(screen[0]) && std::isfinite(screen[1]);
  }
};

// Residual for a single scalar: this frame's departure from carrying on at the
// previous rate, multiplied back by the frame time. Returns false when there is
// not enough history or the frame lengths are implausible.
bool PushScalarSample(Probe::ScalarTrack &track, const double value,
                      const std::int64_t now_ns,
                      double *residual_out) noexcept {
  if (track.count >= 2U) {
    const double dt_now = static_cast<double>(now_ns - track.ns[0]) * 1e-9;
    const double dt_was =
        static_cast<double>(track.ns[0] - track.ns[1]) * 1e-9;
    if (dt_now > 0.002 && dt_now < 0.2 && dt_was > 0.002 && dt_was < 0.2) {
      const double rate_now = (value - track.value[0]) / dt_now;
      const double rate_was = (track.value[0] - track.value[1]) / dt_was;
      const double residual = (rate_now - rate_was) * dt_now;
      if (std::isfinite(residual) && std::fabs(residual) < 180.0) {
        *residual_out = residual;
        track.value[1] = track.value[0];
        track.value[0] = value;
        track.ns[1] = track.ns[0];
        track.ns[0] = now_ns;
        return true;
      }
    }
  }
  track.value[1] = track.value[0];
  track.value[0] = value;
  track.ns[1] = track.ns[0];
  track.ns[0] = now_ns;
  if (track.count < 2U) {
    ++track.count;
  }
  return false;
}

void ResetResidual(std::atomic<std::uint64_t> &peak,
                   std::atomic<std::uint64_t> &sum,
                   std::atomic<std::uint64_t> &count) noexcept {
  peak.store(0, std::memory_order_relaxed);
  sum.store(0, std::memory_order_relaxed);
  count.store(0, std::memory_order_relaxed);
}

void AccumulateResidual(std::atomic<std::uint64_t> &peak,
                        std::atomic<std::uint64_t> &sum,
                        std::atomic<std::uint64_t> &count,
                        const double magnitude,
                        const double scale) noexcept {
  if (!std::isfinite(magnitude) || magnitude < 0.0) {
    return;
  }
  const auto units = static_cast<std::uint64_t>(magnitude * scale + 0.5);
  RaisePeak(peak, units);
  sum.fetch_add(units, std::memory_order_relaxed);
  count.fetch_add(1, std::memory_order_relaxed);
}

void PushScreenSample(const ScreenPointProjector &projector,
                      const double world[3], const std::int64_t now_ns,
                      Probe::ScreenTrack &track,
                      std::atomic<std::uint64_t> &peak,
                      std::atomic<std::uint64_t> &sum,
                      std::atomic<std::uint64_t> &count) noexcept {
  float screen[2]{};
  if (!projector.Project(world, screen)) {
    return;
  }
  if (track.count >= 2U) {
    const double dt_now = static_cast<double>(now_ns - track.ns[0]) * 1e-9;
    const double dt_was = static_cast<double>(track.ns[0] - track.ns[1]) * 1e-9;
    if (dt_now > 0.002 && dt_now < 0.2 && dt_was > 0.002 && dt_was < 0.2) {
      double residual2 = 0.0;
      for (std::size_t axis = 0; axis < 2; ++axis) {
        const double velocity_now =
            (static_cast<double>(screen[axis]) - track.px[0][axis]) / dt_now;
        const double velocity_was =
            (track.px[0][axis] - track.px[1][axis]) / dt_was;
        const double kink = (velocity_now - velocity_was) * dt_now;
        residual2 += kink * kink;
      }
      const double pixels = std::sqrt(residual2);
      // A reset, a teleport or a camera cut would read as one enormous spike.
      if (std::isfinite(pixels) && pixels < 2000.0) {
        const auto tenths = static_cast<std::uint64_t>(pixels * 10.0 + 0.5);
        RaisePeak(peak, tenths);
        sum.fetch_add(tenths, std::memory_order_relaxed);
        count.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  track.px[1][0] = track.px[0][0];
  track.px[1][1] = track.px[0][1];
  track.ns[1] = track.ns[0];
  track.px[0][0] = static_cast<double>(screen[0]);
  track.px[0][1] = static_cast<double>(screen[1]);
  track.ns[0] = now_ns;
  if (track.count < 2U) {
    ++track.count;
  }
  track.valid = true;
}

void ANOMALY_CALL OnProbeFrame(void *user,
                               const AnomalyUe5AhudFrameV1 *frame) noexcept {
  auto *context = static_cast<Context *>(user);
  if (context == nullptr || frame == nullptr) {
    return;
  }
  ++context->probe.frame_calls;
  ++context->probe.frame_window_calls;

  // Judder instrument. Ask the game where it puts a world point on screen this
  // frame, using the same projection it draws with, and measure how far that
  // pixel position departs from carrying on at the previous velocity. Two
  // points, because the two possible faults need telling apart:
  //
  //   anchor  the car position this camera was built from. Rough => the camera
  //           itself is moving in steps.
  //   car     the raw newest Game-tick position, which the Game only moves once
  //           a tick. Rough => the staircase in the data, with the camera
  //           smooth; the picture would then be fine and this number would be
  //           an artefact of which point is measured.
  if (frame->project == nullptr || frame->user == nullptr) {
    return;
  }
  Probe &probe = context->probe;
  const std::int64_t now_ns = NowNanoseconds();
  ScreenPointProjector project{frame};
  if (context->probe_car_valid.load(std::memory_order_relaxed)) {
    const double car[3] = {
        context->probe_car_cm[0].load(std::memory_order_relaxed),
        context->probe_car_cm[1].load(std::memory_order_relaxed),
        context->probe_car_cm[2].load(std::memory_order_relaxed)};
    PushScreenSample(project, car, now_ns, probe.track_car,
                     context->diag.judder_car_peak_dpx,
                     context->diag.judder_car_sum_dpx,
                     context->diag.judder_car_count);
  }
  const double anchor[3] = {
      context->presenter_anchor[0].load(std::memory_order_relaxed),
      context->presenter_anchor[1].load(std::memory_order_relaxed),
      context->presenter_anchor[2].load(std::memory_order_relaxed)};
  PushScreenSample(project, anchor, now_ns, probe.track_anchor,
                   context->diag.judder_peak_dpx,
                   context->diag.judder_sum_dpx,
                   context->diag.judder_count);

  // The camera's own speed. A camera that translates in steps makes the world
  // flow in steps; the car sits still in the middle of the frame throughout.
  {
    const double eye[3] = {
        context->presenter_pose_location[0].load(std::memory_order_relaxed),
        context->presenter_pose_location[1].load(std::memory_order_relaxed),
        context->presenter_pose_location[2].load(std::memory_order_relaxed)};
    if (probe.eye_count >= 1U) {
      const double dt = static_cast<double>(now_ns - probe.eye_ns[0]) * 1e-9;
      if (dt > 0.002 && dt < 0.2) {
        double jump2 = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double speed = (eye[axis] - probe.eye_cm[0][axis]) / dt;
          const double previous =
              probe.eye_count >= 2U
                  ? (probe.eye_cm[0][axis] - probe.eye_cm[1][axis]) /
                        (static_cast<double>(probe.eye_ns[0] -
                                             probe.eye_ns[1]) *
                         1e-9)
                  : speed;
          const double kink = (speed - previous) * dt;
          jump2 += kink * kink;
        }
        const double centimetres = std::sqrt(jump2);
        if (std::isfinite(centimetres) && centimetres < 500.0) {
          const auto ccm =
              static_cast<std::uint64_t>(centimetres * 100.0 + 0.5);
          RaisePeak(context->diag.flow_peak_ccm, ccm);
          context->diag.flow_sum_ccm.fetch_add(ccm, std::memory_order_relaxed);
          context->diag.flow_count.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.eye_cm[1][axis] = probe.eye_cm[0][axis];
      probe.eye_cm[0][axis] = eye[axis];
    }
    probe.eye_ns[1] = probe.eye_ns[0];
    probe.eye_ns[0] = now_ns;
    if (probe.eye_count < 2U) {
      ++probe.eye_count;
    }
  }
  // Frame pacing. A frame that takes half as long again as the average is a
  // visible stutter no matter how smooth the camera's math is, and it is the one
  // source of judder that lives outside the pose entirely.
  {
    const auto micros = static_cast<std::uint64_t>(
        static_cast<double>(now_ns - probe.frame_last_ns) / 1000.0 + 0.5);
    if (probe.frame_last_ns != 0 && micros > 200 && micros < 200000) {
      RaisePeak(context->diag.frame_peak_us, micros);
      context->diag.frame_sum_us.fetch_add(micros, std::memory_order_relaxed);
      context->diag.frame_count.fetch_add(1, std::memory_order_relaxed);
    }
    probe.frame_last_ns = now_ns;
  }

  // Rotation and lens, the two channels a point at the centre cannot see.
  double residual = 0.0;
  if (PushScalarSample(
          probe.track_pitch,
          context->presenter_pose_rotation[0].load(std::memory_order_relaxed),
          now_ns, &residual)) {
    AccumulateResidual(context->diag.pitch_peak_cdeg,
                       context->diag.pitch_sum_cdeg,
                       context->diag.pitch_count, std::fabs(residual), 100.0);
  }
  if (PushScalarSample(
          probe.track_roll,
          context->presenter_pose_rotation[2].load(std::memory_order_relaxed),
          now_ns, &residual)) {
    AccumulateResidual(context->diag.roll_peak_cdeg,
                       context->diag.roll_sum_cdeg,
                       context->diag.roll_count, std::fabs(residual), 100.0);
  }
  if (PushScalarSample(
          probe.track_lens,
          static_cast<double>(
              context->presenter_pose_lens.load(std::memory_order_relaxed)),
          now_ns, &residual)) {
    AccumulateResidual(context->diag.lens_peak_cdeg,
                       context->diag.lens_sum_cdeg,
                       context->diag.lens_count, std::fabs(residual), 100.0);
  }
}

void ReleaseProbeSubscription(Context &context) noexcept {
  if (context.ahud != nullptr && context.probe.subscribed &&
      ProbeAhudReady(context.ahud) && context.probe.subscription.id != 0) {
    // The host drains an already-entered callback before returning, and Stop
    // runs before Unload, so the Context outlives the last callback.
    static_cast<void>(context.ahud->unsubscribe(context.ahud->user,
                                                context.probe.subscription));
  }
  context.probe.subscribed = false;
  context.probe.subscription = {};
}

// Called right after the Game tick has read this tick's snapshots, so
// `camera_yaw_radians` and `probe_bounds_*` describe the same sample.
void UpdateProbeYaw(Context &context) noexcept {
  Probe &probe = context.probe;
  // Vertical bob inherited from the mesh bound radius, measured every tick even
  // when the camera yaw is unavailable.
  if (context.probe_have_extent) {
    const double step =
        std::fabs(context.probe_extent_z_cm - context.probe_last_extent_z_cm);
    if (step > probe.height_step_cm) {
      probe.height_step_cm = step;
    }
  }
  context.probe_last_extent_z_cm = context.probe_extent_z_cm;
  context.probe_have_extent = true;
  probe.camera_yaw_valid = context.camera_yaw_valid;
  probe.camera_minus_bounds_valid = false;
  if (!context.camera_yaw_valid) {
    probe.have_last_camera_yaw = false;
    return;
  }
  const double camera_degrees =
      context.camera_yaw_radians * nfs2015_chase::kDegreesPerRadian;
  probe.camera_yaw_degrees = camera_degrees;
  if (probe.have_last_camera_yaw) {
    // Shortest arc: the yaw crosses +-180 while drifting, and a plain
    // subtraction would report 360 there.
    const double step = std::fabs(
        std::remainder(camera_degrees - probe.last_camera_yaw_degrees, 360.0));
    if (step > probe.camera_yaw_step_max_ddeg) {
      probe.camera_yaw_step_max_ddeg = step;
    }
    probe.camera_yaw_step_sum_ddeg += step;
    ++probe.camera_yaw_step_count;
  }
  probe.last_camera_yaw_degrees = camera_degrees;
  probe.have_last_camera_yaw = true;

  const double dx = context.probe_bounds_dx_cm;
  const double dy = context.probe_bounds_dy_cm;
  const double offset_cm = std::sqrt(dx * dx + dy * dy);
  probe.bounds_offset_cm = offset_cm;
  probe.bounds_yaw_valid =
      offset_cm >= kProbeMinimumBoundsOffsetCm &&
      offset_cm <= kProbeMaximumBoundsOffsetCm;
  if (!probe.bounds_yaw_valid) {
    return;
  }
  probe.bounds_yaw_degrees =
      std::atan2(dy, dx) * nfs2015_chase::kDegreesPerRadian;
  probe.camera_minus_bounds_degrees = std::remainder(
      camera_degrees - probe.bounds_yaw_degrees, 360.0);
  probe.camera_minus_bounds_valid = true;
}

void RunProbe(Context &context, const double dt) noexcept {
  Probe &probe = context.probe;
  UpdateProbeYaw(context);
  // Subscribe to the once-per-frame callback. The binding may still be
  // resolving on the first ticks, so retry before calling it missing.
  if (probe.subscribe_state != kProbeOk && probe.subscribe_attempts < 20 &&
      context.ahud != nullptr && ProbeAhudReady(context.ahud) &&
      context.elapsed >= probe.next_subscribe_elapsed) {
    ++probe.subscribe_attempts;
    AnomalyGenerationHandleV1 handle{};
    const auto status = context.ahud->subscribe(context.ahud->user, OnProbeFrame,
                                                &context, &handle);
    probe.subscribe_code = status.code;
    if (status.code == ANOMALY_STATUS_V1_OK && handle.id != 0) {
      probe.subscription = handle;
      probe.subscribed = true;
      probe.subscribe_state = kProbeOk;
    } else {
      probe.subscribe_state = kProbeMissing;
      probe.next_subscribe_elapsed = context.elapsed + 2.0;
    }
  }
  // Turn the frame counter into a rate once a second.
  if (probe.subscribed && dt > 0.0) {
    probe.frame_window_elapsed += dt;
    if (probe.frame_window_elapsed >= 1.0) {
      probe.frame_hz = static_cast<double>(probe.frame_window_calls) /
                       probe.frame_window_elapsed;
      probe.frame_window_calls = 0;
      probe.frame_window_elapsed = 0.0;
    }
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
  context.display.vehicle_gap_ticks =
      diag.vehicle_gap_ticks.load(std::memory_order_relaxed);
  context.display.player_sequence_jumps =
      diag.player_sequence_jumps.load(std::memory_order_relaxed);
  context.display.vehicle_polls =
      diag.vehicle_polls.load(std::memory_order_relaxed);
  context.display.camera_yaw_esp_misses =
      diag.camera_yaw_esp_misses.load(std::memory_order_relaxed);
  context.display.slip_signed_speed_kmh =
      diag.slip_signed_speed_kmh.load(std::memory_order_relaxed);
  context.display.slip_from_position_degrees =
      diag.slip_from_position_degrees.load(std::memory_order_relaxed);
  context.display.slip_from_speed_degrees =
      diag.slip_from_speed_degrees.load(std::memory_order_relaxed);
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
  context.display.diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  context.display.diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  context.display.diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  context.display.diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
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
  // Optional engine access for the probe. Declared optional in the manifest, so
  // a host without `ue5.ahud` simply leaves this null and the probe reports
  // "off" instead of failing the load.
  context->ahud = Query<AnomalyUe5AhudServiceV1>(
      host, ANOMALY_UE5_AHUD_SERVICE_V1_ID, ANOMALY_UE5_AHUD_SERVICE_V1_VERSION);
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
  context->diag.vehicle_gap_ticks.store(0, std::memory_order_relaxed);
  context->diag.player_sequence_jumps.store(0, std::memory_order_relaxed);
  context->diag.vehicle_polls.store(0, std::memory_order_relaxed);
  context->diag.camera_yaw_esp_misses.store(0, std::memory_order_relaxed);
  context->diag.slip_signed_speed_kmh.store(0.0, std::memory_order_relaxed);
  context->diag.slip_from_position_degrees.store(0.0,
                                                 std::memory_order_relaxed);
  context->diag.slip_from_speed_degrees.store(0.0,
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
  context->detour_hz = 0.0;
  context->next_vehicle_query_elapsed = 0.0;
  context->vehicle_cached_valid = false;
  context->vehicle_cached_speed_valid = false;
  context->vehicle_cached_speed_kmh = 0.0;
  context->vehicle_cached_fresh = false;
  context->brake_last_target = 0.0;
  context->vehicle_cached_handle = {};
  context->base_diagnostic_logged = false;
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
  ++context->sample_epoch;
  context->sample_mean_interval_ns.store(0.0, std::memory_order_relaxed);
  context->borrow_valid = false;
  context->borrow_last_ns = 0;
  context->car_ring.Clear();
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
  ReleaseProbeSubscription(*context);
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
        context->diag.slip_max_step_ddeg.store(0,
                                               std::memory_order_relaxed);
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    RetryHookIfNeeded(*context);
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

void DrawCountLine(const AnomalyUiServiceV1 *ui,
                   const nfs2015_chase_cam::Localizer &localizer,
                   const char *key, const char *fallback,
                   const std::uint64_t value) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "%llu",
                static_cast<unsigned long long>(value));
  DrawStatusLine(ui, localizer, key, fallback, buffer);
}

// Which channel is actually twitching, as one word.
//
// Four quantities are now measured per frame and they are not in the same unit,
// so comparing them needs a threshold each: a pixel of car-on-screen wobble is
// about 0.05 degrees, and a tenth of a degree of rotation or lens is about two
// pixels. Score each against its own threshold and name the worst one. The
// point is to stop the loop of reading numbers back and interpreting them.
enum class JudderChannel {
  kNone = 0,
  kPosition,
  kPitch,
  kRoll,
  kLens,
  kReset,
  kFlow,
  kFrame,
};

[[nodiscard]] JudderChannel WorstJudderChannel(
    const DisplayState &display) noexcept {
  constexpr double kPositionThresholdPx = 2.0;
  constexpr double kAngleThresholdDegrees = 0.10;
  JudderChannel worst = JudderChannel::kNone;
  double score = 1.0;
  const auto consider = [&score, &worst](const JudderChannel channel,
                                         const double value,
                                         const double threshold) {
    if (!(threshold > 0.0) || !std::isfinite(value)) {
      return;
    }
    const double candidate = value / threshold;
    if (candidate > score) {
      score = candidate;
      worst = channel;
    }
  };
  if (display.judder_valid) {
    consider(JudderChannel::kPosition, display.judder_avg_px,
             kPositionThresholdPx);
  }
  if (display.rotation_valid) {
    consider(JudderChannel::kPitch, display.pitch_avg_degrees,
             kAngleThresholdDegrees);
    consider(JudderChannel::kRoll, display.roll_avg_degrees,
             kAngleThresholdDegrees);
  }
  if (display.lens_valid) {
    consider(JudderChannel::kLens, display.lens_avg_degrees,
             kAngleThresholdDegrees);
  }
  // One reset in half a second is already two visible snaps a second, and each
  // one is a large event, so it outweighs everything else by construction.
  // A centimetre of camera position per frame is a fifth of a degree at five
  // metres, so two centimetres is where the world visibly changes speed.
  consider(JudderChannel::kFlow, display.flow_avg_cm, 2.0);
  // A frame half again as long as the average is a visible stutter. Scored as
  // how far the longest frame of the window runs past four thirds of the mean.
  if (display.frame_valid && display.frame_mean_ms > 1.0) {
    const double ratio = display.frame_peak_ms / display.frame_mean_ms;
    consider(JudderChannel::kFrame, (ratio - 1.0) * display.frame_mean_ms,
             0.4 * display.frame_mean_ms);
  }
  consider(JudderChannel::kReset,
           static_cast<double>(display.resets_in_window), 1.0);
  return worst;
}

// T0: the counters behind PLAN-camera-timing.md. Collapsed by default so they
// do not compete with the settings, but they are the only way to tell a silent
// degradation (source flip, torn read, dropped tick) from a tuning issue.
void DrawDiagnostics(const AnomalyUiServiceV1 *ui,
                     const nfs2015_chase_cam::Localizer &localizer,
                     const DisplayState &display) {
  char line[128]{};
  // One word first: which channel is twitching. Everything below it is the raw
  // evidence for that verdict.
  {
    const JudderChannel channel = WorstJudderChannel(display);
    const char *fallbacks[8] = {"none yet", "position", "pitch",
                                "roll",     "lens",     "camera reset",
                                "world flow", "frame pacing"};
    char key[48]{};
    std::snprintf(key, sizeof(key), "diag.worst.%d",
                  static_cast<int>(channel));
    DrawStatusLine(ui, localizer, key,
                   fallbacks[static_cast<int>(channel)],
                   channel == JudderChannel::kNone ? "-" : "<= twitching");
  }
  // Judder instrument and the two defect amplitudes, then the probe. Read-only
  // reconnaissance, so it comes before everything else: the diagnostics body
  // scrolls and these are the numbers this round of work is looking at.
  // Every header value carries its own tag, because these lines have been
  // confused with each other when the units looked alike.
  if (display.judder_valid) {
    std::snprintf(line, sizeof(line), "A %.1f / %.1f px",
                  display.judder_peak_px, display.judder_avg_px);
  } else {
    std::snprintf(line, sizeof(line), "A --");
  }
  DrawStatusLine(ui, localizer, "diag.judder",
                 "A: anchor through the game projection, peak/avg px", line);
  if (display.rotation_valid) {
    std::snprintf(line, sizeof(line), "P %.2f / %.2f  R %.2f / %.2f deg",
                  display.pitch_peak_degrees, display.pitch_avg_degrees,
                  display.roll_peak_degrees, display.roll_avg_degrees);
  } else {
    std::snprintf(line, sizeof(line), "P --  R --");
  }
  DrawStatusLine(ui, localizer, "diag.rotation",
                 "P/R: pitch and roll residual per frame, peak/avg deg", line);
  if (display.lens_valid) {
    std::snprintf(line, sizeof(line), "F %.2f / %.2f deg",
                  display.lens_peak_degrees, display.lens_avg_degrees);
  } else {
    std::snprintf(line, sizeof(line), "F --");
  }
  DrawStatusLine(ui, localizer, "diag.lens_residual",
                 "F: field of view residual per frame, peak/avg deg", line);
  if (display.flow_valid) {
    std::snprintf(line, sizeof(line), "W %.2f / %.2f cm",
                  display.flow_peak_cm, display.flow_avg_cm);
  } else {
    std::snprintf(line, sizeof(line), "W --");
  }
  DrawStatusLine(ui, localizer, "diag.flow",
                 "W: the camera's own speed jump per frame, peak/avg cm",
                 line);
  if (display.frame_valid) {
    std::snprintf(line, sizeof(line), "T %.1f / %.1f ms",
                  display.frame_peak_ms, display.frame_mean_ms);
  } else {
    std::snprintf(line, sizeof(line), "T --");
  }
  DrawStatusLine(ui, localizer, "diag.frame_pacing",
                 "T: frame length, peak / mean ms", line);

  if (!display.probe_ahud_service) {
    std::snprintf(line, sizeof(line), "no service");
  } else if (display.probe_subscribe_state == kProbeOk) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f Hz", display.probe_frame_hz,
                  display.update_hz);
  } else if (display.probe_subscribe_state == kProbeMissing) {
    std::snprintf(line, sizeof(line), "code %u",
                  display.probe_subscribe_code);
  } else {
    std::snprintf(line, sizeof(line), "wait");
  }
  DrawStatusLine(ui, localizer, "diag.probe_frame_rate",
                 "Probe frames/s vs Game ticks/s, <1.0 leaves steps visible",
                 line);
  std::snprintf(line, sizeof(line), "%.1f cm", display.probe_bounds_offset_cm);
  DrawStatusLine(ui, localizer, "diag.probe_bounds_offset",
                 "Probe bounds-centre lever arm, <20 cm unusable", line);
  if (display.probe_bounds_yaw_valid) {
    std::snprintf(line, sizeof(line), "%.1f deg",
                  display.probe_camera_minus_bounds_degrees);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.probe_camera_vs_bounds",
                 "Probe camera yaw minus bounds yaw, drift shows the slip",
                 line);
  std::snprintf(line, sizeof(line), "%.1f / %.1f Hz", display.detour_hz,
                display.update_hz);
  DrawStatusLine(ui, localizer, "diag.detour_rate",
                 "Presenter / Game tick", line);
  std::snprintf(line, sizeof(line), "%.2f / %.2f / %.2f / %.2f ms",
                display.tick_ms_last, display.tick_ms_avg, display.tick_ms_p99,
                display.tick_ms_max);
  DrawStatusLine(ui, localizer, "diag.tick_cost",
                 "Camera tick (last/avg/p99/max)", line);
  // The presentation numbers come first: the diagnostics body scrolls, so
  // anything appended after the counters is off screen on a small window.
  if (display.render_speed_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f m/s",
                  display.render_speed_floor_mps,
                  display.render_speed_peak_mps);
  } else {
    std::snprintf(line, sizeof(line), "-- / -- m/s");
  }
  DrawStatusLine(ui, localizer, "diag.render_speed",
                 "Camera-vs-car speed, 0.5 s, <3 m/s", line);
  if (display.render_turn_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f deg/s",
                  display.render_turn_floor_dps, display.render_turn_peak_dps);
  } else {
    std::snprintf(line, sizeof(line), "-- / -- deg/s");
  }
  DrawStatusLine(ui, localizer, "diag.render_turn",
                 "Camera-vs-car turn rate, 0.5 s, should match", line);
  std::snprintf(line, sizeof(line), "%.1f cm", display.car_step_cm);
  DrawStatusLine(ui, localizer, "diag.car_step",
                 "Anchor step per tick, the scale for the two above", line);
  if (display.judder_car_valid) {
    std::snprintf(line, sizeof(line), "C %.1f / %.1f px",
                  display.judder_car_peak_px, display.judder_car_avg_px);
  } else {
    std::snprintf(line, sizeof(line), "C --");
  }
  DrawStatusLine(ui, localizer, "diag.judder_car",
                 "C: raw Game-tick car position, same projection, px", line);
  if (display.tick_interval_valid) {
    std::snprintf(line, sizeof(line), "T %.1f / %.1f ms",
                  display.tick_interval_min_ms, display.tick_interval_max_ms);
  } else {
    std::snprintf(line, sizeof(line), "T --");
  }
  DrawStatusLine(ui, localizer, "diag.tick_interval",
                 "T: Game tick interval, min / max ms", line);
  std::snprintf(line, sizeof(line), "H %.1f cm", display.probe_height_step_cm);
  DrawStatusLine(ui, localizer, "diag.height_step",
                 "Mesh bound radius step per tick, moves the eye up and down",
                 line);
  std::snprintf(line, sizeof(line), "%.2f / %.2f deg",
                display.probe_camera_yaw_step_max_degrees,
                display.probe_camera_yaw_step_avg_degrees);
  DrawStatusLine(ui, localizer, "diag.probe_camera_yaw_step",
                 "Game camera yaw step per tick, max/avg, >0.3 is noise", line);
  DrawCountLine(ui, localizer, "diag.snap", "View snaps",
                display.snap_total);
  static constexpr const char *kSnapKeys[kSnapReasonCount] = {
      "diag.snap_motion", "diag.snap_teleport", "diag.snap_inactive",
      "diag.snap_preset", "diag.snap_user_reset"};
  static constexpr const char *kSnapFallbacks[kSnapReasonCount] = {
      "  motion reset", "  teleport step", "  camera idle",
      "  preset switch", "  user reset"};
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    if (display.snap_by_reason[index] == 0) {
      continue;
    }
    DrawCountLine(ui, localizer, kSnapKeys[index], kSnapFallbacks[index],
                  display.snap_by_reason[index]);
  }
  DrawCountLine(ui, localizer, "diag.yaw_switches",
                "Car yaw: camera vs slip-limit", display.yaw_source_switches);
  DrawCountLine(ui, localizer, "diag.observed_failures",
                "Observed pose read failures",
                display.observed_read_failures);
  DrawCountLine(ui, localizer, "diag.lease_failures", "Hook lease failures",
                display.lease_failures);
  DrawCountLine(ui, localizer, "diag.torn_reads", "Torn view reads",
                display.view_read_torn);
  DrawCountLine(ui, localizer, "diag.detour_missing",
                "Detour calls without a pose", display.detour_no_publish);
  DrawCountLine(ui, localizer, "diag.detour_threads", "Detour threads",
                display.detour_thread_count);
  DrawCountLine(ui, localizer, "diag.vehicle_polls",
                "Vehicle service polls", display.vehicle_polls);
  DrawCountLine(ui, localizer, "diag.camera_yaw_fallback",
                "Ticks without the Game-tick car yaw",
                display.camera_yaw_esp_misses);
  std::snprintf(line, sizeof(line), "%.1f km/h",
                display.slip_signed_speed_kmh);
  DrawStatusLine(ui, localizer, "diag.signed_speed",
                 "Vehicle forward speed (signed)", line);
  std::snprintf(line, sizeof(line), "%.1f / %.1f deg",
                display.slip_from_position_degrees,
                display.slip_from_speed_degrees);
  DrawStatusLine(ui, localizer, "diag.slip_compare",
                 "Slip: from position / from speed, <3 deg apart", line);
  DrawCountLine(ui, localizer, "diag.slip_flips",
                "Slip sign changes, >3 suspicious",
                display.slip_sign_changes);
  // The thresholds are printed next to the values so the panel can be read
  // without knowing what "large" means for each number.
  std::snprintf(line, sizeof(line), "%.1f deg",
                display.slip_max_step_degrees);
  DrawStatusLine(ui, localizer, "diag.slip_step",
                 "Largest slip step per tick, 0.5 s, <2 deg", line);
  DrawCountLine(ui, localizer, "diag.vehicle_gaps",
                "Ticks on a stale vehicle snapshot",
                display.vehicle_gap_ticks);
  DrawCountLine(ui, localizer, "diag.sequence_jumps",
                "Player sequence jumps", display.player_sequence_jumps);
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

      const std::string flat_label = context->localizer.Label(
          "setting.diagnostic_flat_pose",
          "Diagnostic: position and yaw only", "chase-flat-pose");
      int flat_pose = display.diagnostic_flat_pose ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(flat_label), &flat_pose) != 0) {
        context->diagnostic_flat_pose.store(flat_pose != 0,
                                            std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string zero_lag_label = context->localizer.Label(
          "setting.diagnostic_zero_lag",
          "Diagnostic: zero lag, newest tick position", "chase-zero-lag");
      int zero_lag = display.diagnostic_zero_lag ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(zero_lag_label), &zero_lag) != 0) {
        context->diagnostic_zero_lag.store(zero_lag != 0,
                                           std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string skip_position_label = context->localizer.Label(
          "setting.diagnostic_skip_position",
          "Diagnostic: keep the Game's position", "chase-skip-position");
      int skip_position = display.diagnostic_skip_position ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(skip_position_label),
                       &skip_position) != 0) {
        context->diagnostic_skip_position.store(skip_position != 0,
                                                std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string skip_rotation_label = context->localizer.Label(
          "setting.diagnostic_skip_rotation",
          "Diagnostic: keep the Game's rotation", "chase-skip-rotation");
      int skip_rotation = display.diagnostic_skip_rotation ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(skip_rotation_label),
                       &skip_rotation) != 0) {
        context->diagnostic_skip_rotation.store(skip_rotation != 0,
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
      const std::string diagnostics_label = context->localizer.Label(
          "setting.diagnostics", "Diagnostics", "chase-diagnostics");
      int diagnostics_open = context->diagnostics_open ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(diagnostics_label),
                       &diagnostics_open) != 0) {
        context->diagnostics_open = diagnostics_open != 0;
      }
      if (context->diagnostics_open) {
        DrawDiagnostics(ui, context->localizer, display);
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
