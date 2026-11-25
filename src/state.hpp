// The plugin's shared state: the diagnostic enums, one structure per subsystem,
// and the single Context that ties them together. Field order and comments are
// part of the threading contract documented in Context, so they are moved
// verbatim from the single-file plugin.
#pragma once

#include "constants.hpp"
#include "settings_groups.hpp"

#include "anomaly/sdk/cpp.hpp"
#include "anomaly/sdk/services/core.h"
#include "anomaly/sdk/services/interop.h"
#include "anomaly/sdk/services/json.h"
#include "anomaly/sdk/services/nte.h"
#include "anomaly/sdk/services/ui.h"
#include "anomaly/sdk/services/ui_resources.h"
#include "anomaly/sdk/services/ue5.h"

#include "burnout_shot.hpp"
#include "car_sample.hpp"
#include "chase_cam.hpp"
#include "latency_window.hpp"
#include "localization.hpp"
#include "orbit_shot.hpp"
#include "orbit_ui_hide.hpp"
#include "wheel_steer.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace nfs2015_chase_cam::detail {

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
  // The rendered vehicle proxy's chassis forward. Unlike the game-camera yaw
  // above, this is the car's actual heading and does not inherit the camera's
  // own smoothing or framing offset.
  kYawSourceBody = 4,
};

// What produced the speed shown in the panel. Anomaly 2.7.0 has no exact
// vehicle speed service, so the camera currently always reports `Derived`.
// `Exact` remains reserved for task group 3 if the public-ABI query returns.
enum SpeedSource : std::uint8_t {
  kSpeedSourceUnavailable = 0,
  kSpeedSourceDerived = 1,
  kSpeedSourceExact = 2,
};

// Phase 0 burnout reconnaissance. Everything here is read-only and lives on
// the Game tick; none of it feeds the camera yet. The function owners and
// parameter sizes were verified against the live UE reflection table with
// `anomaly-cli ue functions <name>` (Anomaly 2.7.0). Keep the exact paths:
// resolving a function on the wrong owner is the usual reason a getter looks
// unavailable even though the game has it.
enum BurnoutProbeReceiver : std::uint8_t {
  kBurnoutReceiverVehicle = 0,
  kBurnoutReceiverMovement = 1,
  kBurnoutReceiverRuntimeInfo = 2,
};

enum BurnoutProbeValue : std::uint8_t {
  kBurnoutValueBool = 0,
  kBurnoutValueFloat = 1,
  kBurnoutValueInt = 2,
  kBurnoutValuePointer = 3,
  kBurnoutValueStruct = 4,
};

enum BurnoutProbeStatus : std::uint8_t {
  kBurnoutProbeUnresolved = 0,
  kBurnoutProbeResolved = 1,
  kBurnoutProbeOk = 2,
  kBurnoutProbeTooSmall = 3,
};

enum BurnoutProbeIndex : std::size_t {
  kBurnoutProbeIsBurnout = 0,
  kBurnoutProbeThrottle,
  kBurnoutProbeBrake,
  kBurnoutProbeSteering,
  kBurnoutProbeSkidAngle,
  kBurnoutProbePlayerThrottle,
  kBurnoutProbeWheelBurnout,
  kBurnoutProbeWheelBurnoutRep,
  kBurnoutProbeWheelAngularSpeed,
  kBurnoutProbeRuntimeInfo,
  kBurnoutProbeWheelCount,
  kBurnoutProbeInputInfo,
  kBurnoutProbeCount
};

struct BurnoutProbeSpec final {
  const char *label;
  const char *path;
  BurnoutProbeReceiver receiver;
  BurnoutProbeValue value;
  std::uint16_t return_offset;
  // 0 = no parameter, 1 = uint8 wheel index, 4 = int32 wheel index.
  std::uint8_t input_size;
};

inline constexpr BurnoutProbeSpec kBurnoutProbeSpecs[] = {
    {"isBurnout",
     "/Script/HTGame.HTVehicleMovementComponent:GetVehicleIsBurnout",
     kBurnoutReceiverMovement, kBurnoutValueBool, 0, 0},
    {"throttle",
     "/Script/HTGame.HTVehicleMovementComponent:GetVehicleThrottle",
     kBurnoutReceiverMovement, kBurnoutValueFloat, 0, 0},
    {"brake",
     "/Script/HTGame.HTVehicleMovementComponent:GetVehicleBrake",
     kBurnoutReceiverMovement, kBurnoutValueFloat, 0, 0},
    {"steering",
     "/Script/HTGame.HTVehicleMovementComponent:GetRawSteeringInput",
     kBurnoutReceiverMovement, kBurnoutValueFloat, 0, 0},
    {"skidAngle",
     "/Script/HTGame.HTVehicleMovementComponent:GetVehicleSkidAngle",
     kBurnoutReceiverMovement, kBurnoutValueFloat, 0, 0},
    {"playerThrottle",
     "/Script/HTGame.HTWheeledVehicleDrivable:GetPlayerThrottleInput",
     kBurnoutReceiverVehicle, kBurnoutValueFloat, 0, 0},
    {"wheelBurnout",
     "/Script/HTGame.HTVehicleMovementComponent:GetWheelIsBurnout",
     kBurnoutReceiverMovement, kBurnoutValueBool, 4, 1},
    {"wheelBurnoutRep",
     "/Script/HTGame.HTWheeledVehicleDrivable:GetWheelIsBurnoutRep",
     kBurnoutReceiverVehicle, kBurnoutValueBool, 4, 1},
    {"wheelAngularSpeed",
     "/Script/HTGame.HTVehicleMovementComponent:GetWheelAngularSpeed",
     kBurnoutReceiverMovement, kBurnoutValueFloat, 4, 1},
    {"runtimeInfo",
     "/Script/HTGame.HTVehicleRuntimeInfoInterface:GetCurrentRuntimeInfo",
     kBurnoutReceiverVehicle, kBurnoutValuePointer, 0, 0},
    {"wheelCount",
     "/Script/HTGame.HTWheeledVehicleRuntimeInfoBase:GetWheelCount",
     kBurnoutReceiverRuntimeInfo, kBurnoutValueInt, 0, 0},
    {"inputInfo",
     "/Script/HTGame.HTVehicleRuntimeInfoBase:GetInputInfo",
     kBurnoutReceiverRuntimeInfo, kBurnoutValueStruct, 0, 0},
};

static_assert(std::size(kBurnoutProbeSpecs) ==
              static_cast<std::size_t>(kBurnoutProbeCount));

inline constexpr std::size_t kBurnoutProbeMaxWheels = 8;

struct BurnoutProbeResult final {
  AnomalyGenerationHandleV1 function{};
  std::uint8_t status{kBurnoutProbeUnresolved};
  std::uint32_t last_code{};
  std::uint32_t calls{};
  std::uint32_t failures{};
  double next_retry{};
  bool logged{};
  bool has_output{};
  std::uint16_t output_size{};
  alignas(16) std::uint8_t output[32]{};
};

struct BurnoutProbeState final {
  std::array<BurnoutProbeResult, kBurnoutProbeCount> results{};
  AnomalyGenerationHandleV1 vehicle_handle{};
  AnomalyGenerationHandleV1 movement_handle{};
  AnomalyGenerationHandleV1 runtime_info_handle{};
  std::uintptr_t vehicle_object{};
  std::uintptr_t movement_object{};
  std::uintptr_t runtime_info_object{};
  double next_direct{};
  double next_wheel{};
  double next_runtime{};
  double next_runtime_signals{};
  double summary_due{};
  double last_input_log{};
  double next_active_log{};
  bool summary_logged{};
  bool have_last_state{};
  bool last_burnout{};
  bool last_handbrake{};
  bool last_handbrake12{};
  bool last_handbrake0{};
  std::uint32_t last_wheel_mask{};
  std::uint8_t last_input_raw[16]{};
  std::size_t last_input_raw_size{};

  bool is_burnout_known{};
  bool is_burnout{};
  bool throttle_known{};
  double throttle{};
  bool brake_known{};
  double brake{};
  // BrakeInput from GetInputInfo's HTVehicleInputInfo (float @8). Unlike
  // GetVehicleBrake, which reads 1.0 while coasting at rest, this is the actual
  // pedal value used to decide whether the car is parked.
  bool brake_pedal_known{};
  double brake_pedal{};
  bool steering_known{};
  double steering{};
  bool skid_known{};
  double skid_angle{};
  bool player_throttle_known{};
  double player_throttle{};
  bool handbrake_known{};
  bool handbrake{};
  bool handbrake12_known{};
  bool handbrake12{};
  bool handbrake0_known{};
  bool handbrake0{};
  bool wheel_count_known{};
  std::int32_t wheel_count{};
  bool wheel_burnout_known[kBurnoutProbeMaxWheels]{};
  bool wheel_burnout[kBurnoutProbeMaxWheels]{};
  bool wheel_rep_known[kBurnoutProbeMaxWheels]{};
  bool wheel_rep[kBurnoutProbeMaxWheels]{};
  bool wheel_angular_known[kBurnoutProbeMaxWheels]{};
  double wheel_angular[kBurnoutProbeMaxWheels]{};
  std::uint8_t input_raw[16]{};
  std::size_t input_raw_size{};
};

constexpr std::size_t kDiagnosticThreadSlots = 8;

struct Diagnostics final {
  std::atomic<std::uint64_t> snap_by_reason[kSnapReasonCount]{};
  std::atomic<std::uint64_t> yaw_source_switches{};
  std::atomic<std::uint64_t> lease_failures{};
  std::atomic<std::uint64_t> view_read_torn{};
  std::atomic<std::uint64_t> detour_calls{};
  std::atomic<std::uint64_t> detour_calls_window{};
  // Active view but no published pose: the frame the game camera shows through.
  std::atomic<std::uint64_t> detour_no_publish{};
  // 2.1.0 game-thread path. `view_applies` counts every GetViewPoint call the
  // adapter actually wrote (the UI projection, the scene view, and the HUD).
  // `query_skips` counts calls that saw `pose_attempted` but no pose: the scene
  // advance failed this frame, so the post-scene consumers deliberately stay
  // on the game camera.
  std::atomic<std::uint64_t> view_applies{};
  std::atomic<std::uint64_t> query_skips{};
  // Task group 6. `cut_applied` counts scene views where our pending cut was
  // actually ORed into FSceneViewInitOptions::bInCameraCut. `pose_invalidated`
  // counts transitions from a published pose to no pose (scene advance failed
  // or the view went inactive), so the next frame's early UI cannot replay a
  // stale pose. The extrapolation counters explain why the model kept running
  // while the live ComponentToWorld was unavailable.
  std::atomic<std::uint64_t> cut_applied{};
  std::atomic<std::uint64_t> cut_skipped_no_view{};
  std::atomic<std::uint64_t> pose_invalidated{};
  std::atomic<std::uint64_t> extrapolated_frames{};
  std::atomic<std::uint64_t> extrapolation_timeouts{};
  // TG-1: frames the chase pose was withheld because the game's live view-mode
  // tag was not ThirdPerson. This is the belt-and-suspenders counter for plan
  // SS2.4: a non-zero value means the presenter's flag lagged the game.
  std::atomic<std::uint64_t> tag_veto_frames{};
  // Q1: which threads call the hooked getter.
  std::atomic<std::uint32_t> detour_threads[kDiagnosticThreadSlots]{};
  std::atomic<std::uint32_t> detour_thread_count{};
  std::atomic<std::uint64_t> player_sequence_jumps{};
  // T3.1: ticks that fell back to the game-camera ESP yaw because
  // the Game-tick ESP camera tail was unavailable. Should stay 0.
  std::atomic<std::uint64_t> camera_yaw_esp_misses{};
  // Experiment (PLAN-camera-timing.md T3.4). Two independent ways to measure
  // the slip angle, side by side, plus the presenter's per-frame correction.
  // Nothing here changes behaviour; the numbers decide which fix is worth
  // building.
  std::atomic<double> slip_from_position_degrees{};
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

// Game-thread vehicle runtime. It replaces the old render-thread probe:
// profile gate and `ue5.GWorld` resolution, the player pawn chain, the vehicle
// mesh's ComponentToWorld, and the presence/body-yaw sample used by the Game
// tick. The camera itself reads the live transform at the scene-view phase;
// nothing here feeds a view from another thread.
struct VehicleRuntime final {
  // The profile hash is the only runtime witness for the hardcoded offsets.
  // A missing or mismatched profile closes the gate instead of reading a
  // layout that may have moved.
  std::uint8_t profile_state{kProfileGateWaiting};
  std::uint8_t logged_profile_state{0xFF};
  std::atomic_bool version_gate{};

  // `ue5.GWorld` resolution and the current local player chain.
  std::uint8_t resolve_state{kVehicleRuntimeWait};
  std::uint32_t resolve_code{};
  double next_resolve_elapsed{};
  std::uintptr_t gworld_slot{};
  std::atomic<std::uintptr_t> root{};
  // Same-chain cross-check against the host's player snapshot. A stale offset
  // table is caught here before the camera reads a transform through it.
  bool chain_verified{};
  bool chain_failed{};
  std::uint32_t chain_fail_streak{};
  double chain_max_cm{};
  std::uint64_t chain_mismatches{};

  // Game-tick presence and body heading from the vehicle mesh's
  // ComponentToWorld. `vehicle_sample_ns` is refreshed on every successful
  // AttachParent read, including "no parent", so the Game tick can distinguish
  // on-foot from a failed read. `vehicle_present_ns` is the last present
  // sample.
  std::atomic_bool vehicle_present{};
  std::atomic<std::int64_t> vehicle_present_ns{};
  std::atomic<std::int64_t> vehicle_sample_ns{};
  std::atomic<std::uintptr_t> vehicle_parent{};
  std::atomic<std::uint64_t> vehicle_read_failures{};
  std::atomic<double> body_yaw{};
  std::atomic<std::int64_t> body_yaw_ns{};
  std::atomic_bool body_yaw_valid{};
  // A torn parent read must not clear the vehicle on the first miss; keep the
  // last sample for at most this many 20 Hz polls. Game thread only.
  std::uint32_t vehicle_miss_streak{};

  void Reset() noexcept {
    profile_state = kProfileGateWaiting;
    logged_profile_state = 0xFF;
    version_gate.store(false, std::memory_order_relaxed);
    resolve_state = kVehicleRuntimeWait;
    resolve_code = 0;
    next_resolve_elapsed = 0.0;
    gworld_slot = 0;
    root.store(0, std::memory_order_relaxed);
    chain_verified = false;
    chain_failed = false;
    chain_fail_streak = 0;
    chain_max_cm = 0.0;
    chain_mismatches = 0;
    vehicle_present.store(false, std::memory_order_relaxed);
    vehicle_present_ns.store(0, std::memory_order_relaxed);
    vehicle_sample_ns.store(0, std::memory_order_relaxed);
    vehicle_parent.store(0, std::memory_order_relaxed);
    vehicle_read_failures.store(0, std::memory_order_relaxed);
    body_yaw.store(0.0, std::memory_order_relaxed);
    body_yaw_ns.store(0, std::memory_order_relaxed);
    body_yaw_valid.store(false, std::memory_order_relaxed);
    vehicle_miss_streak = 0;
  }
};

struct DisplayState final {
  bool started{};
  bool enabled{};
  bool hook_ready{};
  bool active{};
  bool driving{};
  bool reversing{};
  bool player_valid{};
  bool fov_enabled{};
  std::uint32_t preset{};
  // Mode of each settings group (PLAN-settings-ui-groups.md). Normalized on the
  // copy: the panel never sees a mode its group cannot express.
  GroupMode group_modes[kSettingsGroupCount]{};
  std::uint8_t speed_source{kSpeedSourceUnavailable};
  bool speed_sign_known{};
  double speed_kmh{};
  // Phase 0 burnout probe. Copied from Context::burnout_probe on the Game
  // tick; nothing here changes the camera.
  bool burnout_probe_is_burnout_known{};
  bool burnout_probe_is_burnout{};
  bool burnout_probe_throttle_known{};
  double burnout_probe_throttle{};
  bool burnout_probe_brake_known{};
  double burnout_probe_brake{};
  bool burnout_probe_steering_known{};
  double burnout_probe_steering{};
  bool burnout_probe_skid_known{};
  double burnout_probe_skid_angle{};
  bool burnout_probe_player_throttle_known{};
  double burnout_probe_player_throttle{};
  bool burnout_probe_handbrake12_known{};
  bool burnout_probe_handbrake12{};
  bool burnout_probe_handbrake0_known{};
  bool burnout_probe_handbrake0{};
  bool burnout_probe_handbrake_known{};
  bool burnout_probe_handbrake{};
  bool burnout_probe_wheel_count_known{};
  std::int32_t burnout_probe_wheel_count{};
  bool burnout_probe_wheel_burnout_known[kBurnoutProbeMaxWheels]{};
  bool burnout_probe_wheel_burnout[kBurnoutProbeMaxWheels]{};
  bool burnout_probe_wheel_rep_known[kBurnoutProbeMaxWheels]{};
  bool burnout_probe_wheel_rep[kBurnoutProbeMaxWheels]{};
  bool burnout_probe_wheel_angular_known[kBurnoutProbeMaxWheels]{};
  double burnout_probe_wheel_angular[kBurnoutProbeMaxWheels]{};
  std::uint8_t burnout_probe_input_raw[16]{};
  std::size_t burnout_probe_input_raw_size{};
  std::uint32_t burnout_probe_resolved{};
  std::uint32_t burnout_probe_ok{};
  std::uint32_t burnout_probe_failed{};
  std::uint32_t burnout_probe_status[kBurnoutProbeCount]{};
  std::uint32_t burnout_probe_code[kBurnoutProbeCount]{};
  // Phase 2 sample facts (not yet consumed by the camera).
  bool burnout_signal_known{};
  bool burnout_direct{};
  bool burnout_fallback{};
  bool burnout_active{};
  bool burnout_left{};
  bool burnout_right{};
  double burnout_intensity{};
  double burnout_throttle{};
  double burnout_handbrake{};
  double burnout_steering{};
  double burnout_speed_mps{};
  double burnout_hold_remaining{};
  bool wheel_steer_enabled{kDefaultWheelSteerEnabled};
  bool wheel_steer_active{};
  bool wheel_steer_engaged{};
  bool wheel_steer_force_turn{};
  std::uint32_t wheel_steer_wheels{};
  double wheel_steer_angle_degrees{};
  double wheel_steer_target_degrees{};
  double wheel_steer_entry_speed_kmh{kDefaultWheelSteerEntrySpeedKmh};
  double wheel_steer_max_degrees{kDefaultWheelSteerMaxDegrees};
  double wheel_steer_gain{kDefaultWheelSteerGain};
  double wheel_steer_full_slip_degrees{kDefaultWheelSteerFullSlipDegrees};
  double wheel_steer_rate_degrees_per_second{
      kDefaultWheelSteerRateDegreesPerSecond};
  double wheel_steer_force_turn_delay_seconds{
      kDefaultWheelSteerForceTurnDelaySeconds};
  double wheel_steer_sample_percent{kDefaultWheelSteerSamplePercent};
  double slip_degrees{};
  double drift{};
  double fov_horizontal{};
  double camera_distance_m{};
  double camera_distance_raw_m{};
  // Live eye height above the ground, from the last presenter pose.
  double camera_height_m{};
  double update_hz{};
  double view_output_hz{};
  double interpolation_ratio{};
  double camera_distance_setting_m{kDefaultCameraDistanceM};
  double position_spring{kDefaultPositionSpring};
  double yaw_spring{kDefaultYawSpring};
  double drift_yaw_spring{kDefaultDriftYawSpring};
  double look_yaw_spring{kDefaultLookYawSpring};
  double yaw_return_spring{kDefaultYawReturnSpring};
  double pitch_spring{kDefaultPitchSpring};
  double roll_spring{kDefaultRollSpring};
  double camera_height_setting_m{kDefaultCameraHeightM};
  double aspect_ratio{kDefaultAspectRatio};
  double speed_distance_scale{kDefaultSpeedDistanceScale};
  double roll_scale{kDefaultRollScale};
  double pitch_scale{kDefaultPitchScale};
  double shake_scale{kDefaultShakeScale};
  bool burnout_enabled{kDefaultBurnoutEnabled};
  double burnout_entry_delay{kDefaultBurnoutEntryDelay};
  double burnout_entry_time{kDefaultBurnoutEntryTime};
  double burnout_exit_delay{kDefaultBurnoutExitDelay};
  double burnout_exit_time{kDefaultBurnoutExitTime};
  double burnout_launch_exit_time{kDefaultBurnoutLaunchExitTime};
  double burnout_fov{kDefaultBurnoutFov};
  double burnout_shake{kDefaultBurnoutShake};
  std::uint32_t burnout_side_mode{kDefaultBurnoutSideMode};
  double burnout_entry_speed{kDefaultBurnoutEntrySpeed};
  double burnout_hold_speed{kDefaultBurnoutHoldSpeed};
  bool orbit_enabled{kDefaultOrbitEnabled};
  bool orbit_hide_ui{kDefaultOrbitHideUi};
  double orbit_entry_delay{kDefaultOrbitEntryDelay};
  double orbit_period{kDefaultOrbitPeriod};
  double orbit_radius{kDefaultOrbitRadius};
  double orbit_height{kDefaultOrbitHeight};
  // Where the orbit aims, above the car origin along the body up axis. The eye
  // height above and this one are independent; their difference is the
  // elevation angle.
  double orbit_look_at_height{kDefaultOrbitLookAtHeight};
  double orbit_weave{kDefaultOrbitWeave};
  double orbit_target_sway{kDefaultOrbitTargetSway};
  double orbit_fov{kDefaultOrbitFov};
  double orbit_shake_strength{kDefaultOrbitShakeStrength};
  double orbit_shake_frequency{kDefaultOrbitShakeFrequency};
  double burnout_shot_blend{};
  int burnout_shot_side{1};
  std::uint64_t burnout_shot_fallbacks{};
  double orbit_blend{};
  nfs2015_chase::OrbitPhase orbit_state{nfs2015_chase::OrbitPhase::kIdle};
  nfs2015_chase::OrbitUiHidePhase orbit_ui_phase{
      nfs2015_chase::OrbitUiHidePhase::kIdle};
  bool orbit_ui_was_hidden{};
  bool orbit_ui_hidden{};
  bool orbit_ui_intervened{};
  bool orbit_ui_readback_supported{};
  bool orbit_ui_readback_visible{};
  std::uint32_t orbit_ui_hide_requests{};
  std::uint32_t orbit_ui_restore_requests{};
  std::uint32_t orbit_ui_errors{};
  double orbit_hold_seconds{};
  double orbit_time{};
  double orbit_phase0_degrees{};
  bool orbit_input{};
  double orbit_speed_mps{};
  double orbit_shake_x_m{};
  double orbit_shake_y_m{};
  // Low-frequency sway after its strength scale and the 1 s damper, in metres
  // along the camera's local right / up, plus the roll noise in degrees. All
  // three are read-only diagnostics of the model's own state.
  double orbit_target_sway_x_m{};
  double orbit_target_sway_y_m{};
  double orbit_roll_noise_degrees{};
  bool slope_follow{true};
  double slope_strength{kDefaultSlopeStrength};
  bool roll_only_in_drift{true};
  bool diagnostic_flat_pose{};
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
  std::uint64_t lease_failures{};
  std::uint64_t view_read_torn{};
  std::uint64_t detour_calls{};
  std::uint64_t detour_no_publish{};
  std::uint64_t view_applies{};
  std::uint64_t query_skips{};
  std::uint64_t cut_applied{};
  std::uint64_t cut_skipped_no_view{};
  std::uint64_t pose_invalidated{};
  std::uint64_t extrapolated_frames{};
  std::uint64_t extrapolation_timeouts{};
  std::uint64_t detour_thread_count{};
  std::uint64_t player_sequence_jumps{};
  std::uint64_t camera_yaw_esp_misses{};
  double slip_from_position_degrees{};
  std::uint64_t slip_sign_changes{};
  double slip_max_step_degrees{};
  bool render_speed_valid{};
  bool render_turn_valid{};
  double render_speed_floor_mps{};
  double render_speed_peak_mps{};
  double render_turn_floor_dps{};
  double render_turn_peak_dps{};

  // Game-thread vehicle runtime. These fields are the chain gate and the
  // vehicle presence signal.
  std::uint8_t runtime_state{};
  std::uint32_t runtime_code{};
  std::uint8_t runtime_gate{kProfileGateOff};
  double runtime_chain_cm{};
  std::uint64_t runtime_chain_mismatches{};
  bool vehicle_present{};
  bool vehicle_presence_known{};
  std::uint64_t vehicle_read_failures{};

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

// Facts copied from the reflected burnout probe once per Game tick. This is
// deliberately separate from the probe's raw diagnostic state: the sample
// pipeline consumes a small, stable view, while the probe keeps counters and
// the last raw output for the panel.
struct BurnoutSignals final {
  bool known{};
  bool direct_known{};
  bool direct{};
  bool handbrake_known{};
  bool handbrake{};
  bool throttle_known{};
  double throttle{};
  bool player_throttle_known{};
  double player_throttle{};
  bool brake_known{};
  double brake{};
  bool brake_pedal_known{};
  double brake_pedal{};
  bool steering_known{};
  double steering{};
  bool left_known{};
  bool left{};
  bool right_known{};
  bool right{};
};

// Game-tick hysteresis for the sample. The render-side shot has its own
// entry/exit delays; this state only bridges the 20 Hz poll and applies the
// speed/ground gates and the intensity ramp.
struct BurnoutDetectionState final {
  bool active{};
  bool raw_active{};
  bool direct{};
  bool fallback{};
  bool have_last_log{};
  bool last_log_active{};
  bool last_log_direct{};
  bool last_log_fallback{};
  double hold_until{};
  double intensity{};
};

// Game-thread state for the visual wheel override. NTE's wheel controller owns
// both the steering pose and the rolling spin. A fixed RelativeRotation written
// on top of that controller precesses as soon as the wheel rolls, so while the
// override is active we park the controller with bEnableWheelAnimation=0 and
// drive the visual steer and roll ourselves. The stock relative rotations and
// the animation flag are restored on every exit path.
struct WheelSteerState final {
  std::uintptr_t vehicle{};
  std::uintptr_t mesh{};
  std::uintptr_t anim{};
  std::array<std::uintptr_t, kMaximumWheelSteerWheels> wheels{};
  std::array<bool, kMaximumWheelSteerWheels> front{};
  std::array<std::array<double, 3>, kMaximumWheelSteerWheels> base_rotator{};
  std::array<bool, kMaximumWheelSteerWheels> base_captured{};
  // Socket orientation relative to the vehicle mesh, captured after the game's
  // wheel controller has been parked. The front wheel steer is computed against
  // this frozen socket, which removes the rolling-spin feedback entirely.
  std::array<nfs2015_chase::Quat, kMaximumWheelSteerWheels> socket_local{};
  std::array<bool, kMaximumWheelSteerWheels> socket_captured{};
  std::size_t wheel_count{};
  std::size_t front_count{};
  std::size_t rear_count{};
  std::uint8_t animation_was_enabled{};
  bool animation_saved{};
  bool animation_disabled{};
  int settle_ticks{};
  double roll_angle{};
  double applied_angle{};
  double target_angle{};
  double target_blend{};
  double stock_angle{};
  double entry_stock_angle{};
  bool entry_stock_valid{};
  double front_travel_angle{};
  double rear_travel_angle{};
  double yaw_rate{};
  nfs2015_chase::Vec3 front_axle_offset{};
  nfs2015_chase::Vec3 rear_axle_offset{};
  bool axle_offsets_valid{};
  nfs2015_chase::Quat last_mesh{};
  bool have_last_mesh{};
  nfs2015_chase::WheelSteerDriftState drift{};
  nfs2015_chase::WheelSteerPhase phase{
      nfs2015_chase::WheelSteerPhase::grip};
  bool engaged{};
  bool active{};
  bool quick_turn{};
  bool force_turn{};
  bool resolved{};
  bool failure_logged{};
  bool geometry_logged{};
  bool activation_logged{};
  double next_resolve{};
};

// Z-key view-mode recon (PLAN-zkey-phase2-h1.md task group 0).
//
// Task group 0 is log-only: the detour below is installed, every call is
// recorded, and every call is forwarded to the game unchanged. Nothing here
// influences the view mode yet, so a wrong offset can only cost log lines.
//
// The tag ids are resolved by text at the start of the session; the game
// assigns FName ids as names are registered, so a recorded id is not portable
// across runs and must never be hardcoded.
struct ViewModeState final {
  std::atomic<std::uintptr_t> pc{};
  std::atomic<std::uintptr_t> world_slot{};
  std::atomic<std::uint32_t> tag_third_person{};
  std::atomic<std::uint32_t> tag_first_person{};
  std::atomic<std::uint32_t> tag_fixed{};
  std::atomic<std::uint32_t> tag_seat{};
  std::atomic<std::uint32_t> tag_driving{};
  std::atomic_bool names_ready{};
  std::atomic<std::uint64_t> hook_id{};
  std::atomic<std::uint64_t> hook_generation{};
  std::atomic<std::uintptr_t> hook_target{};
  // Detour-side diagnostics. Atomics because the toggle is only assumed to run
  // on the Game thread, not guaranteed to.
  std::atomic<std::uint64_t> calls{};
  std::atomic<std::uint64_t> logged{};
  std::atomic<std::uintptr_t> last_site{};
  std::atomic<std::uint32_t> forwarded{};
  // Game-thread latches: each failure is reported once, then the retry timer
  // decides whether asking again can change the answer.
  bool names_failure_logged{};
  bool hook_failure_logged{};
  bool vtable_failure_logged{};
  bool shape_logged{};
  bool pattern_unavailable{};
  bool vtable_mismatch{};
  double next_retry{};
  double next_tags_retry{};
  // Per-tick watch: the game changes the view mode on its own (a teleport or a
  // transition resets it), so the plugin has to notice rather than infer. Plain
  // fields: the Game tick is the only writer.
  std::uint64_t last_tag{};
  std::int32_t last_limit{};
  std::int32_t last_count{};
  bool have_tag{};
  // TG-1: the chase slot. The plugin still never keeps its own "which mode am
  // I in" counter - the tag below is a copy of the game's, refreshed every tick
  // by the watcher and read by the presenter and the chase guard.
  std::atomic<std::uint64_t> published_tag{};
  std::atomic_bool published_tag_known{};
  // Set by the presenter: the same `driving` answer the chase model uses. The
  // detour only takes over while it is true (plan SS2.2 / TG-1).
  std::atomic_bool in_vehicle{};
  // Game-thread only. "The player wants the chase camera": latched when a
  // vehicle is entered (entryView=chase) and dropped when the player presses Z
  // out of it or leaves the vehicle.
  bool chase_wanted{};
  // Game-thread only (TG-2, plan SS2.3.2). A vehicle was entered while the game
  // was not in a view the plugin can classify (Fixed / Seat / unreadable), so
  // `entryView` has not been applied yet. Consumed once the game reports Third
  // or First again; cleared when the vehicle is left.
  bool pending_entry{};
  // Game-thread only: the "waiting for a readable tag" line is printed once per
  // entry, not once per tick.
  bool entry_pending_logged{};
};

// Game-thread-only runtime cache for the parked-orbit HUD hide. The pure
// OrbitUiHide machine owns Q2 semantics; this struct owns the guest handles,
// the object-registry scan and the readback targets. None of it is touched by
// the render thread or by the settings UI.
struct OrbitUiRuntimeState final {
  nfs2015_chase::OrbitUiHide machine{};

  AnomalyGenerationHandleV1 layout_class{};
  AnomalyGenerationHandleV1 hide_all_layer{};
  AnomalyGenerationHandleV1 revert_layer_show{};
  AnomalyGenerationHandleV1 get_render_opacity{};
  AnomalyGenerationHandleV1 set_render_opacity{};
  AnomalyGenerationHandleV1 set_render_translation{};
  AnomalyGenerationHandleV1 layout{};
  std::array<AnomalyGenerationHandleV1, 4> readback{};
  // The path queries can resolve the class's CDO WidgetTree. Only handles
  // after the live layout are safe to animate; the CDO entries are read-only
  // path candidates.
  std::array<bool, 4> readback_live{};
  // GameLayer_Stack is the menu/game layer; GameplayMain_Stack is the HUD.
  // ESC only reveals the former, so the game's own HUD hide animation cannot
  // replay over an already hidden HUD.
  std::array<bool, 4> readback_menu{};
  std::uint32_t readback_count{};

  // Game-thread opacity ramp. HideAllLayer/RevertLayerShow flip the layer
  // RenderOpacity immediately; the game's ESC menu instead fades the main form.
  // Interpolate the live stack opacity for the same short, simple transition,
  // then let the native function commit the final state.
  struct Fade final {
    bool active{};
    bool hiding{};
    double start{};
    double duration{};
    float from{};
    float to{};
  } fade;
  // ESC reveals the menu layer after a full RevertLayerShow. Reassert the
  // HUD-hidden/menu-visible split for a few frames because the game's own ESC
  // handler also animates the HUD.
  double menu_hold_until{};
  // False after the opacity-only path is available: orbit hides only the HUD
  // layer and leaves the menu layer live, so F1/F2/B/C/ESC can open their UIs
  // on the first press. True only when SetRenderOpacity is unavailable and we
  // fall back to HideAllLayer.
  bool full_layer_hide{};

  std::uint32_t layout_name_id{};
  std::uint32_t layout_scan_index{};
  std::uint32_t layout_scan_count{};
  std::uint32_t readback_scan_index{};
  std::uint32_t readback_scan_count{};

  std::uint64_t generation{};
  bool layout_scanning{};
  bool layout_resolved{};
  bool readback_scanning{};
  bool readback_ready{};
  bool readback_paths_tried{};

  // A marker survives a plugin unload. The next generation uses it to restore
  // a HUD that the previous generation hid but could not restore from the
  // non-Game-thread on_stop callback.
  bool startup_restore_pending{};
  bool startup_restore_logged{};
  double next_startup_restore{};
  bool marker_error_logged{};
  AnomalyGenerationHandleV1 marker_clear_task{};

  // The presenter resets the orbit whenever the chase slot is left and
  // re-entered. Track that generation too: a HUD hidden before a view
  // transition must not survive into the new view.
  std::uint64_t view_epoch{};
  bool view_epoch_initialized{};

  bool services_unavailable_logged{};
  bool resolve_failed_logged{};
  bool readback_unavailable_logged{};
  bool last_invoke_ok{true};

  double next_resolve{};
  nfs2015_chase::OrbitUiHidePhase logged_phase{
      nfs2015_chase::OrbitUiHidePhase::kIdle};
};

struct Context final {
  nfs2015_chase_cam::Localizer localizer;
  const AnomalyCoreServiceV1 *core{};
  const AnomalyConfigServiceV1 *config{};
  // Optional plugin-private storage, used only for the orbit HUD recovery
  // marker. It is optional because older hosts need not expose storage.
  const AnomalyStorageServiceV1 *storage{};
  const AnomalySchedulerServiceV1 *scheduler{};
  // Optional input service: ESC must open the phone menu even while the orbit
  // owns the HUD. `was_pressed` is polled on the Game tick.
  const AnomalyInputServiceV1 *input{};
  const AnomalyJsonServiceV1 *json{};
  const AnomalyUiServiceV1 *ui{};
  const AnomalyWindowServiceV1 *window{};
  AnomalyGenerationHandleV1 window_handle{};
  const AnomalySignatureServiceV1 *signature{};
  const AnomalyHookServiceV1 *hook{};
  const AnomalyNteSessionServiceV1 *session{};
  const AnomalyNtePlayerServiceV1 *player{};
  // The old 2.6 vehicle snapshot called BP_GetCurrentDriveVehicle through the
  // Host. 2.7 removed that service, but the same reflected getters are still
  // callable through the generic UE5 object/ProcessEvent services.
  const AnomalyUe5ObjectsServiceV1 *objects{};
  const AnomalyUe5ProcessEventServiceV1 *events{};
  // Optional engine access. Only the probe uses this; it may be null on a host
  // without the `ue5.ahud` feature, and every use is guarded.
  const AnomalyUe5AhudServiceV1 *ahud{};
  // Optional build identity. It gates the hardcoded vehicle/GWorld offsets;
  // a host without it leaves the game-thread camera disabled.
  const AnomalyUe5BuildServiceV1 *build{};
  // Optional name pool access. `find_utf8` (names service v2) is the only way
  // to turn "CameraViewMode.ThirdPerson" into the id this process assigned it;
  // without it the Z cycle stays disabled and Z remains native.
  const AnomalyUe5NamesServiceV1 *names{};

  AnomalyGenerationHandleV1 settings_schema{};
  // The 2.1.0 game-thread hooks.
  std::atomic<std::uint64_t> game_view_point_hook_id{};
  std::atomic<std::uint64_t> game_view_point_hook_generation{};
  std::atomic<std::uintptr_t> game_view_point_hook_target{};
  std::atomic<std::uint64_t> scene_view_init_hook_id{};
  std::atomic<std::uint64_t> scene_view_init_hook_generation{};
  std::atomic<std::uintptr_t> scene_view_init_hook_target{};

  // Settings: Render/hotkey write, Game reads.
  std::atomic_bool enabled{true};
  std::atomic<std::uint32_t> preset{0};
  std::atomic<double> camera_distance_m{kDefaultCameraDistanceM};
  std::atomic<double> position_spring{kDefaultPositionSpring};
  std::atomic<double> yaw_spring{kDefaultYawSpring};
  std::atomic<double> drift_yaw_spring{kDefaultDriftYawSpring};
  std::atomic<double> look_yaw_spring{kDefaultLookYawSpring};
  std::atomic<double> yaw_return_spring{kDefaultYawReturnSpring};
  std::atomic<double> pitch_spring{kDefaultPitchSpring};
  std::atomic<double> roll_spring{kDefaultRollSpring};
  std::atomic<double> camera_height_m{kDefaultCameraHeightM};
  std::atomic<double> aspect_ratio{kDefaultAspectRatio};
  std::atomic<double> speed_distance_scale{kDefaultSpeedDistanceScale};
  std::atomic<double> roll_scale{kDefaultRollScale};
  std::atomic<double> pitch_scale{kDefaultPitchScale};
  std::atomic<double> shake_scale{kDefaultShakeScale};
  // Burnout close-up settings. `burnoutShake` is separate from the chase shake
  // so the two cameras can be tuned independently.
  std::atomic_bool wheel_steer_enabled{kDefaultWheelSteerEnabled};
  std::atomic<double> wheel_steer_entry_speed_kmh{
      kDefaultWheelSteerEntrySpeedKmh};
  std::atomic<double> wheel_steer_max_degrees{kDefaultWheelSteerMaxDegrees};
  std::atomic<double> wheel_steer_gain{kDefaultWheelSteerGain};
  std::atomic<double> wheel_steer_full_slip_degrees{
      kDefaultWheelSteerFullSlipDegrees};
  std::atomic<double> wheel_steer_rate_degrees_per_second{
      kDefaultWheelSteerRateDegreesPerSecond};
  std::atomic<double> wheel_steer_force_turn_delay_seconds{
      kDefaultWheelSteerForceTurnDelaySeconds};
  std::atomic<double> wheel_steer_sample_percent{
      kDefaultWheelSteerSamplePercent};
  std::atomic_bool burnout_enabled{kDefaultBurnoutEnabled};
  std::atomic<double> burnout_entry_delay{kDefaultBurnoutEntryDelay};
  std::atomic<double> burnout_entry_time{kDefaultBurnoutEntryTime};
  std::atomic<double> burnout_exit_delay{kDefaultBurnoutExitDelay};
  std::atomic<double> burnout_exit_time{kDefaultBurnoutExitTime};
  std::atomic<double> burnout_launch_exit_time{
      kDefaultBurnoutLaunchExitTime};
  std::atomic<double> burnout_fov{kDefaultBurnoutFov};
  std::atomic<double> burnout_shake{kDefaultBurnoutShake};
  std::atomic<std::uint32_t> burnout_side_mode{kDefaultBurnoutSideMode};
  std::atomic<double> burnout_entry_speed{kDefaultBurnoutEntrySpeed};
  std::atomic<double> burnout_hold_speed{kDefaultBurnoutHoldSpeed};
  // Parked-orbit settings. Entry/exit durations stay in the model; the orbit
  // height and framing values are exposed because the user can feel them.
  std::atomic_bool orbit_enabled{kDefaultOrbitEnabled};
  std::atomic_bool orbit_hide_ui{kDefaultOrbitHideUi};
  std::atomic<double> orbit_entry_delay{kDefaultOrbitEntryDelay};
  std::atomic<double> orbit_period{kDefaultOrbitPeriod};
  std::atomic<double> orbit_radius{kDefaultOrbitRadius};
  std::atomic<double> orbit_height{kDefaultOrbitHeight};
  std::atomic<double> orbit_look_at_height{kDefaultOrbitLookAtHeight};
  std::atomic<double> orbit_weave{kDefaultOrbitWeave};
  std::atomic<double> orbit_target_sway{kDefaultOrbitTargetSway};
  std::atomic<double> orbit_fov{kDefaultOrbitFov};
  std::atomic<double> orbit_shake_strength{kDefaultOrbitShakeStrength};
  std::atomic<double> orbit_shake_frequency{kDefaultOrbitShakeFrequency};
  std::atomic_bool slope_follow{true};
  std::atomic<double> slope_strength{kDefaultSlopeStrength};
  std::atomic<double> slope_max_gradient_degrees{
      kDefaultSlopeMaxGradientDegrees};
  std::atomic<double> slope_up_look_at{kDefaultSlopeUpLookAt};
  std::atomic<double> slope_up_rest{kDefaultSlopeUpRest};
  std::atomic<double> slope_down_look_at{kDefaultSlopeDownLookAt};
  std::atomic<double> slope_down_rest{kDefaultSlopeDownRest};
  std::atomic_bool roll_only_in_drift{true};
  // Bisect switch: hold every attitude term at zero and stop writing the lens,
  // so only position and yaw survive. One click decides whether the twitch is
  // inside the attitude model at all, without reading a single number.
  std::atomic_bool diagnostic_flat_pose{};
  // Bisect switches for what the view writes. Turning one off leaves the Game's
  // own value in place for that half.
  std::atomic_bool diagnostic_skip_position{};
  std::atomic_bool diagnostic_skip_rotation{};
  std::atomic_bool fov_enabled{true};
  // Grouped settings (PLAN-settings-ui-groups.md task group 1). The atomics
  // above remain the effective values every consumer reads; ApplyGroup() is
  // their only grouped writer. The custom numbers live behind this mutex so a
  // group apply copies a coherent block and then releases the lock before it
  // touches any settings atomic.
  std::mutex custom_settings_mutex;
  CustomSettings custom_settings;
  std::atomic<std::uint32_t> group_modes[kSettingsGroupCount]{};
  std::atomic<std::uint64_t> settings_revision{};
  std::atomic<std::uint64_t> persisted_settings_revision{};
  std::atomic<std::uint64_t> settings_changed_at{};
  // The debounced write runs on the host scheduler's worker thread: the host
  // rejects configuration I/O from the Game and Render callbacks, which is
  // where the settings are edited. Only one task may be in flight at a time;
  // the task clears the flag after it has written (or failed to write).
  std::atomic_bool settings_save_pending{};
  // The status of the last write failure, so a silent rejection is logged once
  // per distinct status instead of once per retry.
  std::atomic<std::uint32_t> settings_save_reported_error{
      ANOMALY_STATUS_V1_OK};

  // Hook state. `hook_ready` means both game-thread hooks are installed.
  std::atomic_bool hook_ready{};
  double next_hook_retry{};
  // A deterministic pattern miss cannot be fixed by scanning the module again.
  // Latch it so the 2 s hook retry does not re-scan .text forever.
  bool game_view_point_pattern_unavailable{};
  bool scene_view_init_pattern_unavailable{};
  bool game_view_point_hook_failure_logged{};
  bool scene_view_init_hook_failure_logged{};

  // Bumped when the chase slot is left or re-entered. The presenter drops a
  // latched orbit when it changes, so returning from first person or a cutscene
  // does not resume an orbit immediately.
  std::atomic<std::uint64_t> view_epoch{};
  std::atomic_bool view_active{};
  // 2.1.0 scene-phase gate. The model advances once at the entry of
  // CalcSceneViewInitOptions, after the vehicle has run, and sets pose_ready.
  // Every GetViewPoint call after that copies the pose. Earlier calls (the UMG
  // projection in World Tick) copy the pose from the previous scene, which
  // matches the previous-frame car transform they see. `pose_attempted` is set
  // even when the scene advance fails, so the rest of the frame fails open to
  // the game camera instead of retrying and mixing poses. on_update clears
  // both.
  // `view_build_depth` brackets CalcSceneViewInitOptions; it also identifies
  // the scene consumers and is used by task group 6's cut write.
  std::atomic<std::uint32_t> view_build_depth{};
  std::atomic_bool pose_attempted{};
  std::atomic_bool pose_ready{};

  double brake_last_target{};

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
  // Phase 3: render-clock burnout shot state. It is advanced and read only
  // while presenter_lock is held, alongside the chase model.
  nfs2015_chase::BurnoutShot burnout_shot{};
  // Parked-orbit state. Like the burnout shot it is advanced and read only
  // while presenter_lock is held, alongside the chase model.
  nfs2015_chase::OrbitShot orbit_shot{};
  OrbitUiRuntimeState orbit_ui{};
  std::uint32_t configured_preset{0};
  bool chase_initialized{};
  // The last completed Game-tick car state. It is copied once at the
  // scene-view advance, then the live VehicleMesh ComponentToWorld replaces its
  // position and attitude. No interpolation and no render-clock mapping.
  std::atomic<std::uint64_t> latest_car_sample_sequence{};
  nfs2015_chase::CarSample latest_car_sample{};
  std::atomic_bool latest_car_sample_valid{};
  // Task group 6: last successful scene-phase car read. It is used for the
  // scene teleport check and for the short missing-transform extrapolation.
  // Both the writer and reader are the Game thread; no atomics are needed.
  nfs2015_chase::CarSample last_scene_car_sample{};
  bool last_scene_car_valid{};
  std::uint32_t scene_extrapolation_frames{};
  // A snap requested by either the Game tick or the scene advance. The
  // CalcSceneViewInitOptions detour consumes it only for an active, allowed
  // view and ORs it into the engine's own bInCameraCut. Keeping it pending
  // while the camera is inactive is deliberate: the first view after re-entry
  // is the one that needs the cut.
  std::atomic_bool pending_cut{};
  std::atomic_flag presenter_lock = ATOMIC_FLAG_INIT;
  std::int64_t presenter_last_ns{};
  std::uint64_t presenter_epoch{};
  std::uint64_t presenter_view_epoch{};
  // Last orbit phase a transition log was emitted for.
  std::uint32_t presenter_orbit_logged_mode{};
  bool presenter_initialized{};
  std::int64_t presenter_pose_ns{};
  std::atomic<std::uint64_t> presenter_pose_sequence{};
  std::atomic<double> presenter_pose_location[3]{};
  std::atomic<double> presenter_pose_rotation[3]{};
  std::atomic<double> presenter_car_cm[3]{};
  std::atomic<float> presenter_pose_lens{};
  std::atomic<bool> presenter_pose_fov{};
  std::atomic<bool> presenter_pose_valid{};
  std::atomic<double> presenter_anchor[3]{};
  std::atomic<double> presenter_drift{};
  std::atomic<double> presenter_distance_cm{};
  // Eye height above the model origin (ground) in centimetres, from the last
  // published pose. The settings panel shows it beside the live distance.
  std::atomic<double> presenter_height_cm{};
  // Diagnostics for the mixed chase/shot camera. The blend is written under
  // the presenter lock and read by diagnostics.
  std::atomic<double> presenter_burnout_blend{};
  std::atomic<int> presenter_burnout_side{1};
  std::atomic<std::uint64_t> presenter_burnout_fallbacks{};
  // Parked-orbit diagnostics. Written by the presenter, read by the Game-tick
  // display snapshot. The mode is the numeric OrbitPhase value.
  std::atomic<double> presenter_orbit_blend{};
  std::atomic<std::uint32_t> presenter_orbit_mode{};
  std::atomic<double> presenter_orbit_hold_seconds{};
  std::atomic<double> presenter_orbit_time{};
  std::atomic<double> presenter_orbit_phase0_degrees{};
  std::atomic<double> presenter_orbit_speed_mps{};
  std::atomic_bool presenter_orbit_input{};
  std::atomic<double> presenter_orbit_shake_x_m{};
  std::atomic<double> presenter_orbit_shake_y_m{};
  std::atomic<double> presenter_orbit_target_sway_x_m{};
  std::atomic<double> presenter_orbit_target_sway_y_m{};
  std::atomic<double> presenter_orbit_roll_noise_degrees{};
  std::int64_t sample_last_ns{};
  // Bumped by the Game tick when the model must restart (world, preset, reset).
  std::uint64_t sample_epoch{};
  std::atomic_bool reset_requested{};
  std::atomic<std::uint64_t> reset_feedback_until{};
  std::atomic_bool hook_retry_requested{};
  AnomalyGenerationHandleV1 world{};
  AnomalyGenerationHandleV1 player_handle{};
  bool have_last_position{};
  bool have_last_yaw{};
  bool reversing{};
  bool speed_sign_known{true};
  bool vehicle_present_last{};
  double last_vehicle_present_time{};
  std::uintptr_t last_vehicle_parent{};
  // The old 2.6 vehicle service polled BP_GetCurrentDriveVehicle every 50 ms.
  // Reproduce that exact presence signal through the generic UE5 services.
  std::array<AnomalyGenerationHandleV1, 3> vehicle_getter_handles{};
  std::array<double, 3> next_vehicle_getter_retry{};
  double next_vehicle_query_time{};
  bool vehicle_query_known{};
  bool vehicle_query_present{};
  std::uintptr_t vehicle_query_object{};
  bool vehicle_query_speed_known{};
  bool vehicle_query_speed_fresh{};
  double vehicle_query_speed_kmh{};
  // Exact-speed cache. The reflected function and the movement component
  // handle stay valid while the vehicle object does, and ue5.objects'
  // generation invalidates all of them when the registry is rebuilt.
  std::uint64_t vehicle_objects_generation{};
  std::uintptr_t vehicle_speed_object{};
  AnomalyGenerationHandleV1 vehicle_speed_function{};
  AnomalyGenerationHandleV1 vehicle_movement_handle{};
  AnomalyGenerationHandleV1 vehicle_wheeled_class{};
  bool vehicle_speed_wheeled{};
  double next_vehicle_wheeled_class_retry{};
  double next_vehicle_speed_function_retry{};
  bool vehicle_speed_diag_logged{};
  bool vehicle_speed_active_logged{};
  double last_vehicle_speed_mps{-1.0};
  double last_vehicle_speed_time{};
  bool last_speed_exact{};
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
  bool last_view_active{};
  bool last_fov_enabled{};
  // TG-1: latches for the chase slot. `view_off_seconds` is how long the plugin
  // has not seen a vehicle; a value above the threshold is what makes the next
  // driving tick count as "got in", which is the only thing that re-arms the
  // chase camera. Debounced because the vehicle getter and the pawn chain both
  // drop out for a tick now and then, and a gap must not look like an exit.
  double view_off_seconds{1.0};
  bool view_slot_chase_last{};
  // T3.1: game-camera yaw from the player ESP snapshot, sampled by the host
  // on this Game tick.
  double camera_yaw_radians{};
  bool camera_yaw_valid{};
  bool view_snap{true};

  // Diagnostics (T0). Game thread owns the plain fields; the detour only
  // touches Context::diag atomics.
  Diagnostics diag{};
  Probe probe{};
  // Game-thread profile gate, player chain and vehicle presence.
  VehicleRuntime vehicle_runtime{};
  // Phase 0 burnout reconnaissance. Game-thread only; read-only.
  BurnoutProbeState burnout_probe{};
  // Phase 2: the compact facts and the Game-tick hysteresis that feed
  // CarSample::burnout. Neither is read by the camera yet.
  BurnoutSignals burnout_signals{};
  BurnoutDetectionState burnout_detection{};
  // Visual front-wheel override. Game-thread only.
  WheelSteerState wheel_steer{};
  // Z-key view-mode recon (task group 0: log only).
  ViewModeState view_mode{};
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
  // Panel state, UI thread only. The advanced block starts closed so the first
  // view is the seven group choices; the diagnostics panel keeps its own switch.
  bool advanced_open{};
  bool diagnostics_open{};

  // Game writes, Render copies.
  mutable std::mutex display_mutex;
  DisplayState display{};
};

}  // namespace nfs2015_chase_cam::detail
