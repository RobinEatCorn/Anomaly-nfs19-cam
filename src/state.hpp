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
  std::atomic<std::uint64_t> observed_read_failures{};
  std::atomic<std::uint64_t> lease_failures{};
  std::atomic<std::uint64_t> view_read_torn{};
  std::atomic<std::uint64_t> detour_calls{};
  std::atomic<std::uint64_t> detour_calls_window{};
  // Active view but no published pose: the frame the game camera shows through.
  std::atomic<std::uint64_t> detour_no_publish{};
  // TG-1: frames the chase pose was withheld because the game's live view-mode
  // tag was not ThirdPerson. This is the belt-and-suspenders counter for plan
  // SS2.4: a non-zero value means the presenter's flag lagged the game.
  std::atomic<std::uint64_t> tag_veto_frames{};
  // Q1: which threads call the hooked getter.
  std::atomic<std::uint32_t> detour_threads[kDiagnosticThreadSlots]{};
  std::atomic<std::uint32_t> detour_thread_count{};
  std::atomic<std::uint64_t> player_sequence_jumps{};
  // T3.1: ticks that fell back to the render-thread camera observation because
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

// PLAN-render-thread-hook.md task group 0: read-only phase probe. The host
// samples `player.position` on the Game tick, but the Game draws the car from
// whatever the render thread sees at draw time. This walks the host's own
// player chain from `ue5.GWorld` on both clocks so the difference can be read
// instead of argued about. Nothing here feeds the camera.
//
// The Game-tick read is the control: it must equal the host snapshot exactly.
// If it does not, the hardcoded offsets no longer describe this build and the
// render-thread number cannot be trusted either.
struct RenderXProbe final {
  // Game tick: resolution state, the GWorld slot, and the cached Pawn chain.
  std::uint8_t resolve_state{kRenderXWait};
  std::uint32_t resolve_code{};
  double next_resolve_elapsed{};
  std::uintptr_t gworld_slot{};
  std::atomic<std::uintptr_t> root{};
  // Game tick: the host snapshot the render-thread read is compared with. The
  // velocity turns the difference into a signed lead along the direction of
  // travel, which is what separates "same field, read later" from "some other
  // field that happens to be near". Measured from this probe's own successive
  // snapshots, so it keeps working with the camera switched off.
  std::atomic<double> player_cm[3]{};
  std::atomic<double> velocity_cmps[3]{};
  std::atomic_bool player_valid{};
  // Which thread actually performs the X read. If the detour runs on the Game
  // thread, a "render-time" reading is not render-time at all, and the whole
  // premise needs revisiting before any anchoring work.
  std::atomic<std::uint32_t> game_thread_id{};
  std::atomic<std::uint64_t> samples_on_game_thread{};
  double last_player_cm[3]{};
  double last_player_time{};
  bool have_last_player{};
  // Game tick: same-chain cross-check, window max and session mismatch count.
  double chain_max_cm{};
  std::uint64_t chain_mismatches{};
  // Render thread: phase samples. Reset with the host's 0.5 s windows.
  std::atomic<std::uint64_t> read_failures{};
  std::atomic<std::uint64_t> sample_count{};
  std::atomic<std::uint64_t> peak_dcm{};
  std::atomic<std::uint64_t> sum_dcm{};
  std::atomic<std::uint64_t> peak_dms{};
  std::atomic<std::uint64_t> sum_dms{};
  std::atomic<std::uint64_t> speed_sum_cmps{};
  std::atomic<std::int64_t> lead_sum_dcm{};
  std::atomic<std::int64_t> lead_sum_dms{};
  std::atomic<std::int64_t> lead_count{};

  // Task group 1: version gate and the anchor snapshot the presenter reads.
  // `insurance_state` and `chain_failed` are Game-tick only; the detour reads
  // `anchor_enabled` and the snapshot fields.
  std::uint8_t insurance_state{kAnchorInsuranceWaiting};
  std::uint8_t logged_insurance_state{0xFF};
  // Fail closed for the route B hooks that dereference the game's FViewInfo /
  // FSceneRenderer layout. The profile hash is the only runtime witness we
  // have for those offsets; patterns only prove that the function entry still
  // exists. A missing/mismatched profile disables the camera instead of
  // writing through a moved field.
  std::atomic_bool version_gate{};
  bool chain_verified{};
  bool chain_failed{};
  std::uint32_t chain_fail_streak{};
  std::atomic_bool anchor_enabled{};
  std::atomic_bool anchor_valid{};
  std::atomic<double> anchor_cm[3]{};
  std::atomic<std::int64_t> anchor_ns{};
  std::atomic<std::int64_t> last_read_ns{};
  std::atomic<std::uint64_t> anchor_read_failures{};
  std::atomic<std::uint64_t> anchor_fallbacks{};
  std::atomic<std::uint8_t> last_used{kAnchorStateDisabled};
  // Task group 3: the vehicle anchor is the player capsule's AttachParent
  // (the `VehicleMesh` SkeletalMesh). `vehicle_anchor_cm` is the scene proxy's
  // LocalToWorld translation, or the parent's ComponentToWorld when the proxy
  // did not validate, read in the same pass as `anchor_cm`; `vehicle_parent`
  // is published for diagnostics only. `vehicle_read_failures` counts a
  // non-null parent whose transform did not validate; a missing parent (on
  // foot) is not a failure.
  std::atomic_bool vehicle_anchor_valid{};
  std::atomic<double> vehicle_anchor_cm[3]{};
  std::atomic<std::int64_t> vehicle_anchor_ns{};
  std::atomic<std::uintptr_t> vehicle_parent{};
  // Game-tick presence signal derived from the same AttachParent read. The
  // timestamp is refreshed on every valid parent sample, even while the car is
  // parked and OnTransformChanged is not firing.
  std::atomic_bool vehicle_present{};
  std::atomic<std::int64_t> vehicle_present_ns{};
  // Last successful presence sample, present or absent. Unlike
  // `vehicle_present_ns` (last present), this lets the Game tick distinguish
  // "no parent, so definitely on foot" from "the parent read failed".
  std::atomic<std::int64_t> vehicle_sample_ns{};
  std::atomic<std::uint64_t> vehicle_read_failures{};
  std::atomic<std::uint64_t> vehicle_fallbacks{};
  // A torn AttachParent/proxy read must not clear the render-thread proxy on
  // the first miss: the prepare path would then hand one frame to the game
  // camera. Keep the last valid proxy for a couple of 20 Hz polls, then fail
  // closed if the vehicle really disappeared. Game-tick only.
  std::atomic<std::uint32_t> vehicle_anchor_miss_streak{};
  // True when the anchor came from the scene proxy rather than the component
  // transform; the count is the number of valid parent frames that had to use
  // the component transform because the proxy was null or did not validate.
  std::atomic_bool vehicle_proxy_used{};
  std::atomic<std::uint64_t> vehicle_proxy_fallbacks{};
  // Task group 3 A2: the render thread's own transform notification. `watched_proxy`
  // is published by the Game tick; the detour compares it by pointer. The
  // published triple is guarded by `transform_sequence` (odd while writing), so
  // the camera never mixes two frames of a moving car.
  std::atomic<std::uintptr_t> watched_proxy{};
  std::atomic<std::uint64_t> transform_sequence{};
  std::atomic_bool transform_valid{};
  std::atomic<std::uintptr_t> transform_proxy{};
  std::atomic<double> transform_cm[3]{};
  std::atomic<double> transform_forward[3]{};
  std::atomic_bool transform_orientation_valid{};
  std::atomic<std::int64_t> transform_ns{};
  std::atomic<std::uint64_t> transform_updates{};
  // Thread identity of the render-thread transform hook. The view hook is
  // classified against it: a setup call from the Game thread (the renderer's
  // constructor) must not be mistaken for a render-thread view build.
  std::atomic<std::uint32_t> render_thread_id{};
  std::atomic<std::uint64_t> view_uniform_calls{};
  std::atomic<std::uint64_t> view_uniform_render_calls{};
  std::atomic<std::uint64_t> view_uniform_game_calls{};
  std::atomic<std::uint64_t> view_uniform_age_peak_us{};
  std::atomic<std::uint64_t> view_uniform_age_sum_us{};
  std::atomic<std::uint64_t> view_uniform_age_count{};
  std::atomic<std::uint64_t> route_b_applied{};
  std::atomic<std::uint64_t> route_b_fallbacks{};
  std::atomic<std::uint64_t> route_b_witness_failures{};
  std::atomic<std::uint64_t> route_b_last_age_us{};
  // Pre-visibility applications. This is the only camera writer: the late
  // SetupUniformBufferParameters hook no longer moves the view, because a
  // post-Prepare pose would leave culling, motion vectors and TAA history on a
  // different camera (the one-frame ghost).
  std::atomic<std::uint64_t> route_b_early_applied{};
  std::atomic<std::uint64_t> route_b_early_fallbacks{};
  std::atomic<std::uint64_t> route_b_early_last_age_us{};
  std::atomic<std::int64_t> route_b_early_ns{};
  std::atomic<std::uint64_t> route_b_early_direct{};
  std::atomic<std::uint64_t> route_b_early_presenter{};
  std::atomic<std::uint64_t> route_b_early_held{};
  // TEMP read-only timing diagnostics. These fields never write the camera.
  std::atomic<std::uint64_t> timing_frame{};
  std::atomic<std::uint64_t> timing_early_count{};
  std::atomic<std::uint64_t> timing_late_count{};
  std::atomic<std::uint64_t> timing_transform_before{};
  std::atomic<std::uint64_t> timing_transform_between{};
  std::atomic<std::uint64_t> timing_transform_after{};
  std::atomic<std::int64_t> timing_early_ns{};
  std::atomic<std::int64_t> timing_late_ns{};
  std::atomic_bool timing_late_seen{};
  std::atomic<double> timing_early_car[3]{};
  std::atomic<double> timing_late_car[3]{};
  std::atomic<double> timing_early_view[3]{};
  std::atomic<double> timing_late_view[3]{};
  std::atomic<double> timing_presenter_car[3]{};
  std::atomic<std::uint64_t> timing_car_delta_peak_dmm{};
  std::atomic<std::uint64_t> timing_car_delta_sum_dmm{};
  std::atomic<std::uint64_t> timing_car_delta_count{};
  std::atomic<std::uint64_t> timing_view_delta_peak_dmm{};
  std::atomic<std::uint64_t> timing_view_delta_sum_dmm{};
  std::atomic<std::uint64_t> timing_view_delta_count{};
  // Render-thread-only last pose. If a proxy read is torn for one frame, repeat
  // this pose briefly instead of dropping the frame to the game camera. Reset
  // only when the session starts, before any callback can be in flight.
  bool route_b_held_pose_valid{};
  double route_b_held_eye_cm[3]{};
  double route_b_held_rotation[3]{};
  std::int64_t route_b_held_ns{};
  std::atomic<std::uint64_t> route_b_render_applied{};
  std::atomic<std::uint64_t> route_b_history_updated{};
  // Latest NFS chase-model pose, published by the presenter on the Game
  // thread and consumed by route B on the render thread. `sequence` is the
  // usual odd/even seqlock; the transform remains the position truth, the
  // model only supplies its camera offset and attitude.
  std::atomic<std::uint64_t> route_b_model_sequence{};
  std::atomic_bool route_b_model_valid{};
  std::atomic<std::int64_t> route_b_model_ns{};
  std::atomic<double> route_b_model_car_cm[3]{};
  std::atomic<double> route_b_model_eye_cm[3]{};
  std::atomic<double> route_b_model_rotation[3]{};
  std::atomic<double> route_b_model_camera_scale{1.0};
  std::atomic<double> route_b_model_origin_offset_cm{};
  std::atomic<float> route_b_model_lens{};
  std::atomic<std::uint64_t> prepare_calls{};
  std::atomic<std::uint64_t> prepare_thread_mismatch{};
  std::atomic<std::uint64_t> prepare_anchor_off{};
  std::atomic<std::uint64_t> prepare_no_views{};
  std::atomic<std::uint64_t> prepare_pose_fail{};
  std::atomic<std::uint64_t> prepare_fullscreen_views{};
  // The exact views written before visibility. Kept for diagnostics only:
  // route B no longer writes the camera from the late view hook, so a stale
  // list cannot move a view after culling/TAA history were captured.
  std::atomic<std::uint32_t> route_b_early_view_count{};
  std::atomic<bool> route_b_early_views_valid{};
  std::atomic<std::uint64_t> route_b_prepare_generation{};
  std::atomic<std::uint64_t> route_b_transform_generation{};
  std::atomic<std::uintptr_t> route_b_prepare_renderer{};
  std::atomic<std::uintptr_t> route_b_active_renderer{};
  std::atomic<std::int64_t> route_b_prepare_ns{};
  std::atomic<std::uintptr_t> route_b_early_views[8]{};
  std::atomic<std::uintptr_t> route_b_early_view_states[8]{};
  // Task group 3 A2 isolation: the hook's own timestamp against the Game-tick
  // proxy read in the same detour pass. A stable phase means the render value
  // is the same transform read on another clock; a moving phase means the hook
  // itself is stepping or stale. Read-only, reset with the 0.5 s windows.
  std::atomic<std::uint64_t> render_proxy_peak_dcm{};
  std::atomic<std::uint64_t> render_proxy_sum_dcm{};
  std::atomic<std::uint64_t> render_proxy_count{};
  std::atomic<std::uint64_t> render_proxy_peak_dms{};
  std::atomic<std::uint64_t> render_proxy_sum_dms{};
  std::atomic<std::uint64_t> render_proxy_time_count{};
  std::atomic<std::int64_t> render_proxy_lead_sum_dcm{};
  std::atomic<std::int64_t> render_proxy_lead_sum_dms{};
  std::atomic<std::int64_t> render_proxy_lead_count{};
  std::atomic<std::uint64_t> render_age_peak_us{};
  std::atomic<std::uint64_t> render_age_sum_us{};
  std::atomic<std::uint64_t> render_age_count{};
  std::atomic_bool vehicle_render_used{};

  void Reset() noexcept {
    resolve_state = kRenderXWait;
    resolve_code = 0;
    next_resolve_elapsed = 0.0;
    gworld_slot = 0;
    root.store(0, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      player_cm[axis].store(0.0, std::memory_order_relaxed);
      velocity_cmps[axis].store(0.0, std::memory_order_relaxed);
    }
    player_valid.store(false, std::memory_order_relaxed);
    game_thread_id.store(0, std::memory_order_relaxed);
    samples_on_game_thread.store(0, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      last_player_cm[axis] = 0.0;
    }
    last_player_time = 0.0;
    have_last_player = false;
    chain_max_cm = 0.0;
    chain_mismatches = 0;
    read_failures.store(0, std::memory_order_relaxed);
    sample_count.store(0, std::memory_order_relaxed);
    peak_dcm.store(0, std::memory_order_relaxed);
    sum_dcm.store(0, std::memory_order_relaxed);
    peak_dms.store(0, std::memory_order_relaxed);
    sum_dms.store(0, std::memory_order_relaxed);
    speed_sum_cmps.store(0, std::memory_order_relaxed);
    lead_sum_dcm.store(0, std::memory_order_relaxed);
    lead_sum_dms.store(0, std::memory_order_relaxed);
    lead_count.store(0, std::memory_order_relaxed);
    insurance_state = kAnchorInsuranceWaiting;
    logged_insurance_state = 0xFF;
    version_gate.store(false, std::memory_order_relaxed);
    chain_verified = false;
    chain_failed = false;
    chain_fail_streak = 0;
    anchor_enabled.store(false, std::memory_order_relaxed);
    anchor_valid.store(false, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      anchor_cm[axis].store(0.0, std::memory_order_relaxed);
    }
    anchor_ns.store(0, std::memory_order_relaxed);
    last_read_ns.store(0, std::memory_order_relaxed);
    anchor_read_failures.store(0, std::memory_order_relaxed);
    anchor_fallbacks.store(0, std::memory_order_relaxed);
    last_used.store(kAnchorStateDisabled, std::memory_order_relaxed);
    vehicle_anchor_valid.store(false, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      vehicle_anchor_cm[axis].store(0.0, std::memory_order_relaxed);
    }
    vehicle_anchor_ns.store(0, std::memory_order_relaxed);
    vehicle_parent.store(0, std::memory_order_relaxed);
    vehicle_present.store(false, std::memory_order_relaxed);
    vehicle_present_ns.store(0, std::memory_order_relaxed);
    vehicle_sample_ns.store(0, std::memory_order_relaxed);
    vehicle_read_failures.store(0, std::memory_order_relaxed);
    vehicle_fallbacks.store(0, std::memory_order_relaxed);
    vehicle_anchor_miss_streak.store(0, std::memory_order_relaxed);
    vehicle_proxy_used.store(false, std::memory_order_relaxed);
    vehicle_proxy_fallbacks.store(0, std::memory_order_relaxed);
    watched_proxy.store(0, std::memory_order_relaxed);
    transform_sequence.store(0, std::memory_order_relaxed);
    transform_valid.store(false, std::memory_order_relaxed);
    transform_proxy.store(0, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      transform_cm[axis].store(0.0, std::memory_order_relaxed);
      transform_forward[axis].store(0.0, std::memory_order_relaxed);
    }
    transform_orientation_valid.store(false, std::memory_order_relaxed);
    transform_ns.store(0, std::memory_order_relaxed);
    transform_updates.store(0, std::memory_order_relaxed);
    render_thread_id.store(0, std::memory_order_relaxed);
    view_uniform_calls.store(0, std::memory_order_relaxed);
    view_uniform_render_calls.store(0, std::memory_order_relaxed);
    view_uniform_game_calls.store(0, std::memory_order_relaxed);
    view_uniform_age_peak_us.store(0, std::memory_order_relaxed);
    view_uniform_age_sum_us.store(0, std::memory_order_relaxed);
    view_uniform_age_count.store(0, std::memory_order_relaxed);
    route_b_applied.store(0, std::memory_order_relaxed);
    route_b_fallbacks.store(0, std::memory_order_relaxed);
    route_b_witness_failures.store(0, std::memory_order_relaxed);
    route_b_last_age_us.store(0, std::memory_order_relaxed);
    route_b_early_applied.store(0, std::memory_order_relaxed);
    route_b_early_fallbacks.store(0, std::memory_order_relaxed);
    route_b_early_last_age_us.store(0, std::memory_order_relaxed);
    route_b_early_ns.store(0, std::memory_order_relaxed);
    route_b_early_direct.store(0, std::memory_order_relaxed);
    route_b_early_presenter.store(0, std::memory_order_relaxed);
    route_b_early_held.store(0, std::memory_order_relaxed);
    route_b_held_pose_valid = false;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      route_b_held_eye_cm[axis] = 0.0;
      route_b_held_rotation[axis] = 0.0;
    }
    route_b_held_ns = 0;
    route_b_render_applied.store(0, std::memory_order_relaxed);
    route_b_history_updated.store(0, std::memory_order_relaxed);
    route_b_model_sequence.store(0, std::memory_order_relaxed);
    route_b_model_valid.store(false, std::memory_order_relaxed);
    route_b_model_ns.store(0, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      route_b_model_car_cm[axis].store(0.0, std::memory_order_relaxed);
      route_b_model_eye_cm[axis].store(0.0, std::memory_order_relaxed);
      route_b_model_rotation[axis].store(0.0, std::memory_order_relaxed);
    }
    route_b_model_camera_scale.store(1.0, std::memory_order_relaxed);
    route_b_model_origin_offset_cm.store(0.0, std::memory_order_relaxed);
    route_b_model_lens.store(0.0F, std::memory_order_relaxed);
    prepare_calls.store(0, std::memory_order_relaxed);
    prepare_thread_mismatch.store(0, std::memory_order_relaxed);
    prepare_anchor_off.store(0, std::memory_order_relaxed);
    prepare_no_views.store(0, std::memory_order_relaxed);
    prepare_pose_fail.store(0, std::memory_order_relaxed);
    prepare_fullscreen_views.store(0, std::memory_order_relaxed);
    route_b_early_view_count.store(0, std::memory_order_relaxed);
    route_b_early_views_valid.store(false, std::memory_order_relaxed);
    route_b_prepare_generation.store(0, std::memory_order_relaxed);
    route_b_transform_generation.store(0, std::memory_order_relaxed);
    route_b_prepare_renderer.store(0, std::memory_order_relaxed);
    route_b_active_renderer.store(0, std::memory_order_relaxed);
    route_b_prepare_ns.store(0, std::memory_order_relaxed);
    for (auto &view : route_b_early_views) {
      view.store(0, std::memory_order_relaxed);
    }
    for (auto &state : route_b_early_view_states) {
      state.store(0, std::memory_order_relaxed);
    }
    render_proxy_peak_dcm.store(0, std::memory_order_relaxed);
    render_proxy_sum_dcm.store(0, std::memory_order_relaxed);
    render_proxy_count.store(0, std::memory_order_relaxed);
    render_proxy_peak_dms.store(0, std::memory_order_relaxed);
    render_proxy_sum_dms.store(0, std::memory_order_relaxed);
    render_proxy_time_count.store(0, std::memory_order_relaxed);
    render_proxy_lead_sum_dcm.store(0, std::memory_order_relaxed);
    render_proxy_lead_sum_dms.store(0, std::memory_order_relaxed);
    render_proxy_lead_count.store(0, std::memory_order_relaxed);
    render_age_peak_us.store(0, std::memory_order_relaxed);
    render_age_sum_us.store(0, std::memory_order_relaxed);
    render_age_count.store(0, std::memory_order_relaxed);
    vehicle_render_used.store(false, std::memory_order_relaxed);
  }

  // Called by the Game tick every 0.5 s, next to the other diagnostic windows.
  void ResetWindow() noexcept {
    peak_dcm.store(0, std::memory_order_relaxed);
    sum_dcm.store(0, std::memory_order_relaxed);
    peak_dms.store(0, std::memory_order_relaxed);
    sum_dms.store(0, std::memory_order_relaxed);
    speed_sum_cmps.store(0, std::memory_order_relaxed);
    sample_count.store(0, std::memory_order_relaxed);
    lead_sum_dcm.store(0, std::memory_order_relaxed);
    lead_sum_dms.store(0, std::memory_order_relaxed);
    lead_count.store(0, std::memory_order_relaxed);
    render_proxy_peak_dcm.store(0, std::memory_order_relaxed);
    render_proxy_sum_dcm.store(0, std::memory_order_relaxed);
    render_proxy_count.store(0, std::memory_order_relaxed);
    render_proxy_peak_dms.store(0, std::memory_order_relaxed);
    render_proxy_sum_dms.store(0, std::memory_order_relaxed);
    render_proxy_time_count.store(0, std::memory_order_relaxed);
    render_proxy_lead_sum_dcm.store(0, std::memory_order_relaxed);
    render_proxy_lead_sum_dms.store(0, std::memory_order_relaxed);
    render_proxy_lead_count.store(0, std::memory_order_relaxed);
    render_age_peak_us.store(0, std::memory_order_relaxed);
    render_age_sum_us.store(0, std::memory_order_relaxed);
    render_age_count.store(0, std::memory_order_relaxed);
    samples_on_game_thread.store(0, std::memory_order_relaxed);
    chain_max_cm = 0.0;
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
  double update_hz{};
  double view_output_hz{};
  double interpolation_ratio{};
  double camera_scale{kDefaultCameraScale};
  double position_spring{kDefaultPositionSpring};
  double yaw_spring{kDefaultYawSpring};
  double drift_yaw_spring{kDefaultDriftYawSpring};
  double look_yaw_spring{kDefaultLookYawSpring};
  double yaw_return_spring{kDefaultYawReturnSpring};
  double pitch_spring{kDefaultPitchSpring};
  double roll_spring{kDefaultRollSpring};
  double origin_offset_cm{kDefaultOriginOffsetCm};
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
  double burnout_shot_blend{};
  int burnout_shot_side{1};
  std::uint64_t burnout_shot_fallbacks{};
  bool slope_follow{true};
  double slope_strength{kDefaultSlopeStrength};
  bool roll_only_in_drift{true};
  bool diagnostic_flat_pose{};
  bool diagnostic_zero_lag{};
  double borrow_strength{};
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
  double car_step_cm{};

  // Render-thread X probe (PLAN-render-thread-hook.md task group 0). Copied
  // from Context::render_x on the Game tick.
  std::uint8_t render_x_state{};
  std::uint32_t render_x_code{};
  std::uint64_t render_x_failures{};
  std::uint64_t render_x_samples{};
  bool render_x_valid{};
  double render_x_avg_cm{};
  double render_x_peak_cm{};
  double render_x_avg_ms{};
  double render_x_peak_ms{};
  double render_x_lead_cm{};
  double render_x_lead_ms{};
  double render_x_speed_kmh{};
  double render_x_chain_cm{};
  std::uint64_t render_x_chain_mismatches{};
  std::uint64_t render_x_gt_samples{};
  std::uint32_t anchor_source{kAnchorSourceDefault};
  std::uint8_t anchor_effective{kAnchorStateDisabled};
  std::uint8_t anchor_gate{kAnchorGateOff};
  std::uint64_t anchor_fallbacks{};
  std::uint64_t anchor_read_failures{};
  bool vehicle_anchor_valid{};
  bool vehicle_present{};
  bool vehicle_presence_known{};
  bool vehicle_proxy_used{};
  bool vehicle_render_used{};
  bool transform_hook_valid{};
  std::uint64_t vehicle_fallbacks{};
  std::uint64_t vehicle_read_failures{};
  std::uint64_t vehicle_proxy_fallbacks{};
  std::uint64_t transform_updates{};
  bool view_uniform_hook_valid{};
  std::uint64_t view_uniform_calls{};
  std::uint64_t view_uniform_render_calls{};
  std::uint64_t view_uniform_game_calls{};
  std::uint32_t render_thread_id{};
  bool view_uniform_age_valid{};
  double view_uniform_age_avg_ms{};
  double view_uniform_age_peak_ms{};
  std::uint64_t route_b_applied{};
  std::uint64_t route_b_fallbacks{};
  std::uint64_t route_b_witness_failures{};
  std::uint64_t route_b_early_applied{};
  std::uint64_t route_b_early_fallbacks{};
  double route_b_last_age_ms{};
  double route_b_early_last_age_ms{};
  std::uint64_t prepare_calls{};
  std::uint64_t prepare_thread_mismatch{};
  std::uint64_t prepare_anchor_off{};
  std::uint64_t prepare_no_views{};
  std::uint64_t prepare_pose_fail{};
  std::uint64_t prepare_fullscreen_views{};
  std::uint64_t route_b_early_direct{};
  std::uint64_t route_b_early_held{};
  bool render_proxy_valid{};
  double render_proxy_avg_cm{};
  double render_proxy_peak_cm{};
  double render_proxy_avg_ms{};
  double render_proxy_peak_ms{};
  double render_proxy_lead_cm{};
  double render_proxy_lead_ms{};
  bool render_age_valid{};
  double render_age_avg_ms{};
  double render_age_peak_ms{};

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
  // by the watcher and read by the presenter and the render-thread guard.
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

struct Context final {
  nfs2015_chase_cam::Localizer localizer;
  const AnomalyCoreServiceV1 *core{};
  const AnomalyConfigServiceV1 *config{};
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
  // Optional build identity. It gates the hardcoded offset table used by the X
  // anchor; a host without it simply leaves the X path disabled.
  const AnomalyUe5BuildServiceV1 *build{};
  // Optional name pool access. `find_utf8` (names service v2) is the only way
  // to turn "CameraViewMode.ThirdPerson" into the id this process assigned it;
  // without it the Z cycle stays disabled and Z remains native.
  const AnomalyUe5NamesServiceV1 *names{};

  AnomalyGenerationHandleV1 settings_schema{};
  std::atomic<std::uint64_t> view_hook_id{};
  std::atomic<std::uint64_t> view_hook_generation{};
  std::atomic<std::uint64_t> transform_hook_id{};
  std::atomic<std::uint64_t> transform_hook_generation{};
  std::atomic<std::uintptr_t> transform_hook_target{};
  std::atomic<std::uint64_t> view_uniform_hook_id{};
  std::atomic<std::uint64_t> view_uniform_hook_generation{};
  std::atomic<std::uintptr_t> view_uniform_hook_target{};
  std::atomic<std::uint64_t> prepare_view_state_hook_id{};
  std::atomic<std::uint64_t> prepare_view_state_hook_generation{};
  std::atomic<std::uintptr_t> prepare_view_state_hook_target{};

  // Settings: Render/hotkey write, Game reads.
  std::atomic_bool enabled{true};
  std::atomic<std::uint32_t> preset{0};
  std::atomic<double> camera_scale{kDefaultCameraScale};
  std::atomic<double> position_spring{kDefaultPositionSpring};
  std::atomic<double> yaw_spring{kDefaultYawSpring};
  std::atomic<double> drift_yaw_spring{kDefaultDriftYawSpring};
  std::atomic<double> look_yaw_spring{kDefaultLookYawSpring};
  std::atomic<double> yaw_return_spring{kDefaultYawReturnSpring};
  std::atomic<double> pitch_spring{kDefaultPitchSpring};
  std::atomic<double> roll_spring{kDefaultRollSpring};
  std::atomic<double> origin_offset_cm{kDefaultOriginOffsetCm};
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
  // Render-synced car position estimate. The Game draws the car at an instant
  // our Game-tick samples cannot see, but the Game's own camera position is
  // produced at that same instant. We low-pass the Game camera's offset from
  // our sampled car to recover the Game camera's framing, then subtract it
  // from the Game camera position to approximate where the Game actually drew
  // the car this frame. The chase model can then anchor itself to that point,
  // which is the quantity that was missing before.
  bool game_offset_valid{};
  double game_offset_cm[3]{};
  std::int64_t game_offset_last_ns{};
  // How much of the render-synced car position to use, 0..1. At zero this is
  // the plain NFS model anchored to the Game-tick sample; at one the model is
  // anchored to the Game's drawn car position.
  std::atomic<double> borrow_strength{1.0};
  // Which position source feeds the anchor: X (detour-time `root+280`), the
  // borrow estimate, or the plain tick sample. The X path falls back to borrow
  // when the build gate is closed, so the camera never depends on it.
  std::atomic<std::uint32_t> anchor_source{kAnchorSourceDefault};
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
  bool transform_hook_failure_logged{};
  bool view_uniform_hook_failure_logged{};
  bool view_uniform_diag_logged{};
  bool route_b_diag_logged{};
  bool route_b_early_failure_logged{};
  std::uint64_t route_b_timing_last_log_ms{};
  std::uint32_t route_b_timing_windows{};
  bool route_b_pattern_failure_logged{};
  bool prepare_view_state_hook_failure_logged{};
  bool prepare_view_state_pattern_failure_logged{};
  // A deterministic pattern miss cannot be fixed by scanning the module again.
  // Latch it so the 2 s hook retry does not re-scan .text forever; the camera
  // stays on whatever path remains available.
  bool early_route_b_unavailable{};
  std::uint32_t prepare_hook_pattern_failures{};
  bool transform_pattern_unavailable{};
  bool view_uniform_pattern_unavailable{};
  bool view_point_pattern_unavailable{};

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
  std::atomic<double> presenter_car_cm[3]{};
  std::atomic<float> presenter_pose_lens{};
  std::atomic<bool> presenter_pose_fov{};
  std::atomic<bool> presenter_pose_valid{};
  std::atomic<double> presenter_anchor[3]{};
  std::atomic<double> presenter_drift{};
  std::atomic<double> presenter_distance_cm{};
  // Diagnostics for the mixed chase/shot camera. The blend is written under
  // the presenter lock and read by route B / diagnostics.
  std::atomic<double> presenter_burnout_blend{};
  std::atomic<int> presenter_burnout_side{1};
  std::atomic<std::uint64_t> presenter_burnout_fallbacks{};
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
  // Latest body pitch published by the render-thread transform hook, radians
  // (nose up positive). Game-tick only, so plain fields are enough.
  // `render_pitch_floor_ns` rejects a snapshot from before the last reset or
  // teleport; `render_pitch_ns` is the snapshot's own timestamp.
  bool render_pitch_valid{};
  double render_pitch_radians{};
  std::int64_t render_pitch_ns{};
  std::int64_t render_pitch_floor_ns{};
  std::uintptr_t render_pitch_proxy{};
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
  // T3.1: game-camera yaw from the player ESP snapshot, i.e. sampled by the
  // host on this Game tick instead of observed on the render thread.
  double camera_yaw_radians{};
  bool camera_yaw_valid{};
  bool view_snap{true};

  // Diagnostics (T0). Game thread owns the plain fields; the detour only
  // touches Context::diag atomics.
  Diagnostics diag{};
  Probe probe{};
  // PLAN-render-thread-hook.md task group 0. Read-only: it never reaches the
  // camera or the publisher.
  RenderXProbe render_x{};
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
  bool diagnostics_open{};

  // Game writes, Render copies.
  mutable std::mutex display_mutex;
  DisplayState display{};
};

}  // namespace nfs2015_chase_cam::detail
