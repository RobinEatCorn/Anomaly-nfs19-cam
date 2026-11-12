// Settings schema, tunable defaults, hook patterns and engine offsets shared by
// every module of the plugin.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include <windows.h>

namespace nfs2015_chase_cam::detail {

constexpr std::string_view kPluginId =
    "robineatcorn.nfs2015-chase-cam";
constexpr std::string_view kSettingsSchemaId = "nfs2015-chase-cam-settings-v1";
constexpr std::string_view kSettingsWindowId = "nfs2015-chase-cam-settings-v1";
constexpr std::uint32_t kSettingsSchemaVersion = 1;
constexpr std::size_t kMaximumSettingsBytes = 4096;
constexpr std::uint64_t kSettingsSaveDelayMilliseconds = 500;
constexpr std::uint32_t kToggleKey = VK_F7;

constexpr double kDegreesToRadians =
    3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees =
    180.0 / 3.14159265358979323846;
constexpr double kDefaultCameraScale = 1.0;
constexpr double kMinimumCameraScale = 0.6;
constexpr double kMaximumCameraScale = 1.6;
constexpr double kDefaultPositionSpring = 7.5;
constexpr double kMinimumPositionSpring = 0.5;
constexpr double kMaximumPositionSpring = 30.0;
constexpr double kDefaultYawSpring = 5.4;
constexpr double kMinimumYawSpring = 0.5;
constexpr double kMaximumYawSpring = 30.0;
constexpr double kDefaultDriftYawSpring = 2.7;
constexpr double kMinimumDriftYawSpring = 0.5;
constexpr double kMaximumDriftYawSpring = 30.0;
// Composition default; the retail drift spring converts to 2.7.
constexpr double kDefaultLookYawSpring = 15.0;
constexpr double kMinimumLookYawSpring = 0.5;
constexpr double kMaximumLookYawSpring = 30.0;
constexpr double kDefaultYawReturnSpring = 4.5;
constexpr double kMinimumYawReturnSpring = 0.5;
constexpr double kMaximumYawReturnSpring = 30.0;
constexpr double kDefaultPitchSpring = 33.0;
constexpr double kMinimumPitchSpring = 1.0;
constexpr double kMaximumPitchSpring = 120.0;
constexpr double kDefaultRollSpring = 60.0;
constexpr double kMinimumRollSpring = 1.0;
constexpr double kMaximumRollSpring = 200.0;
// Route B anchors the NFS camera to the vehicle proxy's ground pivot. The
// NFS preset's local rest height is ground-relative, so the old +70 cm centre
// offset must be cancelled by default; otherwise the first install sits about
// 70 cm too high. Existing configs keep their explicit value.
constexpr double kDefaultOriginOffsetCm = -70.0;
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
// Burnout close-up defaults. These mirror BurnoutShotConfig so the persisted
// settings and the pure camera model agree when a field is absent. The effect
// is opt-in so a fresh install does not hijack the chase camera on a launch.
constexpr bool kDefaultBurnoutEnabled = false;
constexpr double kDefaultBurnoutEntryDelay = 0.5;
constexpr double kMinimumBurnoutEntryDelay = 0.0;
constexpr double kMaximumBurnoutEntryDelay = 2.0;
constexpr double kDefaultBurnoutEntryTime = 1.0;
constexpr double kMinimumBurnoutEntryTime = 0.1;
constexpr double kMaximumBurnoutEntryTime = 4.0;
constexpr double kDefaultBurnoutExitDelay = 0.2;
constexpr double kMinimumBurnoutExitDelay = 0.0;
constexpr double kMaximumBurnoutExitDelay = 2.0;
constexpr double kDefaultBurnoutExitTime = 2.5;
constexpr double kMinimumBurnoutExitTime = 0.1;
constexpr double kMaximumBurnoutExitTime = 5.0;
constexpr double kDefaultBurnoutLaunchExitTime = 1.8;
constexpr double kMinimumBurnoutLaunchExitTime = 0.1;
constexpr double kMaximumBurnoutLaunchExitTime = 5.0;
constexpr double kDefaultBurnoutFov = 45.0;
constexpr double kMinimumBurnoutFov = 20.0;
constexpr double kMaximumBurnoutFov = 90.0;
constexpr double kDefaultBurnoutShake = 1.0;
constexpr double kMinimumBurnoutShake = 0.0;
constexpr double kMaximumBurnoutShake = 2.0;
constexpr std::uint32_t kDefaultBurnoutSideMode = 0;
constexpr std::uint32_t kMaximumBurnoutSideMode = 3;
constexpr double kDefaultBurnoutEntrySpeed = 1.0;
constexpr double kMinimumBurnoutEntrySpeed = 0.0;
constexpr double kMaximumBurnoutEntrySpeed = 10.0;
constexpr double kDefaultBurnoutHoldSpeed = 3.0;
constexpr double kMinimumBurnoutHoldSpeed = 0.0;
constexpr double kMaximumBurnoutHoldSpeed = 20.0;
// Slope following. The UI exposes only the master switch and the composition
// strength; the gradient scalars stay in the settings file.
constexpr double kDefaultSlopeStrength = 1.0;
constexpr double kMinimumSlopeStrength = 0.0;
constexpr double kMaximumSlopeStrength = 2.0;
constexpr double kDefaultSlopeMaxGradientDegrees = 10.0;
constexpr double kMinimumSlopeMaxGradientDegrees = 0.0;
constexpr double kMaximumSlopeMaxGradientDegrees = 30.0;
constexpr double kDefaultSlopeUpLookAt = 0.5;
constexpr double kDefaultSlopeUpRest = 2.0;
constexpr double kDefaultSlopeDownLookAt = 0.3;
constexpr double kDefaultSlopeDownRest = 0.25;
constexpr double kMinimumSlopeLookAt = 0.0;
constexpr double kMaximumSlopeLookAt = 2.0;
constexpr double kMinimumSlopeRest = 0.0;
constexpr double kMaximumSlopeRest = 4.0;
// NFS 2015-style visual front-wheel countersteer. The visual clamp stays
// explicit so the front wheel cannot point sideways at a large slip angle.
// NFS 2015's 30 mph gate is tuned around faster cars; NTE defaults to a
// 54 km/h entry gate, exposed as a slider. That gate is entry-only: once a
// drift is engaged, low speed and low forward speed are not exit conditions.
constexpr bool kDefaultWheelSteerEnabled = true;
constexpr double kDefaultWheelSteerMaxDegrees = 32.0;
constexpr double kMinimumWheelSteerMaxDegrees = 20.0;
constexpr double kMaximumWheelSteerMaxDegrees = 50.0;
constexpr double kDefaultWheelSteerGain = 1.0;
constexpr double kMinimumWheelSteerGain = 0.5;
constexpr double kMaximumWheelSteerGain = 1.5;
constexpr double kDefaultWheelSteerFullSlipDegrees = 40.0;
constexpr double kMinimumWheelSteerFullSlipDegrees = 10.0;
constexpr double kMaximumWheelSteerFullSlipDegrees = 60.0;
constexpr double kDefaultWheelSteerRateDegreesPerSecond = 120.0;
constexpr double kMinimumWheelSteerRateDegreesPerSecond = 60.0;
constexpr double kMaximumWheelSteerRateDegreesPerSecond = 1200.0;
// 2.0.2 shipped these three values together. 2.0.3 changes the scale (front
// axle target, 40 degree full slip, 32 degree cap, 120 deg/s). Only an
// untouched old trio is migrated; a user who deliberately changed one field
// keeps their saved values.
constexpr double kLegacyWheelSteerMaxDegrees = 40.0;
constexpr double kLegacyWheelSteerFullSlipDegrees = 25.0;
constexpr double kLegacyWheelSteerRateDegreesPerSecond = 360.0;
// NFS 2015's 30 mph gate is tuned around faster cars. NTE tends to drift at
// lower speeds, so the default entry gate is 54 km/h and the panel exposes it
// as a slider. This is deliberately entry-only: once the drift state is
// engaged, speed is not an exit condition.
constexpr double kDefaultWheelSteerEntrySpeedKmh = 54.0;
constexpr double kMinimumWheelSteerEntrySpeedKmh = 20.0;
constexpr double kMaximumWheelSteerEntrySpeedKmh = 160.0;
constexpr double kWheelSteerEnterSlipDegrees = 6.5;
constexpr double kWheelSteerExitSlipDegrees = 3.0;
constexpr double kWheelSteerHoldSeconds = 0.5;
constexpr double kWheelSteerExitHoldSeconds = 0.25;
constexpr double kWheelSteerBlendInSeconds = 0.20;
constexpr double kWheelSteerBlendOutSeconds = 0.30;
constexpr double kWheelSteerStockMaxDegrees = 35.0;
constexpr double kWheelSteerCounterAuthorityDegrees = 5.0;
constexpr double kWheelSteerRestoreToleranceDegrees = 1.5;
constexpr double kWheelSteerTurnInSeconds = 0.18;
constexpr double kWheelSteerTurnInMinSeconds = 0.10;
constexpr double kWheelSteerTurnInMaxSeconds = 0.30;
constexpr double kWheelSteerTurnInStartSlipDegrees = 8.0;
constexpr double kWheelSteerTurnInFullSlipDegrees = 22.0;
constexpr double kWheelSteerMinimumTurnInAlpha = 0.08;
constexpr double kWheelSteerTurnInRateDegreesPerSecond = 300.0;
constexpr double kWheelSteerTurnInMaxRateDegreesPerSecond = 900.0;
constexpr double kWheelSteerTurnInReleaseSlipDegrees = 18.0;
constexpr double kWheelSteerReentryCooldownSeconds = 0.5;
constexpr double kWheelSteerStandstillSpeedMps = 0.5;
constexpr double kWheelSteerStandstillGraceSeconds = 0.5;
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
// Presence comes from the reflected vehicle getters (20 Hz poll). When those
// services are unavailable, the player capsule's AttachParent is the fallback:
// fresh means the render-thread read saw a valid vehicle parent. `recent`
// covers one missed poll while entering/exiting or while the renderer is
// streaming. Speed is used only when neither signal can answer.
constexpr double kVehiclePresenceGraceSeconds = 0.20;
// TG-1: how long the plugin has to have been *out* of a vehicle before the next
// driving tick counts as "got in". Above the presence grace on purpose: the
// 0.20 s grace already absorbs one missed 20 Hz poll, and this one decides
// whether the chase camera comes back on its own. A gap shorter than this (a
// pawn-chain hiccup, a paused tick) must not resurrect a camera the player
// dismissed with Z.
constexpr double kChaseEntryGapSeconds = 0.75;
constexpr std::int64_t kVehiclePresenceFreshNs = 250'000'000;
// The 2.6 vehicle service also returned GetForwardSpeedKmH. 2.7 removed the
// service, not the reflected function: the movement component sits at the
// offset the 2.6 profile validated, and the getter is still callable through
// ue5.process-event. Exact speed is the smooth signal the old plugin used for
// reverse, braking pitch and longitudinal acceleration.
constexpr std::uint32_t kObjectInternalIndexOffset = 0x0C;
constexpr std::uint32_t kObjectClassOffset = 0x10;
constexpr std::uint32_t kObjectNameOffset = 0x18;
constexpr std::uint32_t kUstructSuperStructOffset = 0x40;
constexpr std::uint32_t kVehicleMovementComponentOffset = 0x378;
// UE accepts both the Class:Function and Class.Function forms for a UFunction
// path; try both so the exact-speed recovery does not depend on which one the
// active build's StaticFindObject handles.
constexpr std::string_view kVehicleSpeedFunctionPaths[2] = {
    "/Script/HTGame.HTVehicleMovementComponent:GetForwardSpeedKmH",
    "/Script/HTGame.HTVehicleMovementComponent.GetForwardSpeedKmH"};
constexpr double kReverseSpeedKmh = 1.0;
// Reverse is inferred from the angle between travel and the game-camera yaw.
// Enter well past 90 degrees, leave earlier, and ignore walking-speed noise.
constexpr double kReverseDecisionMinSpeedMps = 1.0;
constexpr double kReverseEnterCosine = -0.70;  // about 134 degrees
constexpr double kReverseExitCosine = -0.40;   // about 114 degrees
// Position differentiation is noisier than the removed exact-speed sample.
// Braking uses a larger dead zone and slower filters so suspension/step noise
// cannot pump the pitch.
constexpr double kDerivedBrakingPitchDeadzoneMps2 = 2.2;
constexpr double kDerivedBrakingDecelerationSmoothing = 4.0;
constexpr double kDerivedBrakingInputSmoothing = 6.0;
// A frame hitch is not a brake. If the next motion sample arrives this late,
// discard it instead of differentiating a one-off 100 ms gap into nose dive.
constexpr double kBrakingMaxSampleIntervalSeconds = 0.10;
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
// Multiple render passes call the detour for one frame. Treat calls
// closer than this as the same frame so the model advances once per
// frame, not once per pass. 60 fps frames are 16.7 ms apart and the
// passes within a frame are usually well under 2 ms, so 8 ms sits in
// the middle.
constexpr std::int64_t kPresenterSameFrameNs = 8'000'000;
// The camera-to-car distance is the direct screen-size driver. Low-pass it so
// Game-tick sampling/spring phase noise cannot make the car breathe in frame.
constexpr double kViewDistanceSmoothing = 4.0;
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
constexpr double kHookRetrySeconds = 2.0;
// The signature service can still be settling on the first Game tick. A
// pattern miss is therefore retried for a few 2 s hook windows before the
// early route B path is latched off; a single transient miss must not leave the
// camera on the late hook forever.
constexpr std::uint32_t kPrepareHookMaxPatternFailures = 5;
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

// ---------------------------------------------------------------------------
// Z-key view-mode recon (PLAN-zkey-phase2-h1.md task group 0).
//
// `HTPlayerController::ToggleSwitchCameraViewMode(bool bForward)`, the single
// point the game's own Z key ends up in. The prologue compares the view-mode
// instance count (`cmp [rcx+0x1E90],0`) before it reads the argument, so those
// 22 bytes are unique in this build's .text. Resolving it gives the function,
// not an instruction inside it; the vtable slot below is the second witness.
constexpr std::string_view kToggleSwitchCameraViewModePattern =
    "40 53 55 57 48 83 EC 30 83 B9 90 1E 00 00 00 0F B6 FA 48 8B D9";
// `PC->vtable[kToggleVtableOffset]` must equal the resolved target. A plugin
// (or a game update) that moves the entry point invalidates the hook even when
// the pattern still matches.
constexpr std::uint32_t kToggleVtableOffset = 0x15C8;
// The view-mode block on the PlayerController. `tag` is an FGameplayTag (an
// FName: 8 bytes, Number always 0); the array holds `{FGameplayTag; UObject*}`
// entries with a 16-byte stride. Every field is only meaningful for the
// profile the plugin was verified against, which is why the same hash gate that
// guards the render hooks guards this one.
constexpr std::uint32_t kPcViewModeTagOffset = 0x1E80;
constexpr std::uint32_t kPcViewModeArrayOffset = 0x1E88;
constexpr std::uint32_t kPcViewModeCountOffset = 0x1E90;
constexpr std::uint32_t kPcViewModeCapacityOffset = 0x1E94;
constexpr std::uint32_t kPcLimitManualToggleOffset = 0x1E98;
constexpr std::uint32_t kPcToggleCooldownOffset = 0x1E9C;
constexpr std::uint32_t kPcToggleGraceOffset = 0x1EA0;
constexpr std::uint32_t kPcToggleCooldownClockOffset = 0x1EA8;
constexpr std::uint32_t kPcToggleGraceUsedOffset = 0x1EB0;
// One read covers tag..graceUsed inclusive (0x1E80..0x1EB0).
constexpr std::uint32_t kPcViewModeStateSpan = 0x38;
constexpr std::uint32_t kViewModeEntryStride = 0x10;
constexpr std::size_t kMaximumLoggedViewModes = 8;
// TG-0 only records; a scripted path can call the toggle every frame, so the
// log stops being interesting long before the session does.
constexpr std::uint64_t kToggleLogAllCalls = 400;
constexpr std::uint64_t kToggleLogStrideAfter = 10;

// Render-thread car position probe (PLAN-render-thread-hook.md task group 0).
// The profile's `ue5.GWorld` entry: `mov rbx,[rip+disp32]` is the first
// instruction, 7 bytes long, and the displacement sits at +3. Resolving the
// pattern gives the instruction, so the global slot is
// `instruction + instructionSize + displacement`.
constexpr std::string_view kGWorldPattern =
    "48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01";
constexpr std::uint32_t kGWorldRipOffset = 3;
constexpr std::uint32_t kGWorldInstructionSize = 7;
// Offsets from the host's own `ReadCurrentPlayerLocation`, i.e. the profile's
// `nte-current.json` layout. Every one of them is a hardcoded guess about the
// running build; the Game-tick cross-check below is what catches a stale table.
constexpr std::uint32_t kWorldGameInstanceOffset = 560;
constexpr std::uint32_t kGameInstanceLocalPlayersOffset = 56;
constexpr std::uint32_t kLocalPlayerControllerOffset = 48;
constexpr std::uint32_t kControllerPawnOffset = 776;
constexpr std::uint32_t kActorRootComponentOffset = 456;
constexpr std::uint32_t kSceneComponentLocationOffset = 280;
// The vehicle anchor reads the player SceneComponent's AttachParent. While
// driving, the character capsule is attached to the vehicle's SkeletalMesh
// component (object name `VehicleMesh`); its bounds origin is the car's visual
// centre, while the capsule's own bounds origin is the seat. The parent pointer
// and the parent transform are read in the same detour pass, so their relative
// offset is not polluted by the car moving between two reads.
constexpr std::uint32_t kSceneComponentAttachParentOffset = 0xD8;
constexpr std::uint32_t kSceneComponentRelativeRotationOffset = 0x168;
constexpr std::uint32_t kSceneComponentTransformOffset = 0x1F0;
constexpr std::uint32_t kSceneComponentTransformTranslationOffset = 0x210;
// NTE player vehicles expose their visual wheel components as a TArray of
// StaticMeshComponents. The animation controller normally owns all four; the
// drift override parks it and writes the front steer plus the matching roll on
// every wheel.
constexpr std::uint32_t kVehicleMeshOffset = 0x348;
constexpr std::uint32_t kVehicleFrontWheelsOffset = 0x12D8;
constexpr std::uint32_t kVehicleRearWheelsOffset = 0x12E8;
constexpr std::uint32_t kSceneComponentAnimScriptInstanceOffset = 0x938;
constexpr std::uint32_t kVehicleAnimWheelAnimationOffset = 0xC98;
constexpr std::uint32_t kTArrayDataOffset = 0x0;
constexpr std::uint32_t kTArrayNumOffset = 0x8;
constexpr std::size_t kMaximumFrontWheels = 4;
constexpr std::size_t kMaximumRearWheels = 4;
constexpr std::size_t kMaximumWheelSteerWheels =
    kMaximumFrontWheels + kMaximumRearWheels;
// The hand-driven visual roll is only a phase ramp. The game's tyre radius is
// not exposed through a stable reflection path, so this value is deliberately
// a tuning constant rather than a physics read. It only changes how fast the
// visual wheel spins while the override owns the wheel.
constexpr double kWheelVisualRollRadiusMeters = 0.35;
constexpr double kVehicleAnchorMaxOffsetCm = 1000.0;
// The vehicle mesh's ComponentToWorld translation is the chassis pivot at the
// ground; the bounds centre used by the old X path sits about this much above
// it. Keeping the height in the anchor keeps the camera's existing framing when
// the source switches from the capsule/bounds to the component transform.
constexpr double kVehicleAnchorCenterHeightCm = 70.0;
// The renderer keeps a parallel `FPrimitiveSceneProxy` for every rendered
// primitive. For the vehicle SkeletalMesh the component points at it from
// +0x340; the proxy points back at its owner from +0x228. Its LocalToWorld is
// a row-major 4x4 double matrix at +0xE0, so the translation is the last row at
// +0x140. The value at +0x160 is the bounds origin: it is derived from the
// mesh and moves with a damaged door, so it must not be used as the camera
// anchor. The owner back-pointer and the distance to ComponentToWorld make a
// wrong offset fall back instead of moving the camera to a random object.
// These are hardcoded for the profile above; the profile-hash gate still
// applies to the whole vehicle path.
constexpr std::uint32_t kVehicleProxyOffset = 0x340;
constexpr std::uint32_t kVehicleProxyOwnerOffset = 0x228;
constexpr std::uint32_t kVehicleProxyTranslationOffset = 0x140;
// FPrimitiveSceneProxy::OnTransformChanged, the virtual hook SetTransform
// calls after it has copied the new matrix and bounds into the proxy. The
// skeletal-mesh implementation reads the proxy's LocalToWorld (+0xE0) and
// forwards it to the mesh object, so its translation (+0x140) is the exact
// render-thread transform the car is drawn with. Hooking this function sees
// only skeletal proxies, not every primitive SetTransform touches. The
// pattern pins the implementation's prologue and its first proxy fields; it
// is resolved at runtime instead of trusting a hardcoded address.
constexpr std::string_view kOnTransformChangedPattern =
    "48 89 5C 24 08 48 89 6C 24 10 48 89 74 24 18 57 48 83 EC 20 "
    "48 8B B9 50 02 00 00 48 8B E9 48 8B 89 F0 01 00 00";
// FViewInfo::SetupUniformBufferParameters (2.6.0). This function runs while
// the renderer is building the view's uniform buffer, after primitive render
// transforms have been applied. The pattern is unique in the dumped .text.
constexpr std::string_view kSetupUniformBufferPattern =
    "48 8B C4 55 53 56 57 41 54 41 55 41 56 41 57 48 8D A8 28 FB FF FF "
    "48 81 EC 98 05 00 00";
// FViewMatrices constructor from FMinimalInitializer (2.6.0), also unique in
// the dumped .text. Route B needs the game's own derived matrices rather than
// a partial hand-written copy of them.
constexpr std::string_view kViewMatricesCtorPattern =
    "48 8B C4 48 89 58 18 55 56 57 41 54 41 55 41 56 41 57 48 8D A8 B8 FB "
    "FF FF 48 81 EC 10 05 00 00";
// FSceneRenderer::PrepareViewStateForVisibility (2.6.0). This is the last
// point before visibility tasks are launched and before the view's temporal
// history is advanced. Route B must be written here, not in the later
// SetupUniformBufferParameters hook: culling and TAA both consume the pose in
// this function.
constexpr std::string_view kPrepareViewStatePattern =
    "40 55 41 56 41 57 48 8D AC 24 70 E9 FF FF B8 90 17 00 00 E8 ?? ?? ?? "
    "?? 48 2B E0 48 8B 05";
// FSceneView::UpdateViewMatrix (2.6.0). Rebuilds ViewMatrices from the fields
// written by route B, including ViewFrustum, but not CullingFrustum.
constexpr std::string_view kSceneViewUpdateMatrixPattern =
    "40 53 48 83 EC 60 0F 10 81 F0 0F 00 00 48 8B D9 48 8B 05 ?? ?? ?? ?? "
    "F2 0F 10 89 00 10 00 00 0F 11 44 24 48 0F 10 81 08 10 00 00 F2 0F 11 "
    "4C 24";
// FSceneView::SetupViewFrustum (2.6.0). Recomputes ViewFrustum and, for the
// non-stereo case, copies it into CullingFrustum/CullingOrigin.
constexpr std::string_view kSceneViewSetupFrustumPattern =
    "4C 8B DC 55 57 49 8D AB 58 FB FF FF 48 81 EC 98 05 00 00 41 0F 29 73 "
    "D8 45 0F 29 4B C8";
constexpr std::uintptr_t kViewMatricesOffset = 0x3E0;
constexpr std::uintptr_t kViewLocationOffset = 0xFF0;
constexpr std::uintptr_t kViewRotationOffset = 0x1008;
constexpr std::uintptr_t kViewStateOffset = 0x2330;
constexpr std::uintptr_t kViewStatePrevFrameOffset = 0x740;
constexpr std::uintptr_t kViewStatePrevFrameMatricesOffset = 0x750;
constexpr std::uintptr_t kViewMatricesSize = 0xC10;
constexpr std::uintptr_t kViewInfoRectOffset = 0x2320;
constexpr std::uintptr_t kViewFovOffset = 0x2044;
constexpr std::uintptr_t kViewDesiredFovOffset = 0x2048;
// FSceneRenderer::AllViews is a pointer array; PrepareViewStateForVisibility
// iterates it. This is the right list for the pre-visibility write because it
// includes every view the renderer will actually cull for.
constexpr std::uintptr_t kSceneRendererAllViewsOffset = 0x3C8;
constexpr std::uintptr_t kSceneRendererAllViewCountOffset = 0x3D0;
constexpr std::int32_t kRouteBMaxEarlyViews = 64;
constexpr std::int64_t kVehicleRenderTransformMaxAgeNs = 50'000'000;
// If a proxy/build pose read tears for one frame, repeat the last pose this
// briefly rather than handing one frame to the game camera. Three frames at
// 60 fps; a persistent failure still fails closed.
constexpr std::int64_t kRouteBHeldPoseNs = 50'000'000;
// While the car is moving, the camera can bridge a missed render transform for
// this long. Beyond it the proxy is treated as gone (pause, stream, on foot)
// and the rig returns to the flat body frame instead of keeping a stale slope.
constexpr std::int64_t kRenderPitchHoldNs = 250'000'000;
// `OnTransformChanged` only fires when the proxy transform changes. A parked
// car can sit on one slope indefinitely without a new publish, so below this
// speed the last pitch stays valid; the hold expiry above only applies while
// the car is moving, where an old snapshot really means a render hitch.
constexpr double kRenderPitchStationarySpeedCmps = 200.0;
// Route B's local mount. The car proxy translation is the chassis pivot at the
// ground; the +0.70 m centre height matches the existing vehicle anchor. The
// look point is deliberately in front of the nose so the chase framing does
// not depend on the game's camera at all.
constexpr double kRouteBCenterHeightM = 0.70;
constexpr double kRouteBRestXM = -5.20;
constexpr double kRouteBRestZM = 1.55;
constexpr double kRouteBLookAheadM = 8.00;
constexpr double kRouteBLookHeightM = 0.80;
constexpr std::int32_t kRouteBMinViewWidth = 320;
constexpr std::int32_t kRouteBMinViewHeight = 180;
// The proxy is written by the render thread while the detour reads it on its
// own thread. A torn or one-frame-old copy must still describe the same car,
// so accept only a translation within this radius of ComponentToWorld. At
// 300 km/h one frame is under 150 cm, so two metres leaves room for the writer
// and still rejects a freed/reused proxy.
constexpr double kVehicleProxyMaxOffsetCm = 2000.0;
// The phase sample only means something while the car is moving: at a
// standstill both clocks read the same point and the mean would be diluted.
// Ten metres per second keeps city crawling out of the average.
constexpr double kRenderXMinimumSpeedCmps = 1000.0;
// The same chain read on the Game tick has to reproduce the host snapshot. A
// centimetre covers double-rounding; anything larger is a wrong offset.
constexpr double kRenderXChainToleranceCm = 1.0;
constexpr double kRenderXResolveRetrySeconds = 2.0;
constexpr std::uint8_t kRenderXWait = 0;
constexpr std::uint8_t kRenderXReady = 1;
constexpr std::uint8_t kRenderXFailed = 2;

// Version insurance for the X anchor (PLAN-render-thread-hook.md task group 1).
// The offset table above was verified against these Anomaly NTE profiles;
// `anomaly-cli --pid <pid> ue` reports the same value as `profileHash`. The
// profile is hashed as its raw JSON text, so any edit - even whitespace or line
// endings - moves this value and disables the X path instead of reading a
// layout that moved. Both known 2.7.0 spellings are accepted: the installed
// CRLF profile and the repository's LF profile. On a mismatch the camera keeps
// working through the borrow estimate, and the panel says why.
constexpr std::string_view kAcceptedProfileHashes[] = {
    // Anomaly 2.7.0 installed runtime (CRLF JSON).
    "c6d97fd2e506e75a7da060096a1614b80ff1a3a6a3e1eaf185e25eb5b55c1f52",
    // Anomaly 2.7.0 repository profile (LF JSON).
    "c37a7b3d1802c74927797675c1723da01afaf637028315e5e265599d5119db2a",
};
constexpr std::uint8_t kAnchorInsuranceWaiting = 0;
constexpr std::uint8_t kAnchorInsuranceReady = 1;
constexpr std::uint8_t kAnchorInsuranceServiceMissing = 2;
constexpr std::uint8_t kAnchorInsuranceHashMismatch = 3;
constexpr std::uint8_t kAnchorInsuranceReadFailed = 4;
// What anchors the car position. `X` reads `root+280` in the detour; `Borrow`
// is the old game-camera estimate; `Model` is the untouched tick sample.
constexpr std::uint32_t kAnchorSourceX = 0;
constexpr std::uint32_t kAnchorSourceBorrow = 1;
constexpr std::uint32_t kAnchorSourceModel = 2;
// The renderer's car transform: read the player SceneComponent's AttachParent
// (the vehicle SkeletalMesh), prefer its scene proxy's LocalToWorld
// translation, and fall back to the component transform. Falls back to X when
// the character is not in a vehicle.
constexpr std::uint32_t kAnchorSourceVehicle = 3;
constexpr std::uint32_t kAnchorSourceDefault = kAnchorSourceVehicle;

// What the panel reports. These are also what the detour publishes as the
// anchor it actually used this frame.
constexpr std::uint8_t kAnchorStateX = 0;
constexpr std::uint8_t kAnchorStateBorrow = 1;
constexpr std::uint8_t kAnchorStateDisabled = 2;
constexpr std::uint8_t kAnchorStateHashMismatch = 3;
constexpr std::uint8_t kAnchorStateResolveFailed = 4;
constexpr std::uint8_t kAnchorStateVehicle = 5;
constexpr std::uint8_t kAnchorGateOk = 0;
constexpr std::uint8_t kAnchorGateHash = 1;
constexpr std::uint8_t kAnchorGateResolve = 2;
constexpr std::uint8_t kAnchorGateService = 3;
constexpr std::uint8_t kAnchorGateOffset = 4;
constexpr std::uint8_t kAnchorGateOff = 5;
constexpr std::uint8_t kAnchorGateRead = 6;
// A wrong offset is caught by the Game-tick cross-check. Require this many
// consecutive mismatches so one hitch cannot disable the path for a session.
constexpr std::uint32_t kAnchorChainMismatchLimit = 3;
// The detour reads X at most once per 8 ms; the presenter runs on the same
// call or the next pass. Fifty ms covers a frame hitch without accepting a
// stale position from before a pause.
constexpr std::int64_t kAnchorSnapshotMaxAgeNs = 50'000'000;

constexpr std::string_view kSettingsSchema = R"json(
{
  "type":"object",
  "additionalProperties":false,
  "required":["enabled","preset","cameraScale","originOffsetCm","aspectRatio","fovEnabled"],
  "properties":{
    "enabled":{"type":"boolean"},
    "preset":{"type":"integer","minimum":0,"maximum":1},
    "cameraScale":{"type":"number","minimum":0.6,"maximum":1.6},
    "positionSpring":{"type":"number","minimum":0.5,"maximum":30.0},
    "yawSpring":{"type":"number","minimum":0.5,"maximum":30.0},
    "driftYawSpring":{"type":"number","minimum":0.5,"maximum":30.0},
    "lookYawSpring":{"type":"number","minimum":0.5,"maximum":30.0},
    "yawReturnSpring":{"type":"number","minimum":0.5,"maximum":30.0},
    "pitchSpring":{"type":"number","minimum":1.0,"maximum":120.0},
    "rollSpring":{"type":"number","minimum":1.0,"maximum":200.0},
    "originOffsetCm":{"type":"number","minimum":-150.0,"maximum":150.0},
    "aspectRatio":{"type":"number","minimum":1.0,"maximum":3.5},
    "fovEnabled":{"type":"boolean"},
    "speedDistanceScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "rollScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "pitchScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "shakeScale":{"type":"number","minimum":0.0,"maximum":2.0},
    "rollOnlyInDrift":{"type":"boolean"},
    "anchorSource":{"type":"integer","minimum":0,"maximum":3},
    "borrowStrength":{"type":"number","minimum":0.0,"maximum":1.0},
    "diagnosticFlatPose":{"type":"boolean"},
    "diagnosticZeroLag":{"type":"boolean"},
    "diagnosticSkipPosition":{"type":"boolean"},
    "diagnosticSkipRotation":{"type":"boolean"},
    "slopeFollow":{"type":"boolean"},
    "slopeStrength":{"type":"number","minimum":0.0,"maximum":2.0},
    "slopeMaxGradientDegrees":{"type":"number","minimum":0.0,"maximum":30.0},
    "slopeUpLookAt":{"type":"number","minimum":0.0,"maximum":2.0},
    "slopeUpRest":{"type":"number","minimum":0.0,"maximum":4.0},
    "slopeDownLookAt":{"type":"number","minimum":0.0,"maximum":2.0},
    "slopeDownRest":{"type":"number","minimum":0.0,"maximum":4.0},
    "burnoutEnabled":{"type":"boolean"},
    "burnoutEntryDelay":{"type":"number","minimum":0.0,"maximum":2.0},
    "burnoutEntryTime":{"type":"number","minimum":0.1,"maximum":4.0},
    "burnoutExitDelay":{"type":"number","minimum":0.0,"maximum":2.0},
    "burnoutExitTime":{"type":"number","minimum":0.1,"maximum":5.0},
    "burnoutLaunchExitTime":{"type":"number","minimum":0.1,"maximum":5.0},
    "burnoutFov":{"type":"number","minimum":20.0,"maximum":90.0},
    "burnoutShake":{"type":"number","minimum":0.0,"maximum":2.0},
    "burnoutSideMode":{"type":"integer","minimum":0,"maximum":3},
    "burnoutEntrySpeed":{"type":"number","minimum":0.0,"maximum":10.0},
    "burnoutHoldSpeed":{"type":"number","minimum":0.0,"maximum":20.0},
    "wheelSteerEnabled":{"type":"boolean"},
    "wheelSteerEntrySpeedKmh":{"type":"number","minimum":20.0,"maximum":160.0},
    "wheelSteerMaxDegrees":{"type":"number","minimum":20.0,"maximum":50.0},
    "wheelSteerGain":{"type":"number","minimum":0.5,"maximum":1.5},
    "wheelSteerFullSlipDegrees":{"type":"number","minimum":10.0,"maximum":60.0},
    "wheelSteerRateDegreesPerSecond":{"type":"number","minimum":60.0,"maximum":1200.0}
  }
}
)json";

// The 2.6 vehicle getters may be missing on a cold world; retry at this rate
// instead of every tick. Shared by the presence poll and the wheel-steer take.
constexpr double kVehicleGetterRetrySeconds = 2.0;

}  // namespace nfs2015_chase_cam::detail
