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

constexpr double kDegreesToRadians =
    3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees =
    180.0 / 3.14159265358979323846;
constexpr double kDefaultCameraScale = 1.0;
constexpr double kMinimumCameraScale = 0.6;
constexpr double kMaximumCameraScale = 1.6;
// User-facing metric rig geometry. The camera applies these as per-axis
// factors around the fixed ground origin; the old scale/origin pair below
// remains only for loading old documents and for the derived fields written
// for older builds.
constexpr double kDefaultCameraDistanceM = 5.10;
constexpr double kMinimumCameraDistanceM = 3.0;
constexpr double kMaximumCameraDistanceM = 8.2;
constexpr double kDefaultCameraHeightM = 1.49;
constexpr double kMinimumCameraHeightM = 0.45;
constexpr double kMaximumCameraHeightM = 4.6;
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
// The NFS preset's local rest height is ground-relative; the user-facing
// height lives in cameraHeightM. These two values are retained for migration
// and for the derived fields written so older builds can load a new document.
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
// Parked orbit defaults. After the car has been parked and untouched for the
// entry delay, the camera circles it. The feature is on out of the box, like
// the slope follow and the drift front wheels: the group's Default runs it on
// these built-in numbers, Custom runs it on the user's numbers, and Opposite
// is the only state that turns it off.
//
// The framing is the NFS 2015 reference composition, not a hand-tuned guess:
// 5.05 m radius, 1.0 m eye height, 0.75 m look-at height and 28 degrees
// vertical FOV. The eye sits 0.25 m above the pivot, which is 2.84 degrees of
// elevation on the 5.05 m radius; 43 authored degrees minus the orbit
// controller's 15 gives the lens. The older 4.0 / 0.724 / 0.671 / 45 values are
// still reachable by typing them into the Custom sliders; a document that still
// carries them untouched is migrated by LoadSettings().
constexpr bool kDefaultOrbitEnabled = true;
constexpr double kDefaultOrbitEntryDelay = 8.0;
constexpr double kMinimumOrbitEntryDelay = 0.0;
constexpr double kMaximumOrbitEntryDelay = 60.0;
constexpr double kDefaultOrbitPeriod = 60.0;
constexpr double kMinimumOrbitPeriod = 10.0;
constexpr double kMaximumOrbitPeriod = 600.0;
constexpr double kDefaultOrbitRadius = 5.05;
constexpr double kMinimumOrbitRadius = 2.0;
constexpr double kMaximumOrbitRadius = 12.0;
// Camera height above the car origin, in metres along the body up axis. With
// the 5.05 m radius and the 0.75 m look-at height below, the eye sits 0.25 m
// above the pivot: asin(0.25 / 5.05) = 2.84 degrees of elevation.
constexpr double kDefaultOrbitHeight = 1.0;
constexpr double kMinimumOrbitHeight = 0.3;
constexpr double kMaximumOrbitHeight = 3.0;
// Where the camera looks, measured above the car origin along the body up axis.
// Targeting the origin frames the tyres instead of the car, so it stays above
// the wheel centre at 0.75 m. The eye height above and this one are independent
// sliders; their difference sets the elevation angle and the 3D slant radius
// stays at the orbit radius.
constexpr double kDefaultOrbitLookAtHeight = 0.75;
constexpr double kMinimumOrbitLookAtHeight = 0.3;
constexpr double kMaximumOrbitLookAtHeight = 3.0;
constexpr double kDefaultOrbitWeave = 1.0;
constexpr double kMinimumOrbitWeave = 0.0;
constexpr double kMaximumOrbitWeave = 3.0;
// Look-at low-frequency sway (Instance192/Instance188 -> Instance279): the
// reaction that moves only the aim point, not the camera. 1.0 keeps the asset
// keys as they are, 0 disables the layer, and 2 doubles the swing while the
// amplitude is still B-grade evidence.
constexpr double kDefaultOrbitTargetSway = 1.0;
constexpr double kMinimumOrbitTargetSway = 0.0;
constexpr double kMaximumOrbitTargetSway = 2.0;
// Vertical FOV. 28 degrees is the runtime value: the chase preset authors 43
// and the orbit controller subtracts 15. The horizontal equivalent is computed
// from the current value in the panel instead of being baked into a label.
constexpr double kDefaultOrbitFov = 28.0;
constexpr double kMinimumOrbitFov = 20.0;
constexpr double kMaximumOrbitFov = 70.0;
// Look-at noise (Instance18) and the roll noise (Instance16) share this band,
// so 2 Hz is both the asset frequency and the default for the slider.
constexpr double kDefaultOrbitShakeStrength = 1.0;
constexpr double kMinimumOrbitShakeStrength = 0.0;
constexpr double kMaximumOrbitShakeStrength = 2.0;
constexpr double kDefaultOrbitShakeFrequency = 2.0;
// Hide the game HUD during the parked orbit. The runtime resolves the
// W_OverallUILayout API; this switch is exposed in the settings panel later.
constexpr bool kDefaultOrbitHideUi = true;
constexpr double kMinimumOrbitShakeFrequency = 2.0;
constexpr double kMaximumOrbitShakeFrequency = 8.0;
// The orbit numbers the 2.0.6 build shipped. Its writer stored the built-in
// numbers in the custom block even while the group was on Default, so a value
// that still equals one of these was never tuned. A document without
// orbitTargetSway predates the current set, and LoadSettings() moves such a
// value to today's default; a value the user changed, and every document that
// carries orbitTargetSway, is left alone.
constexpr double kLegacyOrbitRadius = 4.0;
constexpr double kLegacyOrbitHeight = 0.724;
constexpr double kLegacyOrbitLookAtHeight = 0.671;
constexpr double kLegacyOrbitFov = 45.0;
constexpr double kLegacyOrbitShakeFrequency = 5.0;
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
constexpr double kDefaultWheelSteerRateDegreesPerSecond = 150.0;
constexpr double kMinimumWheelSteerRateDegreesPerSecond = 60.0;
constexpr double kMaximumWheelSteerRateDegreesPerSecond = 1200.0;
// When the drift-side key is held this long without quick_turn owning the
// wheel, force_turn takes over the visual target until the key is released or
// the underlying state returns to grip. Unlike quick_turn it is not one-shot.
constexpr double kDefaultWheelSteerForceTurnDelaySeconds = 0.4;
constexpr double kMinimumWheelSteerForceTurnDelaySeconds = 0.1;
constexpr double kMaximumWheelSteerForceTurnDelaySeconds = 2.0;
// Where along the front axle the countersteer travel angle is sampled, as a
// percentage of half the front track. Positive is the inside of the drift and
// negative the outside; the sign mirrors with the drift direction, so the same
// value produces the same visual result for left and right drifts. Zero keeps
// the historical front-axle-centre behaviour.
constexpr double kDefaultWheelSteerSamplePercent = 0.0;
constexpr double kMinimumWheelSteerSamplePercent = -100.0;
constexpr double kMaximumWheelSteerSamplePercent = 100.0;
// 2.0.2 shipped these three values together. 2.0.3 changes the scale (front
// axle target, 40 degree full slip, 32 degree cap, 150 deg/s). Only an
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
constexpr double kWheelSteerDirectionChangeSlipDegrees = 10.0;
constexpr double kWheelSteerExitSlipDegrees = 3.0;
constexpr double kWheelSteerHoldSeconds = 0.5;
constexpr double kWheelSteerExitHoldSeconds = 0.25;
constexpr double kWheelSteerBlendInSeconds = 0.20;
constexpr double kWheelSteerBlendOutSeconds = 0.30;
constexpr double kWheelSteerStockMaxDegrees = 35.0;
constexpr double kWheelSteerCounterAuthorityDegrees = 5.0;
constexpr double kWheelSteerRestoreToleranceDegrees = 1.5;
constexpr double kWheelSteerEntryStockStartSlipDegrees = 8.0;
constexpr double kWheelSteerEntryStockFullSlipDegrees = 22.0;
constexpr double kWheelSteerReentryCooldownSeconds = 0.5;
constexpr double kWheelSteerQuickTurnMemorySeconds = 1.5;
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
// fresh means the Game-tick read saw a valid vehicle parent. `recent` covers
// one missed poll while entering/exiting or while the renderer is streaming.
// Speed is used only when neither signal can answer.
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
// The game-thread path reads the live VehicleMesh ComponentToWorld at the
// scene-view phase, so there is no source selection or fallback estimator.
constexpr double kMaximumPresenterStepSeconds = 0.1;
// The camera's yaw is the nose reference until it strays this far from the
// direction of travel: past it the camera is describing something other than
// the car (a wall, a spin, a scripted shot), and the travel direction is the
// safer answer.
constexpr double kMaximumCameraYawDifference = 1.4;  // ~80 degrees
constexpr double kHookRetrySeconds = 2.0;

// 2.1.0 game-thread view path. `ULocalPlayer::GetViewPoint` is the engine's
// own view-point producer; `CalcSceneViewInitOptions` brackets the first view
// construction (its GetProjectionData call reaches GetViewPoint once). Both
// patterns were taken from the running 3.1.0 build and re-verified against the
// 3.1.2 image: still unique in .text, and the two entry points only moved by
// the same +0x230 the surrounding linker shuffle explains. The field offsets
// below were re-read from the 3.1.2 disassembly (see RELEASE-NOTES-2.1.1.md).
// They stay behind the profile-hash gate; a mismatch leaves the native camera
// alone.
constexpr std::string_view kLocalPlayerGetViewPointPattern =
    "48 89 5C 24 08 48 89 6C 24 10 48 89 74 24 18 48 "
    "89 7C 24 20 41 56 48 83 EC 60 48 8B FA 48 8B E9 "
    "E8 8B 85 FF FF 4C 8D 77 30 4C 8B C7 4C 8D 4F 18";
constexpr std::string_view kCalcSceneViewInitOptionsPattern =
    "40 56 57 41 54 41 55 41 56 48 83 EC 30 48 83 79 "
    "30 00 4D 8B E9 4D 8B F0 48 8B FA 48 8B F1 0F 84 "
    "29 03 00 00 0F 57 C0 66 0F 2F 81 90 00 00 00 0F";
// The prefix of FMinimalViewInfo the new path reads and writes. The full
// engine object is 0xC70 bytes; these four fields were confirmed by the
// game's own UStruct reflection and a runtime FOV write in task group 1.
constexpr std::uint32_t kMinimalViewInfoLocationOffset = 0x00;
constexpr std::uint32_t kMinimalViewInfoRotationOffset = 0x18;
constexpr std::uint32_t kMinimalViewInfoFovOffset = 0x30;
constexpr std::uint32_t kMinimalViewInfoDesiredFovOffset = 0x34;
// `FSceneViewInitOptions::bInCameraCut` (SceneView.h). The game's own
// CalcSceneViewInitOptions writes it from the camera manager's cut bit after
// GetProjectionData returns; the detour only ORs our pending cut into that
// value, so the game's own cuts are preserved.
constexpr std::uintptr_t kSceneViewInitOptionsCameraCutOffset = 0x2C8;

// Missing scene-car transform: extrapolate for at most two consecutive scene
// advances. A longer gap is a hitch or a stream/pause, not missing data; it
// becomes a cut instead of a camera that keeps flying on old velocity.
constexpr std::uint32_t kSceneExtrapolationMaxFrames = 2;
constexpr std::int64_t kSceneExtrapolationMaxAgeNs = 150'000'000;

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
// guards the camera hooks guards this one.
constexpr std::uint32_t kPcViewModeTagOffset = 0x1E80;
constexpr std::uint32_t kPcViewModeArrayOffset = 0x1E88;
constexpr std::uint32_t kPcViewModeCountOffset = 0x1E90;
constexpr std::uint32_t kPcViewModeCapacityOffset = 0x1E94;
constexpr std::uint32_t kPcLimitManualToggleOffset = 0x1E98;
constexpr std::uint32_t kPcToggleCooldownOffset = 0x1E9C;
constexpr std::uint32_t kPcToggleGraceOffset = 0x1EA0;
constexpr std::uint32_t kPcToggleCooldownClockOffset = 0x1EA8;
constexpr std::uint32_t kPcToggleGraceUsedOffset = 0x1EB0;
constexpr std::uint32_t kPcViewModeStateSpan = 0x38;
constexpr std::uint32_t kViewModeEntryStride = 0x10;
constexpr std::size_t kMaximumLoggedViewModes = 8;
constexpr std::uint64_t kToggleLogAllCalls = 400;
constexpr std::uint64_t kToggleLogStrideAfter = 10;
// Photo ("抓拍", F8) and any other game-owned cine camera. While the player
// controller shows its own camera, the game attaches a `CineCameraComponent`
// to the controller and stores it here; the member is null the rest of the
// time. The game does NOT change the view-mode tag for this, so the tag guard
// cannot see it.
constexpr std::uint32_t kPcCineCameraOffset = 0x1760;

// `ue5.GWorld` entry: `mov rbx,[rip+disp32]` is the first instruction, 7 bytes
// long, and the displacement sits at +3. Resolving the pattern gives the
// instruction, so the global slot is
// `instruction + instructionSize + displacement`.
constexpr std::string_view kGWorldPattern =
    "48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01";
constexpr std::uint32_t kGWorldRipOffset = 3;
constexpr std::uint32_t kGWorldInstructionSize = 7;
// Offsets from the host's own `ReadCurrentPlayerLocation`, i.e. the profile's
// `nte-current.json` layout. Every one of them is a hardcoded guess about the
// running build; the Game-tick cross-check catches a stale table.
constexpr std::uint32_t kWorldGameInstanceOffset = 560;
constexpr std::uint32_t kGameInstanceLocalPlayersOffset = 56;
constexpr std::uint32_t kLocalPlayerControllerOffset = 48;
constexpr std::uint32_t kControllerPawnOffset = 776;
constexpr std::uint32_t kActorRootComponentOffset = 456;
constexpr std::uint32_t kSceneComponentLocationOffset = 280;
// The vehicle transform follows the player SceneComponent's AttachParent.
// While driving, the character capsule is attached to the vehicle's
// SkeletalMesh component (object name `VehicleMesh`). The parent pointer and
// the parent transform are read in the same pass, so their relative offset is
// not polluted by the car moving between two reads.
constexpr std::uint32_t kSceneComponentAttachParentOffset = 0xD8;
constexpr std::uint32_t kSceneComponentRelativeRotationOffset = 0x168;
constexpr std::uint32_t kSceneComponentTransformOffset = 0x1F0;
constexpr std::uint32_t kSceneComponentTransformTranslationOffset = 0x210;
// NTE player vehicles expose their visual wheel components as a TArray of
// StaticMeshComponents. The animation controller normally owns all four; the
// drift override parks it and writes the front steer plus the matching roll on
// every wheel. `FrontWheels` / `RearWheels` are declared on
// `HTWheeledVehicleDrivable` (not on a blueprint), so these two offsets cover
// every wheeled vehicle. 3.1.2 inserted one 8-byte field before them (3.1.0:
// 0x12D8/0x12E8), verified from the live class's `PropertyLink` reflection:
// FrontWheels@0x12E0 num=2 {Wheel_FR, Wheel_FL},
// RearWheels @0x12F0 num=2 {Wheel_RL, Wheel_RR}.
constexpr std::uint32_t kVehicleMeshOffset = 0x348;
constexpr std::uint32_t kVehicleFrontWheelsOffset = 0x12E0;
constexpr std::uint32_t kVehicleRearWheelsOffset = 0x12F0;
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
// a tuning constant rather than a physics read.
constexpr double kWheelVisualRollRadiusMeters = 0.35;
// A parent transform farther than this from the pawn root is a stale or
// recycled pointer, not the car.
constexpr double kVehicleAnchorMaxOffsetCm = 1000.0;
// The vehicle mesh's ComponentToWorld translation is the chassis pivot at the
// ground; the model's fixed ground origin sits this much below the body centre.
constexpr double kVehicleAnchorCenterHeightCm = 70.0;
// The same chain read on the Game tick has to reproduce the host snapshot. A
// centimetre covers double-rounding; anything larger is a wrong offset.
constexpr double kVehicleChainToleranceCm = 1.0;
constexpr std::uint32_t kVehicleChainMismatchLimit = 3;
constexpr double kVehicleRuntimeResolveRetrySeconds = 2.0;
constexpr std::uint8_t kVehicleRuntimeWait = 0;
constexpr std::uint8_t kVehicleRuntimeReady = 1;
constexpr std::uint8_t kVehicleRuntimeFailed = 2;
// Body yaw is sampled once per Game tick. A snapshot older than this is not a
// useful heading for the next tick.
constexpr std::int64_t kVehicleBodyYawMaxAgeNs = 50'000'000;

// Version insurance for the hardcoded offsets. The offset table above was
// verified against these Anomaly NTE profiles; `anomaly-cli --pid <pid> ue`
// reports the same value as `profileHash`. The profile is hashed as its raw
// JSON text, so any edit - even whitespace or line endings - moves this value
// and disables the camera hooks instead of reading a layout that moved.
// Known 2.7.0, 3.0.0, 3.1.0/3.1.1 and 3.1.2 spellings are accepted: for each
// release, the installed CRLF profile and the repository's LF profile. The
// 3.0.0 profile is the same HTGame.exe as 2.7.0, and 3.1.0 only adds
// combat/account entries on top of the same camera layout and GWorld symbol.
// 3.1.2 ships a new HTGame.exe, but its profile only adds `cooked.vehicle.*`
// schema keys: every existing layout value is byte-identical, and the camera
// patterns, FMinimalViewInfo fields, `bInCameraCut`, the PC view-mode block
// and the toggle vtable slot were re-verified against the live 3.1.2 image.
// On a mismatch no camera hook is installed, the game camera stays native, and
// the panel says `hash mismatch`.
constexpr std::string_view kAcceptedProfileHashes[] = {
    // Anomaly 2.7.0 installed runtime (CRLF JSON).
    "c6d97fd2e506e75a7da060096a1614b80ff1a3a6a3e1eaf185e25eb5b55c1f52",
    // Anomaly 2.7.0 repository profile (LF JSON).
    "c37a7b3d1802c74927797675c1723da01afaf637028315e5e265599d5119db2a",
    // Anomaly 3.0.0 installed runtime (CRLF JSON). Same layout as 2.7.0:
    // same HTGame.exe, same ue5.GWorld RVA, all plugin patterns still unique.
    "93eefb64683fe66d202cd405079e15540fe1ee0d6f5b40b7eacf3889c6eff1ff",
    // Anomaly 3.0.0 repository profile (LF JSON).
    "ddba763c22f45a2354b48cc123eef217717637088046ea5353afa0380bd22a20",
    // Anomaly 3.1.0 / 3.1.1 installed runtime (CRLF JSON). 3.1.1 ships the
    // same NTE profile; every camera layout key and ue5.GWorld are unchanged.
    "08a3feb998c74f99375cebd6bd8866e409663680b9fe3da229274f4c9c2e249a",
    // Anomaly 3.1.0 repository profile (LF JSON, tag v3.1.0).
    "9acaf134e5279ade33f1b3c555a114b60871511717d9131850fe4a7bca2d5a82",
    // Anomaly 3.1.2 installed runtime (CRLF JSON). New HTGame.exe (RVA shift),
    // unchanged camera layout; see the block comment above.
    "7341beb0ffae6e43c017d20b51a357d30249d4516dd487d5ac91c0cd2fafdf7d",
    // Anomaly 3.1.2 repository profile (LF JSON).
    "228b0ac3642a682688442706e517af4963c3700a475386ee7291acde25336501",
};
constexpr std::uint8_t kProfileGateWaiting = 0;
constexpr std::uint8_t kProfileGateReady = 1;
constexpr std::uint8_t kProfileGateServiceMissing = 2;
constexpr std::uint8_t kProfileGateHashMismatch = 3;
constexpr std::uint8_t kProfileGateReadFailed = 4;
constexpr std::uint8_t kProfileGateOk = 0;
constexpr std::uint8_t kProfileGateHash = 1;
constexpr std::uint8_t kProfileGateResolve = 2;
constexpr std::uint8_t kProfileGateService = 3;
constexpr std::uint8_t kProfileGateOffset = 4;
constexpr std::uint8_t kProfileGateOff = 5;
constexpr std::uint8_t kProfileGateRead = 6;

constexpr std::string_view kSettingsSchema = R"json(
{
  "type":"object",
  "additionalProperties":false,
  "required":["enabled","preset","cameraScale","originOffsetCm","aspectRatio","fovEnabled"],
  "properties":{
    "enabled":{"type":"boolean"},
    "preset":{"type":"integer","minimum":0,"maximum":1},
    "cameraScale":{"type":"number","minimum":0.6,"maximum":1.6},
    "cameraDistanceM":{"type":"number","minimum":3.0,"maximum":8.2},
    "cameraHeightM":{"type":"number","minimum":0.45,"maximum":4.6},
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
    "diagnosticFlatPose":{"type":"boolean"},
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
    "orbitHideUi":{"type":"boolean"},
    "orbitEntryDelay":{"type":"number","minimum":0.0,"maximum":60.0},
    "orbitPeriod":{"type":"number","minimum":10.0,"maximum":600.0},
    "orbitRadius":{"type":"number","minimum":2.0,"maximum":12.0},
    "orbitHeight":{"type":"number","minimum":0.3,"maximum":3.0},
    "orbitLookAtHeight":{"type":"number","minimum":0.3,"maximum":3.0},
    "orbitWeave":{"type":"number","minimum":0.0,"maximum":3.0},
    "orbitFov":{"type":"number","minimum":20.0,"maximum":70.0},
    "orbitShakeStrength":{"type":"number","minimum":0.0,"maximum":2.0},
    "orbitShakeFrequency":{"type":"number","minimum":2.0,"maximum":8.0},
    "orbitTargetSway":{"type":"number","minimum":0.0,"maximum":2.0},
    "wheelSteerEnabled":{"type":"boolean"},
    "wheelSteerEntrySpeedKmh":{"type":"number","minimum":20.0,"maximum":160.0},
    "wheelSteerMaxDegrees":{"type":"number","minimum":20.0,"maximum":50.0},
    "wheelSteerGain":{"type":"number","minimum":0.5,"maximum":1.5},
    "wheelSteerFullSlipDegrees":{"type":"number","minimum":10.0,"maximum":60.0},
    "wheelSteerRateDegreesPerSecond":{"type":"number","minimum":60.0,"maximum":1200.0},
    "wheelSteerForceTurnDelay":{"type":"number","minimum":0.1,"maximum":2.0},
    "wheelSteerSamplePercent":{"type":"number","minimum":-100.0,"maximum":100.0},
    "nfsLensMode":{"type":"integer","minimum":0,"maximum":2},
    "rigMode":{"type":"integer","minimum":0,"maximum":2},
    "springsMode":{"type":"integer","minimum":0,"maximum":2},
    "dynamicsMode":{"type":"integer","minimum":0,"maximum":2},
    "slopeMode":{"type":"integer","minimum":0,"maximum":2},
    "wheelSteerMode":{"type":"integer","minimum":0,"maximum":2},
    "burnoutMode":{"type":"integer","minimum":0,"maximum":2},
    "orbitMode":{"type":"integer","minimum":0,"maximum":2}
  }
}
)json";

// The 2.6 vehicle getters may be missing on a cold world; retry at this rate
// instead of every tick. Shared by the presence poll and the wheel-steer take.
constexpr double kVehicleGetterRetrySeconds = 2.0;

}  // namespace nfs2015_chase_cam::detail
