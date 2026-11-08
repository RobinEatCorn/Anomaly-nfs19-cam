// NFS 2015 Chase Cam — third-party Anomaly plugin (ABI v1).
//
// This package is built against the installed Anomaly SDK only: it includes no
// host-internal headers and carries no JSON library of its own. Settings are
// persisted through `anomaly.config` and decoded through `anomaly.json`.
//
// Feeds the clean-room NFS 2015 chase-camera model in `chase_cam.hpp` from
// Anomaly's NTE snapshots, then replaces the view point produced by the game's
// camera manager. The game's own third-person camera is used as the car-yaw
// sensor; the player snapshot supplies the car position. Anomaly 2.7.0 removed
// `anomaly.nte.vehicle`, so vehicle presence is recovered by calling the same
// reflected getters that service used, through `ue5.objects` +
// `ue5.process-event`; the player capsule's AttachParent remains the anchor and
// the fallback. Speed comes from differentiating the player position and is
// signed against the resolved car yaw, with hysteresis for reverse. No GObjects
// scan.
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
#include <iterator>
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
    "robineatcorn.nfs2015-chase-cam";
constexpr std::string_view kSettingsSchemaId = "nfs2015-chase-cam-settings-v1";
constexpr std::string_view kSettingsWindowId = "nfs2015-chase-cam-settings-v1";
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
constexpr std::uint32_t kSceneComponentTransformOffset = 0x1F0;
constexpr std::uint32_t kSceneComponentTransformTranslationOffset = 0x210;
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
constexpr std::uint32_t kAnchorSourceDefault = kAnchorSourceX;
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
    "slopeDownRest":{"type":"number","minimum":0.0,"maximum":4.0}
  }
}
)json";

using CameraViewPointFn = void(ANOMALY_CALL *)(void *, double *, double *);
// The virtual transform notification. The second argument is the scene/command
// context SetTransform forwards; it is passed through unchanged.
using OnTransformChangedFn = void(ANOMALY_CALL *)(void *, void *);
// FViewInfo::SetupUniformBufferParameters. The first four arguments are
// pointers; the fifth is a small flag and the sixth an output pointer. It runs
// late in the renderer's view setup, once per view, and is the stable place to
// observe the view the renderer is actually about to use.
using SetupUniformBufferFn = void(ANOMALY_CALL *)(void *, void *, void *,
                                                  void *, std::uint64_t,
                                                  void *) noexcept;
// FViewMatrices::FViewMatrices(const FSceneViewInitOptions&). The constructor
// derives all view/projection matrices from the supplied rotation/projection
// and origin. Calling it is preferable to reimplementing its derived matrices.
using ViewMatricesCtorFn = void(ANOMALY_CALL *)(void *, const void *) noexcept;
// FSceneRenderer::PrepareViewStateForVisibility. Called once per renderer
// immediately before visibility tasks are launched; the second argument points
// at the scene textures config embedded in the renderer.
using PrepareViewStateFn = void(ANOMALY_CALL *)(void *, const void *) noexcept;
// FSceneView::UpdateViewMatrix() and the static FSceneView::SetupViewFrustum().
// The former rebuilds ViewMatrices from ViewLocation/ViewRotation; the latter
// derives both ViewFrustum and CullingFrustum afterwards.
using SceneViewUpdateMatrixFn = void(ANOMALY_CALL *)(void *) noexcept;
using SceneViewSetupFrustumFn = void(ANOMALY_CALL *)(void *) noexcept;

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

// Body pitch (nose up positive) from the vehicle proxy's forward/nose vector.
// The vector may be fully normalised (z = sin pitch) or horizontally
// normalised (z = tan pitch, the historical publish format), so atan2 is the
// correct inverse for both; asin(z) would only work for the former.
[[nodiscard]] bool BodyPitchFromForward(const double forward[3],
                                        double &pitch) noexcept {
  if (!std::isfinite(forward[0]) || !std::isfinite(forward[1]) ||
      !std::isfinite(forward[2])) {
    return false;
  }
  const double horizontal_squared =
      forward[0] * forward[0] + forward[1] * forward[1];
  if (!(horizontal_squared > 1e-8)) {
    return false;
  }
  pitch = std::atan2(forward[2], std::sqrt(horizontal_squared));
  return std::isfinite(pitch);
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

bool WindowReady(const AnomalyWindowServiceV1 *service) noexcept {
  return service != nullptr && service->register_window != nullptr &&
         service->release_window != nullptr && service->set_open != nullptr &&
         service->state != nullptr && service->begin != nullptr &&
         service->end != nullptr;
}

bool PlayerReady(const AnomalyNtePlayerServiceV1 *service) noexcept {
  return service != nullptr && service->snapshot != nullptr;
}

bool PlayerHandlesReady(const AnomalyNtePlayerServiceV1 *service) noexcept {
  return HasField<AnomalyNtePlayerServiceV1,
                  decltype(AnomalyNtePlayerServiceV1::get_object_handles)>(
             service,
             offsetof(AnomalyNtePlayerServiceV1, get_object_handles)) &&
         service->get_object_handles != nullptr;
}

bool Ue5ObjectsReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept {
  // find_exact is the appended V1 tail; older hosts only expose the registry
  // snapshots and cannot resolve a UFunction by path.
  return HasField<AnomalyUe5ObjectsServiceV1,
                  decltype(AnomalyUe5ObjectsServiceV1::find_exact)>(
             service, offsetof(AnomalyUe5ObjectsServiceV1, find_exact)) &&
         service->find_exact != nullptr;
}

bool Ue5ProcessEventInvokeReady(
    const AnomalyUe5ProcessEventServiceV1 *service) noexcept {
  return HasField<AnomalyUe5ProcessEventServiceV1,
                  decltype(AnomalyUe5ProcessEventServiceV1::invoke)>(
             service, offsetof(AnomalyUe5ProcessEventServiceV1, invoke)) &&
         service->invoke != nullptr;
}

// Exact-speed support needs the registry snapshot call to turn the raw vehicle
// pointer returned by BP_GetCurrentDriveVehicle into a ProcessEvent handle.
bool Ue5ObjectSnapshotReady(const AnomalyUe5ObjectsServiceV1 *service) noexcept {
  return Ue5ObjectsReady(service) && service->generation != nullptr &&
         service->snapshot_at != nullptr;
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

// What produced the speed shown in the panel. Anomaly 2.7.0 has no exact
// vehicle speed service, so the camera currently always reports `Derived`.
// `Exact` remains reserved for task group 3 if the public-ABI query returns.
enum SpeedSource : std::uint8_t {
  kSpeedSourceUnavailable = 0,
  kSpeedSourceDerived = 1,
  kSpeedSourceExact = 2,
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
  std::uint8_t speed_source{kSpeedSourceUnavailable};
  bool speed_sign_known{};
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
  bool slope_follow{true};
  double slope_strength{kDefaultSlopeStrength};
  bool roll_only_in_drift{};
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

  AnomalyGenerationHandleV1 settings_schema{};
  AnomalyGenerationHandleV1 toggle_hotkey{};
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
  std::atomic<double> origin_offset_cm{kDefaultOriginOffsetCm};
  std::atomic<double> aspect_ratio{kDefaultAspectRatio};
  std::atomic<double> speed_distance_scale{kDefaultSpeedDistanceScale};
  std::atomic<double> roll_scale{kDefaultRollScale};
  std::atomic<double> pitch_scale{kDefaultPitchScale};
  std::atomic<double> shake_scale{kDefaultShakeScale};
  std::atomic_bool slope_follow{true};
  std::atomic<double> slope_strength{kDefaultSlopeStrength};
  std::atomic<double> slope_max_gradient_degrees{
      kDefaultSlopeMaxGradientDegrees};
  std::atomic<double> slope_up_look_at{kDefaultSlopeUpLookAt};
  std::atomic<double> slope_up_rest{kDefaultSlopeUpRest};
  std::atomic<double> slope_down_look_at{kDefaultSlopeDownLookAt};
  std::atomic<double> slope_down_rest{kDefaultSlopeDownRest};
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
  // The origin offset that was baked into the published presenter pose.
  // The early render hook shifts the pose from the presenter's car anchor to
  // this frame's VehicleMesh; adding the difference between the current and
  // baked-in offset keeps the user trim effective without applying it twice.
  std::atomic<double> presenter_origin_offset_cm{kDefaultOriginOffsetCm};
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
std::atomic<OnTransformChangedFn> g_on_transform_changed_original{};
std::atomic<SetupUniformBufferFn> g_setup_uniform_buffer_original{};
std::atomic<ViewMatricesCtorFn> g_view_matrices_ctor{};
std::atomic<PrepareViewStateFn> g_prepare_view_state_original{};
std::atomic<SceneViewUpdateMatrixFn> g_scene_view_update_matrix{};
std::atomic<SceneViewSetupFrustumFn> g_scene_view_setup_frustum{};

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

AnomalyStatusV1 ResolveSignatureStatus(const Context &context,
                                       const std::string_view pattern,
                                       std::uintptr_t &address) noexcept {
  address = 0;
  if (!SignatureReady(context.signature)) {
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE);
  }
  return context.signature->resolve(
      context.signature->user, StringView("HTGame.exe"), StringView(".text"),
      StringView(pattern), &address);
}

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
                    "%zu accepted 2.7.0 profiles); camera hooks disabled until "
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

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept;

// Experimental: apply the fresh render proxy transform after the car's
// OnTransformChanged has updated LocalToWorld. This is deliberately a
// separate writer from ApplyRouteBEarly; it is used to test whether the
// transform notification is still before the visibility task data consumes
// the view.
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept;

// Task group 3 A2. The render thread calls this once for every primitive whose
// transform changed. Most calls are other objects: the watched-proxy pointer
// check is the only work before the original call. For the vehicle proxy we
// copy the matrix's translation (last row, +0x60 in the FMatrix) into the
// probe's seqlock before returning.
void ANOMALY_CALL RenderTransformDetour(void *self,
                                           void *scene) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->transform_hook_id.load(std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->transform_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  double translation[3]{};
  double forward[3]{};
  bool publish = false;
  bool orientation_valid = false;
  if (context != nullptr && self != nullptr &&
      context->render_x.version_gate.load(std::memory_order_acquire) &&
      reinterpret_cast<std::uintptr_t>(self) ==
          context->render_x.watched_proxy.load(std::memory_order_relaxed)) {
    const auto *value = reinterpret_cast<const double *>(
        reinterpret_cast<std::uintptr_t>(self) +
        kVehicleProxyTranslationOffset);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      translation[axis] = value[axis];
    }
    // The proxy matrix is row-major; row 0 is the local +X (nose) basis.
    // Normalise the full vector, not only its horizontal part: consumers use
    // forward.z as sin(pitch) for the body frame. The proxy may carry a
    // component scale.
    const auto *matrix = reinterpret_cast<const double *>(
        reinterpret_cast<std::uintptr_t>(self) + 0xE0);
    forward[0] = matrix[0];
    forward[1] = matrix[1];
    forward[2] = matrix[2];
    const double length_squared = forward[0] * forward[0] +
                                  forward[1] * forward[1] +
                                  forward[2] * forward[2];
    if (std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
        std::isfinite(forward[2]) && length_squared > 1e-8) {
      const double inverse_length = 1.0 / std::sqrt(length_squared);
      forward[0] *= inverse_length;
      forward[1] *= inverse_length;
      forward[2] *= inverse_length;
      orientation_valid = true;
    }
    publish = std::isfinite(translation[0]) &&
              std::isfinite(translation[1]) &&
              std::isfinite(translation[2]);
  }

  OnTransformChangedFn original =
      g_on_transform_changed_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, scene);
    }
  } catch (...) {
  }

  if (publish) {
    RenderXProbe &probe = context->render_x;
    probe.render_thread_id.store(static_cast<std::uint32_t>(GetCurrentThreadId()),
                                 std::memory_order_release);
    probe.transform_sequence.fetch_add(1, std::memory_order_acq_rel);
    probe.transform_proxy.store(reinterpret_cast<std::uintptr_t>(self),
                                std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.transform_cm[axis].store(translation[axis],
                                     std::memory_order_relaxed);
      probe.transform_forward[axis].store(forward[axis],
                                          std::memory_order_relaxed);
    }
    probe.transform_orientation_valid.store(orientation_valid,
                                             std::memory_order_relaxed);
    probe.transform_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    probe.transform_updates.fetch_add(1, std::memory_order_relaxed);
    probe.transform_valid.store(true, std::memory_order_release);
    probe.transform_sequence.fetch_add(1, std::memory_order_release);
    // TEMP read-only timing diagnostics: classify this transform update
    // against the last early camera write. It never changes the camera.
    if (probe.timing_frame.load(std::memory_order_relaxed) != 0) {
      const std::int64_t early_ns =
          probe.timing_early_ns.load(std::memory_order_relaxed);
      const std::int64_t event_ns = NowNanoseconds();
      if (probe.timing_late_seen.load(std::memory_order_acquire)) {
        probe.timing_transform_after.fetch_add(1,
                                               std::memory_order_relaxed);
      } else if (early_ns != 0 && event_ns >= early_ns) {
        probe.timing_transform_between.fetch_add(1,
                                                 std::memory_order_relaxed);
      } else {
        probe.timing_transform_before.fetch_add(1,
                                                std::memory_order_relaxed);
      }
    }
    // The VehicleMesh ComponentToWorld is already available at the Game
    // camera point and matches the final proxy. Do not write the view here:
    // this point is after PrepareViewStateForVisibility, so a late camera
    // change would desynchronise the history and visibility state.
  }

  // Release the callback lease even if the host is already tearing the hook
  // service down. `begin_callback` succeeded, so this lease is ours; skipping
  // `end_callback` because `HookReady` turned false during shutdown would leak
  // an in-flight callback and make the host quarantine the next generation.
  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

// Route B's initializer, matching FMinimalInitializer in the 2.6.0 build.
// Every offset here is pinned by static_assert: the render-thread constructor
// reads these fields directly, so a layout drift must fail at compile time
// rather than move the camera from a misread matrix.
struct alignas(16) RouteBViewInit final {
  double view_rotation[4][4]{};
  double projection[4][4]{};
  double view_origin[3]{};
  std::int32_t constrained_rect[4]{};
  std::int32_t stereo_pass{};
  bool use_faux_ortho{};
};
static_assert(offsetof(RouteBViewInit, projection) == 0x80);
static_assert(offsetof(RouteBViewInit, view_origin) == 0x100);
static_assert(offsetof(RouteBViewInit, constrained_rect) == 0x118);
static_assert(offsetof(RouteBViewInit, stereo_pass) == 0x128);
static_assert(offsetof(RouteBViewInit, use_faux_ortho) == 0x12C);
static_assert(sizeof(RouteBViewInit) == 0x130);

void Multiply4x4(const double a[4][4], const double b[4][4],
                 double out[4][4]) noexcept {
  double result[4][4]{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      double sum = 0.0;
      for (std::size_t inner = 0; inner < 4; ++inner) {
        sum += a[row][inner] * b[inner][column];
      }
      result[row][column] = sum;
    }
  }
  std::memcpy(out, result, sizeof(result));
}

// FInverseRotationMatrix(ViewRotation) * ViewPlanesMatrix, exactly the
// composition FViewMatrices uses for a view rotation (degrees).
void MakeViewRotationMatrix(const double pitch_degrees,
                            const double yaw_degrees,
                            const double roll_degrees,
                            double out[4][4]) noexcept {
  const double yaw = yaw_degrees * kDegreesToRadians;
  const double pitch = pitch_degrees * kDegreesToRadians;
  const double roll = roll_degrees * kDegreesToRadians;
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);

  const double yaw_matrix[4][4] = {
      {cy, -sy, 0.0, 0.0},
      {sy, cy, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double pitch_matrix[4][4] = {
      {cp, 0.0, -sp, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {sp, 0.0, cp, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double roll_matrix[4][4] = {
      {1.0, 0.0, 0.0, 0.0},
      {0.0, cr, sr, 0.0},
      {0.0, -sr, cr, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double planes[4][4] = {
      {0.0, 0.0, 1.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  double yaw_pitch[4][4]{};
  double yaw_pitch_roll[4][4]{};
  Multiply4x4(yaw_matrix, pitch_matrix, yaw_pitch);
  Multiply4x4(yaw_pitch, roll_matrix, yaw_pitch_roll);
  Multiply4x4(yaw_pitch_roll, planes, out);
}

bool ReadRouteBSnapshot(const RenderXProbe &probe, double car_cm[3],
                        double forward[3], std::uintptr_t &proxy,
                        std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint64_t first =
        probe.transform_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0) {
      continue;
    }
    const std::uintptr_t watched =
        probe.watched_proxy.load(std::memory_order_relaxed);
    const std::uintptr_t current_proxy =
        probe.transform_proxy.load(std::memory_order_relaxed);
    const bool valid =
        probe.transform_valid.load(std::memory_order_relaxed);
    const bool orientation_valid =
        probe.transform_orientation_valid.load(std::memory_order_relaxed);
    double current_car[3]{};
    double current_forward[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      current_car[axis] =
          probe.transform_cm[axis].load(std::memory_order_relaxed);
      current_forward[axis] =
          probe.transform_forward[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t current_timestamp =
        probe.transform_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.transform_sequence.load(std::memory_order_acquire);
    if (first != second || !valid || !orientation_valid ||
        watched == 0 || current_proxy != watched ||
        current_timestamp == 0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = current_car[axis];
      forward[axis] = current_forward[axis];
    }
    proxy = current_proxy;
    timestamp_ns = current_timestamp;
    return std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
           std::isfinite(car_cm[2]) && std::isfinite(forward[0]) &&
           std::isfinite(forward[1]) && std::isfinite(forward[2]);
  }
  return false;
}

// The camera model's pitch source. Same seqlock and proxy checks as route B,
// but only the body pitch is needed. `atan2` handles both the old horizontally
// normalised vector and the new 3D one.
bool ReadRenderPitch(const RenderXProbe &probe, double &pitch,
                     std::uintptr_t &proxy,
                     std::int64_t &timestamp_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, snapshot_proxy,
                          snapshot_ns)) {
    return false;
  }
  if (!BodyPitchFromForward(forward, pitch)) {
    return false;
  }
  proxy = snapshot_proxy;
  timestamp_ns = snapshot_ns;
  return true;
}

// Update the cached body pitch from the render-thread snapshot and return the
// value for this Game tick. Fresh snapshots win; while moving, a short render
// gap holds the last good value and then falls back to flat. A parked car keeps
// the last orientation because the transform hook may not fire again until it
// moves. The snapshot must also be newer than the last reset/teleport floor, so
// an old slope cannot leak into the new frame.
double UpdateRenderPitch(Context &context) noexcept {
  double pitch = 0.0;
  std::uintptr_t proxy = 0;
  std::int64_t snapshot_ns = 0;
  const bool read = ReadRenderPitch(context.render_x, pitch, proxy,
                                    snapshot_ns);
  const std::uintptr_t watched =
      context.render_x.watched_proxy.load(std::memory_order_relaxed);
  // A different watched proxy is a different car. Drop the cached attitude
  // before trying the new one; otherwise a parked car switch could keep the
  // old car's slope indefinitely.
  if (context.render_pitch_valid && watched != 0 &&
      watched != context.render_pitch_proxy) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
  }
  const std::int64_t now_ns = NowNanoseconds();
  if (read && proxy == watched &&
      snapshot_ns >= context.render_pitch_floor_ns) {
    const std::int64_t age = now_ns - snapshot_ns;
    if (age >= 0 && age <= kVehicleRenderTransformMaxAgeNs) {
      context.render_pitch_valid = true;
      context.render_pitch_radians = pitch;
      context.render_pitch_ns = snapshot_ns;
      context.render_pitch_proxy = proxy;
    }
  }
  if (!context.render_pitch_valid) {
    return 0.0;
  }
  const std::int64_t held = now_ns - context.render_pitch_ns;
  const double speed_cmps =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const bool parked = !(speed_cmps >= kRenderPitchStationarySpeedCmps);
  if (held < 0 || (!parked && held > kRenderPitchHoldNs)) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
    return 0.0;
  }
  return context.render_pitch_radians;
}

// The route B fallback is the only path that applies body pitch outside the
// chase model. It must obey the same master switch and flat-pose diagnostic so
// disabling slope follow also disables the fallback tilt.
[[nodiscard]] bool RouteBFollowPitch(const Context &context) noexcept {
  return context.slope_follow.load(std::memory_order_acquire) &&
         !context.diagnostic_flat_pose.load(std::memory_order_acquire);
}

bool BuildRouteBPoseFromCar(const double car_cm[3],
                            const double forward[3],
                            const bool follow_pitch, double eye[3],
                            double rotation[3]) noexcept {
  // The fallback uses the same body basis as the chase model's position frame.
  // `forward` may be fully normalised or only horizontally normalised; the
  // helper handles both. The master switch and the flat-pose diagnostic keep
  // the fallback yaw-only too.
  double body_pitch = 0.0;
  if (!BodyPitchFromForward(forward, body_pitch)) {
    return false;
  }
  if (!follow_pitch) {
    body_pitch = 0.0;
  }
  const double car_yaw = std::atan2(forward[1], forward[0]);
  // FViewMatrices and FViewInfo store world positions in Unreal units
  // (centimetres), unlike the chase model's metres. Keep this conversion
  // explicit: a metre-valued ViewOrigin puts the camera near the world origin
  // and leaves only the skybox visible.
  const nfs2015_chase::Vec3 car{
      car_cm[0],
      car_cm[1],
      car_cm[2] + kVehicleAnchorCenterHeightCm,
  };
  const nfs2015_chase::Vec3 rest{kRouteBRestXM * 100.0, 0.0,
                                 kRouteBRestZM * 100.0};
  const nfs2015_chase::Vec3 look_offset{kRouteBLookAheadM * 100.0, 0.0,
                                        kRouteBLookHeightM * 100.0};
  // `rest` is in centimetres, matching `car_cm`; the basis is unitless, so
  // CarLocalToWorld keeps the centimetre scale.
  const nfs2015_chase::CarBasis basis =
      nfs2015_chase::MakeCarBasis(car_yaw, body_pitch);
  const nfs2015_chase::Vec3 eye_vec =
      nfs2015_chase::Add(car, nfs2015_chase::CarLocalToWorld(rest, basis));
  // The look target stays in the world-horizontal nose frame: the rig climbs
  // with the body, but the horizon must not tilt with the slope.
  const nfs2015_chase::Vec3 look_vec =
      nfs2015_chase::Add(car, nfs2015_chase::RotateYaw(car_yaw, look_offset));
  const nfs2015_chase::Vec3 delta =
      nfs2015_chase::Sub(look_vec, eye_vec);
  const double delta_horizontal =
      std::sqrt(delta.x * delta.x + delta.y * delta.y);
  if (!(delta_horizontal > 1e-4)) {
    return false;
  }
  eye[0] = eye_vec.x;
  eye[1] = eye_vec.y;
  eye[2] = eye_vec.z;
  rotation[0] = std::atan2(delta.z, delta_horizontal) * kRadiansToDegrees;
  rotation[1] = std::atan2(delta.y, delta.x) * kRadiansToDegrees;
  rotation[2] = 0.0;
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

bool ReadRouteBModelPose(const RenderXProbe &probe, double car_cm[3],
                         double eye_cm[3], double rotation[3],
                         std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t first =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0 || !probe.route_b_model_valid.load(
                                     std::memory_order_relaxed)) {
      continue;
    }
    double local_car[3]{};
    double local_eye[3]{};
    double local_rotation[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      local_car[axis] =
          probe.route_b_model_car_cm[axis].load(std::memory_order_relaxed);
      local_eye[axis] =
          probe.route_b_model_eye_cm[axis].load(std::memory_order_relaxed);
      local_rotation[axis] =
          probe.route_b_model_rotation[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t timestamp =
        probe.route_b_model_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if (first != second || timestamp == 0 || !std::isfinite(local_car[0]) ||
        !std::isfinite(local_car[1]) || !std::isfinite(local_car[2]) ||
        !std::isfinite(local_eye[0]) || !std::isfinite(local_eye[1]) ||
        !std::isfinite(local_eye[2]) || !std::isfinite(local_rotation[0]) ||
        !std::isfinite(local_rotation[1]) ||
        !std::isfinite(local_rotation[2])) {
      continue;
    }
    const double dx = local_eye[0] - local_car[0];
    const double dy = local_eye[1] - local_car[1];
    const double dz = local_eye[2] - local_car[2];
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
        dx * dx + dy * dy + dz * dz > 5000.0 * 5000.0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = local_car[axis];
      eye_cm[axis] = local_eye[axis];
      rotation[axis] = local_rotation[axis];
    }
    timestamp_ns = timestamp;
    return true;
  }
  return false;
}

// The NFS camera is still the model's camera: the render transform only
// supplies this frame's car translation. Moving the model pose by that delta
// keeps the NFS offset/attitude (drift swing, speed dolly, roll) while making
// the camera and the drawn car share one render frame. If the model has not
// produced a pose yet, the old rigid mount is the safe fallback.
bool BuildRouteBPoseFromModel(const RenderXProbe &probe,
                              const double car_cm[3],
                              const double forward[3], double eye[3],
                              double rotation[3]) noexcept {
  double model_car[3]{};
  double model_eye[3]{};
  double model_rotation[3]{};
  std::int64_t model_ns = 0;
  if (!ReadRouteBModelPose(probe, model_car, model_eye, model_rotation,
                           model_ns)) {
    return false;
  }
  const std::int64_t age = NowNanoseconds() - model_ns;
  if (age < 0 || age > 100'000'000) {
    return false;
  }
  // `model_eye` already contains the body-frame slope from the chase model.
  // Only its translation moves to this render frame; applying the pitch again
  // here would double the slope.
  const double fresh_car[3] = {
      car_cm[0],
      car_cm[1],
      car_cm[2] + kVehicleAnchorCenterHeightCm,
  };
  double delta[3]{};
  double distance_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    delta[axis] = fresh_car[axis] - model_car[axis];
    distance_squared += delta[axis] * delta[axis];
  }
  if (!std::isfinite(distance_squared) ||
      distance_squared > kVehicleProxyMaxOffsetCm * kVehicleProxyMaxOffsetCm) {
    return false;
  }
  const double scale = nfs2015_chase::Clamp(
      probe.route_b_model_camera_scale.load(std::memory_order_relaxed),
      kMinimumCameraScale, kMaximumCameraScale);
  const double origin_offset_cm =
      probe.route_b_model_origin_offset_cm.load(std::memory_order_relaxed);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double model_offset = model_eye[axis] - model_car[axis];
    eye[axis] = fresh_car[axis] + model_offset * scale;
    rotation[axis] = model_rotation[axis];
  }
  // `origin_offset_cm` is a user-facing vertical trim. The model already
  // contains the game-tick copy of it, so the render anchor above must not
  // apply it twice; adding it here makes the slider move route B.
  eye[2] += origin_offset_cm;
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]) &&
         std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
         std::isfinite(forward[2]);
}

bool BuildRouteBPose(const RenderXProbe &probe, const bool follow_pitch,
                     double eye[3], double rotation[3],
                     std::int64_t &age_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t proxy{};
  std::int64_t transform_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, proxy, transform_ns)) {
    return false;
  }
  const std::int64_t now = NowNanoseconds();
  age_ns = now - transform_ns;
  if (age_ns < 0 || age_ns > kVehicleRenderTransformMaxAgeNs) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

// The pre-visibility pass can run before the render transform notification
// publishes this frame's proxy matrix. The proxy itself, however, is already
// the object the renderer draws the car from. Read its current LocalToWorld
// directly (not as a prediction): if the matrix has already advanced, this
// keeps route B on the exact same car frame as the draw; if it has not, the
// snapshot fallback is used.
bool ReadPresenterPose(const Context &context, double location[3],
                       double rotation[3], float *lens,
                       bool *fov_enabled,
                       double car_cm[3] = nullptr) noexcept;
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept;

// The Game thread already published the exact NFS camera pose it is using
// (`CameraViewPointDetour`). Applying that pose before visibility avoids
// rebuilding a camera from a car transform that the render thread has not
// received yet: at this point the transform snapshot is often one frame old,
// while the presenter pose is the camera the game itself would draw with.
//
// The presenter pose is translated by the difference between the car it was
// built from and the car the renderer is about to draw. That is a same-frame
// anchor, not a speed/prediction compensation: no extrapolation, no latency
// estimate, only the transform read from the watched proxy on this frame.
bool BuildRouteBPoseFromPresenter(Context &context, const RenderXProbe &probe,
                                  double eye[3],
                                  double rotation[3]) noexcept {
  double presenter_car[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  if (!ReadPresenterPose(context, eye, rotation, &lens, &fov_enabled,
                         presenter_car)) {
    return false;
  }
  double fresh_car[3]{};
  double forward[3]{};
  if (ReadRouteBDirectCar(probe, fresh_car, forward)) {
    // Move the whole presenter pose onto this frame's VehicleMesh transform.
    // Aligning only X/Y left the camera on the Game tick's Z, so on a slope
    // the drawn car moved vertically under a camera that did not: visible
    // up/down bobbing. Z must come from the same ComponentToWorld as X/Y.
    for (std::size_t axis = 0; axis < 3; ++axis) {
      eye[axis] += fresh_car[axis] - presenter_car[axis];
    }
    // `presenter_car` contains the origin offset baked into the presenter
    // pose. Moving to the raw VehicleMesh therefore cancels that offset;
    // add back only the user's current trim relative to the baked-in value.
    const double current_offset =
        context.origin_offset_cm.load(std::memory_order_acquire);
    const double presenter_offset =
        context.presenter_origin_offset_cm.load(std::memory_order_acquire);
    eye[2] += current_offset - presenter_offset;
  }
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

// Read the car transform the renderer is using. The VehicleMesh component
// transform is the authoritative value at the Game camera point: the probe
// showed parent-final = 0.00 cm while the watched proxy was still one frame
// behind. Prefer that component; the proxy remains the fallback for frames
// where the parent pointer is not available. Returns the raw translation
// (without the anchor centre height).
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept {
  const std::uintptr_t parent =
      probe.vehicle_parent.load(std::memory_order_relaxed);
  if (parent != 0) {
    const auto *translation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformTranslationOffset);
    const auto *orientation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformOffset);
    const double qx = orientation[0];
    const double qy = orientation[1];
    const double qz = orientation[2];
    const double qw = orientation[3];
    const double norm2 = qx * qx + qy * qy + qz * qz + qw * qw;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = translation[axis];
    }
    if (std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
        std::isfinite(car_cm[2]) && norm2 > 0.81 && norm2 < 1.21) {
      const double inverse_norm = 1.0 / std::sqrt(norm2);
      const double x = qx * inverse_norm;
      const double y = qy * inverse_norm;
      const double z = qz * inverse_norm;
      const double w = qw * inverse_norm;
      // Rotate the local +X (nose) axis by the ComponentToWorld quaternion.
      const double tx = 0.0;
      const double ty = 2.0 * z;
      const double tz = -2.0 * y;
      forward[0] = 1.0 + w * tx + (y * tz - z * ty);
      forward[1] = w * ty + (z * tx - x * tz);
      forward[2] = w * tz + (x * ty - y * tx);
      const double length_squared = forward[0] * forward[0] +
                                    forward[1] * forward[1] +
                                    forward[2] * forward[2];
      if (std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
          std::isfinite(forward[2]) && length_squared > 1e-8) {
        const double inverse_length = 1.0 / std::sqrt(length_squared);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          forward[axis] *= inverse_length;
        }
        return true;
      }
    }
  }

  double snapshot_car[3]{};
  double snapshot_forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  const bool have_snapshot =
      ReadRouteBSnapshot(probe, snapshot_car, snapshot_forward,
                         snapshot_proxy, snapshot_ns);
  const std::uintptr_t watched =
      probe.watched_proxy.load(std::memory_order_relaxed);
  if (watched == 0) {
    return false;
  }
  const bool snapshot_matches = have_snapshot && snapshot_proxy == watched;

  const auto *matrix = reinterpret_cast<const double *>(watched + 0xE0);
  const auto *translation = reinterpret_cast<const double *>(
      watched + kVehicleProxyTranslationOffset);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    car_cm[axis] = translation[axis];
    forward[axis] = matrix[axis];
  }
  if (!std::isfinite(car_cm[0]) || !std::isfinite(car_cm[1]) ||
      !std::isfinite(car_cm[2]) || !std::isfinite(forward[0]) ||
      !std::isfinite(forward[1]) || !std::isfinite(forward[2])) {
    return false;
  }
  const double length_squared = forward[0] * forward[0] +
                                forward[1] * forward[1] +
                                forward[2] * forward[2];
  if (!(length_squared > 1e-8)) {
    return false;
  }
  const double inverse_length = 1.0 / std::sqrt(length_squared);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    forward[axis] *= inverse_length;
  }

  double reference_cm[3]{};
  bool have_reference = false;
  double limit_cm = 0.0;
  if (snapshot_matches) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] = snapshot_car[axis];
    }
    have_reference = true;
    limit_cm = kVehicleProxyMaxOffsetCm;
  } else if (probe.vehicle_anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.vehicle_anchor_cm[axis].load(std::memory_order_relaxed);
    }
    reference_cm[2] -= kVehicleAnchorCenterHeightCm;
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  } else if (probe.anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.anchor_cm[axis].load(std::memory_order_relaxed);
    }
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  }
  if (have_reference) {
    double distance_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = car_cm[axis] - reference_cm[axis];
      distance_squared += delta * delta;
    }
    if (!std::isfinite(distance_squared) ||
        distance_squared > limit_cm * limit_cm) {
      return false;
    }
  }
  return true;
}

bool BuildRouteBPoseDirect(const RenderXProbe &probe, const bool follow_pitch,
                           double eye[3],
                           double rotation[3]) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  if (!ReadRouteBDirectCar(probe, car_cm, forward)) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

bool IsRouteBFullScreenView(void *view) noexcept {
  if (view == nullptr ||
      reinterpret_cast<std::uintptr_t>(view) < 0x10000) {
    return false;
  }
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(
                  reinterpret_cast<std::uintptr_t>(view) +
                  kViewInfoRectOffset),
              sizeof(rect));
  return rect[2] >= kRouteBMinViewWidth &&
         rect[3] >= kRouteBMinViewHeight;
}

double Determinant4x4(const double matrix[16]) noexcept {
  return matrix[0] *
             (matrix[5] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) +
              matrix[7] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13])) -
         matrix[1] *
             (matrix[4] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12])) +
         matrix[2] *
             (matrix[4] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12])) -
         matrix[3] *
             (matrix[4] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12]) +
              matrix[6] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12]));
}

// A hash match proves the offsets were verified against this profile; it does
// not prove that a particular FViewInfo is still a live full-screen view. Check
// the fields route B is about to read and write before touching them, so a
// changed view layout fails closed instead of writing a camera pose into an
// unrelated object.
bool IsRouteBViewWritable(void *view) noexcept {
  if (!IsRouteBFullScreenView(view)) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);
  double location[3]{};
  double rotation[3]{};
  double projection[16]{};
  float fov = 0.0F;
  std::memcpy(location,
              reinterpret_cast<const void *>(address + kViewLocationOffset),
              sizeof(location));
  std::memcpy(rotation,
              reinterpret_cast<const void *>(address + kViewRotationOffset),
              sizeof(rotation));
  std::memcpy(projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(projection));
  std::memcpy(&fov, reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(fov));
  for (double value : location) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : rotation) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : projection) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (!(std::fabs(Determinant4x4(projection)) > 1e-12) ||
      !(fov > 1.0F) || !(fov < 179.0F)) {
    return false;
  }
  return true;
}

// Write the final route B pose into one view, before visibility and before
// PrepareViewStateForVisibility snapshots the temporal history. This is the
// only place the camera may be moved: a post-Prepare write would leave the
// culling frustum, motion vectors and TAA history describing a different
// camera, which is visible as a one-frame ghost/flash.
//
// The FViewMatrices constructor is used instead of scaling the stored
// projection in place. In-place scaling leaves InvProjectionMatrix,
// ProjectionNoAAMatrix and ScreenToClipMatrix describing the game's lens; TAA,
// SSR and the depth reconstruction read those on the same frame. The
// constructor refreshes every derived matrix from the projection and rotation
// the renderer actually uses.
bool ApplyRouteBPoseToView(void *view, const double eye[3],
                           const double rotation[3],
                           const RenderXProbe &probe) noexcept {
  if (view == nullptr || !IsRouteBFullScreenView(view) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  ViewMatricesCtorFn ctor =
      g_view_matrices_ctor.load(std::memory_order_acquire);
  if (ctor == nullptr) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);

  RouteBViewInit init{};
  std::memcpy(init.projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(init.projection));

  // The model lens replaces the game's lens for the same frame. Keep the
  // depth range and TAA jitter terms untouched; only the X/Y scale changes.
  const float desired =
      probe.route_b_model_lens.load(std::memory_order_relaxed);
  float current = 0.0F;
  std::memcpy(&current,
              reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(current));
  bool lens_changed = false;
  if (std::isfinite(current) && current > 1.0F && current < 179.0F &&
      desired > 1.0F && desired < 179.0F &&
      std::fabs(current - desired) >= 0.01F) {
    const double current_radians =
        static_cast<double>(current) * kDegreesToRadians;
    const double desired_radians =
        static_cast<double>(desired) * kDegreesToRadians;
    const double denominator = std::tan(desired_radians * 0.5);
    if (std::fabs(denominator) > 1e-6) {
      const double factor = std::tan(current_radians * 0.5) / denominator;
      if (std::isfinite(factor) && factor > 0.0) {
        init.projection[0][0] *= factor;
        init.projection[1][1] *= factor;
        lens_changed = true;
      }
    }
  }

  MakeViewRotationMatrix(rotation[0], rotation[1], rotation[2],
                         init.view_rotation);
  init.view_origin[0] = eye[0];
  init.view_origin[1] = eye[1];
  init.view_origin[2] = eye[2];
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(address + kViewInfoRectOffset),
              sizeof(rect));
  std::memcpy(init.constrained_rect, rect, sizeof(init.constrained_rect));
  init.stereo_pass = 0;
  init.use_faux_ortho = false;

  try {
    ctor(reinterpret_cast<void *>(address + kViewMatricesOffset), &init);
  } catch (...) {
    return false;
  }

  if (lens_changed) {
    std::memcpy(reinterpret_cast<void *>(address + kViewFovOffset), &desired,
                sizeof(desired));
    std::memcpy(reinterpret_cast<void *>(address + kViewDesiredFovOffset),
                &desired, sizeof(desired));
  }
  std::memcpy(reinterpret_cast<void *>(address + kViewLocationOffset), eye,
              sizeof(eye[0]) * 3);
  std::memcpy(reinterpret_cast<void *>(address + kViewRotationOffset),
              rotation, sizeof(rotation[0]) * 3);

  // Keep ShadowViewMatrices and ViewFrustum in sync; the constructor only
  // rebuilt the view matrices. SetupViewFrustum then refreshes both the culling
  // frustum and its origin for the non-stereo visibility path.
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update != nullptr) {
    update(view);
  }
  SceneViewSetupFrustumFn setup_frustum =
      g_scene_view_setup_frustum.load(std::memory_order_acquire);
  if (setup_frustum != nullptr) {
    setup_frustum(view);
  }
  return true;
}

// Apply route B before FSceneRenderer::PrepareViewStateForVisibility. At this
// point the view matrices are still the game's, but visibility tasks have not
// been launched and the temporal history has not been advanced. Rebuilding the
// matrices here makes the frustum and the TAA history describe the same chase
// camera the renderer will later use.
bool ApplyRouteBEarly(Context &context, void *renderer) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!probe.version_gate.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // A failed attempt this frame must not leave an older frame's view list
  // looking fresh to the diagnostics.
  probe.route_b_early_views_valid.store(false, std::memory_order_release);
  probe.route_b_early_view_count.store(0, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(0, std::memory_order_relaxed);
  for (auto &state : probe.route_b_early_view_states) {
    state.store(0, std::memory_order_relaxed);
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    probe.prepare_thread_mismatch.fetch_add(1, std::memory_order_relaxed);
  }
  if (renderer == nullptr) {
    return false;
  }
  if (!probe.anchor_enabled.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update == nullptr) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  const auto renderer_address = reinterpret_cast<std::uintptr_t>(renderer);
  void *views = nullptr;
  std::int32_t count = 0;
  std::memcpy(&views,
              reinterpret_cast<const void *>(renderer_address +
                                             kSceneRendererAllViewsOffset),
              sizeof(views));
  std::memcpy(&count,
              reinterpret_cast<const void *>(
                  renderer_address + kSceneRendererAllViewCountOffset),
              sizeof(count));
  if (views == nullptr || count <= 0 || count > kRouteBMaxEarlyViews) {
    probe.prepare_no_views.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  double eye[3]{};
  double rotation[3]{};
  std::int64_t age_ns = 0;
  const std::int64_t now_ns = NowNanoseconds();
  const bool follow_pitch = RouteBFollowPitch(context);
  // The pre-visibility call can run one render frame before the transform
  // notification is published; the 2.7 log showed a 23 ms snapshot age here.
  // Using that snapshot makes the camera ride the previous car transform and
  // the car visibly shakes. The proxy itself is already the object the
  // renderer draws this frame, so read it directly first. The notification
  // snapshot is only a fallback when the proxy pointer is not available.
  // The presenter pose is the artifact-free path: it is the NFS camera the
  // Game thread already produced, and it is aligned horizontally here with
  // the fresh VehicleMesh transform. Its Z carries the model height and the
  // origin-offset slider. The model-pose path remains a fallback.
  bool have_pose =
      BuildRouteBPoseFromPresenter(context, probe, eye, rotation);
  if (have_pose) {
    probe.route_b_early_presenter.fetch_add(1, std::memory_order_relaxed);
  } else if (BuildRouteBPoseDirect(probe, follow_pitch, eye, rotation)) {
    probe.route_b_early_direct.fetch_add(1, std::memory_order_relaxed);
    have_pose = true;
  } else if (BuildRouteBPose(probe, follow_pitch, eye, rotation, age_ns)) {
    have_pose = true;
  }
  if (!have_pose) {
    const std::int64_t held_age = now_ns - probe.route_b_held_ns;
    if (probe.route_b_held_pose_valid && held_age >= 0 &&
        held_age <= kRouteBHeldPoseNs) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        eye[axis] = probe.route_b_held_eye_cm[axis];
        rotation[axis] = probe.route_b_held_rotation[axis];
      }
      probe.route_b_early_held.fetch_add(1, std::memory_order_relaxed);
      have_pose = true;
    }
  }
  if (!have_pose) {
    probe.prepare_pose_fail.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Keep the last applied pose for the one-frame hold above. The pose is in
  // world centimetres, like the view fields it is copied into.
  probe.route_b_held_pose_valid = true;
  probe.route_b_held_ns = now_ns;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.route_b_held_eye_cm[axis] = eye[axis];
    probe.route_b_held_rotation[axis] = rotation[axis];
  }

  auto *view_array = static_cast<void *const *>(views);
  std::uint64_t applied = 0;
  for (std::int32_t index = 0; index < count; ++index) {
    void *view = view_array[index];
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    // One authoritative write, before visibility: pose, lens and every derived
    // matrix (including InvProjection) are rebuilt together.
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    if (applied <
        sizeof(probe.route_b_early_views) /
            sizeof(probe.route_b_early_views[0])) {
      probe.route_b_early_views[applied].store(
          reinterpret_cast<std::uintptr_t>(view), std::memory_order_relaxed);
      std::uintptr_t state = 0;
      std::memcpy(&state,
                  reinterpret_cast<const void *>(view_address +
                                                 kViewStateOffset),
                  sizeof(state));
      probe.route_b_early_view_states[applied].store(
          state, std::memory_order_relaxed);
    }
    ++applied;
  }

  probe.prepare_fullscreen_views.fetch_add(applied, std::memory_order_relaxed);
  if (applied == 0) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  probe.route_b_early_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_early_last_age_us.store(
      static_cast<std::uint64_t>(age_ns / 1000), std::memory_order_relaxed);
  const std::int64_t prepared_ns = NowNanoseconds();
  probe.route_b_early_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_early_view_count.store(
      static_cast<std::uint32_t>(applied), std::memory_order_release);
  probe.route_b_prepare_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(
      reinterpret_cast<std::uintptr_t>(renderer), std::memory_order_release);
  probe.route_b_prepare_generation.fetch_add(1, std::memory_order_acq_rel);
  probe.route_b_early_views_valid.store(true, std::memory_order_release);

  // TEMP read-only timing diagnostics: snapshot the car/camera sources at the
  // exact point where the early camera is written. No writes to game state.
  {
    probe.timing_frame.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_count.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    probe.timing_late_seen.store(false, std::memory_order_release);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.timing_early_view[axis].store(eye[axis],
                                          std::memory_order_relaxed);
    }
    double car_cm[3]{};
    double forward[3]{};
    if (ReadRouteBDirectCar(probe, car_cm, forward)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_early_car[axis].store(car_cm[axis],
                                           std::memory_order_relaxed);
      }
    }
    double presenter_car[3]{};
    double presenter_eye[3]{};
    double presenter_rotation[3]{};
    float presenter_lens = 0.0F;
    bool presenter_fov = false;
    if (ReadPresenterPose(context, presenter_eye, presenter_rotation,
                          &presenter_lens, &presenter_fov,
                          presenter_car)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_presenter_car[axis].store(
            presenter_car[axis], std::memory_order_relaxed);
      }
    }
  }
  return true;
}

// Experimental transform-hook writer. It is intentionally guarded by the
// view list captured by ApplyRouteBEarly and by the prepare generation, so it
// writes at most once per prepared frame. This is not a late uniform fallback.
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!context.started.load(std::memory_order_acquire) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return;
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    return;
  }
  if (!probe.route_b_early_views_valid.load(std::memory_order_acquire)) {
    return;
  }
  const std::uintptr_t prepared_renderer =
      probe.route_b_prepare_renderer.load(std::memory_order_relaxed);
  const std::uintptr_t active_renderer =
      probe.route_b_active_renderer.load(std::memory_order_acquire);
  if (prepared_renderer == 0 ||
      (active_renderer != 0 && active_renderer == prepared_renderer)) {
    return;
  }
  const std::uint64_t generation =
      probe.route_b_prepare_generation.load(std::memory_order_acquire);
  const std::uint64_t last_generation =
      probe.route_b_transform_generation.load(std::memory_order_acquire);
  if (generation == 0 || generation == last_generation) {
    return;
  }
  const std::int64_t prepared_ns =
      probe.route_b_prepare_ns.load(std::memory_order_relaxed);
  const std::int64_t now_ns = NowNanoseconds();
  if (prepared_ns == 0 || now_ns < prepared_ns ||
      now_ns - prepared_ns > 100'000'000) {
    return;
  }
  const std::uint32_t count =
      probe.route_b_early_view_count.load(std::memory_order_acquire);
  if (count == 0 ||
      count > sizeof(probe.route_b_early_views) /
                  sizeof(probe.route_b_early_views[0])) {
    return;
  }
  double eye[3]{};
  double rotation[3]{};
  const bool follow_pitch = RouteBFollowPitch(context);
  if (!BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) &&
      !BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation)) {
    return;
  }
  std::uint64_t applied = 0;
  for (std::uint32_t index = 0; index < count; ++index) {
    void *view = reinterpret_cast<void *>(
        probe.route_b_early_views[index].load(std::memory_order_relaxed));
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    std::uintptr_t state = 0;
    std::memcpy(&state,
                reinterpret_cast<const void *>(view_address +
                                               kViewStateOffset),
                sizeof(state));
    if (state == 0 ||
        state != probe.route_b_early_view_states[index].load(
                     std::memory_order_relaxed)) {
      continue;
    }
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    ++applied;
  }
  if (applied == 0) {
    return;
  }
  probe.route_b_render_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_transform_generation.store(generation,
                                            std::memory_order_release);
  probe.route_b_early_last_age_us.store(0, std::memory_order_relaxed);
  probe.route_b_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
}

// Diagnostic-only view hook. It records whether the renderer's late view setup
// runs on the same thread as the render transform hook, and how old the
// transform snapshot is. It deliberately never touches the camera: changing
// the pose after PrepareViewStateForVisibility would leave this frame's
// culling, motion vectors and TAA history describing a different camera, which
// is exactly the one-frame ghost this hook used to cause.
//
// The hook is kept installed because the transform/variable diagnostics are
// useful and because removing an established hook target would change the
// plugin's failure surface. Route B itself is applied only in
// ApplyRouteBEarly, before visibility.
bool IsRouteBEarlyView(const RenderXProbe &probe, void *view) noexcept {
  if (view == nullptr ||
      !probe.route_b_early_views_valid.load(std::memory_order_acquire)) {
    return false;
  }
  const std::uint32_t count =
      probe.route_b_early_view_count.load(std::memory_order_relaxed);
  const std::uint32_t limit =
      count < sizeof(probe.route_b_early_views) /
                  sizeof(probe.route_b_early_views[0])
          ? count
          : static_cast<std::uint32_t>(
                sizeof(probe.route_b_early_views) /
                sizeof(probe.route_b_early_views[0]));
  const auto address = reinterpret_cast<std::uintptr_t>(view);
  std::uintptr_t state = 0;
  std::memcpy(&state,
              reinterpret_cast<const void *>(address + kViewStateOffset),
              sizeof(state));
  for (std::uint32_t index = 0; index < limit; ++index) {
    if (probe.route_b_early_views[index].load(
            std::memory_order_relaxed) == address &&
        probe.route_b_early_view_states[index].load(
            std::memory_order_relaxed) == state) {
      return true;
    }
  }
  return false;
}

void ANOMALY_CALL ViewUniformDetour(void *self, void *view_matrices,
                                    void *prev_view_matrices,
                                    void *out_parameters,
                                    std::uint64_t flag,
                                    void *uniform_buffer) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->view_uniform_hook_id.load(std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->view_uniform_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  if (context != nullptr) {
    RenderXProbe &probe = context->render_x;
    probe.view_uniform_calls.fetch_add(1, std::memory_order_relaxed);
    const std::uint32_t thread = static_cast<std::uint32_t>(GetCurrentThreadId());
    const std::uint32_t render_thread =
        probe.render_thread_id.load(std::memory_order_acquire);
    if (render_thread != 0 && thread == render_thread) {
      probe.view_uniform_render_calls.fetch_add(1,
                                                std::memory_order_relaxed);
      if (probe.transform_valid.load(std::memory_order_acquire)) {
        const std::int64_t transform_ns =
            probe.transform_ns.load(std::memory_order_relaxed);
        const std::int64_t age = NowNanoseconds() - transform_ns;
        // Only a plausible in-flight age is a timing sample. A huge value is
        // a stale/never-written snapshot, not a camera frame.
        if (transform_ns != 0 && age >= 0 && age <= 1'000'000'000) {
          const auto age_us = static_cast<std::uint64_t>(age / 1000);
          RaisePeak(probe.view_uniform_age_peak_us, age_us);
          probe.view_uniform_age_sum_us.fetch_add(age_us,
                                                  std::memory_order_relaxed);
          probe.view_uniform_age_count.fetch_add(1,
                                                  std::memory_order_relaxed);
        }
      }
    } else {
      probe.view_uniform_game_calls.fetch_add(1,
                                               std::memory_order_relaxed);
    }
  }

  SetupUniformBufferFn original =
      g_setup_uniform_buffer_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, view_matrices, prev_view_matrices, out_parameters, flag,
               uniform_buffer);
    }
  } catch (...) {
  }

  // TEMP read-only timing diagnostics: compare the car/view at the late
  // uniform setup with the values captured by the early camera write.
  if (context != nullptr && IsRouteBEarlyView(context->render_x, self) &&
      context->render_x.timing_frame.load(std::memory_order_relaxed) != 0 &&
      !context->render_x.timing_late_seen.exchange(
          true, std::memory_order_acq_rel)) {
    RenderXProbe &probe = context->render_x;
    const auto address = reinterpret_cast<std::uintptr_t>(self);
    double late_view[3]{};
    std::memcpy(late_view,
                reinterpret_cast<const void *>(address +
                                               kViewLocationOffset),
                sizeof(late_view));
    double late_car[3]{};
    double late_forward[3]{};
    const bool have_car =
        ReadRouteBDirectCar(probe, late_car, late_forward);
    probe.timing_late_count.fetch_add(1, std::memory_order_relaxed);
    probe.timing_late_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    double car_distance = 0.0;
    double view_distance = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.timing_late_view[axis].store(late_view[axis],
                                         std::memory_order_relaxed);
      if (have_car) {
        probe.timing_late_car[axis].store(late_car[axis],
                                          std::memory_order_relaxed);
        const double car_delta =
            late_car[axis] -
            probe.timing_early_car[axis].load(std::memory_order_relaxed);
        car_distance += car_delta * car_delta;
      }
      const double view_delta =
          late_view[axis] -
          probe.timing_early_view[axis].load(std::memory_order_relaxed);
      view_distance += view_delta * view_delta;
    }
    const auto store_distance = [](std::atomic<std::uint64_t> &peak,
                                   std::atomic<std::uint64_t> &sum,
                                   std::atomic<std::uint64_t> &count,
                                   const double distance_cm) {
      if (!std::isfinite(distance_cm)) {
        return;
      }
      const auto value = static_cast<std::uint64_t>(
          distance_cm * 10.0 + 0.5);
      RaisePeak(peak, value);
      sum.fetch_add(value, std::memory_order_relaxed);
      count.fetch_add(1, std::memory_order_relaxed);
    };
    if (have_car) {
      store_distance(probe.timing_car_delta_peak_dmm,
                     probe.timing_car_delta_sum_dmm,
                     probe.timing_car_delta_count,
                     std::sqrt(car_distance));
    }
    store_distance(probe.timing_view_delta_peak_dmm,
                   probe.timing_view_delta_sum_dmm,
                   probe.timing_view_delta_count,
                   std::sqrt(view_distance));
  }

  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

// Pre-visibility route B. This runs immediately before culling tasks are
// launched and before PrepareViewStateForVisibility snapshots the matrices
// into the temporal history. It is the only camera writer; the late
// view-uniform hook is diagnostic and must never move a view.
void ANOMALY_CALL PrepareViewStateDetour(void *self,
                                         const void *scene_textures_config) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  const std::uint64_t hook_id =
      context == nullptr
          ? 0
          : context->prepare_view_state_hook_id.load(
                std::memory_order_acquire);
  try {
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->prepare_view_state_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
    }
  } catch (...) {
    leased = false;
  }

  if (context != nullptr) {
    context->render_x.prepare_calls.fetch_add(1, std::memory_order_relaxed);
    context->render_x.route_b_active_renderer.store(
        reinterpret_cast<std::uintptr_t>(self), std::memory_order_release);
    if (context->enabled.load(std::memory_order_acquire)) {
      static_cast<void>(ApplyRouteBEarly(*context, self));
    }
  }

  PrepareViewStateFn original =
      g_prepare_view_state_original.load(std::memory_order_acquire);
  try {
    if (original != nullptr) {
      original(self, scene_textures_config);
    }
  } catch (...) {
  }

  if (context != nullptr &&
      context->render_x.route_b_active_renderer.load(
          std::memory_order_relaxed) ==
          reinterpret_cast<std::uintptr_t>(self)) {
    context->render_x.route_b_active_renderer.store(
        0, std::memory_order_release);
  }

  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

bool EnsurePrepareViewStateHook(Context &context) noexcept {
  if (context.early_route_b_unavailable) {
    return false;
  }
  const auto note_pattern_miss = [&context]() {
    ++context.prepare_hook_pattern_failures;
    if (context.prepare_hook_pattern_failures >=
        kPrepareHookMaxPatternFailures) {
      context.early_route_b_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: early route B pattern still unresolved after "
          "the retry window; route B disabled for this session");
    }
  };
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.prepare_view_state_hook_id.load(std::memory_order_acquire) !=
          0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }

  std::uintptr_t target =
      context.prepare_view_state_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kPrepareViewStatePattern, target)) {
      note_pattern_miss();
      if (!context.prepare_view_state_pattern_failure_logged) {
        context.prepare_view_state_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: PrepareViewStateForVisibility pattern not "
            "found; early route B is retrying, no late camera write");
      }
      return false;
    }
    context.prepare_view_state_hook_target.store(target,
                                                  std::memory_order_release);
  }

  if (g_scene_view_update_matrix.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t update_matrix = 0;
    if (!ResolveSignature(context, kSceneViewUpdateMatrixPattern,
                          update_matrix)) {
      note_pattern_miss();
      if (!context.route_b_pattern_failure_logged) {
        context.route_b_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: FSceneView::UpdateViewMatrix pattern not "
            "found; early route B is retrying, no late camera write");
      }
      return false;
    }
    g_scene_view_update_matrix.store(
        reinterpret_cast<SceneViewUpdateMatrixFn>(update_matrix),
        std::memory_order_release);
  }

  if (g_scene_view_setup_frustum.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t setup_frustum = 0;
    if (!ResolveSignature(context, kSceneViewSetupFrustumPattern,
                          setup_frustum)) {
      if (!context.route_b_pattern_failure_logged) {
        context.route_b_pattern_failure_logged = true;
        Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
            "nfs2015 chase cam: FSceneView::SetupViewFrustum pattern not "
            "found; CullingFrustum may trail route B");
      }
    } else {
      g_scene_view_setup_frustum.store(
          reinterpret_cast<SceneViewSetupFrustumFn>(setup_frustum),
          std::memory_order_release);
    }
  }

  // Route B rebuilds FViewMatrices through the engine constructor so every
  // derived matrix (InvProjection, ProjectionNoAA, ScreenToClip) matches the
  // lens it writes. Resolve the constructor here as well: the prepare hook can
  // be installed before the diagnostic view-uniform hook.
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t matrices_ctor = 0;
    if (ResolveSignature(context, kViewMatricesCtorPattern, matrices_ctor)) {
      g_view_matrices_ctor.store(
          reinterpret_cast<ViewMatricesCtorFn>(matrices_ctor),
          std::memory_order_release);
    }
  }
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    note_pattern_miss();
    if (!context.route_b_pattern_failure_logged) {
      context.route_b_pattern_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: FViewMatrices ctor pattern not found; early "
          "route B is retrying, no late camera write");
    }
    return false;
  }

  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&PrepareViewStateDetour);
  request.label = StringView("nfs2015-chase-cam-prepare-view-state");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.prepare_view_state_hook_id.store(0, std::memory_order_release);
    context.prepare_view_state_hook_generation.store(
        0, std::memory_order_release);
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_hook_id.load(std::memory_order_acquire) == 0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.prepare_view_state_hook_failure_logged) {
      context.prepare_view_state_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: prepare-view-state hook creation failed");
    }
    return false;
  }
  context.prepare_view_state_hook_generation.store(
      handle.generation, std::memory_order_release);
  g_prepare_view_state_original.store(
      reinterpret_cast<PrepareViewStateFn>(original),
      std::memory_order_release);
  context.prepare_view_state_hook_id.store(handle.id,
                                            std::memory_order_release);
  context.prepare_view_state_hook_failure_logged = false;
  context.prepare_hook_pattern_failures = 0;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: pre-visibility route B hook installed");
  return true;
}

bool EnsureViewUniformHook(Context &context) noexcept {
  if (context.view_uniform_pattern_unavailable) {
    return false;
  }
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.view_uniform_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.view_uniform_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kSetupUniformBufferPattern, target)) {
      context.view_uniform_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: SetupUniformBufferParameters pattern not found; "
          "view-uniform diagnostics disabled (route B is unaffected)");
      return false;
    }
    context.view_uniform_hook_target.store(target, std::memory_order_release);
  }
  if (g_view_matrices_ctor.load(std::memory_order_acquire) == nullptr) {
    std::uintptr_t matrices_ctor = 0;
    if (ResolveSignature(context, kViewMatricesCtorPattern, matrices_ctor)) {
      g_view_matrices_ctor.store(
          reinterpret_cast<ViewMatricesCtorFn>(matrices_ctor),
          std::memory_order_release);
    } else if (!context.route_b_pattern_failure_logged) {
      context.route_b_pattern_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: FViewMatrices ctor pattern not found; route B "
          "will use the original camera");
    }
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&ViewUniformDetour);
  request.label = StringView("nfs2015-chase-cam-view-uniform");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.view_uniform_hook_id.store(0, std::memory_order_release);
    context.view_uniform_hook_generation.store(0, std::memory_order_release);
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.view_uniform_hook_failure_logged) {
      context.view_uniform_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: view uniform hook creation failed");
    }
    return false;
  }
  context.view_uniform_hook_generation.store(handle.generation,
                                               std::memory_order_release);
  g_setup_uniform_buffer_original.store(
      reinterpret_cast<SetupUniformBufferFn>(original),
      std::memory_order_release);
  context.view_uniform_hook_id.store(handle.id, std::memory_order_release);
  context.view_uniform_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: render view hook installed");
  return true;
}

bool EnsureTransformHook(Context &context) noexcept {
  if (context.transform_pattern_unavailable) {
    return false;
  }
  if (!context.render_x.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  if (context.transform_hook_id.load(std::memory_order_acquire) != 0 &&
      HookReady(context.hook)) {
    return true;
  }
  if (!HookReady(context.hook)) {
    return false;
  }
  std::uintptr_t target =
      context.transform_hook_target.load(std::memory_order_acquire);
  if (target == 0) {
    if (!ResolveSignature(context, kOnTransformChangedPattern, target)) {
      context.transform_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: OnTransformChanged pattern not found; render "
          "transform hook disabled for this session");
      return false;
    }
    context.transform_hook_target.store(target, std::memory_order_release);
  }
  AnomalyHookRequestV1 request{sizeof(request)};
  request.kind = ANOMALY_HOOK_V1_FUNCTION;
  request.target = target;
  request.detour = reinterpret_cast<void *>(&RenderTransformDetour);
  request.label = StringView("nfs2015-chase-cam-on-transform-changed");
  std::uintptr_t original{};
  AnomalyGenerationHandleV1 handle{};
  g_active.store(&context, std::memory_order_release);
  const auto status =
      context.hook->create(context.hook->user, &request, &original, &handle);
  if (status.code != ANOMALY_STATUS_V1_OK || handle.id == 0 || original == 0) {
    if (handle.id != 0 && HookReady(context.hook)) {
      static_cast<void>(
          context.hook->release(context.hook->user, handle));
    }
    context.transform_hook_id.store(0, std::memory_order_release);
    context.transform_hook_generation.store(0, std::memory_order_release);
    if (context.view_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
    if (!context.transform_hook_failure_logged) {
      context.transform_hook_failure_logged = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: render transform hook creation failed");
    }
    return false;
  }
  context.transform_hook_generation.store(handle.generation,
                                           std::memory_order_release);
  g_on_transform_changed_original.store(
      reinterpret_cast<OnTransformChangedFn>(original),
                                 std::memory_order_release);
  context.transform_hook_id.store(handle.id, std::memory_order_release);
  context.transform_hook_failure_logged = false;
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
      "nfs2015 chase cam: render transform hook installed");
  return true;
}

bool EnsureHook(Context &context) noexcept {
  if (context.view_point_pattern_unavailable) {
    return false;
  }
  // 2.5 has no build service. Preserve its legacy view-point hook rather than
  // turning an optional service into a hard requirement; 2.6+ with a build
  // service present is fail-closed on a hash mismatch.
  const bool legacy_without_build =
      context.render_x.insurance_state == kAnchorInsuranceServiceMissing;
  if (!context.render_x.version_gate.load(std::memory_order_acquire) &&
      !legacy_without_build) {
    return false;
  }
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
      context.view_point_pattern_unavailable = true;
      Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING,
          "nfs2015 chase cam: CameraManager view-point pattern not found; "
          "camera hook disabled for this session");
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
    if (context.transform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.view_uniform_hook_id.load(std::memory_order_acquire) == 0 &&
        context.prepare_view_state_hook_id.load(std::memory_order_acquire) ==
            0) {
      g_active.store(nullptr, std::memory_order_release);
    }
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
  const std::uint64_t transform_id =
      context.transform_hook_id.load(std::memory_order_acquire);
  bool transform_released = false;
  if (transform_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        transform_id,
        context.transform_hook_generation.load(std::memory_order_acquire)};
    transform_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (transform_released) {
    g_on_transform_changed_original.store(nullptr,
                                          std::memory_order_release);
  }
  context.transform_hook_id.store(0, std::memory_order_release);
  context.transform_hook_generation.store(0, std::memory_order_release);
  const std::uint64_t view_uniform_id =
      context.view_uniform_hook_id.load(std::memory_order_acquire);
  bool view_uniform_released = false;
  if (view_uniform_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        view_uniform_id,
        context.view_uniform_hook_generation.load(std::memory_order_acquire)};
    view_uniform_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (view_uniform_released) {
    g_setup_uniform_buffer_original.store(nullptr, std::memory_order_release);
  }
  context.view_uniform_hook_id.store(0, std::memory_order_release);
  context.view_uniform_hook_generation.store(0, std::memory_order_release);
  const std::uint64_t prepare_view_state_id =
      context.prepare_view_state_hook_id.load(std::memory_order_acquire);
  bool prepare_view_state_released = false;
  if (prepare_view_state_id != 0 && HookReady(context.hook)) {
    const AnomalyGenerationHandleV1 handle{
        prepare_view_state_id,
        context.prepare_view_state_hook_generation.load(
            std::memory_order_acquire)};
    prepare_view_state_released =
        context.hook->release(context.hook->user, handle).code ==
        ANOMALY_STATUS_V1_OK;
  }
  if (prepare_view_state_released) {
    g_prepare_view_state_original.store(nullptr, std::memory_order_release);
  }
  context.prepare_view_state_hook_id.store(0, std::memory_order_release);
  context.prepare_view_state_hook_generation.store(0,
                                                    std::memory_order_release);
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
                   const double shake_scale,
                   const double slope_strength,
                   const double slope_max_gradient_degrees,
                   const double slope_up_look_at,
                   const double slope_up_rest,
                   const double slope_down_look_at,
                   const double slope_down_rest) noexcept {
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
         shake_scale <= kMaximumShakeScale &&
         std::isfinite(slope_strength) &&
         slope_strength >= kMinimumSlopeStrength &&
         slope_strength <= kMaximumSlopeStrength &&
         std::isfinite(slope_max_gradient_degrees) &&
         slope_max_gradient_degrees >= kMinimumSlopeMaxGradientDegrees &&
         slope_max_gradient_degrees <= kMaximumSlopeMaxGradientDegrees &&
         std::isfinite(slope_up_look_at) &&
         slope_up_look_at >= kMinimumSlopeLookAt &&
         slope_up_look_at <= kMaximumSlopeLookAt &&
         std::isfinite(slope_up_rest) &&
         slope_up_rest >= kMinimumSlopeRest &&
         slope_up_rest <= kMaximumSlopeRest &&
         std::isfinite(slope_down_look_at) &&
         slope_down_look_at >= kMinimumSlopeLookAt &&
         slope_down_look_at <= kMaximumSlopeLookAt &&
         std::isfinite(slope_down_rest) &&
         slope_down_rest >= kMinimumSlopeRest &&
         slope_down_rest <= kMaximumSlopeRest && preset <= 1U;
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
  const bool slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  const double slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  const double slope_max_gradient_degrees =
      context.slope_max_gradient_degrees.load(std::memory_order_acquire);
  const double slope_up_look_at =
      context.slope_up_look_at.load(std::memory_order_acquire);
  const double slope_up_rest =
      context.slope_up_rest.load(std::memory_order_acquire);
  const double slope_down_look_at =
      context.slope_down_look_at.load(std::memory_order_acquire);
  const double slope_down_rest =
      context.slope_down_rest.load(std::memory_order_acquire);
  const bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  const bool fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  const bool diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  const bool diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  const double borrow_strength =
      context.borrow_strength.load(std::memory_order_acquire);
  const std::uint32_t anchor_source =
      context.anchor_source.load(std::memory_order_acquire);
  const bool diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  const bool diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
  if (!ConfigReady(context.config) ||
      !SettingsValid(preset, camera_scale, origin_offset_cm, aspect_ratio,
                     speed_distance_scale, roll_scale, pitch_scale,
                     shake_scale, slope_strength,
                     slope_max_gradient_degrees, slope_up_look_at,
                     slope_up_rest, slope_down_look_at,
                     slope_down_rest) ||
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
      AppendText(cursor, end, ",\"slopeFollow\":") &&
      AppendText(cursor, end, slope_follow ? "true" : "false") &&
      AppendText(cursor, end, ",\"slopeStrength\":") &&
      AppendNumber(cursor, end, slope_strength) &&
      AppendText(cursor, end, ",\"slopeMaxGradientDegrees\":") &&
      AppendNumber(cursor, end, slope_max_gradient_degrees) &&
      AppendText(cursor, end, ",\"slopeUpLookAt\":") &&
      AppendNumber(cursor, end, slope_up_look_at) &&
      AppendText(cursor, end, ",\"slopeUpRest\":") &&
      AppendNumber(cursor, end, slope_up_rest) &&
      AppendText(cursor, end, ",\"slopeDownLookAt\":") &&
      AppendNumber(cursor, end, slope_down_look_at) &&
      AppendText(cursor, end, ",\"slopeDownRest\":") &&
      AppendNumber(cursor, end, slope_down_rest) &&
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
      AppendText(cursor, end, ",\"borrowStrength\":") &&
      AppendNumber(cursor, end, borrow_strength) &&
      AppendText(cursor, end, ",\"anchorSource\":") &&
      AppendNumber(cursor, end, static_cast<double>(anchor_source)) &&
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
  context.slope_follow.store(true, std::memory_order_release);
  context.slope_strength.store(kDefaultSlopeStrength,
                               std::memory_order_release);
  context.slope_max_gradient_degrees.store(
      kDefaultSlopeMaxGradientDegrees, std::memory_order_release);
  context.slope_up_look_at.store(kDefaultSlopeUpLookAt,
                                 std::memory_order_release);
  context.slope_up_rest.store(kDefaultSlopeUpRest,
                              std::memory_order_release);
  context.slope_down_look_at.store(kDefaultSlopeDownLookAt,
                                   std::memory_order_release);
  context.slope_down_rest.store(kDefaultSlopeDownRest,
                                std::memory_order_release);
  context.roll_only_in_drift.store(false, std::memory_order_release);
  context.diagnostic_flat_pose.store(false, std::memory_order_release);
  context.diagnostic_zero_lag.store(false, std::memory_order_release);
  context.borrow_strength.store(1.0, std::memory_order_release);
  context.anchor_source.store(kAnchorSourceDefault, std::memory_order_release);
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
  // The schema has 24 properties today. Keep headroom so a future optional
  // field does not make existing documents unreadable.
  if (json.kind(json.user, root, &kind).code != ANOMALY_STATUS_V1_OK ||
      kind != ANOMALY_JSON_V1_OBJECT ||
      json.object_size(json.user, root, &field_count).code !=
          ANOMALY_STATUS_V1_OK ||
      field_count < 6 || field_count > 32) {
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
  double borrow_strength = 1.0;
  double anchor_source_number = static_cast<double>(kAnchorSourceDefault);
  bool diagnostic_skip_position = false;
  bool diagnostic_skip_rotation = false;
  bool slope_follow = true;
  double slope_strength = kDefaultSlopeStrength;
  double slope_max_gradient_degrees = kDefaultSlopeMaxGradientDegrees;
  double slope_up_look_at = kDefaultSlopeUpLookAt;
  double slope_up_rest = kDefaultSlopeUpRest;
  double slope_down_look_at = kDefaultSlopeDownLookAt;
  double slope_down_rest = kDefaultSlopeDownRest;
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
  static_cast<void>(JsonObjectNumber(json, root, "borrowStrength",
                                     borrow_strength));
  static_cast<void>(JsonObjectNumber(json, root, "anchorSource",
                                     anchor_source_number));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticZeroLag",
                                      diagnostic_zero_lag));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticSkipPosition",
                                      diagnostic_skip_position));
  static_cast<void>(JsonObjectBoolean(json, root, "diagnosticSkipRotation",
                                      diagnostic_skip_rotation));
  static_cast<void>(JsonObjectBoolean(json, root, "slopeFollow",
                                      slope_follow));
  static_cast<void>(JsonObjectNumber(json, root, "slopeStrength",
                                     slope_strength));
  static_cast<void>(JsonObjectNumber(json, root, "slopeMaxGradientDegrees",
                                     slope_max_gradient_degrees));
  static_cast<void>(JsonObjectNumber(json, root, "slopeUpLookAt",
                                     slope_up_look_at));
  static_cast<void>(JsonObjectNumber(json, root, "slopeUpRest",
                                     slope_up_rest));
  static_cast<void>(JsonObjectNumber(json, root, "slopeDownLookAt",
                                     slope_down_look_at));
  static_cast<void>(JsonObjectNumber(json, root, "slopeDownRest",
                                     slope_down_rest));
  if (!SettingsValid(preset, camera_scale, origin_offset_cm, aspect_ratio,
                     speed_distance_scale, roll_scale, pitch_scale,
                     shake_scale, slope_strength,
                     slope_max_gradient_degrees, slope_up_look_at,
                     slope_up_rest, slope_down_look_at,
                     slope_down_rest)) {
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
  context.slope_follow.store(slope_follow, std::memory_order_release);
  context.slope_strength.store(slope_strength, std::memory_order_release);
  context.slope_max_gradient_degrees.store(
      slope_max_gradient_degrees, std::memory_order_release);
  context.slope_up_look_at.store(slope_up_look_at,
                                 std::memory_order_release);
  context.slope_up_rest.store(slope_up_rest, std::memory_order_release);
  context.slope_down_look_at.store(slope_down_look_at,
                                   std::memory_order_release);
  context.slope_down_rest.store(slope_down_rest, std::memory_order_release);
  context.roll_only_in_drift.store(roll_only_in_drift,
                                   std::memory_order_release);
  context.diagnostic_flat_pose.store(diagnostic_flat_pose,
                                     std::memory_order_release);
  context.borrow_strength.store(
      nfs2015_chase::Clamp(borrow_strength, 0.0, 1.0),
      std::memory_order_release);
  context.anchor_source.store(
      std::isfinite(anchor_source_number) && anchor_source_number >= 0.0 &&
              anchor_source_number <= 3.0 &&
              std::floor(anchor_source_number) == anchor_source_number
          ? static_cast<std::uint32_t>(anchor_source_number)
          : kAnchorSourceDefault,
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
                      const double *game_eye_cm, double location[3],
                      double rotation[3], float *lens, bool *fov_enabled,
                      bool *advanced) noexcept;

void RequestPresenterReset(Context &context) noexcept;

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
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
      // Task group 0 probe: the render thread's own read of the host's player
      // chain. Deliberately before the camera work, and independent of it.
      SampleRenderXPhase(*context);
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
        // The Game's own position is still in `location` here: the original
        // ran before us and we have not written yet.
        const double game_eye_cm[3] = {location[0], location[1],
                                       location[2]};
        bool advanced = false;
        if (AdvancePresenter(*context, render_now_ns, game_eye_cm,
                             present_location, present_rotation,
                             &present_lens, &present_fov_enabled, &advanced)) {
          // T3.4e: how fast the eye moves *relative to the car*, which is the
          // only thing that can look like twitching. The absolute eye speed is
          // dominated by the car's own travel, so it can never separate a
          // smooth camera from a juddering one. Sample only when this call
          // advanced the model: later passes of the same frame reuse the cached
          // pose, and dividing its step by a pass-length dt inflates the peak
          // by the whole frame motion (PLAN-camera-timing.md §4.1).
          if (advanced) {
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
            // `AdvancePresenter` applies the Game-camera borrow inside its
            // lock while producing the pose; when it returned a cached pose
            // (same frame, or another thread won the lock) the borrow is
            // already reflected there.
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

  // Same rule as the transform detour: release a lease that was actually
  // acquired, even while the host is already stopping the hook service.
  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
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
  // The camera model is anchored to the rigid Pawn root. The mesh bounds
  // centre and extent move with body roll and pitch; anchoring the camera to
  // them made the eye inherit that motion 1:1, which reads as twitch. Keep the
  // bounds fields only for the probe, never for the model origin.
  origin_cm[0] = base_cm[0];
  origin_cm[1] = base_cm[1];
  origin_cm[2] = base_cm[2] + origin_offset_cm;
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

double WrappedAngle(const double angle) noexcept {
  return nfs2015_chase::WrapPi(angle);
}

// A world change, teleport, or model reset must not carry the old slope's
// pitch into the new frame. The floor also rejects a still-fresh snapshot that
// was published before the reset.
void ResetRenderPitch(Context &context) noexcept {
  context.render_pitch_valid = false;
  context.render_pitch_radians = 0.0;
  context.render_pitch_ns = 0;
  context.render_pitch_floor_ns = NowNanoseconds();
  context.render_pitch_proxy = 0;
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
  context.reversing = false;
  context.speed_sign_known = true;
  context.last_vehicle_parent = 0;
  context.last_vehicle_speed_mps = -1.0;
  context.last_vehicle_speed_time = 0.0;
  context.last_speed_exact = false;
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

void UpdateBrakeInput(Context &context, const bool speed_valid,
                      const bool speed_fresh, const bool derived,
                      const double speed_kmh, const double dt) noexcept {
  const double input_smoothing =
      derived ? kDerivedBrakingInputSmoothing : kBrakingInputSmoothing;
  const double deceleration_smoothing =
      derived ? kDerivedBrakingDecelerationSmoothing
              : kBrakingDecelerationSmoothing;
  if (!speed_valid) {
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
        context.vehicle_acceleration_mps2, 0.0, deceleration_smoothing, dt);
    return;
  }
  if (!speed_fresh) {
    // The motion sample is held between host snapshots, so there is no new
    // speed delta to differentiate. Differentiating it at tick rate would read
    // as zero acceleration on the held ticks and a spike on the sampled one,
    // i.e. a sawtooth in the braking pitch.
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input,
        context.elapsed <= context.brake_hold_until
            ? context.brake_last_target
            : 0.0,
        input_smoothing, dt);
    return;
  }
  // Braking pitch follows speed magnitude: reversing decelerates too, and the
  // camera model already receives the signed speed separately.
  const double speed_mps = std::fabs(speed_kmh) / 3.6;
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
  if (derived && elapsed > kBrakingMaxSampleIntervalSeconds) {
    // Only the differentiated position is invalid across a hitch; the exact
    // speed getter is a direct sample and can span a longer poll gap.
    context.brake_deceleration_mps2 = 0.0;
    context.brake_input = nfs2015_chase::Damp(
        context.brake_input, 0.0, input_smoothing, dt);
    return;
  }
  const double raw_acceleration = (speed_mps - previous_mps) / elapsed;
  context.vehicle_acceleration_mps2 = nfs2015_chase::Damp(
      context.vehicle_acceleration_mps2,
      nfs2015_chase::Clamp(raw_acceleration, -30.0, 30.0),
      deceleration_smoothing, elapsed);
  const double deceleration =
      nfs2015_chase::Clamp(-raw_acceleration, 0.0, 30.0);
  context.brake_deceleration_mps2 = nfs2015_chase::Damp(
      context.brake_deceleration_mps2, deceleration,
      deceleration_smoothing, elapsed);
  const double deadzone =
      derived ? kDerivedBrakingPitchDeadzoneMps2
              : kBrakingPitchDeadzoneMps2;
  const double excess =
      (std::max)(0.0, context.brake_deceleration_mps2 - deadzone);
  const double target = nfs2015_chase::Clamp(
      excess / (kBrakingPitchFullMps2 - deadzone), 0.0, 1.0);
  if (target > 0.05) {
    context.brake_hold_until = context.elapsed + kBrakingHoldSeconds;
  }
  const double gated =
      context.elapsed <= context.brake_hold_until ? target : 0.0;
  context.brake_last_target = gated;
  context.brake_input = nfs2015_chase::Damp(
      context.brake_input, gated, input_smoothing, dt);
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
          ResetRenderPitch(context);
          // A teleport is not a brake. Clear the derived-speed history so the
          // next sample cannot turn the jump to zero into a full pitch input.
          context.last_vehicle_speed_mps = -1.0;
          context.last_vehicle_speed_time = context.elapsed;
          context.brake_deceleration_mps2 = 0.0;
          context.brake_input = 0.0;
          context.brake_hold_until = 0.0;
          context.brake_last_target = 0.0;
          context.vehicle_acceleration_mps2 = 0.0;
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
                    const double velocity_mps, const bool have_velocity,
                    const bool exact_speed,
                    const double signed_speed_kmh) noexcept {
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
  context.speed_sign_known = exact_speed || have_camera_yaw;
  if (exact_speed) {
    // GetForwardSpeedKmH is signed, so reverse is stateless again exactly as
    // it was in 2.6. The derived-speed hysteresis is only the fallback.
    context.reversing = signed_speed_kmh < -kReverseSpeedKmh;
  }
  if (!have_camera_yaw) {
    NoteYawSource(context, kYawSourceNoObservation);
    if (!have_velocity) {
      return context.have_last_yaw ? context.last_car_yaw : 0.0;
    }
    // Without a camera reference the sign is unknown. Reuse the last known
    // reverse state rather than flipping the camera to the nose in one tick.
    return context.reversing
               ? WrappedAngle(velocity_yaw + nfs2015_chase::kPi)
               : velocity_yaw;
  }
  if (!have_velocity) {
    // Too slow for the travel direction to mean anything: the camera is the
    // only continuous reference left.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
  // Reversing travels along the nose plus 180 degrees, so the camera stays
  // aligned with the nose instead of ending up in front of the car. When the
  // exact signed speed is unavailable, infer the sign from the angle between
  // travel and the game-camera yaw. The two thresholds form a hysteresis band
  // so a drift or a camera swing cannot chatter the state; below walking speed
  // the last decision is held.
  const double alignment =
      std::cos(WrappedAngle(velocity_yaw - camera_yaw));
  if (!exact_speed && velocity_mps >= kReverseDecisionMinSpeedMps) {
    if (context.reversing) {
      if (alignment > kReverseExitCosine) {
        context.reversing = false;
      }
    } else if (alignment < kReverseEnterCosine) {
      context.reversing = true;
    }
  }
  const bool reversing = context.reversing;
  if (!reversing && velocity_mps < kReverseDecisionMinSpeedMps &&
      alignment < kReverseEnterCosine) {
    // Below the reverse decision speed the travel direction is too noisy to
    // commit to a gear. The camera still points along the nose, so keep it
    // instead of flipping the rig to the noisy velocity vector.
    NoteYawSource(context, kYawSourceObserved);
    return camera_yaw;
  }
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
  ResetRenderPitch(context);
  context.chase_initialized = false;
  ++context.sample_epoch;
}

// The model owns its state; other render threads must read a consistent pose.
void PublishPresenterPose(Context &context, const double location[3],
                          const double rotation[3], const double car_cm[3],
                          const float lens, const bool fov_enabled) noexcept {
  const std::uint64_t sequence =
      context.presenter_pose_sequence.load(std::memory_order_relaxed);
  context.presenter_pose_sequence.store(sequence + 1U,
                                        std::memory_order_release);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    context.presenter_pose_location[axis].store(location[axis],
                                                std::memory_order_relaxed);
    context.presenter_pose_rotation[axis].store(rotation[axis],
                                               std::memory_order_relaxed);
    context.presenter_car_cm[axis].store(car_cm[axis],
                                        std::memory_order_relaxed);
  }
  context.presenter_origin_offset_cm.store(
      context.origin_offset_cm.load(std::memory_order_acquire),
      std::memory_order_relaxed);
  context.presenter_pose_lens.store(lens, std::memory_order_relaxed);
  context.presenter_pose_fov.store(fov_enabled, std::memory_order_relaxed);
  context.presenter_pose_valid.store(true, std::memory_order_relaxed);
  context.presenter_pose_sequence.store(sequence + 2U,
                                        std::memory_order_release);
}

[[nodiscard]] bool ReadPresenterPose(const Context &context, double location[3],
                                     double rotation[3], float *lens,
                                     bool *fov_enabled,
                                     double car_cm[3]) noexcept {
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
      if (car_cm != nullptr) {
        car_cm[axis] = context.presenter_car_cm[axis].load(
            std::memory_order_relaxed);
      }
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

void PublishRouteBModelPose(Context &context,
                            const nfs2015_chase::CarSample &sample,
                            const nfs2015_chase::CameraState &view,
                            const std::int64_t now_ns) noexcept {
  RenderXProbe &probe = context.render_x;
  probe.route_b_model_sequence.fetch_add(1, std::memory_order_acq_rel);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double model_car =
        axis == 0 ? sample.car.position.x
                  : (axis == 1 ? sample.car.position.y
                               : sample.car.position.z);
    const double model_eye =
        axis == 0 ? view.position.x
                  : (axis == 1 ? view.position.y : view.position.z);
    const double model_rotation =
        axis == 0 ? view.pitch_degrees
                  : (axis == 1 ? view.yaw_degrees : view.roll_degrees);
    probe.route_b_model_car_cm[axis].store(model_car * 100.0,
                                           std::memory_order_relaxed);
    probe.route_b_model_eye_cm[axis].store(model_eye * 100.0,
                                           std::memory_order_relaxed);
    probe.route_b_model_rotation[axis].store(model_rotation,
                                             std::memory_order_relaxed);
  }
  probe.route_b_model_camera_scale.store(
      nfs2015_chase::Clamp(sample.camera_scale, kMinimumCameraScale,
                           kMaximumCameraScale),
      std::memory_order_relaxed);
  probe.route_b_model_origin_offset_cm.store(
      context.origin_offset_cm.load(std::memory_order_acquire),
      std::memory_order_relaxed);
  probe.route_b_model_lens.store(
      sample.fov_enabled
          ? static_cast<float>(
                view.HorizontalFovDegrees(sample.aspect_ratio))
          : 0.0F,
      std::memory_order_relaxed);
  probe.route_b_model_ns.store(now_ns, std::memory_order_relaxed);
  probe.route_b_model_valid.store(true, std::memory_order_relaxed);
  probe.route_b_model_sequence.fetch_add(1, std::memory_order_release);
}

// Advance the model to `now_ns` and produce the pose for this render call.
// Returns false when there is nothing to present (no sample yet, or another
// thread is advancing and no cached pose exists).
bool AdvancePresenter(Context &context, const std::int64_t now_ns,
                      const double *game_eye_cm, double location[3],
                      double rotation[3], float *lens, bool *fov_enabled,
                      bool *advanced) noexcept {
  *advanced = false;
  if (!TryLockPresenter(context)) {
    return ReadPresenterPose(context, location, rotation, lens, fov_enabled);
  }
  struct Unlock final {
    Context &context;
    ~Unlock() { UnlockPresenter(context); }
  } unlock{context};

  // The detour runs once per render pass, and one frame has many passes. If
  // every pass advanced the model, the model time would be the pass-to-pass
  // interval (a noisy fraction of a millisecond), not the frame interval.
  // Advance once per frame and hand every later pass of that frame the cached
  // pose. The threshold is smaller than the shortest frame at 60 fps
  // (16.7 ms) and larger than the longest observed within-frame pass gap.
  if (context.presenter_pose_valid.load(std::memory_order_relaxed)) {
    const std::int64_t since = now_ns - context.presenter_pose_ns;
    if (since >= -kPresenterSameFrameNs && since < kPresenterSameFrameNs) {
      return ReadPresenterPose(context, location, rotation, lens, fov_enabled,
                              nullptr);
    }
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

  // A bumped epoch means the model must restart (world change, teleport,
  // preset switch). Restart the offset estimate too, so it initialises from
  // the current frame instead of converging over a third of a second.
  if (context.presenter_initialized && context.presenter_epoch != sample.epoch) {
    context.game_offset_valid = false;
    context.game_offset_last_ns = 0;
  }

  // Anchor the model to the car position produced at this detour instant.
  // Four sources, selected by `anchor_source`:
  //   X       the player root+280 read the same detour took (task group 1),
  //   Vehicle the player capsule's AttachParent (VehicleMesh) scene proxy
  //           LocalToWorld, with the component transform as fallback,
  //   Borrow  the old game-camera estimate,
  //   Model   the untouched Game-tick sample.
  // X only enters the anchor; velocity and yaw still come from the Game tick.
  if (game_eye_cm != nullptr) {
    const double our_car_cm[3] = {
        sample.car.position.x * 100.0, sample.car.position.y * 100.0,
        sample.car.position.z * 100.0};
    const double borrow_strength =
        context.borrow_strength.load(std::memory_order_relaxed);
    const std::uint32_t anchor_source =
        context.anchor_source.load(std::memory_order_relaxed);
    double raw_est_cm[3]{};

    bool x_valid = false;
    if (anchor_source == kAnchorSourceX ||
        anchor_source == kAnchorSourceVehicle) {
      x_valid =
          context.render_x.anchor_enabled.load(std::memory_order_acquire) &&
          context.render_x.anchor_valid.load(std::memory_order_acquire);
      if (x_valid) {
        const std::int64_t age =
            now_ns - context.render_x.anchor_ns.load(std::memory_order_relaxed);
        x_valid = age >= 0 && age <= kAnchorSnapshotMaxAgeNs;
      }
    }
    bool vehicle_valid = false;
    if (anchor_source == kAnchorSourceVehicle) {
      vehicle_valid =
          context.render_x.anchor_enabled.load(std::memory_order_acquire) &&
          context.render_x.vehicle_anchor_valid.load(
              std::memory_order_acquire);
      if (vehicle_valid) {
        const std::int64_t age =
            now_ns - context.render_x.vehicle_anchor_ns.load(
                         std::memory_order_relaxed);
        vehicle_valid = age >= 0 && age <= kAnchorSnapshotMaxAgeNs;
      }
      if (!vehicle_valid) {
        context.render_x.vehicle_fallbacks.fetch_add(
            1, std::memory_order_relaxed);
      }
    }

    if (anchor_source == kAnchorSourceVehicle && vehicle_valid) {
      // The vehicle source reads the capsule's AttachParent (the `VehicleMesh`
      // SkeletalMesh) in the same detour pass. Its ComponentToWorld is the
      // chassis transform the renderer uses; the capsule's own origin is the
      // seat, a child transform that can lag the rendered car.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] = context.render_x.vehicle_anchor_cm[axis].load(
            std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateVehicle,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceX && x_valid) {
      // X is the raw `root+280`; the model's car origin is that value plus the
      // user's `originOffsetCm`, applied on Z exactly as `QueryPlayer` does.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] =
            context.render_x.anchor_cm[axis].load(std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      // The borrow offset is not tracking in this mode. If the X path falls
      // back, the offset re-initialises from the current Game camera instead
      // of resuming a stale estimate.
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateX,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceVehicle && x_valid) {
      // The vehicle parent was unavailable this frame (on foot, or a read that
      // did not validate). The character X is a better fallback than borrow.
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] =
            context.render_x.anchor_cm[axis].load(std::memory_order_relaxed);
      }
      raw_est_cm[2] +=
          context.origin_offset_cm.load(std::memory_order_relaxed);
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateX,
                                       std::memory_order_relaxed);
    } else if (anchor_source == kAnchorSourceModel) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        raw_est_cm[axis] = our_car_cm[axis];
      }
      context.game_offset_valid = false;
      context.game_offset_last_ns = 0;
      context.render_x.last_used.store(kAnchorStateDisabled,
                                       std::memory_order_relaxed);
    } else {
      if (anchor_source == kAnchorSourceX ||
          anchor_source == kAnchorSourceVehicle) {
        context.render_x.anchor_fallbacks.fetch_add(
            1, std::memory_order_relaxed);
      }
      const double borrow_dt =
          context.game_offset_last_ns == 0
              ? 0.0
              : static_cast<double>(now_ns - context.game_offset_last_ns) * 1e-9;
      context.game_offset_last_ns = now_ns;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double offset = game_eye_cm[axis] - our_car_cm[axis];
        if (!context.game_offset_valid) {
          context.game_offset_cm[axis] = offset;
        } else if (borrow_dt > 1e-5 && borrow_dt < 0.2) {
          constexpr double kGameOffsetRate = 3.3;
          const double blend =
              1.0 - std::exp(-kGameOffsetRate * borrow_dt);
          context.game_offset_cm[axis] +=
              (offset - context.game_offset_cm[axis]) * blend;
        }
        raw_est_cm[axis] =
            game_eye_cm[axis] - context.game_offset_cm[axis];
      }
      context.game_offset_valid = true;
      context.render_x.last_used.store(kAnchorStateBorrow,
                                       std::memory_order_relaxed);
    }

    // `borrow_strength` still interpolates between the plain tick sample and
    // the selected source, so 0 is the pure model and 1 is fully anchored.
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double car_cm =
          our_car_cm[axis] +
          (raw_est_cm[axis] - our_car_cm[axis]) * borrow_strength;
      if (axis == 0) {
        sample.car.position.x = car_cm / 100.0;
      } else if (axis == 1) {
        sample.car.position.y = car_cm / 100.0;
      } else {
        sample.car.position.z = car_cm / 100.0;
      }
      sample.anchor_cm[axis] = car_cm;
      sample.origin_cm[axis] = car_cm;
    }
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
  PublishRouteBModelPose(context, sample, view, now_ns);

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
  const double car_cm[3] = {
      sample.car.position.x * 100.0,
      sample.car.position.y * 100.0,
      sample.car.position.z * 100.0,
  };
  PublishPresenterPose(context, location, rotation, car_cm, *lens,
                       *fov_enabled);
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
  *advanced = true;

  return true;
}


struct VehiclePresence {
  bool known{};
  bool present{};
  std::uintptr_t object{};
  bool speed_known{};
  bool speed_fresh{};
  double speed_kmh{};
};

// The 2.6 Host's anomaly.nte.vehicle snapshot called these three reflected
// getters; the service is gone in 2.7 but the game functions are unchanged.
// A successful call that returns null is an authoritative "not driving"; a
// failed call only means "unknown" and leaves the speed fallback available.
constexpr double kVehicleQueryIntervalSeconds = 0.05;
constexpr double kVehicleGetterRetrySeconds = 2.0;
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
  const bool slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  const double slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  const double slope_max_gradient_degrees =
      context.slope_max_gradient_degrees.load(std::memory_order_acquire);
  const double slope_up_look_at =
      context.slope_up_look_at.load(std::memory_order_acquire);
  const double slope_up_rest =
      context.slope_up_rest.load(std::memory_order_acquire);
  const double slope_down_look_at =
      context.slope_down_look_at.load(std::memory_order_acquire);
  const double slope_down_rest =
      context.slope_down_rest.load(std::memory_order_acquire);
  bool roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  // The bisect switch: no attitude, no lens. Position and yaw only.
  const bool diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  const double effective_roll_scale = diagnostic_flat_pose ? 0.0 : roll_scale;
  const double effective_pitch_scale = diagnostic_flat_pose ? 0.0 : pitch_scale;
  const double effective_shake_scale = diagnostic_flat_pose ? 0.0 : shake_scale;
  const bool effective_slope_follow = !diagnostic_flat_pose && slope_follow;
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
  context.chase.SetSlopeFollow(effective_slope_follow);
  context.chase.SetSlopeStrength(slope_strength);
  context.chase.SetSlopeMaxGradientDegrees(slope_max_gradient_degrees);
  context.chase.SetSlopeGradientParams(
      {slope_up_look_at, slope_up_rest},
      {slope_down_look_at, slope_down_rest});
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
  // Task group 0 probe: cache the pawn chain and publish the snapshot the
  // render-thread read is compared with. Read-only; the camera ignores it.
  UpdateRenderXProbe(context, base_cm, player_valid);
  // The camera anchor is the Pawn root location that `QueryPlayer` already
  // fills. `base_cm` cancels out of the published world position (the detour
  // adds it back), so it only has to be stable and cheap. Resolving the
  // rendered body through `component_bounds` used to cost ~2 ms of the Game
  // tick: the host walks the class's whole reflected property list by name on
  // every call, decoding an FName twice per property and reading it through
  // ReadProcessMemory. Nothing about the camera needs that.
  // Anomaly 2.7.0 removed `anomaly.nte.vehicle`, not the game's vehicle
  // getters. Poll the same reflected functions the old Host snapshot used:
  // BP_GetCurrentDriveVehicle / BP_GetCurrentDirvingVehicle /
  // GetBoardWaterVehicle. Their answer is authoritative even when parked; the
  // AttachParent sample remains the anchor source and the fallback when the
  // generic UE5 services are unavailable.
  RenderXProbe &render_x = context.render_x;
  const VehiclePresence vehicle_query =
      enabled && player_valid ? QueryVehiclePresence(context)
                              : VehiclePresence{};
  const std::int64_t now_ns = NowNanoseconds();
  const std::int64_t vehicle_sample_ns =
      render_x.vehicle_sample_ns.load(std::memory_order_relaxed);
  const bool attach_sample_fresh =
      vehicle_sample_ns > 0 && now_ns >= vehicle_sample_ns &&
      now_ns - vehicle_sample_ns <= kVehiclePresenceFreshNs;
  const bool attach_present_fresh =
      attach_sample_fresh &&
      render_x.vehicle_present.load(std::memory_order_acquire);
  const bool presence_known = vehicle_query.known || attach_sample_fresh;
  const bool presence_now =
      vehicle_query.known ? vehicle_query.present : attach_present_fresh;
  const std::uintptr_t vehicle_parent =
      render_x.vehicle_parent.load(std::memory_order_relaxed);
  if (presence_now) {
    context.last_vehicle_present_time = context.elapsed;
  }
  const bool vehicle_recent =
      context.last_vehicle_present_time > 0.0 &&
      context.elapsed >= context.last_vehicle_present_time &&
      context.elapsed - context.last_vehicle_present_time <=
          kVehiclePresenceGraceSeconds;
  const bool vehicle_now = presence_now || vehicle_recent;
  const std::uintptr_t presence_identity =
      vehicle_query.known
          ? vehicle_query.object
          : (attach_present_fresh ? vehicle_parent : 0);

  bool identity_changed = false;
  if (player_valid && player_handle.id != 0 &&
      (player_handle.id != context.player_handle.id ||
       player_handle.generation != context.player_handle.generation)) {
    context.player_handle = player_handle;
    identity_changed = true;
  }
  if (presence_identity != 0 && context.last_vehicle_parent != 0 &&
      context.last_vehicle_parent != presence_identity) {
    identity_changed = true;
  }
  // Entering at a standstill must discard walking velocity, but a presence
  // sample that appears after the speed fallback has already taken over must
  // not reset the model mid-drive.
  const bool vehicle_entered = presence_now &&
                               !context.vehicle_present_last &&
                               !context.last_view_active;
  if (identity_changed || vehicle_entered) {
    ResetMotion(context);
  }
  if (presence_identity != 0) {
    context.last_vehicle_parent = presence_identity;
  } else if (!vehicle_now) {
    context.last_vehicle_parent = 0;
  }
  context.vehicle_present_last = vehicle_now;

  if (!enabled || !player_valid) {
    PublishInactive(context);
    ResetMotion(context);
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.reversing = false;
    context.display.player_valid = player_valid;
    context.display.speed_source = kSpeedSourceUnavailable;
    context.display.speed_sign_known = false;
    return;
  }

  // Keep the motion history alive while the camera is inactive: the fallback
  // speed gate is built from consecutive positions, and resetting every idle
  // tick would make it impossible for that speed to ever rise above zero.
  const bool had_motion = context.have_last_position;
  const std::uint64_t previous_sequence = context.last_player_sequence;
  UpdateMotion(context, origin_cm, player_sequence);
  const bool player_sample_fresh =
      !had_motion || player_sequence != previous_sequence;

  double velocity_length =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const double velocity_mps = velocity_length / 100.0;
  const bool speed_plausible =
      velocity_mps >= kMinimumVehicleSpeedMps && velocity_mps <
          kMaximumPlausibleSpeedCmps / 100.0;
  // Speed is a last resort for hosts where the getter and the AttachParent
  // sample are both unavailable. It must never turn sprinting on foot into a
  // car when a presence answer exists.
  const bool driving = vehicle_now || (speed_plausible && !presence_known);
  if (!driving) {
    const bool was_active = context.last_view_active;
    PublishInactive(context);
    if (was_active) {
      ResetMotion(context);
    }
    context.vehicle_present_last = false;
    context.last_vehicle_present_time = 0.0;
    context.last_vehicle_parent = 0;
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = false;
    context.display.reversing = false;
    context.display.player_valid = true;
    context.display.speed_source = kSpeedSourceUnavailable;
    context.display.speed_sign_known = false;
    context.display.speed_kmh = 0.0;
    return;
  }

  // The exact forward speed is the 2.6 signal: use it for magnitude and
  // reverse, and keep the position direction only for the lateral component
  // (slip/roll). That is what the old service did when HAS_SPEED was set.
  const bool exact_speed = vehicle_query.present && vehicle_query.speed_known;
  const bool exact_speed_fresh = exact_speed && vehicle_query.speed_fresh;
  if (exact_speed) {
    context.speed_sign_known = true;
    const double target_cmps =
        std::fabs(vehicle_query.speed_kmh) * (100000.0 / 3600.0);
    if (velocity_length > 50.0) {
      const double scale = target_cmps / velocity_length;
      if (scale > 0.5 && scale < 2.0) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          context.velocity_cmps[axis] *= scale;
        }
        velocity_length =
            std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
      }
    }
  }
  const double velocity_yaw =
      velocity_length > 1.0
          ? std::atan2(context.velocity_cmps[1], context.velocity_cmps[0])
          : (context.have_last_yaw ? context.last_car_yaw : 0.0);
  double car_yaw =
      SelectCarYaw(context, velocity_yaw, velocity_length / 100.0,
                   velocity_length > 50.0, exact_speed,
                   vehicle_query.speed_kmh);
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
  // Experiment (T3.4): the slip angle from the position derivative. The
  // signed speed above comes from the same derivative, so the reverse gate and
  // the model see one consistent sign.
  const double slip_position_degrees =
      slip_angle * kRadiansToDegrees;
  context.diag.slip_from_position_degrees.store(
      slip_position_degrees, std::memory_order_relaxed);
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

  if (exact_speed != context.last_speed_exact) {
    // Do not differentiate an exact-speed sample against a position-derived
    // one (or the reverse); that step would read as a phantom brake.
    context.last_vehicle_speed_mps = -1.0;
    context.last_vehicle_speed_time = context.elapsed;
    context.brake_deceleration_mps2 = 0.0;
    context.last_speed_exact = exact_speed;
  }
  // Exact speed is fresh when the 20 Hz vehicle poll dispatched. If it is not
  // available (water vehicle or a failed getter), keep the position-derived
  // fallback, which is fresh when the player sequence advanced.
  UpdateBrakeInput(context, true,
                   exact_speed ? exact_speed_fresh : player_sample_fresh,
                   !exact_speed,
                   exact_speed ? vehicle_query.speed_kmh : speed_mps * 3.6,
                   dt);

  nfs2015_chase::CarState car;
  car.position = {origin_cm[0] / 100.0, origin_cm[1] / 100.0,
                  origin_cm[2] / 100.0};
  car.yaw = car_yaw;
  // The Game-tick snapshot has no body attitude; the rendered vehicle proxy is
  // the only source for the slope frame. A fresh pitch is held briefly across
  // a missed render frame and zeroed after a reset or teleport.
  car.pitch = UpdateRenderPitch(context);
  car.velocity = world_velocity;
  car.yaw_rate = context.yaw_rate;
  nfs2015_chase::Vec3 acceleration{
      context.acceleration_cmps2[0] / 100.0,
      context.acceleration_cmps2[1] / 100.0,
      context.acceleration_cmps2[2] / 100.0};
  if (exact_speed) {
    // Restore the 2.6 split: exact longitudinal acceleration, position-derived
    // lateral acceleration. The longitudinal derivative of position is the
    // noisy part that pumped the dolly/pitch filters.
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
  {
    std::scoped_lock lock(context.display_mutex);
    context.display.enabled = enabled;
    context.display.driving = true;
    context.display.reversing = context.reversing;
    context.display.player_valid = true;
    context.display.speed_source =
        exact_speed ? kSpeedSourceExact : kSpeedSourceDerived;
    context.display.speed_sign_known = context.speed_sign_known;
    context.display.speed_kmh =
        exact_speed ? std::fabs(vehicle_query.speed_kmh) : speed_mps * 3.6;
    context.display.slip_degrees = slip_angle * kRadiansToDegrees;
    context.display.drift =
        context.presenter_drift.load(std::memory_order_relaxed);
    const float presenter_lens =
        context.presenter_pose_lens.load(std::memory_order_relaxed);
    context.display.fov_horizontal =
        fov_enabled && presenter_lens > 1.0F && presenter_lens < 179.0F
            ? static_cast<double>(presenter_lens)
            : 0.0;
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
  if (!context->started.load(std::memory_order_acquire)) {
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
  context.display.player_sequence_jumps =
      diag.player_sequence_jumps.load(std::memory_order_relaxed);
  context.display.camera_yaw_esp_misses =
      diag.camera_yaw_esp_misses.load(std::memory_order_relaxed);
  context.display.slip_from_position_degrees =
      diag.slip_from_position_degrees.load(std::memory_order_relaxed);
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
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint64_t samples =
        render_x.sample_count.load(std::memory_order_relaxed);
    const std::uint64_t sum_dcm =
        render_x.sum_dcm.load(std::memory_order_relaxed);
    const std::uint64_t sum_dms =
        render_x.sum_dms.load(std::memory_order_relaxed);
    const std::int64_t lead_count =
        render_x.lead_count.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dcm =
        render_x.lead_sum_dcm.load(std::memory_order_relaxed);
    const std::int64_t lead_sum_dms =
        render_x.lead_sum_dms.load(std::memory_order_relaxed);
    const std::uint64_t speed_sum_cmps =
        render_x.speed_sum_cmps.load(std::memory_order_relaxed);
    context.display.render_x_state = render_x.resolve_state;
    context.display.render_x_code = render_x.resolve_code;
    context.display.render_x_failures =
        render_x.read_failures.load(std::memory_order_relaxed);
    context.display.render_x_samples = samples;
    context.display.render_x_valid = samples != 0;
    context.display.render_x_peak_cm =
        static_cast<double>(
            render_x.peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_cm =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dcm) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_peak_ms =
        static_cast<double>(
            render_x.peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_x_avg_ms =
        samples == 0 ? 0.0
                     : static_cast<double>(sum_dms) /
                           static_cast<double>(samples) / 10.0;
    context.display.render_x_speed_kmh =
        samples == 0 ? 0.0
                     : static_cast<double>(speed_sum_cmps) /
                           static_cast<double>(samples) * 0.036;
    context.display.render_x_lead_cm =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dcm) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_lead_ms =
        lead_count == 0 ? 0.0
                        : static_cast<double>(lead_sum_dms) /
                              static_cast<double>(lead_count) / 10.0;
    context.display.render_x_chain_cm = render_x.chain_max_cm;
    context.display.render_x_chain_mismatches = render_x.chain_mismatches;
    context.display.render_x_gt_samples =
        render_x.samples_on_game_thread.load(std::memory_order_relaxed);
    const std::uint64_t proxy_count =
        render_x.render_proxy_count.load(std::memory_order_relaxed);
    const std::uint64_t proxy_time_count =
        render_x.render_proxy_time_count.load(std::memory_order_relaxed);
    const std::int64_t proxy_lead_count =
        render_x.render_proxy_lead_count.load(std::memory_order_relaxed);
    const std::uint64_t age_count =
        render_x.render_age_count.load(std::memory_order_relaxed);
    context.display.render_proxy_valid = proxy_count != 0;
    context.display.render_proxy_peak_cm =
        static_cast<double>(
            render_x.render_proxy_peak_dcm.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_cm =
        proxy_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dcm.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_count) / 10.0;
    context.display.render_proxy_peak_ms =
        static_cast<double>(
            render_x.render_proxy_peak_dms.load(std::memory_order_relaxed)) /
        10.0;
    context.display.render_proxy_avg_ms =
        proxy_time_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_sum_dms.load(std::memory_order_relaxed)) /
                  static_cast<double>(proxy_time_count) / 10.0;
    context.display.render_proxy_lead_cm =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dcm.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_proxy_lead_ms =
        proxy_lead_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_proxy_lead_sum_dms.load(
                      std::memory_order_relaxed)) /
                  static_cast<double>(proxy_lead_count) / 10.0;
    context.display.render_age_valid = age_count != 0;
    context.display.render_age_peak_ms =
        static_cast<double>(
            render_x.render_age_peak_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.render_age_avg_ms =
        age_count == 0
            ? 0.0
            : static_cast<double>(
                  render_x.render_age_sum_us.load(std::memory_order_relaxed)) /
                  static_cast<double>(age_count) / 1000.0;
  }
  {
    const RenderXProbe &render_x = context.render_x;
    const std::uint32_t source =
        context.anchor_source.load(std::memory_order_acquire);
    const std::uint8_t insurance = render_x.insurance_state;
    const std::uint8_t last_used =
        render_x.last_used.load(std::memory_order_relaxed);
    context.display.anchor_source = source;
    context.display.anchor_fallbacks =
        render_x.anchor_fallbacks.load(std::memory_order_relaxed);
    context.display.anchor_read_failures =
        render_x.anchor_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_anchor_valid =
        render_x.vehicle_anchor_valid.load(std::memory_order_relaxed);
    context.display.vehicle_present =
        context.vehicle_query_known
            ? context.vehicle_query_present
            : render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_presence_known =
        context.vehicle_query_known ||
        render_x.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_proxy_used =
        render_x.vehicle_proxy_used.load(std::memory_order_relaxed);
    context.display.vehicle_render_used =
        render_x.vehicle_render_used.load(std::memory_order_relaxed);
    context.display.transform_hook_valid =
        context.transform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.transform_updates =
        render_x.transform_updates.load(std::memory_order_relaxed);
    context.display.view_uniform_hook_valid =
        context.view_uniform_hook_id.load(std::memory_order_relaxed) != 0;
    context.display.view_uniform_calls =
        render_x.view_uniform_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_render_calls =
        render_x.view_uniform_render_calls.load(std::memory_order_relaxed);
    context.display.view_uniform_game_calls =
        render_x.view_uniform_game_calls.load(std::memory_order_relaxed);
    context.display.render_thread_id =
        render_x.render_thread_id.load(std::memory_order_relaxed);
    const std::uint64_t view_age_count =
        render_x.view_uniform_age_count.load(std::memory_order_relaxed);
    context.display.view_uniform_age_valid = view_age_count != 0;
    context.display.view_uniform_age_avg_ms =
        view_age_count == 0
            ? 0.0
            : static_cast<double>(render_x.view_uniform_age_sum_us.load(
                  std::memory_order_relaxed)) /
                  static_cast<double>(view_age_count) / 1000.0;
    context.display.view_uniform_age_peak_ms =
        static_cast<double>(render_x.view_uniform_age_peak_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_applied =
        render_x.route_b_applied.load(std::memory_order_relaxed);
    context.display.route_b_fallbacks =
        render_x.route_b_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_witness_failures =
        render_x.route_b_witness_failures.load(std::memory_order_relaxed);
    context.display.route_b_early_applied =
        render_x.route_b_early_applied.load(std::memory_order_relaxed);
    context.display.route_b_early_fallbacks =
        render_x.route_b_early_fallbacks.load(std::memory_order_relaxed);
    context.display.route_b_last_age_ms =
        static_cast<double>(
            render_x.route_b_last_age_us.load(std::memory_order_relaxed)) /
        1000.0;
    context.display.route_b_early_last_age_ms =
        static_cast<double>(render_x.route_b_early_last_age_us.load(
            std::memory_order_relaxed)) /
        1000.0;
    context.display.prepare_calls =
        render_x.prepare_calls.load(std::memory_order_relaxed);
    context.display.prepare_thread_mismatch =
        render_x.prepare_thread_mismatch.load(std::memory_order_relaxed);
    context.display.prepare_anchor_off =
        render_x.prepare_anchor_off.load(std::memory_order_relaxed);
    context.display.prepare_no_views =
        render_x.prepare_no_views.load(std::memory_order_relaxed);
    context.display.prepare_pose_fail =
        render_x.prepare_pose_fail.load(std::memory_order_relaxed);
    context.display.prepare_fullscreen_views =
        render_x.prepare_fullscreen_views.load(std::memory_order_relaxed);
    context.display.route_b_early_direct =
        render_x.route_b_early_direct.load(std::memory_order_relaxed);
    context.display.route_b_early_held =
        render_x.route_b_early_held.load(std::memory_order_relaxed);
    context.display.vehicle_fallbacks =
        render_x.vehicle_fallbacks.load(std::memory_order_relaxed);
    context.display.vehicle_read_failures =
        render_x.vehicle_read_failures.load(std::memory_order_relaxed);
    context.display.vehicle_proxy_fallbacks =
        render_x.vehicle_proxy_fallbacks.load(std::memory_order_relaxed);
    std::uint8_t gate = kAnchorGateOff;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        gate = kAnchorGateHash;
      } else if (insurance == kAnchorInsuranceServiceMissing) {
        gate = kAnchorGateService;
      } else if (insurance == kAnchorInsuranceReadFailed) {
        gate = kAnchorGateRead;
      } else if (render_x.chain_failed) {
        gate = kAnchorGateOffset;
      } else if (render_x.resolve_state == kRenderXFailed) {
        gate = kAnchorGateResolve;
      } else if (insurance == kAnchorInsuranceReady &&
                 render_x.resolve_state == kRenderXReady) {
        gate = kAnchorGateOk;
      }
    }
    context.display.anchor_gate = gate;
    std::uint8_t effective = kAnchorStateDisabled;
    if (source == kAnchorSourceX || source == kAnchorSourceVehicle) {
      if (insurance == kAnchorInsuranceHashMismatch) {
        effective = kAnchorStateHashMismatch;
      } else if (render_x.resolve_state == kRenderXFailed) {
        effective = kAnchorStateResolveFailed;
      } else {
        effective = last_used;
      }
    } else if (source == kAnchorSourceBorrow) {
      effective = kAnchorStateBorrow;
    }
    context.display.anchor_effective = effective;
  }
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
  context.display.slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  context.display.slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  context.display.roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  context.display.diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  context.display.diagnostic_zero_lag =
      context.diagnostic_zero_lag.load(std::memory_order_acquire);
  context.display.borrow_strength =
      context.borrow_strength.load(std::memory_order_acquire);
  context.display.diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  context.display.diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
}

void RetryHookIfNeeded(Context &context) noexcept {
  const bool view_point_ready =
      context.view_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_point_pattern_unavailable;
  const bool transform_ready =
      context.transform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.transform_pattern_unavailable;
  const bool view_uniform_ready =
      context.view_uniform_hook_id.load(std::memory_order_acquire) != 0 ||
      context.view_uniform_pattern_unavailable;
  const bool early_route_b_ready =
      context.prepare_view_state_hook_id.load(std::memory_order_acquire) != 0 ||
      context.early_route_b_unavailable;
  if (view_point_ready && transform_ready && view_uniform_ready &&
      early_route_b_ready) {
    return;
  }
  if (context.hook_retry_requested.exchange(false, std::memory_order_acq_rel)) {
    context.next_hook_retry = 0.0;
  }
  if (context.elapsed < context.next_hook_retry) {
    return;
  }
  context.next_hook_retry = context.elapsed + kHookRetrySeconds;
  // The profile hash is the layout witness for the route B hooks. Patterns
  // prove the entry point exists; they cannot prove that FViewInfo or
  // FSceneRenderer still have the fields we write. If the gate is closed,
  // stay out of the game instead of guessing. The legacy view-point hook is
  // kept only when the host genuinely has no build service (2.5).
  const bool version_gate =
      context.render_x.version_gate.load(std::memory_order_acquire);
  const bool legacy_without_build =
      context.render_x.insurance_state == kAnchorInsuranceServiceMissing;
  if (!version_gate && !legacy_without_build) {
    return;
  }
  if (version_gate) {
    static_cast<void>(EnsurePrepareViewStateHook(context));
    static_cast<void>(EnsureTransformHook(context));
    static_cast<void>(EnsureViewUniformHook(context));
  }
  static_cast<void>(EnsureHook(context));
}

void LogViewUniformHookDiagnostics(Context &context) noexcept {
  if (context.view_uniform_diag_logged) {
    return;
  }
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t calls =
      probe.view_uniform_calls.load(std::memory_order_relaxed);
  const std::uint64_t render_calls =
      probe.view_uniform_render_calls.load(std::memory_order_relaxed);
  if (render_calls < 10 && calls < 200) {
    return;
  }
  const std::uint64_t age_count =
      probe.view_uniform_age_count.load(std::memory_order_relaxed);
  const double age_avg_ms =
      age_count == 0
          ? 0.0
          : static_cast<double>(
                probe.view_uniform_age_sum_us.load(std::memory_order_relaxed)) /
                static_cast<double>(age_count) / 1000.0;
  const double age_peak_ms =
      static_cast<double>(
          probe.view_uniform_age_peak_us.load(std::memory_order_relaxed)) /
      1000.0;
  char message[192]{};
  std::snprintf(
      message, sizeof(message),
      "nfs2015 chase cam: view hook calls=%llu render=%llu game=%llu "
      "transform_thread=%u age avg/peak=%.2f/%.2f ms samples=%llu",
      static_cast<unsigned long long>(calls),
      static_cast<unsigned long long>(render_calls),
      static_cast<unsigned long long>(
          probe.view_uniform_game_calls.load(std::memory_order_relaxed)),
      render_calls == 0
          ? 0U
          : probe.render_thread_id.load(std::memory_order_relaxed),
      age_avg_ms, age_peak_ms,
      static_cast<unsigned long long>(age_count));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.view_uniform_diag_logged = true;
}

void LogRouteBDiagnostics(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const std::uint64_t applied =
      probe.route_b_applied.load(std::memory_order_relaxed);
  const std::uint64_t early_applied =
      probe.route_b_early_applied.load(std::memory_order_relaxed);
  const std::uint64_t prepare_calls =
      probe.prepare_calls.load(std::memory_order_relaxed);
  if (!context.route_b_early_failure_logged && early_applied == 0 &&
      applied == 0 && prepare_calls >= 120) {
    context.route_b_early_failure_logged = true;
    char warning[320]{};
    std::snprintf(
        warning, sizeof(warning),
        "nfs2015 chase cam: route B early path has not applied yet: "
        "prep=%llu off=%llu nv=%llu pf=%llu fs=%llu dir=%llu mm=%llu fb=%llu",
        static_cast<unsigned long long>(prepare_calls),
        static_cast<unsigned long long>(
            probe.prepare_anchor_off.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_no_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_pose_fail.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_fullscreen_views.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_direct.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.prepare_thread_mismatch.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            probe.route_b_early_fallbacks.load(std::memory_order_relaxed)));
    Log(context, ANOMALY_CORE_LOG_LEVEL_V1_WARNING, warning);
  }
  if (context.route_b_diag_logged) {
    return;
  }
  if (applied + early_applied < 10) {
    return;
  }
  char message[256]{};
  std::snprintf(message, sizeof(message),
                "nfs2015 chase cam: route B early=%llu late=%llu "
                "fallbacks=%llu witness=%llu early_age=%.2f ms "
                "late_age=%.2f ms presenter=%llu direct=%llu held=%llu",
                static_cast<unsigned long long>(early_applied),
                static_cast<unsigned long long>(applied),
                static_cast<unsigned long long>(
                    probe.route_b_early_fallbacks.load(
                        std::memory_order_relaxed) +
                    probe.route_b_fallbacks.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_witness_failures.load(
                        std::memory_order_relaxed)),
                static_cast<double>(probe.route_b_early_last_age_us.load(
                    std::memory_order_relaxed)) /
                    1000.0,
                static_cast<double>(
                    probe.route_b_last_age_us.load(std::memory_order_relaxed)) /
                    1000.0,
                static_cast<unsigned long long>(
                    probe.route_b_early_presenter.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_direct.load(
                        std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    probe.route_b_early_held.load(
                        std::memory_order_relaxed)));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, message);
  context.route_b_diag_logged = true;
}

// TEMP read-only timing report. It only logs values already captured by the
// early/transform/late callbacks; it does not change any camera state.
void LogRouteBTiming(Context &context) noexcept {
  const RenderXProbe &probe = context.render_x;
  const auto early = probe.timing_early_count.load(std::memory_order_relaxed);
  if (early == 0 || context.route_b_timing_windows >= 15) {
    return;
  }
  const auto now = GetTickCount64();
  if (context.route_b_timing_last_log_ms != 0 &&
      now - context.route_b_timing_last_log_ms < 2000) {
    return;
  }
  context.route_b_timing_last_log_ms = now;
  ++context.route_b_timing_windows;

  const auto late = probe.timing_late_count.load(std::memory_order_relaxed);
  const auto car_count =
      probe.timing_car_delta_count.load(std::memory_order_relaxed);
  const auto view_count =
      probe.timing_view_delta_count.load(std::memory_order_relaxed);
  const double car_avg =
      car_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_car_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(car_count) / 10.0;
  const double car_peak =
      static_cast<double>(probe.timing_car_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;
  const double view_avg =
      view_count == 0
          ? 0.0
          : static_cast<double>(probe.timing_view_delta_sum_dmm.load(
                std::memory_order_relaxed)) /
                static_cast<double>(view_count) / 10.0;
  const double view_peak =
      static_cast<double>(probe.timing_view_delta_peak_dmm.load(
          std::memory_order_relaxed)) / 10.0;

  char line[384]{};
  std::snprintf(
      line, sizeof(line),
      "nfs2015 chase cam: timing early=%llu late=%llu "
      "transform before/between/after=%llu/%llu/%llu "
      "car_delta avg/peak=%.2f/%.2f cm view_delta avg/peak=%.2f/%.2f cm",
      static_cast<unsigned long long>(early),
      static_cast<unsigned long long>(late),
      static_cast<unsigned long long>(
          probe.timing_transform_before.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_between.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
          probe.timing_transform_after.load(std::memory_order_relaxed)),
      car_avg, car_peak, view_avg, view_peak);
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, line);

  char sample[384]{};
  std::snprintf(
      sample, sizeof(sample),
      "nfs2015 chase cam: timing sample early_car=(%.2f,%.2f,%.2f) "
      "late_car=(%.2f,%.2f,%.2f) presenter_car=(%.2f,%.2f,%.2f) "
      "early_view=(%.2f,%.2f,%.2f) late_view=(%.2f,%.2f,%.2f)",
      probe.timing_early_car[0].load(std::memory_order_relaxed),
      probe.timing_early_car[1].load(std::memory_order_relaxed),
      probe.timing_early_car[2].load(std::memory_order_relaxed),
      probe.timing_late_car[0].load(std::memory_order_relaxed),
      probe.timing_late_car[1].load(std::memory_order_relaxed),
      probe.timing_late_car[2].load(std::memory_order_relaxed),
      probe.timing_presenter_car[0].load(std::memory_order_relaxed),
      probe.timing_presenter_car[1].load(std::memory_order_relaxed),
      probe.timing_presenter_car[2].load(std::memory_order_relaxed),
      probe.timing_early_view[0].load(std::memory_order_relaxed),
      probe.timing_early_view[1].load(std::memory_order_relaxed),
      probe.timing_early_view[2].load(std::memory_order_relaxed),
      probe.timing_late_view[0].load(std::memory_order_relaxed),
      probe.timing_late_view[1].load(std::memory_order_relaxed),
      probe.timing_late_view[2].load(std::memory_order_relaxed));
  Log(context, ANOMALY_CORE_LOG_LEVEL_V1_INFO, sample);
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
  // 2.6.0 managed windows. Optional: 2.5.0 has no `anomaly.window`, in which
  // case Draw falls back to the direct `anomaly.ui` begin/end pair.
  context->window = Query<AnomalyWindowServiceV1>(
      host, ANOMALY_WINDOW_SERVICE_V1_ID, ANOMALY_WINDOW_SERVICE_V1_VERSION);
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
  // 2.6's anomaly.nte.vehicle service is gone. The getters it called are still
  // ordinary reflected UFunctions, so resolve them through the generic UE5
  // services instead of guessing from motion.
  context->objects = Query<AnomalyUe5ObjectsServiceV1>(
      host, ANOMALY_UE5_OBJECTS_SERVICE_V1_ID,
      ANOMALY_UE5_OBJECTS_SERVICE_V1_VERSION);
  context->events = Query<AnomalyUe5ProcessEventServiceV1>(
      host, ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_ID,
      ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_VERSION);
  // Optional engine access for the probe. Declared optional in the manifest, so
  // a host without `ue5.ahud` simply leaves this null and the probe reports
  // "off" instead of failing the load.
  context->ahud = Query<AnomalyUe5AhudServiceV1>(
      host, ANOMALY_UE5_AHUD_SERVICE_V1_ID, ANOMALY_UE5_AHUD_SERVICE_V1_VERSION);
  // Version insurance for the X anchor. Optional: a host without the build
  // service leaves the X path disabled and the borrow estimate in charge.
  context->build = Query<AnomalyUe5BuildServiceV1>(
      host, ANOMALY_UE5_BUILD_SERVICE_V1_ID,
      ANOMALY_UE5_BUILD_SERVICE_V1_VERSION);
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

void RegisterSettingsWindow(Context &context) noexcept {
  if (!WindowReady(context.window) || context.window_handle.id != 0) {
    return;
  }
  try {
    AnomalyWindowSpecV1 spec{};
    spec.struct_size = sizeof(spec);
    spec.flags = 0;
    spec.id = StringView(kSettingsWindowId);
    const std::string title =
        context.localizer.Text("window.title", "NFS 2015 Chase Cam");
    spec.title = StringView(title);
    spec.initial_width = 430.0F;
    spec.initial_height = 720.0F;
    spec.minimum_width = 320.0F;
    spec.minimum_height = 240.0F;
    spec.maximum_width = 900.0F;
    spec.maximum_height = 1200.0F;
    spec.default_open = 1;
    const AnomalyStatusV1 status = context.window->register_window(
        context.window->user, &spec, &context.window_handle);
    if (status.code != ANOMALY_STATUS_V1_OK ||
        context.window_handle.id == 0) {
      context.window_handle = {};
    }
  } catch (...) {
    context.window_handle = {};
  }
}

void ReleaseSettingsWindow(Context &context) noexcept {
  if (context.window_handle.id != 0 && WindowReady(context.window)) {
    static_cast<void>(context.window->release_window(
        context.window->user, context.window_handle));
  }
  context.window_handle = {};
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
  context->vehicle_present_last = false;
  context->last_vehicle_present_time = 0.0;
  context->vehicle_getter_handles = {};
  context->next_vehicle_getter_retry = {};
  context->next_vehicle_query_time = 0.0;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
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
  context->diag.player_sequence_jumps.store(0, std::memory_order_relaxed);
  context->diag.camera_yaw_esp_misses.store(0, std::memory_order_relaxed);
  context->diag.slip_from_position_degrees.store(0.0,
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
  // Task group 1: check the profile hash, then resolve `ue5.GWorld` from the
  // Lifecycle domain where signature learning is allowed. The Game tick
  // retries a failed pattern scan, but never bypasses the hash gate.
  context->render_x.Reset();
  EvaluateAnchorInsurance(*context);
  context->detour_hz = 0.0;
  context->brake_last_target = 0.0;
  static_cast<void>(RegisterToggleHotkey(*context));
  RegisterSettingsWindow(*context);
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
  // Both callbacks check `started` or `g_active` first, so dropping these two
  // before any release means the revocation below has nothing to wait for.
  // Draining a callback that is still running the model is what turns a stop
  // into a timeout, and a timeout into a quarantined plugin that the Host will
  // not load again until the process restarts.
  g_active.store(nullptr, std::memory_order_release);
  context->chase_initialized = false;
  ++context->sample_epoch;
  context->sample_mean_interval_ns.store(0.0, std::memory_order_relaxed);
  context->game_offset_valid = false;
  context->game_offset_last_ns = 0;
  context->car_ring.Clear();
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
  ReleaseProbeSubscription(*context);
  ReleaseSettingsWindow(*context);
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
  context->display.reversing = false;
  context->display.speed_source = kSpeedSourceUnavailable;
  context->display.speed_sign_known = false;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->next_vehicle_query_time = 0.0;
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
        context->render_x.ResetWindow();
        context->diag.slip_max_step_ddeg.store(0,
                                               std::memory_order_relaxed);
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    RetryHookIfNeeded(*context);
    LogViewUniformHookDiagnostics(*context);
    LogRouteBDiagnostics(*context);
    LogRouteBTiming(*context);
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

const char *SpeedSourceKey(const std::uint8_t source) noexcept {
  switch (source) {
    case kSpeedSourceDerived:
      return "speed.source.derived";
    case kSpeedSourceExact:
      return "speed.source.exact";
    default:
      return "speed.source.unavailable";
  }
}

const char *SpeedSourceFallback(const std::uint8_t source) noexcept {
  switch (source) {
    case kSpeedSourceDerived:
      return "derived";
    case kSpeedSourceExact:
      return "exact";
    default:
      return "unavailable";
  }
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
  DrawStatusLine(
      ui, localizer, "diag.speed_source", "Speed source",
      localizer.Text(SpeedSourceKey(display.speed_source),
                     SpeedSourceFallback(display.speed_source)));
  if (display.driving) {
    DrawStatusLine(ui, localizer, "diag.reverse", "Reverse",
                   display.reversing ? localizer.Text("status.yes", "yes")
                                     : localizer.Text("status.no", "no"));
    DrawStatusLine(ui, localizer, "diag.speed_sign", "Speed sign",
                   display.speed_sign_known
                       ? localizer.Text("status.known", "known")
                       : localizer.Text("status.unknown", "unknown"));
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
                 "A px peak/avg", line);
  if (display.rotation_valid) {
    std::snprintf(line, sizeof(line), "P %.2f / %.2f  R %.2f / %.2f deg",
                  display.pitch_peak_degrees, display.pitch_avg_degrees,
                  display.roll_peak_degrees, display.roll_avg_degrees);
  } else {
    std::snprintf(line, sizeof(line), "P --  R --");
  }
  DrawStatusLine(ui, localizer, "diag.rotation",
                 "P/R peak/avg deg", line);
  if (display.lens_valid) {
    std::snprintf(line, sizeof(line), "F %.2f / %.2f deg",
                  display.lens_peak_degrees, display.lens_avg_degrees);
  } else {
    std::snprintf(line, sizeof(line), "F --");
  }
  DrawStatusLine(ui, localizer, "diag.lens_residual",
                 "F peak/avg deg", line);
  std::snprintf(line, sizeof(line), "B %.2f",
                display.borrow_strength);
  DrawStatusLine(ui, localizer, "diag.borrow", "B", line);
  if (display.flow_valid) {
    std::snprintf(line, sizeof(line), "W %.2f / %.2f cm",
                  display.flow_peak_cm, display.flow_avg_cm);
  } else {
    std::snprintf(line, sizeof(line), "W --");
  }
  DrawStatusLine(ui, localizer, "diag.flow",
                 "W peak/avg cm", line);
  if (display.frame_valid) {
    std::snprintf(line, sizeof(line), "T %.1f / %.1f ms",
                  display.frame_peak_ms, display.frame_mean_ms);
  } else {
    std::snprintf(line, sizeof(line), "T --");
  }
  DrawStatusLine(ui, localizer, "diag.frame_pacing",
                 "T peak/avg ms", line);

  // Task group 0: the gate for the whole hook route. Order matters. `chain` is
  // the control (same chain, Game tick, against the host snapshot); if it is
  // not ~0 the offsets are stale and `phase` is meaningless.
  if (display.render_x_state == kRenderXReady) {
    std::snprintf(line, sizeof(line), "ok  n=%llu  gt=%llu/%llu  f=%llu",
                  static_cast<unsigned long long>(display.render_x_samples),
                  static_cast<unsigned long long>(display.render_x_gt_samples),
                  static_cast<unsigned long long>(display.render_x_samples),
                  static_cast<unsigned long long>(
                      display.render_x_failures));
  } else if (display.render_x_state == kRenderXFailed) {
    std::snprintf(line, sizeof(line), "failed %u", display.render_x_code);
  } else {
    std::snprintf(line, sizeof(line), "wait");
  }
  DrawStatusLine(ui, localizer, "diag.render_x_state", "X GWorld", line);
  std::snprintf(line, sizeof(line), "%.2f cm / %llu",
                display.render_x_chain_cm,
                static_cast<unsigned long long>(
                    display.render_x_chain_mismatches));
  DrawStatusLine(ui, localizer, "diag.render_x_chain",
                 "X chain diff / count", line);
  if (display.render_x_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f cm  %.1f / %.1f ms",
                  display.render_x_avg_cm, display.render_x_peak_cm,
                  display.render_x_avg_ms, display.render_x_peak_ms);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.render_x_phase",
                 "X phase avg/peak", line);
  std::snprintf(line, sizeof(line), "%+.1f cm %+.1f ms  %.0f km/h",
                display.render_x_lead_cm, display.render_x_lead_ms,
                display.render_x_speed_kmh);
  DrawStatusLine(ui, localizer, "diag.render_x_lead", "X along travel",
                 line);
  if (display.render_proxy_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f cm  %.1f / %.1f ms",
                  display.render_proxy_avg_cm, display.render_proxy_peak_cm,
                  display.render_proxy_avg_ms, display.render_proxy_peak_ms);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.render_proxy_phase",
                 "Render vs proxy avg/peak", line);
  if (display.render_proxy_valid) {
    std::snprintf(line, sizeof(line), "%+.1f cm %+.1f ms",
                  display.render_proxy_lead_cm, display.render_proxy_lead_ms);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.render_proxy_lead",
                 "Render along travel", line);
  if (display.render_age_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f ms",
                  display.render_age_avg_ms, display.render_age_peak_ms);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.render_age", "Render age avg/peak",
                 line);
  {
    static constexpr const char *kGateValues[7] = {
        "ok", "hash", "pattern", "service", "offset", "off", "read"};
    const std::size_t gate =
        display.anchor_gate < 7U ? display.anchor_gate : 6U;
    DrawStatusLine(ui, localizer, "diag.anchor_gate", "Version gate",
                   kGateValues[gate]);
    std::snprintf(line, sizeof(line), "%llu / %llu",
                  static_cast<unsigned long long>(
                      display.anchor_fallbacks),
                  static_cast<unsigned long long>(
                      display.anchor_read_failures));
    DrawStatusLine(ui, localizer, "diag.anchor_fallback",
                   "X fallback/read", line);
    std::string vehicle_anchor_value = localizer.Text("anchor.vehicle.off", "off");
    if (display.vehicle_anchor_valid) {
      if (display.vehicle_render_used) {
        vehicle_anchor_value =
            localizer.Text("anchor.vehicle.render", "render");
      } else {
        vehicle_anchor_value =
            display.vehicle_proxy_used
                ? localizer.Text("anchor.vehicle.proxy", "proxy")
                : localizer.Text("anchor.vehicle.c2w", "c2w");
      }
    } else if (display.vehicle_read_failures != 0) {
      vehicle_anchor_value = localizer.Text("anchor.vehicle.read", "read");
    }
    DrawStatusLine(ui, localizer, "diag.vehicle_anchor", "Car anchor",
                   vehicle_anchor_value);
    DrawStatusLine(
        ui, localizer, "diag.vehicle_present", "Vehicle present",
        display.vehicle_present
            ? localizer.Text("status.yes", "yes")
            : (display.vehicle_presence_known
                   ? localizer.Text("status.no", "no")
                   : localizer.Text("status.unknown", "unknown")));
    std::snprintf(line, sizeof(line), "%llu / %llu",
                  static_cast<unsigned long long>(
                      display.vehicle_fallbacks),
                  static_cast<unsigned long long>(
                      display.vehicle_read_failures));
    DrawStatusLine(ui, localizer, "diag.vehicle_fallback",
                   "Car fallback/read", line);
    std::snprintf(line, sizeof(line), "%s %llu",
                  display.transform_hook_valid ? "ok" : "wait",
                  static_cast<unsigned long long>(
                      display.transform_updates));
    DrawStatusLine(ui, localizer, "diag.transform_hook", "Render hook",
                   line);
    std::snprintf(line, sizeof(line), "%s R/G %llu/%llu thr %u",
                  display.view_uniform_hook_valid ? "ok" : "wait",
                  static_cast<unsigned long long>(
                      display.view_uniform_render_calls),
                  static_cast<unsigned long long>(
                      display.view_uniform_game_calls),
                  display.render_thread_id);
    DrawStatusLine(ui, localizer, "diag.view_uniform_hook",
                   "View hook R/G", line);
    if (display.view_uniform_age_valid) {
      std::snprintf(line, sizeof(line), "%.2f / %.2f ms",
                    display.view_uniform_age_avg_ms,
                    display.view_uniform_age_peak_ms);
    } else {
      std::snprintf(line, sizeof(line), "--");
    }
    DrawStatusLine(ui, localizer, "diag.view_uniform_age",
                   "View->transform age", line);
    std::snprintf(line, sizeof(line),
                  "%llu(%.2f) / %llu / %llu w %llu  prep %llu mm %llu nv "
                  "%llu pf %llu fs %llu dir %llu held %llu",
                  static_cast<unsigned long long>(
                      display.route_b_early_applied),
                  display.route_b_early_last_age_ms,
                  static_cast<unsigned long long>(
                      display.route_b_applied),
                  static_cast<unsigned long long>(
                      display.route_b_fallbacks +
                      display.route_b_early_fallbacks),
                  static_cast<unsigned long long>(
                      display.route_b_witness_failures),
                  static_cast<unsigned long long>(display.prepare_calls),
                  static_cast<unsigned long long>(
                      display.prepare_thread_mismatch),
                  static_cast<unsigned long long>(display.prepare_no_views),
                  static_cast<unsigned long long>(display.prepare_pose_fail),
                  static_cast<unsigned long long>(
                      display.prepare_fullscreen_views),
                  static_cast<unsigned long long>(
                      display.route_b_early_direct),
                  static_cast<unsigned long long>(
                      display.route_b_early_held));
    DrawStatusLine(ui, localizer, "diag.route_b",
                   "Route B early/held/fallback/witness",
                   line);
  }

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
  DrawStatusLine(ui, localizer, "diag.probe_frame_rate", "Probe fps / tick",
                 line);
  std::snprintf(line, sizeof(line), "%.1f cm", display.probe_bounds_offset_cm);
  DrawStatusLine(ui, localizer, "diag.probe_bounds_offset",
                 "Probe bounds offset", line);
  if (display.probe_bounds_yaw_valid) {
    std::snprintf(line, sizeof(line), "%.1f deg",
                  display.probe_camera_minus_bounds_degrees);
  } else {
    std::snprintf(line, sizeof(line), "--");
  }
  DrawStatusLine(ui, localizer, "diag.probe_camera_vs_bounds",
                 "Probe yaw - bounds", line);
  std::snprintf(line, sizeof(line), "%.1f / %.1f Hz  thr %llu",
                display.detour_hz, display.update_hz,
                static_cast<unsigned long long>(display.detour_thread_count));
  DrawStatusLine(ui, localizer, "diag.detour_rate", "detour / tick", line);
  std::snprintf(line, sizeof(line), "%.2f / %.2f / %.2f / %.2f ms",
                display.tick_ms_last, display.tick_ms_avg, display.tick_ms_p99,
                display.tick_ms_max);
  DrawStatusLine(ui, localizer, "diag.tick_cost", "tick ms last/avg/p99/max",
                 line);
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
                 "cam-car speed m/s", line);
  if (display.render_turn_valid) {
    std::snprintf(line, sizeof(line), "%.1f / %.1f deg/s",
                  display.render_turn_floor_dps, display.render_turn_peak_dps);
  } else {
    std::snprintf(line, sizeof(line), "-- / -- deg/s");
  }
  DrawStatusLine(ui, localizer, "diag.render_turn",
                 "cam-car turn deg/s", line);
  std::snprintf(line, sizeof(line), "%.1f cm", display.car_step_cm);
  DrawStatusLine(ui, localizer, "diag.car_step", "anchor cm/tick", line);
  if (display.judder_car_valid) {
    std::snprintf(line, sizeof(line), "C %.1f / %.1f px",
                  display.judder_car_peak_px, display.judder_car_avg_px);
  } else {
    std::snprintf(line, sizeof(line), "C --");
  }
  DrawStatusLine(ui, localizer, "diag.judder_car",
                 "C px peak/avg", line);
  if (display.tick_interval_valid) {
    std::snprintf(line, sizeof(line), "T %.1f / %.1f ms",
                  display.tick_interval_min_ms, display.tick_interval_max_ms);
  } else {
    std::snprintf(line, sizeof(line), "T --");
  }
  DrawStatusLine(ui, localizer, "diag.tick_interval",
                 "tick interval ms", line);
  std::snprintf(line, sizeof(line), "H %.1f cm", display.probe_height_step_cm);
  DrawStatusLine(ui, localizer, "diag.height_step",
                 "bounds radius cm/tick", line);
  std::snprintf(line, sizeof(line), "%.2f / %.2f deg",
                display.probe_camera_yaw_step_max_degrees,
                display.probe_camera_yaw_step_avg_degrees);
  DrawStatusLine(ui, localizer, "diag.probe_camera_yaw_step",
                 "cam yaw step deg", line);
  DrawCountLine(ui, localizer, "diag.snap", "view snaps",
                display.snap_total);
  static constexpr const char *kSnapKeys[kSnapReasonCount] = {
      "diag.snap_motion", "diag.snap_teleport", "diag.snap_inactive",
      "diag.snap_preset", "diag.snap_user_reset"};
  static constexpr const char *kSnapFallbacks[kSnapReasonCount] = {
      "  motion", "  teleport", "  idle", "  preset", "  user"};
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    if (display.snap_by_reason[index] == 0) {
      continue;
    }
    DrawCountLine(ui, localizer, kSnapKeys[index], kSnapFallbacks[index],
                  display.snap_by_reason[index]);
  }
  DrawCountLine(ui, localizer, "diag.yaw_switches",
                "yaw source switches", display.yaw_source_switches);
  DrawCountLine(ui, localizer, "diag.observed_failures",
                "pose read failures", display.observed_read_failures);
  DrawCountLine(ui, localizer, "diag.lease_failures", "hook lease failures",
                display.lease_failures);
  DrawCountLine(ui, localizer, "diag.torn_reads", "torn reads",
                display.view_read_torn);
  DrawCountLine(ui, localizer, "diag.detour_missing",
                "detour no pose", display.detour_no_publish);
  DrawCountLine(ui, localizer, "diag.camera_yaw_fallback",
                "ticks no car yaw", display.camera_yaw_esp_misses);
  std::snprintf(line, sizeof(line), "%.1f deg",
                display.slip_from_position_degrees);
  DrawStatusLine(ui, localizer, "diag.slip_position", "slip pos deg", line);
  DrawCountLine(ui, localizer, "diag.slip_flips",
                "slip sign flips", display.slip_sign_changes);
  // The thresholds are printed next to the values so the panel can be read
  // without knowing what "large" means for each number.
  std::snprintf(line, sizeof(line), "%.1f deg",
                display.slip_max_step_degrees);
  DrawStatusLine(ui, localizer, "diag.slip_step", "slip step deg/tick", line);
  DrawCountLine(ui, localizer, "diag.sequence_jumps",
                "player seq jumps", display.player_sequence_jumps);
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
  bool managed_window = false;
  try {
    DisplayState display;
    {
      std::scoped_lock lock(context->display_mutex);
      display = context->display;
    }

    int visible = 0;
    if (WindowReady(context->window) && context->window_handle.id != 0) {
      // The host drives the managed window group from the plugin catalog. When
      // the group is closed, `begin` returns OK with visible=0 without opening
      // a host window; calling `end` then would unbalance the host UI stack.
      // Read the group state first, as the built-in plugins do.
      AnomalyWindowStateV1 state{};
      state.struct_size = sizeof(state);
      if (context->window->state(context->window->user,
                                 context->window_handle, &state)
              .code != ANOMALY_STATUS_V1_OK ||
          state.open == 0) {
        return;
      }
      const AnomalyStatusV1 status = context->window->begin(
          context->window->user, context->window_handle, 0, &visible);
      if (status.code != ANOMALY_STATUS_V1_OK) {
        return;
      }
      managed_window = true;
      window_begun = true;
    } else {
      const std::string title =
          context->localizer.Text("window.title", "NFS 2015 Chase Cam");
      ui->set_next_window_size(ui->user, 430.0F, 0.0F, 4U);
      int open = 1;
      visible = ui->begin_window(ui->user, StringView(title), &open, 0);
      window_begun = true;
    }
    if (visible == 0) {
      if (managed_window && WindowReady(context->window)) {
        static_cast<void>(context->window->end(context->window->user,
                                               context->window_handle));
      } else {
        ui->end_window(ui->user);
      }
      return;
    }
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

      const std::string anchor_label =
          context->localizer.Text("setting.anchor_source", "Anchor source");
      ui->text(ui->user, StringView(anchor_label));
      const auto anchor_button = [&](const std::uint32_t source,
                                     const char *key, const char *fallback) {
        std::string label;
        label.append(display.anchor_source == source ? "> " : "  ");
        label.append(context->localizer.Text(key, fallback));
        if (ui->button(ui->user, StringView(label), 0.0F, 0.0F) != 0) {
          context->anchor_source.store(source, std::memory_order_release);
          MarkSettingsDirty(*context);
        }
      };
      anchor_button(kAnchorSourceX, "anchor.x", "X anchored");
      anchor_button(kAnchorSourceVehicle, "anchor.vehicle", "Vehicle mesh");
      anchor_button(kAnchorSourceBorrow, "anchor.borrow", "Borrow estimate");
      anchor_button(kAnchorSourceModel, "anchor.model", "Pure model");

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

      const std::string slope_follow_label = context->localizer.Label(
          "setting.slope_follow", "Follow slope (body pitch)",
          "chase-slope-follow");
      int slope_follow = display.slope_follow ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(slope_follow_label),
                       &slope_follow) != 0) {
        context->slope_follow.store(slope_follow != 0,
                                    std::memory_order_release);
        MarkSettingsDirty(*context);
      }

      const std::string slope_strength_label = context->localizer.Label(
          "setting.slope_strength", "Slope composition strength",
          "chase-slope-strength");
      float slope_strength = static_cast<float>(display.slope_strength);
      if (ui->slider_float(ui->user, StringView(slope_strength_label),
                           &slope_strength,
                           static_cast<float>(kMinimumSlopeStrength),
                           static_cast<float>(kMaximumSlopeStrength)) != 0) {
        context->slope_strength.store(
            nfs2015_chase::Clamp(static_cast<double>(slope_strength),
                                 kMinimumSlopeStrength,
                                 kMaximumSlopeStrength),
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
        // This is a continuous vertical trim, not a pose discontinuity. The
        // old per-frame reset restarted the presenter while the slider was
        // dragged, which made the view flash on a slope.
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

      const std::string borrow_label = context->localizer.Label(
          "setting.borrow_strength",
          "Lock to the Game's camera (0 = pure model)", "chase-borrow");
      float borrow = static_cast<float>(display.borrow_strength);
      if (ui->slider_float(ui->user, StringView(borrow_label), &borrow, 0.0F,
                           1.0F) != 0) {
        context->borrow_strength.store(
            nfs2015_chase::Clamp(static_cast<double>(borrow), 0.0, 1.0),
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
      {
        const char *state_key = "anchor.state.disabled";
        const char *state_fallback = "disabled";
        switch (display.anchor_effective) {
          case kAnchorStateX:
            state_key = "anchor.state.x";
            state_fallback = "X anchored";
            break;
          case kAnchorStateBorrow:
            state_key = "anchor.state.borrow";
            state_fallback = "borrow";
            break;
          case kAnchorStateHashMismatch:
            state_key = "anchor.state.hash_mismatch";
            state_fallback = "hash mismatch";
            break;
          case kAnchorStateResolveFailed:
            state_key = "anchor.state.resolve_failed";
            state_fallback = "pattern failed";
            break;
          case kAnchorStateVehicle:
            state_key = "anchor.state.vehicle";
            state_fallback = "vehicle";
            break;
          default:
            break;
        }
        std::string anchor_value =
            context->localizer.Text(state_key, state_fallback);
        char fallback_count[48]{};
        std::snprintf(fallback_count, sizeof(fallback_count), "  fb %llu",
                      static_cast<unsigned long long>(
                          display.anchor_fallbacks));
        anchor_value.append(fallback_count);
        DrawStatusLine(ui, context->localizer, "status.anchor", "Anchor",
                       anchor_value);
      }
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
        DrawStatusLine(
            ui, context->localizer, "status.speed_source", "Speed source",
            context->localizer.Text(
                SpeedSourceKey(display.speed_source),
                SpeedSourceFallback(display.speed_source)));
        if (display.reversing) {
          DrawStatusLine(ui, context->localizer, "status.reversing",
                         "Direction",
                         context->localizer.Text("status.reverse", "reverse"));
        }
        ui->text(ui->user, StringView(line));
      }
      if (!display.player_valid) {
        ui->text(ui->user,
                 StringView(context->localizer.Text(
                     "hint.player_unavailable",
                     "Player snapshot is unavailable; the camera stays idle.")));
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
    if (managed_window && WindowReady(context->window)) {
      static_cast<void>(context->window->end(context->window->user,
                                             context->window_handle));
    } else {
      ui->end_window(ui->user);
    }
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
      StringView("2.0.1"),
      Load,
      Start,
      Stop,
      Unload,
      Update,
      Draw};
  return anomaly::sdk::Ok();
}
