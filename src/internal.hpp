// Module-level declarations and the small helpers several modules share. This
// header grows as modules are moved out of plugin.cpp.
#pragma once

#include "sdk.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace nfs2015_chase_cam::detail {

// Window extremes, kept lock-free: the Game thread resets these between
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

// Body pitch (nose up positive) from the vehicle proxy's forward/nose vector.
// The vector may be fully normalised (z = sin pitch) or horizontally
// normalised (z = tan pitch, the historical publish format), so atan2 is the
// correct inverse for both; asin(z) would only work for the former.
inline bool BodyPitchFromForward(const double forward[3],
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

// --- ui.cpp ---------------------------------------------------------------
void ANOMALY_CALL Draw(void *plugin_context,
                       const AnomalyUiServiceV1 *ui_v1);

// --- settings.cpp ---------------------------------------------------------
void MarkSettingsDirty(Context &context) noexcept;
// Writes one group's effective values into the settings atomics. It is the
// only grouped writer of those atomics; the caller edits the custom block and
// then asks for the group to be applied again.
void ApplyGroup(Context &context, SettingsGroup group) noexcept;
void ApplyAllGroups(Context &context) noexcept;
void SetDefaults(Context &context) noexcept;
void RestoreDefaults(Context &context) noexcept;
[[nodiscard]] bool PersistSettings(Context &context) noexcept;
[[nodiscard]] bool LoadSettings(Context &context) noexcept;
void ReleaseSettingsSchema(Context &context) noexcept;

// --- ahud_probe.cpp -------------------------------------------------------
[[nodiscard]] bool ProbeAhudReady(
    const AnomalyUe5AhudServiceV1 *service) noexcept;
void ResetResidual(std::atomic<std::uint64_t> &peak,
                   std::atomic<std::uint64_t> &sum,
                   std::atomic<std::uint64_t> &count) noexcept;
void RunProbe(Context &context, double dt) noexcept;
void ReleaseProbeSubscription(Context &context) noexcept;

// --- vehicle.cpp ----------------------------------------------------------
// Shared with burnout.cpp and the tick: what the 20 Hz presence poll knows.
struct VehiclePresence {
  bool known{};
  bool present{};
  std::uintptr_t object{};
  bool speed_known{};
  bool speed_fresh{};
  double speed_kmh{};
};

[[nodiscard]] bool ObjectHandleFromPointer(
    Context &context, std::uintptr_t object,
    AnomalyGenerationHandleV1 &handle) noexcept;
[[nodiscard]] bool VehicleIsWheeled(Context &context,
                                    std::uintptr_t vehicle) noexcept;
void QueryVehicleSpeed(Context &context, std::uintptr_t vehicle,
                       VehiclePresence &presence) noexcept;
[[nodiscard]] std::uint64_t ObjectRegistryGeneration(Context &context) noexcept;
void ResetVehicleObjectCaches(Context &context) noexcept;
[[nodiscard]] bool RefreshVehicleObjectGeneration(Context &context) noexcept;
[[nodiscard]] VehiclePresence QueryVehiclePresence(Context &context) noexcept;

// --- wheel_steer.cpp ------------------------------------------------------
void RestoreWheelPose(Context &context) noexcept;
void ReleaseWheelSteer(Context &context) noexcept;
void UpdateWheelSteer(Context &context, std::uintptr_t vehicle,
                      nfs2015_chase::Vec3 world_velocity,
                      double forward_speed_mps, double speed_mps,
                      double steering_input, bool steering_known,
                      bool reversing, double dt) noexcept;

// --- burnout.cpp ----------------------------------------------------------
void UpdateBurnoutProbe(Context &context, const VehiclePresence &vehicle,
                        double speed_mps, double forward_speed) noexcept;
[[nodiscard]] nfs2015_chase::BurnoutSample
UpdateBurnoutDetection(Context &context, bool on_ground, double speed_mps,
                       double forward_speed, double dt) noexcept;

// --- render_x_probe.cpp ---------------------------------------------------
void TryResolveRenderX(Context &context) noexcept;
void EvaluateAnchorInsurance(Context &context) noexcept;
void UpdateRenderXProbe(Context &context, const double base_cm[3],
                        bool player_valid) noexcept;
void SampleRenderXPhase(Context &context) noexcept;

// --- hooks.cpp ------------------------------------------------------------
[[nodiscard]] bool ResolveSignature(Context &context,
                                    std::string_view pattern,
                                    std::uintptr_t &address) noexcept;
[[nodiscard]] bool ResolveCameraPov(Context &context) noexcept;
AnomalyStatusV1 ResolveSignatureStatus(const Context &context,
                                       std::string_view pattern,
                                       std::uintptr_t &address) noexcept;
[[nodiscard]] bool EnsurePrepareViewStateHook(Context &context) noexcept;
[[nodiscard]] bool EnsureViewUniformHook(Context &context) noexcept;
[[nodiscard]] bool EnsureTransformHook(Context &context) noexcept;
[[nodiscard]] bool EnsureHook(Context &context) noexcept;
void ReleaseHook(Context &context) noexcept;

// --- routeb.cpp -----------------------------------------------------------
[[nodiscard]] double UpdateRenderPitch(Context &context) noexcept;
[[nodiscard]] bool ReadRouteBDirectCar(const RenderXProbe &probe,
                                       double car_cm[3],
                                       double forward[3]) noexcept;
[[nodiscard]] bool ApplyRouteBEarly(Context &context, void *renderer) noexcept;
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept;

// --- presenter.cpp --------------------------------------------------------
[[nodiscard]] bool AdvancePresenter(Context &context, std::int64_t now_ns,
                                    const double *game_eye_cm,
                                    double location[3], double rotation[3],
                                    float *lens, bool *fov_enabled,
                                    bool *advanced) noexcept;
void RequestPresenterReset(Context &context) noexcept;
[[nodiscard]] bool ReadPresenterPose(const Context &context,
                                     double location[3], double rotation[3],
                                     float *lens, bool *fov_enabled,
                                     double car_cm[3] = nullptr) noexcept;

// --- camera_detour.cpp ----------------------------------------------------
void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept;

// --- game_tick.cpp --------------------------------------------------------
[[nodiscard]] bool QueryPlayer(Context &context, double origin_cm[3],
                               double base_cm[3], std::uint64_t &sequence,
                               AnomalyGenerationHandleV1 &handle) noexcept;
double WrappedAngle(double angle) noexcept;
void ResetRenderPitch(Context &context) noexcept;
[[nodiscard]] bool WorldChanged(Context &context) noexcept;
void ResetMotion(Context &context) noexcept;
void UpdateBrakeInput(Context &context, bool speed_valid, bool speed_fresh,
                      bool derived, double speed_kmh, double dt) noexcept;
void UpdateMotion(Context &context, const double origin_cm[3],
                  std::uint64_t sequence) noexcept;
double SelectCarYaw(Context &context, double velocity_yaw, double velocity_mps,
                    bool have_velocity, bool exact_speed,
                    double signed_speed_kmh) noexcept;
void PublishInactive(Context &context) noexcept;

// TG-1: the other half of PublishInactive. The chase slot is entered from two
// places (the presenter tick and the toggle detour, which must not show a frame
// of the game camera after FP -> third person), so the flags it sets live here.
void ActivateView(Context &context, bool fov_enabled) noexcept;

// Reads the live view-mode tag out of the controller the watcher resolved.
// Returns false when the controller is unknown or the read failed; callers must
// treat that as "unknown", never as "not ThirdPerson".
[[nodiscard]] bool ReadLiveViewModeTag(Context &context,
                                      std::uint64_t &tag) noexcept;

// Game-thread gate for the chase slot: the player wants it and the game is in
// ThirdPerson. Unknown tag fails open, so a host without `anomaly.ue5.names`
// keeps the 2.0.3 behaviour instead of losing the camera.
[[nodiscard]] bool ChaseSlotAllowed(Context &context) noexcept;

// Text label for a view-mode tag id ("TP" / "FP" / "Fixed" / "Seat" / hex).
// Only for the diagnostics the presenter prints; comparisons never use text.
[[nodiscard]] const char *ViewModeTagLabel(Context &context,
                                          std::uint64_t tag) noexcept;

// Render-thread guard (plan SS2.4). Reads the tag live, falls back to the
// published copy, and fails open when neither is available.
[[nodiscard]] bool ChaseCameraGuardAllows(Context &context) noexcept;

// TG-2 (plan SS2.3.2 / SS2.3.3). `NoteVehicleExit` is the one place that says
// "no vehicle any more, or the plugin is off": it drops both the chase wish and
// any pending `entryView`. The caller still lowers the overlay itself.
void NoteVehicleExit(Context &context) noexcept;

// The fixed entry rule (plan SS2.3.2) for a vehicle that was just entered, or
// for the plugin being enabled while already driving: Third -> chase camera,
// First -> leave it alone, cutscene -> retry later. Idempotent while
// `pending_entry` is set: it either applies now or leaves the flag for the
// next tick.
void ApplyEntryView(Context &context) noexcept;


// --- presenter.cpp --------------------------------------------------------
[[nodiscard]] bool TryLockPresenter(Context &context) noexcept;
void UnlockPresenter(Context &context) noexcept;
void LockPresenter(Context &context) noexcept;
void PublishPresenterPose(Context &context, const double location[3],
                          const double rotation[3], const double car_cm[3],
                          float lens, bool fov_enabled) noexcept;
void PublishRouteBModelPose(Context &context,
                            const nfs2015_chase::CarSample &sample,
                            const nfs2015_chase::CameraState &view,
                            std::int64_t now_ns) noexcept;
void UpdateCamera(Context &context, double dt) noexcept;

// --- view_mode_cycle.cpp --------------------------------------------------
// Task group 0 of PLAN-zkey-phase2-h1.md: one log-only detour on
// HTPlayerController::ToggleSwitchCameraViewMode. Every entry point here is
// fail-open: when a name, a pattern, a vtable slot or the controller is
// missing, the hook is not installed and Z stays native.
[[nodiscard]] bool EnsureViewModeTags(Context &context) noexcept;
[[nodiscard]] bool EnsureToggleViewModeHook(Context &context) noexcept;
void ReleaseToggleViewModeHook(Context &context) noexcept;
void ResetViewModeRecon(Context &context) noexcept;
void UpdateViewModeRecon(Context &context) noexcept;
std::uint8_t ANOMALY_CALL ToggleViewModeDetour(
    void *self, std::uint8_t b_forward_raw) noexcept;

// --- display.cpp ----------------------------------------------------------
void UpdateDisplay(Context &context) noexcept;
void RetryHookIfNeeded(Context &context) noexcept;
void LogViewUniformHookDiagnostics(Context &context) noexcept;
void LogRouteBDiagnostics(Context &context) noexcept;
void LogRouteBTiming(Context &context) noexcept;

// --- lifecycle.cpp (the ABI v1 surface, called through the descriptor) -----
AnomalyStatusV1 ANOMALY_CALL Load(const AnomalyHostApiV1 *host,
                                  void **plugin_context);
AnomalyStatusV1 ANOMALY_CALL Start(void *plugin_context);
AnomalyStatusV1 ANOMALY_CALL Stop(void *plugin_context,
                                  std::uint32_t deadline_milliseconds);
void ANOMALY_CALL Unload(void *plugin_context);
void ANOMALY_CALL Update(void *plugin_context, double delta_seconds);

}  // namespace nfs2015_chase_cam::detail
