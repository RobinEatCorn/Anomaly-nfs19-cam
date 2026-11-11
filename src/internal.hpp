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
void SetDefaults(Context &context) noexcept;
void RestoreDefaults(Context &context) noexcept;
[[nodiscard]] bool PersistSettings(Context &context) noexcept;
[[nodiscard]] bool LoadSettings(Context &context) noexcept;
void ReleaseSettingsSchema(Context &context) noexcept;
[[nodiscard]] bool RegisterToggleHotkey(Context &context) noexcept;
void ReleaseToggleHotkey(Context &context) noexcept;

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

}  // namespace nfs2015_chase_cam::detail
