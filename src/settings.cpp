// Settings validation, persistence, defaults and the toggle hotkey: the whole
// configuration surface of the plugin, plus the schema registration that goes
// with it.
#include "internal.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <system_error>
#include <string_view>

namespace nfs2015_chase_cam::detail {

void MarkSettingsDirty(Context &context) noexcept {
  context.settings_changed_at.store(GetTickCount64(),
                                    std::memory_order_release);
  context.settings_revision.fetch_add(1, std::memory_order_acq_rel);
}

bool SettingsValid(const std::uint32_t preset, const double camera_scale,
                   const double position_spring, const double yaw_spring,
                   const double drift_yaw_spring,
                   const double look_yaw_spring,
                   const double yaw_return_spring,
                   const double pitch_spring, const double roll_spring,
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
         std::isfinite(position_spring) &&
         position_spring >= kMinimumPositionSpring &&
         position_spring <= kMaximumPositionSpring &&
         std::isfinite(yaw_spring) && yaw_spring >= kMinimumYawSpring &&
         yaw_spring <= kMaximumYawSpring &&
         std::isfinite(drift_yaw_spring) &&
         drift_yaw_spring >= kMinimumDriftYawSpring &&
         drift_yaw_spring <= kMaximumDriftYawSpring &&
         std::isfinite(look_yaw_spring) &&
         look_yaw_spring >= kMinimumLookYawSpring &&
         look_yaw_spring <= kMaximumLookYawSpring &&
         std::isfinite(yaw_return_spring) &&
         yaw_return_spring >= kMinimumYawReturnSpring &&
         yaw_return_spring <= kMaximumYawReturnSpring &&
         std::isfinite(pitch_spring) &&
         pitch_spring >= kMinimumPitchSpring &&
         pitch_spring <= kMaximumPitchSpring &&
         std::isfinite(roll_spring) && roll_spring >= kMinimumRollSpring &&
         roll_spring <= kMaximumRollSpring &&
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

bool BurnoutSettingsValid(const double entry_delay, const double entry_time,
                          const double exit_delay, const double exit_time,
                          const double launch_exit_time, const double fov,
                          const double shake, const double side_mode,
                          const double entry_speed,
                          const double hold_speed) noexcept {
  return std::isfinite(entry_delay) &&
         entry_delay >= kMinimumBurnoutEntryDelay &&
         entry_delay <= kMaximumBurnoutEntryDelay &&
         std::isfinite(entry_time) &&
         entry_time >= kMinimumBurnoutEntryTime &&
         entry_time <= kMaximumBurnoutEntryTime &&
         std::isfinite(exit_delay) &&
         exit_delay >= kMinimumBurnoutExitDelay &&
         exit_delay <= kMaximumBurnoutExitDelay &&
         std::isfinite(exit_time) &&
         exit_time >= kMinimumBurnoutExitTime &&
         exit_time <= kMaximumBurnoutExitTime &&
         std::isfinite(launch_exit_time) &&
         launch_exit_time >= kMinimumBurnoutLaunchExitTime &&
         launch_exit_time <= kMaximumBurnoutLaunchExitTime &&
         std::isfinite(fov) &&
         fov >= kMinimumBurnoutFov && fov <= kMaximumBurnoutFov &&
         std::isfinite(shake) && shake >= kMinimumBurnoutShake &&
         shake <= kMaximumBurnoutShake && std::isfinite(side_mode) &&
         side_mode >= 0.0 &&
         side_mode <= static_cast<double>(kMaximumBurnoutSideMode) &&
         std::floor(side_mode) == side_mode &&
         std::isfinite(entry_speed) &&
         entry_speed >= kMinimumBurnoutEntrySpeed &&
         entry_speed <= kMaximumBurnoutEntrySpeed &&
         std::isfinite(hold_speed) &&
         hold_speed >= kMinimumBurnoutHoldSpeed &&
         hold_speed <= kMaximumBurnoutHoldSpeed && hold_speed >= entry_speed;
}

bool WheelSteerSettingsValid(const double max_degrees, const double gain,
                             const double full_slip_degrees,
                             const double rate_degrees_per_second,
                             const double entry_speed_kmh) noexcept {
  return std::isfinite(entry_speed_kmh) &&
         entry_speed_kmh >= kMinimumWheelSteerEntrySpeedKmh &&
         entry_speed_kmh <= kMaximumWheelSteerEntrySpeedKmh &&
         std::isfinite(max_degrees) &&
         max_degrees >= kMinimumWheelSteerMaxDegrees &&
         max_degrees <= kMaximumWheelSteerMaxDegrees &&
         std::isfinite(gain) && gain >= kMinimumWheelSteerGain &&
         gain <= kMaximumWheelSteerGain &&
         std::isfinite(full_slip_degrees) &&
         full_slip_degrees >= kMinimumWheelSteerFullSlipDegrees &&
         full_slip_degrees <= kMaximumWheelSteerFullSlipDegrees &&
         std::isfinite(rate_degrees_per_second) &&
         rate_degrees_per_second >=
             kMinimumWheelSteerRateDegreesPerSecond &&
         rate_degrees_per_second <=
             kMaximumWheelSteerRateDegreesPerSecond;
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
  const double position_spring =
      context.position_spring.load(std::memory_order_acquire);
  const double yaw_spring =
      context.yaw_spring.load(std::memory_order_acquire);
  const double drift_yaw_spring =
      context.drift_yaw_spring.load(std::memory_order_acquire);
  const double look_yaw_spring =
      context.look_yaw_spring.load(std::memory_order_acquire);
  const double yaw_return_spring =
      context.yaw_return_spring.load(std::memory_order_acquire);
  const double pitch_spring =
      context.pitch_spring.load(std::memory_order_acquire);
  const double roll_spring =
      context.roll_spring.load(std::memory_order_acquire);
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
  const bool burnout_enabled =
      context.burnout_enabled.load(std::memory_order_acquire);
  const double burnout_entry_delay =
      context.burnout_entry_delay.load(std::memory_order_acquire);
  const double burnout_entry_time =
      context.burnout_entry_time.load(std::memory_order_acquire);
  const double burnout_exit_delay =
      context.burnout_exit_delay.load(std::memory_order_acquire);
  const double burnout_exit_time =
      context.burnout_exit_time.load(std::memory_order_acquire);
  const double burnout_launch_exit_time =
      context.burnout_launch_exit_time.load(std::memory_order_acquire);
  const double burnout_fov =
      context.burnout_fov.load(std::memory_order_acquire);
  const double burnout_shake =
      context.burnout_shake.load(std::memory_order_acquire);
  const double burnout_side_mode = static_cast<double>(
      context.burnout_side_mode.load(std::memory_order_acquire));
  const double burnout_entry_speed =
      context.burnout_entry_speed.load(std::memory_order_acquire);
  const double burnout_hold_speed =
      context.burnout_hold_speed.load(std::memory_order_acquire);
  const bool wheel_steer_enabled =
      context.wheel_steer_enabled.load(std::memory_order_acquire);
  const double wheel_steer_entry_speed_kmh =
      context.wheel_steer_entry_speed_kmh.load(std::memory_order_acquire);
  const double wheel_steer_max_degrees =
      context.wheel_steer_max_degrees.load(std::memory_order_acquire);
  const double wheel_steer_gain =
      context.wheel_steer_gain.load(std::memory_order_acquire);
  const double wheel_steer_full_slip_degrees =
      context.wheel_steer_full_slip_degrees.load(std::memory_order_acquire);
  const double wheel_steer_rate_degrees_per_second =
      context.wheel_steer_rate_degrees_per_second.load(
          std::memory_order_acquire);
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
      !SettingsValid(preset, camera_scale, position_spring, yaw_spring,
                     drift_yaw_spring, look_yaw_spring, yaw_return_spring,
                     pitch_spring, roll_spring, origin_offset_cm,
                     aspect_ratio, speed_distance_scale, roll_scale,
                     pitch_scale, shake_scale, slope_strength,
                     slope_max_gradient_degrees, slope_up_look_at,
                     slope_up_rest, slope_down_look_at,
                     slope_down_rest) ||
      !BurnoutSettingsValid(burnout_entry_delay, burnout_entry_time,
                            burnout_exit_delay, burnout_exit_time,
                            burnout_launch_exit_time, burnout_fov,
                            burnout_shake, burnout_side_mode,
                            burnout_entry_speed, burnout_hold_speed) ||
      !WheelSteerSettingsValid(wheel_steer_max_degrees, wheel_steer_gain,
                               wheel_steer_full_slip_degrees,
                               wheel_steer_rate_degrees_per_second,
                               wheel_steer_entry_speed_kmh) ||
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
      AppendText(cursor, end, ",\"positionSpring\":") &&
      AppendNumber(cursor, end, position_spring) &&
      AppendText(cursor, end, ",\"yawSpring\":") &&
      AppendNumber(cursor, end, yaw_spring) &&
      AppendText(cursor, end, ",\"driftYawSpring\":") &&
      AppendNumber(cursor, end, drift_yaw_spring) &&
      AppendText(cursor, end, ",\"lookYawSpring\":") &&
      AppendNumber(cursor, end, look_yaw_spring) &&
      AppendText(cursor, end, ",\"yawReturnSpring\":") &&
      AppendNumber(cursor, end, yaw_return_spring) &&
      AppendText(cursor, end, ",\"pitchSpring\":") &&
      AppendNumber(cursor, end, pitch_spring) &&
      AppendText(cursor, end, ",\"rollSpring\":") &&
      AppendNumber(cursor, end, roll_spring) &&
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
      AppendText(cursor, end, ",\"burnoutEnabled\":") &&
      AppendText(cursor, end, burnout_enabled ? "true" : "false") &&
      AppendText(cursor, end, ",\"burnoutEntryDelay\":") &&
      AppendNumber(cursor, end, burnout_entry_delay) &&
      AppendText(cursor, end, ",\"burnoutEntryTime\":") &&
      AppendNumber(cursor, end, burnout_entry_time) &&
      AppendText(cursor, end, ",\"burnoutExitDelay\":") &&
      AppendNumber(cursor, end, burnout_exit_delay) &&
      AppendText(cursor, end, ",\"burnoutExitTime\":") &&
      AppendNumber(cursor, end, burnout_exit_time) &&
      AppendText(cursor, end, ",\"burnoutLaunchExitTime\":") &&
      AppendNumber(cursor, end, burnout_launch_exit_time) &&
      AppendText(cursor, end, ",\"burnoutFov\":") &&
      AppendNumber(cursor, end, burnout_fov) &&
      AppendText(cursor, end, ",\"burnoutShake\":") &&
      AppendNumber(cursor, end, burnout_shake) &&
      AppendText(cursor, end, ",\"burnoutSideMode\":") &&
      AppendNumber(cursor, end, burnout_side_mode) &&
      AppendText(cursor, end, ",\"burnoutEntrySpeed\":") &&
      AppendNumber(cursor, end, burnout_entry_speed) &&
      AppendText(cursor, end, ",\"burnoutHoldSpeed\":") &&
      AppendNumber(cursor, end, burnout_hold_speed) &&
      AppendText(cursor, end, ",\"wheelSteerEnabled\":") &&
      AppendText(cursor, end, wheel_steer_enabled ? "true" : "false") &&
      AppendText(cursor, end, ",\"wheelSteerEntrySpeedKmh\":") &&
      AppendNumber(cursor, end, wheel_steer_entry_speed_kmh) &&
      AppendText(cursor, end, ",\"wheelSteerMaxDegrees\":") &&
      AppendNumber(cursor, end, wheel_steer_max_degrees) &&
      AppendText(cursor, end, ",\"wheelSteerGain\":") &&
      AppendNumber(cursor, end, wheel_steer_gain) &&
      AppendText(cursor, end, ",\"wheelSteerFullSlipDegrees\":") &&
      AppendNumber(cursor, end, wheel_steer_full_slip_degrees) &&
      AppendText(cursor, end, ",\"wheelSteerRateDegreesPerSecond\":") &&
      AppendNumber(cursor, end, wheel_steer_rate_degrees_per_second) &&
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
  context.position_spring.store(kDefaultPositionSpring,
                                std::memory_order_release);
  context.yaw_spring.store(kDefaultYawSpring, std::memory_order_release);
  context.drift_yaw_spring.store(kDefaultDriftYawSpring,
                                 std::memory_order_release);
  context.look_yaw_spring.store(kDefaultLookYawSpring,
                                std::memory_order_release);
  context.yaw_return_spring.store(kDefaultYawReturnSpring,
                                  std::memory_order_release);
  context.pitch_spring.store(kDefaultPitchSpring, std::memory_order_release);
  context.roll_spring.store(kDefaultRollSpring, std::memory_order_release);
  context.origin_offset_cm.store(kDefaultOriginOffsetCm,
                                 std::memory_order_release);
  context.aspect_ratio.store(kDefaultAspectRatio, std::memory_order_release);
  context.speed_distance_scale.store(kDefaultSpeedDistanceScale,
                                     std::memory_order_release);
  context.roll_scale.store(kDefaultRollScale, std::memory_order_release);
  context.pitch_scale.store(kDefaultPitchScale, std::memory_order_release);
  context.shake_scale.store(kDefaultShakeScale, std::memory_order_release);
  context.burnout_enabled.store(kDefaultBurnoutEnabled,
                                std::memory_order_release);
  context.burnout_entry_delay.store(kDefaultBurnoutEntryDelay,
                                    std::memory_order_release);
  context.burnout_entry_time.store(kDefaultBurnoutEntryTime,
                                   std::memory_order_release);
  context.burnout_exit_delay.store(kDefaultBurnoutExitDelay,
                                   std::memory_order_release);
  context.burnout_exit_time.store(kDefaultBurnoutExitTime,
                                  std::memory_order_release);
  context.burnout_launch_exit_time.store(kDefaultBurnoutLaunchExitTime,
                                         std::memory_order_release);
  context.burnout_fov.store(kDefaultBurnoutFov, std::memory_order_release);
  context.burnout_shake.store(kDefaultBurnoutShake,
                              std::memory_order_release);
  context.burnout_side_mode.store(kDefaultBurnoutSideMode,
                                  std::memory_order_release);
  context.burnout_entry_speed.store(kDefaultBurnoutEntrySpeed,
                                    std::memory_order_release);
  context.burnout_hold_speed.store(kDefaultBurnoutHoldSpeed,
                                   std::memory_order_release);
  context.wheel_steer_enabled.store(kDefaultWheelSteerEnabled,
                                    std::memory_order_release);
  context.wheel_steer_entry_speed_kmh.store(
      kDefaultWheelSteerEntrySpeedKmh, std::memory_order_release);
  context.wheel_steer_max_degrees.store(kDefaultWheelSteerMaxDegrees,
                                        std::memory_order_release);
  context.wheel_steer_gain.store(kDefaultWheelSteerGain,
                                 std::memory_order_release);
  context.wheel_steer_full_slip_degrees.store(
      kDefaultWheelSteerFullSlipDegrees, std::memory_order_release);
  context.wheel_steer_rate_degrees_per_second.store(
      kDefaultWheelSteerRateDegreesPerSecond, std::memory_order_release);
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
  context.roll_only_in_drift.store(true, std::memory_order_release);
  context.diagnostic_flat_pose.store(false, std::memory_order_release);
  context.diagnostic_zero_lag.store(false, std::memory_order_release);
  context.borrow_strength.store(1.0, std::memory_order_release);
  context.anchor_source.store(kAnchorSourceDefault, std::memory_order_release);
  context.diagnostic_skip_position.store(false, std::memory_order_release);
  context.diagnostic_skip_rotation.store(false, std::memory_order_release);
  context.fov_enabled.store(true, std::memory_order_release);
}

void RestoreDefaults(Context &context) noexcept {
  SetDefaults(context);
  context.reset_requested.store(true, std::memory_order_release);
  MarkSettingsDirty(context);
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
  // The schema has 48 properties today. Keep headroom so a future optional
  // field does not make existing documents unreadable.
  if (json.kind(json.user, root, &kind).code != ANOMALY_STATUS_V1_OK ||
      kind != ANOMALY_JSON_V1_OBJECT ||
      json.object_size(json.user, root, &field_count).code !=
          ANOMALY_STATUS_V1_OK ||
      field_count < 6 || field_count > 60) {
    return false;
  }
  bool enabled{};
  bool fov_enabled{};
  double preset_number{};
  double camera_scale{};
  double position_spring = kDefaultPositionSpring;
  double yaw_spring = kDefaultYawSpring;
  double drift_yaw_spring = kDefaultDriftYawSpring;
  double look_yaw_spring = kDefaultLookYawSpring;
  double yaw_return_spring = kDefaultYawReturnSpring;
  double pitch_spring = kDefaultPitchSpring;
  double roll_spring = kDefaultRollSpring;
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
  bool burnout_enabled = kDefaultBurnoutEnabled;
  double burnout_entry_delay = kDefaultBurnoutEntryDelay;
  double burnout_entry_time = kDefaultBurnoutEntryTime;
  double burnout_exit_delay = kDefaultBurnoutExitDelay;
  double burnout_exit_time = kDefaultBurnoutExitTime;
  double burnout_launch_exit_time = kDefaultBurnoutLaunchExitTime;
  double burnout_fov = kDefaultBurnoutFov;
  double burnout_shake = kDefaultBurnoutShake;
  double burnout_side_mode_number =
      static_cast<double>(kDefaultBurnoutSideMode);
  double burnout_entry_speed = kDefaultBurnoutEntrySpeed;
  double burnout_hold_speed = kDefaultBurnoutHoldSpeed;
  bool wheel_steer_enabled = kDefaultWheelSteerEnabled;
  double wheel_steer_entry_speed_kmh =
      kDefaultWheelSteerEntrySpeedKmh;
  double wheel_steer_max_degrees = kDefaultWheelSteerMaxDegrees;
  double wheel_steer_gain = kDefaultWheelSteerGain;
  double wheel_steer_full_slip_degrees =
      kDefaultWheelSteerFullSlipDegrees;
  double wheel_steer_rate_degrees_per_second =
      kDefaultWheelSteerRateDegreesPerSecond;
  bool roll_only_in_drift = true;
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
  static_cast<void>(JsonObjectNumber(json, root, "positionSpring",
                                     position_spring));
  static_cast<void>(JsonObjectNumber(json, root, "yawSpring", yaw_spring));
  static_cast<void>(JsonObjectNumber(json, root, "driftYawSpring",
                                     drift_yaw_spring));
  static_cast<void>(JsonObjectNumber(json, root, "lookYawSpring",
                                     look_yaw_spring));
  static_cast<void>(JsonObjectNumber(json, root, "yawReturnSpring",
                                     yaw_return_spring));
  static_cast<void>(JsonObjectNumber(json, root, "pitchSpring",
                                     pitch_spring));
  static_cast<void>(JsonObjectNumber(json, root, "rollSpring", roll_spring));
  static_cast<void>(JsonObjectNumber(json, root, "speedDistanceScale",
                                     speed_distance_scale));
  static_cast<void>(JsonObjectNumber(json, root, "rollScale", roll_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "pitchScale", pitch_scale));
  static_cast<void>(
      JsonObjectNumber(json, root, "shakeScale", shake_scale));
  static_cast<void>(JsonObjectBoolean(json, root, "burnoutEnabled",
                                      burnout_enabled));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutEntryDelay",
                                     burnout_entry_delay));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutEntryTime",
                                     burnout_entry_time));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutExitDelay",
                                     burnout_exit_delay));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutExitTime",
                                     burnout_exit_time));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutLaunchExitTime",
                                     burnout_launch_exit_time));
  static_cast<void>(
      JsonObjectNumber(json, root, "burnoutFov", burnout_fov));
  static_cast<void>(
      JsonObjectNumber(json, root, "burnoutShake", burnout_shake));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutSideMode",
                                     burnout_side_mode_number));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutEntrySpeed",
                                     burnout_entry_speed));
  static_cast<void>(JsonObjectNumber(json, root, "burnoutHoldSpeed",
                                     burnout_hold_speed));
  static_cast<void>(JsonObjectBoolean(json, root, "wheelSteerEnabled",
                                      wheel_steer_enabled));
  static_cast<void>(JsonObjectNumber(json, root, "wheelSteerEntrySpeedKmh",
                                     wheel_steer_entry_speed_kmh));
  static_cast<void>(JsonObjectNumber(json, root, "wheelSteerMaxDegrees",
                                     wheel_steer_max_degrees));
  static_cast<void>(JsonObjectNumber(json, root, "wheelSteerGain",
                                     wheel_steer_gain));
  static_cast<void>(JsonObjectNumber(json, root, "wheelSteerFullSlipDegrees",
                                     wheel_steer_full_slip_degrees));
  static_cast<void>(JsonObjectNumber(
      json, root, "wheelSteerRateDegreesPerSecond",
      wheel_steer_rate_degrees_per_second));
  if (wheel_steer_max_degrees == kLegacyWheelSteerMaxDegrees &&
      wheel_steer_full_slip_degrees ==
          kLegacyWheelSteerFullSlipDegrees &&
      wheel_steer_rate_degrees_per_second ==
          kLegacyWheelSteerRateDegreesPerSecond) {
    wheel_steer_max_degrees = kDefaultWheelSteerMaxDegrees;
    wheel_steer_full_slip_degrees =
        kDefaultWheelSteerFullSlipDegrees;
    wheel_steer_rate_degrees_per_second =
        kDefaultWheelSteerRateDegreesPerSecond;
  }
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
  if (!SettingsValid(preset, camera_scale, position_spring, yaw_spring,
                     drift_yaw_spring, look_yaw_spring, yaw_return_spring,
                     pitch_spring, roll_spring, origin_offset_cm,
                     aspect_ratio, speed_distance_scale, roll_scale,
                     pitch_scale, shake_scale, slope_strength,
                     slope_max_gradient_degrees, slope_up_look_at,
                     slope_up_rest, slope_down_look_at,
                     slope_down_rest) ||
      !BurnoutSettingsValid(burnout_entry_delay, burnout_entry_time,
                            burnout_exit_delay, burnout_exit_time,
                            burnout_launch_exit_time, burnout_fov,
                            burnout_shake, burnout_side_mode_number,
                            burnout_entry_speed, burnout_hold_speed) ||
      !WheelSteerSettingsValid(wheel_steer_max_degrees, wheel_steer_gain,
                               wheel_steer_full_slip_degrees,
                               wheel_steer_rate_degrees_per_second,
                               wheel_steer_entry_speed_kmh)) {
    return false;
  }
  context.enabled.store(enabled, std::memory_order_release);
  context.preset.store(preset, std::memory_order_release);
  context.camera_scale.store(camera_scale, std::memory_order_release);
  context.position_spring.store(position_spring, std::memory_order_release);
  context.yaw_spring.store(yaw_spring, std::memory_order_release);
  context.drift_yaw_spring.store(drift_yaw_spring,
                                 std::memory_order_release);
  context.look_yaw_spring.store(look_yaw_spring, std::memory_order_release);
  context.yaw_return_spring.store(yaw_return_spring,
                                  std::memory_order_release);
  context.pitch_spring.store(pitch_spring, std::memory_order_release);
  context.roll_spring.store(roll_spring, std::memory_order_release);
  context.origin_offset_cm.store(origin_offset_cm,
                                 std::memory_order_release);
  context.aspect_ratio.store(aspect_ratio, std::memory_order_release);
  context.speed_distance_scale.store(speed_distance_scale,
                                     std::memory_order_release);
  context.roll_scale.store(roll_scale, std::memory_order_release);
  context.pitch_scale.store(pitch_scale, std::memory_order_release);
  context.shake_scale.store(shake_scale, std::memory_order_release);
  context.burnout_enabled.store(burnout_enabled, std::memory_order_release);
  context.burnout_entry_delay.store(burnout_entry_delay,
                                    std::memory_order_release);
  context.burnout_entry_time.store(burnout_entry_time,
                                   std::memory_order_release);
  context.burnout_exit_delay.store(burnout_exit_delay,
                                   std::memory_order_release);
  context.burnout_exit_time.store(burnout_exit_time,
                                  std::memory_order_release);
  context.burnout_launch_exit_time.store(burnout_launch_exit_time,
                                         std::memory_order_release);
  context.burnout_fov.store(burnout_fov, std::memory_order_release);
  context.burnout_shake.store(burnout_shake, std::memory_order_release);
  context.burnout_side_mode.store(
      static_cast<std::uint32_t>(burnout_side_mode_number),
      std::memory_order_release);
  context.burnout_entry_speed.store(burnout_entry_speed,
                                    std::memory_order_release);
  context.burnout_hold_speed.store(burnout_hold_speed,
                                   std::memory_order_release);
  context.wheel_steer_enabled.store(wheel_steer_enabled,
                                    std::memory_order_release);
  context.wheel_steer_entry_speed_kmh.store(
      wheel_steer_entry_speed_kmh, std::memory_order_release);
  context.wheel_steer_max_degrees.store(wheel_steer_max_degrees,
                                        std::memory_order_release);
  context.wheel_steer_gain.store(wheel_steer_gain,
                                 std::memory_order_release);
  context.wheel_steer_full_slip_degrees.store(
      wheel_steer_full_slip_degrees, std::memory_order_release);
  context.wheel_steer_rate_degrees_per_second.store(
      wheel_steer_rate_degrees_per_second, std::memory_order_release);
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

}  // namespace nfs2015_chase_cam::detail
