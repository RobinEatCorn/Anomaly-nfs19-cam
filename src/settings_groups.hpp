// The grouped settings model (PLAN-settings-ui-groups.md task group 1).
//
// Every exception the settings panel offers is one of seven groups. A group is
// either on its built-in defaults, on its own custom numbers, or on the
// opposite of its default switch; the group's enable flags are derived from
// that mode instead of being edited one by one. The values the engine reads
// stay in Context's existing atomics, so no consumer changes; ApplyGroup() is
// their only grouped writer.
#pragma once

#include "constants.hpp"

#include <cstddef>
#include <cstdint>

namespace nfs2015_chase_cam::detail {

enum class SettingsGroup : std::uint32_t {
  NfsLens = 0,
  Rig = 1,
  Springs = 2,
  Dynamics = 3,
  Slope = 4,
  WheelSteer = 5,
  Burnout = 6,
};

constexpr std::size_t kSettingsGroupCount = 7;

// The numbers are the stored JSON values; a saved document keeps its meaning
// only while they stay stable.
enum class GroupMode : std::uint32_t {
  Default = 0,
  Custom = 1,
  Opposite = 2,
};

// Which modes a group can express. The lens only toggles the FOV, the rig and
// the springs have no "off", and the rest offer all three.
[[nodiscard]] constexpr bool GroupModeAllowed(const SettingsGroup group,
                                              const GroupMode mode) noexcept {
  switch (group) {
    case SettingsGroup::NfsLens:
      return mode != GroupMode::Custom;
    case SettingsGroup::Rig:
    case SettingsGroup::Springs:
      return mode != GroupMode::Opposite;
    case SettingsGroup::Dynamics:
    case SettingsGroup::Slope:
    case SettingsGroup::WheelSteer:
    case SettingsGroup::Burnout:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool IsGroupModeValue(
    const std::uint32_t value) noexcept {
  return value <= static_cast<std::uint32_t>(GroupMode::Opposite);
}

// Unknown values and modes the group cannot express fall back to Default, so a
// hand-edited or future document can never leave a group in a state the panel
// cannot draw.
[[nodiscard]] constexpr GroupMode NormalizeGroupMode(
    const SettingsGroup group, const std::uint32_t value) noexcept {
  if (!IsGroupModeValue(value)) {
    return GroupMode::Default;
  }
  const auto mode = static_cast<GroupMode>(value);
  return GroupModeAllowed(group, mode) ? mode : GroupMode::Default;
}

// Only the numbers Custom mode can edit. The custom numbers survive on Default
// and Opposite so switching back restores what the user tuned; only "restore
// defaults" clears them. Enable flags are deliberately absent: they follow
// from the mode.
struct CustomSettings final {
  // Rig geometry.
  double camera_scale{kDefaultCameraScale};
  double origin_offset_cm{kDefaultOriginOffsetCm};
  double aspect_ratio{kDefaultAspectRatio};
  double speed_distance_scale{kDefaultSpeedDistanceScale};
  // Chase-camera springs.
  double position_spring{kDefaultPositionSpring};
  double yaw_spring{kDefaultYawSpring};
  double drift_yaw_spring{kDefaultDriftYawSpring};
  double look_yaw_spring{kDefaultLookYawSpring};
  double yaw_return_spring{kDefaultYawReturnSpring};
  double pitch_spring{kDefaultPitchSpring};
  double roll_spring{kDefaultRollSpring};
  // Attitude response.
  double roll_scale{kDefaultRollScale};
  double pitch_scale{kDefaultPitchScale};
  double shake_scale{kDefaultShakeScale};
  bool roll_only_in_drift{true};
  // Slope following. The gradient shape stays an advanced setting.
  double slope_strength{kDefaultSlopeStrength};
  // Drift-time front wheel steer.
  double wheel_steer_entry_speed_kmh{kDefaultWheelSteerEntrySpeedKmh};
  double wheel_steer_max_degrees{kDefaultWheelSteerMaxDegrees};
  double wheel_steer_gain{kDefaultWheelSteerGain};
  double wheel_steer_full_slip_degrees{kDefaultWheelSteerFullSlipDegrees};
  double wheel_steer_rate_degrees_per_second{
      kDefaultWheelSteerRateDegreesPerSecond};
  double wheel_steer_force_turn_delay_seconds{
      kDefaultWheelSteerForceTurnDelaySeconds};
  double wheel_steer_sample_percent{kDefaultWheelSteerSamplePercent};
  // Burnout close-up.
  double burnout_entry_delay{kDefaultBurnoutEntryDelay};
  double burnout_entry_time{kDefaultBurnoutEntryTime};
  double burnout_exit_delay{kDefaultBurnoutExitDelay};
  double burnout_exit_time{kDefaultBurnoutExitTime};
  double burnout_launch_exit_time{kDefaultBurnoutLaunchExitTime};
  double burnout_fov{kDefaultBurnoutFov};
  double burnout_shake{kDefaultBurnoutShake};
  double burnout_entry_speed{kDefaultBurnoutEntrySpeed};
  double burnout_hold_speed{kDefaultBurnoutHoldSpeed};
  std::uint32_t burnout_side_mode{kDefaultBurnoutSideMode};
};

// ---------------------------------------------------------------------------
// Legacy documents
// ---------------------------------------------------------------------------
// A document written before the groups existed has no mode fields: the numbers
// and the enable switches are the only record of what the user had chosen.
// Reconstructing the mode from them keeps an upgraded install behaving exactly
// as it did, and every number the user ever tuned still lands in the custom
// block, so nothing is lost when a group is switched to Custom later.
//
// Exact comparisons are deliberate: the document was written with these same
// constants, so a value the user never touched parses back to the identical
// double. The switches win over the numbers: a group that was switched off
// keeps its numbers in memory but must not come back enabled.
struct LegacyGroupState final {
  CustomSettings custom;
  bool fov_enabled{true};
  bool slope_follow{true};
  bool wheel_steer_enabled{kDefaultWheelSteerEnabled};
  bool burnout_enabled{kDefaultBurnoutEnabled};
};

[[nodiscard]] constexpr bool RigOnDefaults(
    const CustomSettings &custom) noexcept {
  return custom.camera_scale == kDefaultCameraScale &&
         custom.origin_offset_cm == kDefaultOriginOffsetCm &&
         custom.aspect_ratio == kDefaultAspectRatio &&
         custom.speed_distance_scale == kDefaultSpeedDistanceScale;
}

[[nodiscard]] constexpr bool SpringsOnDefaults(
    const CustomSettings &custom) noexcept {
  return custom.position_spring == kDefaultPositionSpring &&
         custom.yaw_spring == kDefaultYawSpring &&
         custom.drift_yaw_spring == kDefaultDriftYawSpring &&
         custom.look_yaw_spring == kDefaultLookYawSpring &&
         custom.yaw_return_spring == kDefaultYawReturnSpring &&
         custom.pitch_spring == kDefaultPitchSpring &&
         custom.roll_spring == kDefaultRollSpring;
}

[[nodiscard]] constexpr bool DynamicsOnDefaults(
    const CustomSettings &custom) noexcept {
  return custom.roll_scale == kDefaultRollScale &&
         custom.pitch_scale == kDefaultPitchScale &&
         custom.shake_scale == kDefaultShakeScale &&
         custom.roll_only_in_drift;
}

[[nodiscard]] constexpr bool WheelSteerOnDefaults(
    const CustomSettings &custom) noexcept {
  return custom.wheel_steer_entry_speed_kmh ==
             kDefaultWheelSteerEntrySpeedKmh &&
         custom.wheel_steer_max_degrees == kDefaultWheelSteerMaxDegrees &&
         custom.wheel_steer_gain == kDefaultWheelSteerGain &&
         custom.wheel_steer_full_slip_degrees ==
             kDefaultWheelSteerFullSlipDegrees &&
         custom.wheel_steer_rate_degrees_per_second ==
             kDefaultWheelSteerRateDegreesPerSecond &&
         custom.wheel_steer_force_turn_delay_seconds ==
             kDefaultWheelSteerForceTurnDelaySeconds &&
         custom.wheel_steer_sample_percent ==
             kDefaultWheelSteerSamplePercent;
}

[[nodiscard]] constexpr bool BurnoutOnDefaults(
    const CustomSettings &custom) noexcept {
  return custom.burnout_entry_delay == kDefaultBurnoutEntryDelay &&
         custom.burnout_entry_time == kDefaultBurnoutEntryTime &&
         custom.burnout_exit_delay == kDefaultBurnoutExitDelay &&
         custom.burnout_exit_time == kDefaultBurnoutExitTime &&
         custom.burnout_launch_exit_time == kDefaultBurnoutLaunchExitTime &&
         custom.burnout_fov == kDefaultBurnoutFov &&
         custom.burnout_shake == kDefaultBurnoutShake &&
         custom.burnout_entry_speed == kDefaultBurnoutEntrySpeed &&
         custom.burnout_hold_speed == kDefaultBurnoutHoldSpeed &&
         custom.burnout_side_mode == kDefaultBurnoutSideMode;
}

[[nodiscard]] constexpr GroupMode InferLegacyGroupMode(
    const SettingsGroup group, const LegacyGroupState &legacy) noexcept {
  const CustomSettings &custom = legacy.custom;
  switch (group) {
    case SettingsGroup::NfsLens:
      return legacy.fov_enabled ? GroupMode::Default : GroupMode::Opposite;
    case SettingsGroup::Rig:
      return RigOnDefaults(custom) ? GroupMode::Default : GroupMode::Custom;
    case SettingsGroup::Springs:
      return SpringsOnDefaults(custom) ? GroupMode::Default
                                       : GroupMode::Custom;
    case SettingsGroup::Dynamics:
      if (custom.roll_scale == 0.0 && custom.pitch_scale == 0.0 &&
          custom.shake_scale == 0.0) {
        return GroupMode::Opposite;
      }
      return DynamicsOnDefaults(custom) ? GroupMode::Default
                                        : GroupMode::Custom;
    case SettingsGroup::Slope:
      if (!legacy.slope_follow) {
        return GroupMode::Opposite;
      }
      return custom.slope_strength == kDefaultSlopeStrength
                 ? GroupMode::Default
                 : GroupMode::Custom;
    case SettingsGroup::WheelSteer:
      if (!legacy.wheel_steer_enabled) {
        return GroupMode::Opposite;
      }
      return WheelSteerOnDefaults(custom) ? GroupMode::Default
                                          : GroupMode::Custom;
    case SettingsGroup::Burnout:
      if (!legacy.burnout_enabled) {
        return GroupMode::Default;
      }
      return BurnoutOnDefaults(custom) ? GroupMode::Opposite
                                       : GroupMode::Custom;
  }
  return GroupMode::Default;
}

}  // namespace nfs2015_chase_cam::detail
