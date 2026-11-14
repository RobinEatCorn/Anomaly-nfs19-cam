// Acceptance tests for the grouped settings model
// (PLAN-settings-ui-groups.md task group 1).
//
// These tests do not load the game or the Anomaly runtime. They link the real
// settings translation unit and drive ApplyGroup() through a plain Context, so
// every group's three modes are pinned to the effective values the engine
// reads.

#include "../src/internal.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase_cam::detail::ApplyAllGroups;
using nfs2015_chase_cam::detail::ApplyGroup;
using nfs2015_chase_cam::detail::Context;
using nfs2015_chase_cam::detail::CustomSettings;
using nfs2015_chase_cam::detail::GroupMode;
using nfs2015_chase_cam::detail::NormalizeGroupMode;
using nfs2015_chase_cam::detail::SetDefaults;
using nfs2015_chase_cam::detail::SettingsGroup;

void Fail(const char* label, const char* detail, const double actual,
          const double expected) {
  std::fprintf(stderr, "FAIL %s: %s, got %.6f (expected %.6f)\n", label,
               detail, actual, expected);
  std::exit(1);
}

void Expect(const char* label, const bool condition) {
  if (!condition) {
    std::fprintf(stderr, "FAIL %s: condition was false\n", label);
    std::exit(1);
  }
  std::printf("ok   %s\n", label);
}

void ExpectNear(const char* label, const double actual, const double expected,
                const double tolerance = 1e-12) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    Fail(label, "outside tolerance", actual, expected);
  }
  std::printf("ok   %-44s %.6f (expected %.6f)\n", label, actual, expected);
}

void SetMode(Context& context, const SettingsGroup group,
             const GroupMode mode) noexcept {
  context.group_modes[static_cast<std::size_t>(group)].store(
      static_cast<std::uint32_t>(mode), std::memory_order_release);
  ApplyGroup(context, group);
}

// The custom block is only written by the panel, but the model has to keep it
// consistent while a group is not on Custom, so the tests edit it directly.
void EditCustom(Context& context,
                void (*edit)(CustomSettings&)) noexcept {
  std::scoped_lock lock(context.custom_settings_mutex);
  edit(context.custom_settings);
}

[[nodiscard]] CustomSettings CustomCopy(Context& context) {
  std::scoped_lock lock(context.custom_settings_mutex);
  return context.custom_settings;
}

void CheckModeTable() {
  Expect("lens rejects Custom",
         !nfs2015_chase_cam::detail::GroupModeAllowed(SettingsGroup::NfsLens,
                                                      GroupMode::Custom));
  Expect("rig rejects Opposite",
         !nfs2015_chase_cam::detail::GroupModeAllowed(SettingsGroup::Rig,
                                                      GroupMode::Opposite));
  Expect("springs reject Opposite",
         !nfs2015_chase_cam::detail::GroupModeAllowed(SettingsGroup::Springs,
                                                      GroupMode::Opposite));
  Expect("dynamics accepts all three",
         nfs2015_chase_cam::detail::GroupModeAllowed(SettingsGroup::Dynamics,
                                                      GroupMode::Custom) &&
             nfs2015_chase_cam::detail::GroupModeAllowed(
                 SettingsGroup::Dynamics, GroupMode::Opposite));
  Expect("unknown value normalizes to Default",
         NormalizeGroupMode(SettingsGroup::Rig, 99) == GroupMode::Default);
  Expect("impossible mode normalizes to Default",
         NormalizeGroupMode(SettingsGroup::NfsLens,
                            static_cast<std::uint32_t>(GroupMode::Custom)) ==
             GroupMode::Default);
}

void CheckDefaults(Context& context) {
  bool all_default = true;
  for (std::size_t index = 0;
       index < nfs2015_chase_cam::detail::kSettingsGroupCount; ++index) {
    all_default =
        all_default &&
        context.group_modes[index].load() ==
            static_cast<std::uint32_t>(GroupMode::Default);
  }
  Expect("every group starts on Default", all_default);
  ApplyAllGroups(context);
  ExpectNear("fresh camera scale", context.camera_scale.load(),
             nfs2015_chase_cam::detail::kDefaultCameraScale);
  ExpectNear("fresh yaw spring", context.yaw_spring.load(),
             nfs2015_chase_cam::detail::kDefaultYawSpring);
  ExpectNear("fresh roll spring", context.roll_spring.load(),
             nfs2015_chase_cam::detail::kDefaultRollSpring);
  ExpectNear("fresh slope strength", context.slope_strength.load(),
             nfs2015_chase_cam::detail::kDefaultSlopeStrength);
  Expect("fresh FOV enabled", context.fov_enabled.load());
  Expect("fresh slope follows", context.slope_follow.load());
  Expect("fresh wheel steer enabled", context.wheel_steer_enabled.load());
  Expect("fresh burnout disabled", !context.burnout_enabled.load());
  Expect("fresh dynamics roll scale on", context.roll_scale.load() == 1.0);
  Expect("fresh roll only in drift", context.roll_only_in_drift.load());
}

void CheckNfsLens(Context& context) {
  SetMode(context, SettingsGroup::NfsLens, GroupMode::Opposite);
  Expect("lens opposite turns the FOV off", !context.fov_enabled.load());
  SetMode(context, SettingsGroup::NfsLens, GroupMode::Custom);
  Expect("lens custom falls back to Default", context.fov_enabled.load());
  SetMode(context, SettingsGroup::NfsLens, GroupMode::Default);
  Expect("lens default turns the FOV on", context.fov_enabled.load());
}

void CheckRig(Context& context) {
  EditCustom(context, [](CustomSettings& custom) {
    custom.camera_scale = 1.3;
    custom.origin_offset_cm = -40.0;
    custom.aspect_ratio = 2.0;
    custom.speed_distance_scale = 0.5;
  });
  SetMode(context, SettingsGroup::Rig, GroupMode::Custom);
  ExpectNear("rig custom camera scale", context.camera_scale.load(), 1.3);
  ExpectNear("rig custom origin offset", context.origin_offset_cm.load(),
             -40.0);
  ExpectNear("rig custom aspect ratio", context.aspect_ratio.load(), 2.0);
  ExpectNear("rig custom speed distance", context.speed_distance_scale.load(),
             0.5);
  SetMode(context, SettingsGroup::Rig, GroupMode::Default);
  ExpectNear("rig default camera scale", context.camera_scale.load(),
             nfs2015_chase_cam::detail::kDefaultCameraScale);
  ExpectNear("rig default aspect ratio", context.aspect_ratio.load(),
             nfs2015_chase_cam::detail::kDefaultAspectRatio);
  SetMode(context, SettingsGroup::Rig, GroupMode::Opposite);
  ExpectNear("rig opposite behaves like Default", context.camera_scale.load(),
             nfs2015_chase_cam::detail::kDefaultCameraScale);
  SetMode(context, SettingsGroup::Rig, GroupMode::Custom);
  ExpectNear("rig custom values survived Default", context.camera_scale.load(),
             1.3);
}

void CheckSprings(Context& context) {
  EditCustom(context, [](CustomSettings& custom) {
    custom.position_spring = 12.0;
    custom.roll_spring = 90.0;
  });
  SetMode(context, SettingsGroup::Springs, GroupMode::Custom);
  ExpectNear("springs custom position", context.position_spring.load(), 12.0);
  ExpectNear("springs custom roll", context.roll_spring.load(), 90.0);
  ExpectNear("springs custom leaves pitch alone", context.pitch_spring.load(),
             nfs2015_chase_cam::detail::kDefaultPitchSpring);
  SetMode(context, SettingsGroup::Springs, GroupMode::Default);
  ExpectNear("springs default position", context.position_spring.load(),
             nfs2015_chase_cam::detail::kDefaultPositionSpring);
}

void CheckDynamics(Context& context) {
  SetMode(context, SettingsGroup::Dynamics, GroupMode::Opposite);
  ExpectNear("dynamics off roll", context.roll_scale.load(), 0.0);
  ExpectNear("dynamics off pitch", context.pitch_scale.load(), 0.0);
  ExpectNear("dynamics off shake", context.shake_scale.load(), 0.0);
  Expect("dynamics off keeps the drift gate",
         context.roll_only_in_drift.load());
  SetMode(context, SettingsGroup::Dynamics, GroupMode::Default);
  ExpectNear("dynamics default roll", context.roll_scale.load(), 1.0);
  EditCustom(context, [](CustomSettings& custom) {
    custom.roll_scale = 0.4;
    custom.pitch_scale = 0.8;
    custom.shake_scale = 1.5;
    custom.roll_only_in_drift = false;
  });
  SetMode(context, SettingsGroup::Dynamics, GroupMode::Custom);
  ExpectNear("dynamics custom roll", context.roll_scale.load(), 0.4);
  ExpectNear("dynamics custom pitch", context.pitch_scale.load(), 0.8);
  ExpectNear("dynamics custom shake", context.shake_scale.load(), 1.5);
  Expect("dynamics custom drift gate off",
         !context.roll_only_in_drift.load());
}

void CheckSlope(Context& context) {
  SetMode(context, SettingsGroup::Slope, GroupMode::Opposite);
  Expect("slope off stops following", !context.slope_follow.load());
  ExpectNear("slope off restores the default strength",
             context.slope_strength.load(),
             nfs2015_chase_cam::detail::kDefaultSlopeStrength);
  EditCustom(context, [](CustomSettings& custom) {
    custom.slope_strength = 1.7;
  });
  SetMode(context, SettingsGroup::Slope, GroupMode::Custom);
  Expect("slope custom follows", context.slope_follow.load());
  ExpectNear("slope custom strength", context.slope_strength.load(), 1.7);
  SetMode(context, SettingsGroup::Slope, GroupMode::Default);
  ExpectNear("slope default strength", context.slope_strength.load(),
             nfs2015_chase_cam::detail::kDefaultSlopeStrength);
}

void CheckWheelSteer(Context& context) {
  SetMode(context, SettingsGroup::WheelSteer, GroupMode::Opposite);
  Expect("wheel steer off disables it", !context.wheel_steer_enabled.load());
  ExpectNear("wheel steer off restores the default angle",
             context.wheel_steer_max_degrees.load(),
             nfs2015_chase_cam::detail::kDefaultWheelSteerMaxDegrees);
  EditCustom(context, [](CustomSettings& custom) {
    custom.wheel_steer_max_degrees = 45.0;
    custom.wheel_steer_sample_percent = -25.0;
  });
  SetMode(context, SettingsGroup::WheelSteer, GroupMode::Custom);
  Expect("wheel steer custom forces it on", context.wheel_steer_enabled.load());
  ExpectNear("wheel steer custom angle", context.wheel_steer_max_degrees.load(),
             45.0);
  ExpectNear("wheel steer custom sample",
             context.wheel_steer_sample_percent.load(), -25.0);
  SetMode(context, SettingsGroup::WheelSteer, GroupMode::Default);
  Expect("wheel steer default enabled", context.wheel_steer_enabled.load());
  ExpectNear("wheel steer default angle",
             context.wheel_steer_max_degrees.load(),
             nfs2015_chase_cam::detail::kDefaultWheelSteerMaxDegrees);
}

void CheckBurnout(Context& context) {
  SetMode(context, SettingsGroup::Burnout, GroupMode::Default);
  Expect("burnout default is off", !context.burnout_enabled.load());
  ExpectNear("burnout default FOV", context.burnout_fov.load(),
             nfs2015_chase_cam::detail::kDefaultBurnoutFov);
  EditCustom(context, [](CustomSettings& custom) {
    custom.burnout_fov = 60.0;
    custom.burnout_entry_speed = 4.0;
    custom.burnout_hold_speed = 1.0;  // invalid pair on purpose
    custom.burnout_side_mode = 2;
  });
  SetMode(context, SettingsGroup::Burnout, GroupMode::Custom);
  Expect("burnout custom forces it on", context.burnout_enabled.load());
  ExpectNear("burnout custom FOV", context.burnout_fov.load(), 60.0);
  ExpectNear("burnout custom side", context.burnout_side_mode.load(), 2.0);
  ExpectNear("burnout hold speed clamped up to entry",
             context.burnout_hold_speed.load(), 4.0);
  ExpectNear("burnout custom hold speed not rewritten",
             CustomCopy(context).burnout_hold_speed, 1.0);
  SetMode(context, SettingsGroup::Burnout, GroupMode::Opposite);
  Expect("burnout opposite runs it on", context.burnout_enabled.load());
  ExpectNear("burnout opposite default FOV", context.burnout_fov.load(),
             nfs2015_chase_cam::detail::kDefaultBurnoutFov);
  ExpectNear("burnout opposite default side", context.burnout_side_mode.load(),
             nfs2015_chase_cam::detail::kDefaultBurnoutSideMode);
  ExpectNear("burnout opposite default hold",
             context.burnout_hold_speed.load(),
             nfs2015_chase_cam::detail::kDefaultBurnoutHoldSpeed);
}

void CheckRestoreDefaults(Context& context) {
  SetMode(context, SettingsGroup::Burnout, GroupMode::Custom);
  SetDefaults(context);
  Expect("restore defaults resets every mode",
         context.group_modes[static_cast<std::size_t>(
             SettingsGroup::Burnout)]
                 .load() == static_cast<std::uint32_t>(GroupMode::Default));
  ExpectNear("restore defaults clears the custom geometry",
             CustomCopy(context).camera_scale,
             nfs2015_chase_cam::detail::kDefaultCameraScale);
  ExpectNear("restore defaults clears the custom burnout FOV",
             CustomCopy(context).burnout_fov,
             nfs2015_chase_cam::detail::kDefaultBurnoutFov);
  Expect("restore defaults turns the FOV back on",
         context.fov_enabled.load());
  Expect("restore defaults turns the burnout off",
         !context.burnout_enabled.load());
}

}  // namespace

int main() {
  Context context;
  CheckModeTable();
  CheckDefaults(context);
  CheckNfsLens(context);
  CheckRig(context);
  CheckSprings(context);
  CheckDynamics(context);
  CheckSlope(context);
  CheckWheelSteer(context);
  CheckBurnout(context);
  CheckRestoreDefaults(context);
  std::printf("\nall settings group checks passed\n");
  return 0;
}
