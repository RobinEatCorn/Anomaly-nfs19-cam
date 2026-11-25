// Display-thread and diagnostics side of the tick: build the panel snapshot and
// emit the hook and timing log lines.
#include "internal.hpp"

#include <cstdint>
#include <cstdio>

namespace nfs2015_chase_cam::detail {

void UpdateDisplay(Context &context) noexcept {
  std::scoped_lock lock(context.display_mutex);
  context.display.started = context.started.load(std::memory_order_acquire);
  context.display.enabled = context.enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_enabled =
      context.wheel_steer_enabled.load(std::memory_order_acquire);
  context.display.wheel_steer_active = context.wheel_steer.active;
  context.display.wheel_steer_engaged = context.wheel_steer.engaged;
  context.display.wheel_steer_force_turn = context.wheel_steer.force_turn;
  context.display.wheel_steer_wheels =
      static_cast<std::uint32_t>(context.wheel_steer.wheel_count);
  context.display.wheel_steer_angle_degrees =
      context.wheel_steer.applied_angle * kRadiansToDegrees;
  context.display.wheel_steer_target_degrees =
      context.wheel_steer.target_angle * kRadiansToDegrees;
  context.display.wheel_steer_entry_speed_kmh =
      context.wheel_steer_entry_speed_kmh.load(std::memory_order_acquire);
  context.display.wheel_steer_max_degrees =
      context.wheel_steer_max_degrees.load(std::memory_order_acquire);
  context.display.wheel_steer_gain =
      context.wheel_steer_gain.load(std::memory_order_acquire);
  context.display.wheel_steer_full_slip_degrees =
      context.wheel_steer_full_slip_degrees.load(
          std::memory_order_acquire);
  context.display.wheel_steer_rate_degrees_per_second =
      context.wheel_steer_rate_degrees_per_second.load(
          std::memory_order_acquire);
  context.display.wheel_steer_force_turn_delay_seconds =
      context.wheel_steer_force_turn_delay_seconds.load(
          std::memory_order_acquire);
  context.display.wheel_steer_sample_percent =
      context.wheel_steer_sample_percent.load(std::memory_order_acquire);
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
  context.display.lease_failures =
      diag.lease_failures.load(std::memory_order_relaxed);
  context.display.view_read_torn =
      diag.view_read_torn.load(std::memory_order_relaxed);
  context.display.detour_calls =
      diag.detour_calls.load(std::memory_order_relaxed);
  context.display.detour_no_publish =
      diag.detour_no_publish.load(std::memory_order_relaxed);
  context.display.view_applies =
      diag.view_applies.load(std::memory_order_relaxed);
  context.display.query_skips =
      diag.query_skips.load(std::memory_order_relaxed);
  context.display.cut_applied =
      diag.cut_applied.load(std::memory_order_relaxed);
  context.display.cut_skipped_no_view =
      diag.cut_skipped_no_view.load(std::memory_order_relaxed);
  context.display.pose_invalidated =
      diag.pose_invalidated.load(std::memory_order_relaxed);
  context.display.extrapolated_frames =
      diag.extrapolated_frames.load(std::memory_order_relaxed);
  context.display.extrapolation_timeouts =
      diag.extrapolation_timeouts.load(std::memory_order_relaxed);
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
    const VehicleRuntime &runtime = context.vehicle_runtime;
    context.display.runtime_state = runtime.resolve_state;
    context.display.runtime_code = runtime.resolve_code;
    context.display.runtime_chain_cm = runtime.chain_max_cm;
    context.display.runtime_chain_mismatches = runtime.chain_mismatches;
    context.display.vehicle_present =
        runtime.vehicle_present.load(std::memory_order_acquire);
    context.display.vehicle_presence_known =
        runtime.vehicle_sample_ns.load(std::memory_order_relaxed) != 0;
    context.display.vehicle_read_failures =
        runtime.vehicle_read_failures.load(std::memory_order_relaxed);
    std::uint8_t gate = kProfileGateOff;
    if (runtime.profile_state == kProfileGateHashMismatch) {
      gate = kProfileGateHash;
    } else if (runtime.profile_state == kProfileGateServiceMissing) {
      gate = kProfileGateService;
    } else if (runtime.profile_state == kProfileGateReadFailed) {
      gate = kProfileGateRead;
    } else if (runtime.profile_state == kProfileGateReady) {
      if (runtime.chain_failed) {
        gate = kProfileGateOffset;
      } else if (runtime.resolve_state == kVehicleRuntimeFailed) {
        gate = kProfileGateResolve;
      } else if (runtime.resolve_state == kVehicleRuntimeReady) {
        gate = kProfileGateOk;
      }
    }
    context.display.runtime_gate = gate;
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
  for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
    context.display.group_modes[index] = NormalizeGroupMode(
        static_cast<SettingsGroup>(index),
        context.group_modes[index].load(std::memory_order_acquire));
  }
  context.display.fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  context.display.camera_distance_setting_m =
      context.camera_distance_m.load(std::memory_order_acquire);
  context.display.position_spring =
      context.position_spring.load(std::memory_order_acquire);
  context.display.yaw_spring =
      context.yaw_spring.load(std::memory_order_acquire);
  context.display.drift_yaw_spring =
      context.drift_yaw_spring.load(std::memory_order_acquire);
  context.display.look_yaw_spring =
      context.look_yaw_spring.load(std::memory_order_acquire);
  context.display.yaw_return_spring =
      context.yaw_return_spring.load(std::memory_order_acquire);
  context.display.pitch_spring =
      context.pitch_spring.load(std::memory_order_acquire);
  context.display.roll_spring =
      context.roll_spring.load(std::memory_order_acquire);
  context.display.camera_height_setting_m =
      context.camera_height_m.load(std::memory_order_acquire);
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
  context.display.burnout_enabled =
      context.burnout_enabled.load(std::memory_order_acquire);
  context.display.burnout_entry_delay =
      context.burnout_entry_delay.load(std::memory_order_acquire);
  context.display.burnout_entry_time =
      context.burnout_entry_time.load(std::memory_order_acquire);
  context.display.burnout_exit_delay =
      context.burnout_exit_delay.load(std::memory_order_acquire);
  context.display.burnout_exit_time =
      context.burnout_exit_time.load(std::memory_order_acquire);
  context.display.burnout_launch_exit_time =
      context.burnout_launch_exit_time.load(std::memory_order_acquire);
  context.display.burnout_fov =
      context.burnout_fov.load(std::memory_order_acquire);
  context.display.burnout_shake =
      context.burnout_shake.load(std::memory_order_acquire);
  context.display.burnout_side_mode =
      context.burnout_side_mode.load(std::memory_order_acquire);
  context.display.burnout_entry_speed =
      context.burnout_entry_speed.load(std::memory_order_acquire);
  context.display.burnout_hold_speed =
      context.burnout_hold_speed.load(std::memory_order_acquire);
  context.display.orbit_enabled =
      context.orbit_enabled.load(std::memory_order_acquire);
  context.display.orbit_hide_ui =
      context.orbit_hide_ui.load(std::memory_order_acquire);
  context.display.orbit_entry_delay =
      context.orbit_entry_delay.load(std::memory_order_acquire);
  context.display.orbit_period =
      context.orbit_period.load(std::memory_order_acquire);
  context.display.orbit_radius =
      context.orbit_radius.load(std::memory_order_acquire);
  context.display.orbit_height =
      context.orbit_height.load(std::memory_order_acquire);
  context.display.orbit_look_at_height =
      context.orbit_look_at_height.load(std::memory_order_acquire);
  context.display.orbit_weave =
      context.orbit_weave.load(std::memory_order_acquire);
  context.display.orbit_target_sway =
      context.orbit_target_sway.load(std::memory_order_acquire);
  context.display.orbit_fov =
      context.orbit_fov.load(std::memory_order_acquire);
  context.display.orbit_shake_strength =
      context.orbit_shake_strength.load(std::memory_order_acquire);
  context.display.orbit_shake_frequency =
      context.orbit_shake_frequency.load(std::memory_order_acquire);
  context.display.burnout_shot_blend =
      context.presenter_burnout_blend.load(std::memory_order_relaxed);
  context.display.burnout_shot_side =
      context.presenter_burnout_side.load(std::memory_order_relaxed);
  context.display.burnout_shot_fallbacks =
      context.presenter_burnout_fallbacks.load(std::memory_order_relaxed);
  context.display.orbit_blend =
      context.presenter_orbit_blend.load(std::memory_order_relaxed);
  context.display.orbit_state =
      static_cast<nfs2015_chase::OrbitPhase>(
          context.presenter_orbit_mode.load(std::memory_order_relaxed));
  const nfs2015_chase::OrbitUiHideState &orbit_ui =
      context.orbit_ui.machine.state();
  context.display.orbit_ui_phase = orbit_ui.phase;
  context.display.orbit_ui_was_hidden = orbit_ui.was_hidden;
  context.display.orbit_ui_hidden = orbit_ui.hidden_by_us;
  context.display.orbit_ui_intervened = orbit_ui.intervened;
  context.display.orbit_ui_readback_supported = orbit_ui.readback_supported;
  context.display.orbit_ui_readback_visible = orbit_ui.last_visible;
  context.display.orbit_ui_hide_requests = orbit_ui.hide_requests;
  context.display.orbit_ui_restore_requests = orbit_ui.restore_requests;
  context.display.orbit_ui_errors = orbit_ui.errors;
  context.display.orbit_hold_seconds =
      context.presenter_orbit_hold_seconds.load(std::memory_order_relaxed);
  context.display.orbit_time =
      context.presenter_orbit_time.load(std::memory_order_relaxed);
  context.display.orbit_phase0_degrees =
      context.presenter_orbit_phase0_degrees.load(
          std::memory_order_relaxed);
  context.display.orbit_speed_mps =
      context.presenter_orbit_speed_mps.load(std::memory_order_relaxed);
  context.display.orbit_input =
      context.presenter_orbit_input.load(std::memory_order_relaxed);
  context.display.orbit_shake_x_m =
      context.presenter_orbit_shake_x_m.load(std::memory_order_relaxed);
  context.display.orbit_shake_y_m =
      context.presenter_orbit_shake_y_m.load(std::memory_order_relaxed);
  context.display.orbit_target_sway_x_m =
      context.presenter_orbit_target_sway_x_m.load(
          std::memory_order_relaxed);
  context.display.orbit_target_sway_y_m =
      context.presenter_orbit_target_sway_y_m.load(
          std::memory_order_relaxed);
  context.display.orbit_roll_noise_degrees =
      context.presenter_orbit_roll_noise_degrees.load(
          std::memory_order_relaxed);
  context.display.slope_follow =
      context.slope_follow.load(std::memory_order_acquire);
  context.display.slope_strength =
      context.slope_strength.load(std::memory_order_acquire);
  context.display.roll_only_in_drift =
      context.roll_only_in_drift.load(std::memory_order_acquire);
  context.display.diagnostic_flat_pose =
      context.diagnostic_flat_pose.load(std::memory_order_acquire);
  context.display.diagnostic_skip_position =
      context.diagnostic_skip_position.load(std::memory_order_acquire);
  context.display.diagnostic_skip_rotation =
      context.diagnostic_skip_rotation.load(std::memory_order_acquire);
}

void RetryHookIfNeeded(Context &context) noexcept {
  const bool view_point_ready =
      context.game_view_point_hook_id.load(std::memory_order_acquire) != 0 ||
      context.game_view_point_pattern_unavailable;
  const bool scene_init_ready =
      context.scene_view_init_hook_id.load(std::memory_order_acquire) != 0 ||
      context.scene_view_init_pattern_unavailable;
  if (view_point_ready && scene_init_ready) {
    return;
  }
  if (context.hook_retry_requested.exchange(false,
                                             std::memory_order_acq_rel)) {
    context.next_hook_retry = 0.0;
  }
  if (context.elapsed < context.next_hook_retry) {
    return;
  }
  context.next_hook_retry = context.elapsed + kHookRetrySeconds;
  static_cast<void>(EnsureHook(context));
}

}  // namespace nfs2015_chase_cam::detail
