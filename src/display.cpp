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
  for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
    context.display.group_modes[index] = NormalizeGroupMode(
        static_cast<SettingsGroup>(index),
        context.group_modes[index].load(std::memory_order_acquire));
  }
  context.display.fov_enabled =
      context.fov_enabled.load(std::memory_order_acquire);
  context.display.camera_scale =
      context.camera_scale.load(std::memory_order_acquire);
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
  context.display.burnout_shot_blend =
      context.presenter_burnout_blend.load(std::memory_order_relaxed);
  context.display.burnout_shot_side =
      context.presenter_burnout_side.load(std::memory_order_relaxed);
  context.display.burnout_shot_fallbacks =
      context.presenter_burnout_fallbacks.load(std::memory_order_relaxed);
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

}  // namespace nfs2015_chase_cam::detail
