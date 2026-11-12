// The settings window and the diagnostics panel. Everything here runs on the UI
// thread against a snapshot of the shared state, so the only shared pieces it
// needs are the state itself and the settings entry points.
#include "internal.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <string_view>

namespace nfs2015_chase_cam::detail {

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
  // Phase 0: the raw reflected signals that will drive the burnout shot.
  // Kept at the top of the diagnostics body because this is the only data the
  // next phase needs from the current session.
  {
    const auto bit = [](const bool known, const bool value) -> char {
      return known ? (value ? '1' : '0') : '-';
    };
    std::snprintf(
        line, sizeof(line), "known %c  direct %c  fallback %c  active %c  hold %.2fs",
        display.burnout_signal_known ? '1' : '0',
        display.burnout_direct ? '1' : '0',
        display.burnout_fallback ? '1' : '0',
        display.burnout_active ? '1' : '0', display.burnout_hold_remaining);
    DrawStatusLine(ui, localizer, "diag.burnout_detection",
                   "Burnout detection", line);
    std::snprintf(
        line, sizeof(line),
        "L %c  R %c  int %.2f  thr %.2f  HB %.0f  spd %+.2f",
        display.burnout_left ? '1' : '0',
        display.burnout_right ? '1' : '0', display.burnout_intensity,
        display.burnout_throttle, display.burnout_handbrake,
        display.burnout_speed_mps);
    DrawStatusLine(ui, localizer, "diag.burnout_sample", "Burnout sample",
                   line);
    const char *shot_side = display.burnout_shot_side < 0
                                ? "L"
                                : (display.burnout_shot_side > 0 ? "R" : "C");
    std::snprintf(line, sizeof(line), "blend %.2f  side %s  fallbacks %llu",
                  display.burnout_shot_blend, shot_side,
                  static_cast<unsigned long long>(
                      display.burnout_shot_fallbacks));
    DrawStatusLine(ui, localizer, "diag.burnout_shot", "Burnout shot",
                   line);
    DrawStatusLine(
        ui, localizer, "diag.burnout_signal", "Burnout flag",
        display.burnout_probe_is_burnout_known
            ? (display.burnout_probe_is_burnout ? "1" : "0")
            : "unknown");
    std::snprintf(
        line, sizeof(line), "THR %.2f  BRK %.2f  STR %+.2f  SKID %.1f",
        display.burnout_probe_throttle_known
            ? display.burnout_probe_throttle
            : -1.0,
        display.burnout_probe_brake_known ? display.burnout_probe_brake : -1.0,
        display.burnout_probe_steering_known
            ? display.burnout_probe_steering
            : 0.0,
        display.burnout_probe_skid_known ? display.burnout_probe_skid_angle
                                         : 0.0);
    DrawStatusLine(ui, localizer, "diag.burnout_inputs", "Burnout inputs",
                   line);
    std::snprintf(
        line, sizeof(line),
        "n=%d  B %c%c%c%c  R %c%c%c%c  HB12 %c  B0 %c",
        display.burnout_probe_wheel_count_known
            ? static_cast<int>(display.burnout_probe_wheel_count)
            : -1,
        bit(display.burnout_probe_wheel_burnout_known[0],
            display.burnout_probe_wheel_burnout[0]),
        bit(display.burnout_probe_wheel_burnout_known[1],
            display.burnout_probe_wheel_burnout[1]),
        bit(display.burnout_probe_wheel_burnout_known[2],
            display.burnout_probe_wheel_burnout[2]),
        bit(display.burnout_probe_wheel_burnout_known[3],
            display.burnout_probe_wheel_burnout[3]),
        bit(display.burnout_probe_wheel_rep_known[0],
            display.burnout_probe_wheel_rep[0]),
        bit(display.burnout_probe_wheel_rep_known[1],
            display.burnout_probe_wheel_rep[1]),
        bit(display.burnout_probe_wheel_rep_known[2],
            display.burnout_probe_wheel_rep[2]),
        bit(display.burnout_probe_wheel_rep_known[3],
            display.burnout_probe_wheel_rep[3]),
        bit(display.burnout_probe_handbrake12_known,
            display.burnout_probe_handbrake12),
        bit(display.burnout_probe_handbrake0_known,
            display.burnout_probe_handbrake0));
    DrawStatusLine(ui, localizer, "diag.burnout_wheels", "Burnout wheels",
                   line);
    std::snprintf(
        line, sizeof(line), "W %.1f / %.1f / %.1f / %.1f",
        display.burnout_probe_wheel_angular_known[0]
            ? display.burnout_probe_wheel_angular[0]
            : 0.0,
        display.burnout_probe_wheel_angular_known[1]
            ? display.burnout_probe_wheel_angular[1]
            : 0.0,
        display.burnout_probe_wheel_angular_known[2]
            ? display.burnout_probe_wheel_angular[2]
            : 0.0,
        display.burnout_probe_wheel_angular_known[3]
            ? display.burnout_probe_wheel_angular[3]
            : 0.0);
    DrawStatusLine(ui, localizer, "diag.burnout_wheel_speed",
                   "Wheel angular", line);
    std::snprintf(line, sizeof(line), "resolved %u/%zu  ok %u  failed %u",
                  static_cast<unsigned>(display.burnout_probe_resolved),
                  kBurnoutProbeCount,
                  static_cast<unsigned>(display.burnout_probe_ok),
                  static_cast<unsigned>(display.burnout_probe_failed));
    DrawStatusLine(ui, localizer, "diag.burnout_probe", "Burnout probe",
                   line);
    if (display.burnout_probe_input_raw_size != 0) {
      char hex[64]{};
      int used = 0;
      const std::size_t count =
          (std::min)(display.burnout_probe_input_raw_size, std::size_t{8});
      for (std::size_t index = 0; index < count; ++index) {
        used += std::snprintf(hex + used,
                              sizeof(hex) - static_cast<std::size_t>(used),
                              "%02x ",
                              display.burnout_probe_input_raw[index]);
      }
      DrawStatusLine(ui, localizer, "diag.burnout_raw", "InputInfo raw",
                     hex);
    }
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

      const std::string restore_defaults_label = context->localizer.Text(
          "setting.restore_defaults", "Restore defaults");
      if (ui->button(ui->user, StringView(restore_defaults_label), 0.0F,
                     0.0F) != 0) {
        RestoreDefaults(*context);
      }

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

      if (display.preset == 0) {
        ui->separator(ui->user);
        const std::string springs_label = context->localizer.Text(
            "setting.springs", "Camera springs");
        ui->text(ui->user, StringView(springs_label));
        const std::string springs_hint = context->localizer.Text(
            "setting.springs_hint",
            "Higher = faster follow; look spring: more side offset");
        ui->text(ui->user, StringView(springs_hint));

        const auto spring_slider =
            [&](const char *key, const char *fallback,
                const char *widget_key, const double value,
                const double minimum, const double maximum,
                std::atomic<double> &target) {
              const std::string label = context->localizer.Label(
                  key, fallback, widget_key);
              float slider_value = static_cast<float>(value);
              if (ui->slider_float(
                      ui->user, StringView(label), &slider_value,
                      static_cast<float>(minimum),
                      static_cast<float>(maximum)) != 0) {
                target.store(
                    nfs2015_chase::Clamp(
                        static_cast<double>(slider_value), minimum, maximum),
                    std::memory_order_release);
                MarkSettingsDirty(*context);
              }
            };
        spring_slider("setting.position_spring", "Camera position spring",
                      "chase-spring-position", display.position_spring,
                      kMinimumPositionSpring, kMaximumPositionSpring,
                      context->position_spring);
        spring_slider("setting.yaw_spring", "Normal yaw spring",
                      "chase-spring-yaw", display.yaw_spring,
                      kMinimumYawSpring, kMaximumYawSpring,
                      context->yaw_spring);
        spring_slider("setting.drift_yaw_spring",
                      "Drift yaw spring (camera position)",
                      "chase-spring-drift-yaw", display.drift_yaw_spring,
                      kMinimumDriftYawSpring, kMaximumDriftYawSpring,
                      context->drift_yaw_spring);
        spring_slider("setting.look_yaw_spring",
                      "Drift look spring (larger = more side offset)",
                      "chase-look-yaw-spring", display.look_yaw_spring,
                      kMinimumLookYawSpring, kMaximumLookYawSpring,
                      context->look_yaw_spring);
        spring_slider("setting.yaw_return_spring",
                      "Drift yaw return spring",
                      "chase-spring-yaw-return", display.yaw_return_spring,
                      kMinimumYawReturnSpring, kMaximumYawReturnSpring,
                      context->yaw_return_spring);
        spring_slider("setting.pitch_spring", "Pitch spring",
                      "chase-spring-pitch", display.pitch_spring,
                      kMinimumPitchSpring, kMaximumPitchSpring,
                      context->pitch_spring);
        spring_slider("setting.roll_spring", "Roll spring",
                      "chase-spring-roll", display.roll_spring,
                      kMinimumRollSpring, kMaximumRollSpring,
                      context->roll_spring);
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

      const std::string wheel_steer_label = context->localizer.Label(
          "setting.wheel_steer_enabled", "NFS drift front wheels",
          "chase-wheel-steer-enabled");
      int wheel_steer_enabled = display.wheel_steer_enabled ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(wheel_steer_label),
                       &wheel_steer_enabled) != 0) {
        context->wheel_steer_enabled.store(wheel_steer_enabled != 0,
                                           std::memory_order_release);
        MarkSettingsDirty(*context);
      }
      if (display.wheel_steer_enabled) {
        const std::string wheel_hint = context->localizer.Text(
            "setting.wheel_steer_hint",
            "During a drift the front wheels follow the actual travel "
            "direction. Entry speed only gates entering; once engaged, "
            "low speed does not release the countersteer. The countersteer "
            "sample point moves along the front axle and mirrors with the "
            "drift direction: 0 is the axle centre, +100% the inside front "
            "wheel, -100% the outside wheel.");
        ui->text(ui->user, StringView(wheel_hint));
        const auto wheel_slider =
            [&](const char *key, const char *fallback,
                const char *widget_key, const double value,
                const double minimum, const double maximum,
                std::atomic<double> &target) {
              const std::string label = context->localizer.Label(
                  key, fallback, widget_key);
              float slider_value = static_cast<float>(value);
              if (ui->slider_float(
                      ui->user, StringView(label), &slider_value,
                      static_cast<float>(minimum),
                      static_cast<float>(maximum)) != 0) {
                target.store(
                    nfs2015_chase::Clamp(
                        static_cast<double>(slider_value), minimum, maximum),
                    std::memory_order_release);
                MarkSettingsDirty(*context);
              }
            };
        wheel_slider("setting.wheel_steer_entry_speed",
                     "Drift entry speed (km/h)",
                     "chase-wheel-steer-entry-speed",
                     display.wheel_steer_entry_speed_kmh,
                     kMinimumWheelSteerEntrySpeedKmh,
                     kMaximumWheelSteerEntrySpeedKmh,
                     context->wheel_steer_entry_speed_kmh);
        wheel_slider("setting.wheel_steer_max",
                     "Front wheel visual limit (deg)",
                     "chase-wheel-steer-max", display.wheel_steer_max_degrees,
                     kMinimumWheelSteerMaxDegrees,
                     kMaximumWheelSteerMaxDegrees,
                     context->wheel_steer_max_degrees);
        wheel_slider("setting.wheel_steer_gain", "Countersteer gain",
                     "chase-wheel-steer-gain", display.wheel_steer_gain,
                     kMinimumWheelSteerGain, kMaximumWheelSteerGain,
                     context->wheel_steer_gain);
        wheel_slider("setting.wheel_steer_sample",
                     "Countersteer sample point (% half track, -out/+in)",
                     "chase-wheel-steer-sample",
                     display.wheel_steer_sample_percent,
                     kMinimumWheelSteerSamplePercent,
                     kMaximumWheelSteerSamplePercent,
                     context->wheel_steer_sample_percent);
        wheel_slider("setting.wheel_steer_full_slip",
                     "Full-angle slip (deg)", "chase-wheel-steer-full-slip",
                     display.wheel_steer_full_slip_degrees,
                     kMinimumWheelSteerFullSlipDegrees,
                     kMaximumWheelSteerFullSlipDegrees,
                     context->wheel_steer_full_slip_degrees);
        wheel_slider("setting.wheel_steer_rate",
                     "Steer rate (deg/s)", "chase-wheel-steer-rate",
                     display.wheel_steer_rate_degrees_per_second,
                     kMinimumWheelSteerRateDegreesPerSecond,
                     kMaximumWheelSteerRateDegreesPerSecond,
                     context->wheel_steer_rate_degrees_per_second);
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

      ui->separator(ui->user);
      ui->text(ui->user,
               StringView(context->localizer.Text(
                   "setting.burnout", "Burnout close-up (Shift + W)")));
      const std::string burnout_enabled_label = context->localizer.Label(
          "setting.burnout_enabled", "Enable burnout close-up",
          "chase-burnout-enabled");
      int burnout_enabled = display.burnout_enabled ? 1 : 0;
      if (ui->checkbox(ui->user, StringView(burnout_enabled_label),
                       &burnout_enabled) != 0) {
        context->burnout_enabled.store(burnout_enabled != 0,
                                       std::memory_order_release);
        MarkSettingsDirty(*context);
      }
      const auto burnout_slider =
          [&](const char *key, const char *fallback, const char *widget_key,
              const double value, const double minimum, const double maximum,
              std::atomic<double> &target) {
            const std::string label = context->localizer.Label(
                key, fallback, widget_key);
            float slider_value = static_cast<float>(value);
            if (ui->slider_float(
                    ui->user, StringView(label), &slider_value,
                    static_cast<float>(minimum),
                    static_cast<float>(maximum)) != 0) {
              target.store(
                  nfs2015_chase::Clamp(static_cast<double>(slider_value),
                                       minimum, maximum),
                  std::memory_order_release);
              MarkSettingsDirty(*context);
            }
          };
      burnout_slider("setting.burnout_fov", "Shot horizontal FOV",
                     "chase-burnout-fov", display.burnout_fov,
                     kMinimumBurnoutFov, kMaximumBurnoutFov,
                     context->burnout_fov);
      burnout_slider("setting.burnout_shake", "Shot shake strength",
                     "chase-burnout-shake", display.burnout_shake,
                     kMinimumBurnoutShake, kMaximumBurnoutShake,
                     context->burnout_shake);
      burnout_slider("setting.burnout_entry_delay", "Entry delay (s)",
                     "chase-burnout-entry-delay",
                     display.burnout_entry_delay, kMinimumBurnoutEntryDelay,
                     kMaximumBurnoutEntryDelay, context->burnout_entry_delay);
      burnout_slider("setting.burnout_entry_time", "Entry time (s)",
                     "chase-burnout-entry-time", display.burnout_entry_time,
                     kMinimumBurnoutEntryTime, kMaximumBurnoutEntryTime,
                     context->burnout_entry_time);
      burnout_slider("setting.burnout_exit_delay", "Exit delay (s)",
                     "chase-burnout-exit-delay", display.burnout_exit_delay,
                     kMinimumBurnoutExitDelay, kMaximumBurnoutExitDelay,
                     context->burnout_exit_delay);
      burnout_slider("setting.burnout_exit_time", "Exit time (s)",
                     "chase-burnout-exit-time", display.burnout_exit_time,
                     kMinimumBurnoutExitTime, kMaximumBurnoutExitTime,
                     context->burnout_exit_time);
      burnout_slider("setting.burnout_launch_exit_time",
                     "Launch exit time (s)", "chase-burnout-launch-exit-time",
                     display.burnout_launch_exit_time,
                     kMinimumBurnoutLaunchExitTime,
                     kMaximumBurnoutLaunchExitTime,
                     context->burnout_launch_exit_time);
      burnout_slider("setting.burnout_entry_speed", "Entry speed (m/s)",
                     "chase-burnout-entry-speed",
                     display.burnout_entry_speed, kMinimumBurnoutEntrySpeed,
                     kMaximumBurnoutEntrySpeed, context->burnout_entry_speed);
      // Keep the pair valid even when the entry slider is raised.
      const double entry_speed =
          context->burnout_entry_speed.load(std::memory_order_acquire);
      if (context->burnout_hold_speed.load(std::memory_order_acquire) <
          entry_speed) {
        context->burnout_hold_speed.store(entry_speed,
                                          std::memory_order_release);
      }
      // Hold speed must not be below entry speed; clamp the slider's lower
      // bound so the persisted document always passes the validator.
      const double hold_minimum = (std::max)(
          kMinimumBurnoutHoldSpeed,
          context->burnout_entry_speed.load(std::memory_order_acquire));
      burnout_slider("setting.burnout_hold_speed", "Hold speed (m/s)",
                     "chase-burnout-hold-speed",
                     (std::max)(display.burnout_hold_speed, hold_minimum),
                     hold_minimum, kMaximumBurnoutHoldSpeed,
                     context->burnout_hold_speed);
      ui->text(ui->user,
               StringView(context->localizer.Text(
                   "setting.burnout_side", "Shot side (latched on entry)")));
      const auto burnout_side_button = [&](const std::uint32_t mode,
                                           const char *key,
                                           const char *fallback) {
        std::string label;
        label.append(display.burnout_side_mode == mode ? "> " : "  ");
        label.append(context->localizer.Text(key, fallback));
        if (ui->button(ui->user, StringView(label), 0.0F, 0.0F) != 0) {
          context->burnout_side_mode.store(mode, std::memory_order_release);
          MarkSettingsDirty(*context);
        }
      };
      burnout_side_button(0, "burnout.side.auto", "Auto / steering");
      burnout_side_button(1, "burnout.side.left", "Left rear");
      burnout_side_button(2, "burnout.side.right", "Right rear");
      burnout_side_button(3, "burnout.side.center", "Centred");

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
        if (display.wheel_steer_enabled) {
          char wheel_line[192]{};
          std::snprintf(
              wheel_line, sizeof(wheel_line),
              "%+.1f° / %+.1f°  (%u %s, %s)  sample %+.0f%%",
              display.wheel_steer_angle_degrees,
              display.wheel_steer_target_degrees,
              display.wheel_steer_wheels, "wheels",
              display.wheel_steer_active ? "active" : "stock",
              display.wheel_steer_sample_percent);
          DrawStatusLine(ui, context->localizer, "status.wheel_steer",
                         "Front wheel steer", wheel_line);
        }
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
                   "While driving, Z cycles chase, third person and first "
                   "person. Turning this camera off hands Z back to the game.")));
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

}  // namespace nfs2015_chase_cam::detail
