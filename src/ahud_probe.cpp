// The once-per-frame AHUD probe: frame timing, camera error, and the residual
// diagnostics behind vertical/horizontal camera error. Every entry point here
// is a no-op until anomaly.ue5.ahud is published, so this cannot break a
// working install.
#include "internal.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nfs2015_chase_cam::detail {

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

}  // namespace nfs2015_chase_cam::detail
