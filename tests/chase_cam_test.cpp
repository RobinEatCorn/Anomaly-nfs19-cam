// Acceptance test for the NFS 2015 chase-camera port.
//
// The straight-line numbers come from the scenario table in
// `NFS2015-相机算法分析报告.md` §6.3, produced independently by the
// TypeScript reference implementation.
//
// The drift numbers in that table were produced by the same reference, but
// its `desiredPosition()` rotates the local rest offset only by the yaw *lag*,
// not by the car yaw. That leaves the camera on the world axis whenever the
// car is not facing +X (the table's straight case starts at yaw 0, which is
// why it did not show). The C++ port rotates the offset by the camera's
// absolute azimuth, i.e. it applies the car-local pivot orbit that the report
// describes. The drift checks below therefore pin the corrected geometry:
// significant lateral offset, the correct roll/FOV, and a car body visibly
// off-axis, without copying the reference's unrotated-offset numbers.

#include "../car_sample.hpp"
#include "../chase_cam.hpp"
#include "../latency_window.hpp"
#include "../view_blend.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase::CarState;
using nfs2015_chase::ChaseCam;
using nfs2015_chase::Config;
using nfs2015_chase::ForwardFromYaw;
using nfs2015_chase::Vec3;

constexpr double kToleranceDistance = 0.16;
constexpr double kToleranceHeight = 0.05;
constexpr double kToleranceFov = 1.0;

struct Result final {
  double horizontal_distance{};
  double height{};
  double lateral{};
  double car_body_off_axis_degrees{};
  double roll_degrees{};
  double fov_y_degrees{};
};

[[nodiscard]] Result RunScenario(const Config& config, const bool drift) noexcept {
  ChaseCam camera(config);
  CarState car;
  car.position = {0.0, 0.0, 0.0};
  car.yaw = 0.0;
  car.velocity = {20.0, 0.0, 0.0};
  car.speed = 20.0;
  car.acceleration = {0.0, 0.0, 0.0};
  camera.Reset(car);

  constexpr double kDt = 1.0 / 60.0;
  for (int frame = 0; frame < 36; ++frame) {
    if (drift) {
      car.yaw += 1.2 * kDt;
      car.yaw_rate = 1.2;
      car.slip_angle = -0.55;
      car.velocity =
          nfs2015_chase::Mul(ForwardFromYaw(car.yaw + car.slip_angle), 20.0);
      // The report's acceleration.x = 8 is world +X in the reference axes.
      car.acceleration = {0.0, 8.0, 0.0};
    }
    static_cast<void>(camera.Update(kDt, car));
  }
  const auto view = camera.Update(kDt, car);

  const Vec3 offset = nfs2015_chase::Sub(view.position, car.position);
  const Vec3 right = nfs2015_chase::RightFromYaw(car.yaw);
  Result result;
  result.horizontal_distance = std::hypot(offset.x, offset.y);
  result.height = offset.z;
  result.lateral = nfs2015_chase::Dot(offset, right);
  const double view_yaw =
      std::atan2(view.target.y - view.position.y,
                 view.target.x - view.position.x);
  result.car_body_off_axis_degrees = nfs2015_chase::WrapPi(view_yaw - car.yaw) *
                                     nfs2015_chase::kDegreesPerRadian;
  result.roll_degrees = view.roll_degrees;
  result.fov_y_degrees = view.fov_y_degrees;
  return result;
}

void Fail(const char* label, const char* detail, const double actual,
          const double expected) {
  std::fprintf(stderr, "FAIL %s: %s, got %.3f (expected %.3f)\n", label,
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
                const double tolerance) {
  const double difference = std::fabs(actual - expected);
  if (difference > tolerance) {
    Fail(label, "outside tolerance", actual, expected);
  }
  std::printf("ok   %-34s %.3f (expected %.3f)\n", label, actual, expected);
}

void ExpectBetween(const char* label, const double actual, const double low,
                   const double high) {
  if (!(actual >= low && actual <= high)) {
    std::fprintf(stderr, "FAIL %s: %.3f not in [%.3f, %.3f]\n", label, actual,
                 low, high);
    std::exit(1);
  }
  std::printf("ok   %-34s %.3f in [%.2f, %.2f]\n", label, actual, low, high);
}

void CheckStraight(const char* name, const Config& config,
                   const double distance, const double height,
                   const double fov) {
  const Result result = RunScenario(config, false);
  std::printf("%s\n", name);
  ExpectNear("horizontal distance", result.horizontal_distance, distance,
             kToleranceDistance);
  ExpectNear("camera height", result.height, height, kToleranceHeight);
  ExpectNear("lateral offset", result.lateral, 0.0, 0.01);
  ExpectNear("car body off-axis", result.car_body_off_axis_degrees, 0.0, 0.01);
  ExpectNear("roll", result.roll_degrees, 0.0, 0.01);
  ExpectNear("vertical FOV", result.fov_y_degrees, fov, kToleranceFov);
}

void CheckDrift(const char* name, const Config& config,
                const double minimum_distance, const double maximum_distance,
                const double minimum_lateral, const double expected_fov,
                const double minimum_roll) {
  const Result result = RunScenario(config, true);
  std::printf("%s\n", name);
  ExpectBetween("horizontal distance", result.horizontal_distance,
                minimum_distance, maximum_distance);
  ExpectBetween("camera height", result.height, 0.9, 1.6);
  ExpectBetween("lateral offset", result.lateral, minimum_lateral, 6.0);
  ExpectBetween("car body off-axis",
                std::fabs(result.car_body_off_axis_degrees), 20.0, 75.0);
  ExpectBetween("roll (right drift banks left)", result.roll_degrees,
                minimum_roll, -0.1);
  ExpectNear("vertical FOV", result.fov_y_degrees, expected_fov,
             kToleranceFov);
}

}  // namespace

int main() {
  using nfs2015_chase::CinematicPreset;
  using nfs2015_chase::GamePreset;

  // §6.3 straight-line rows: distance / height / FOV.
  CheckStraight("cinematic straight", CinematicPreset(), 5.28, 1.57, 56.8);
  CheckStraight("game straight", GamePreset(), 6.35, 1.78, 48.2);

  // §6.3 drift rows, with the local-offset rotation corrected. The camera
  // stays close, drops, hangs to one side, banks, widens the lens, and leaves
  // the car body well off the view axis.
  CheckDrift("cinematic heavy drift", CinematicPreset(), 3.0, 3.6, 1.2, 58.3,
             -12.0);
  CheckDrift("game heavy drift", GamePreset(), 4.7, 5.3, 2.0, 47.3, -3.0);

  // The game POV stores horizontal FOV. The retail 70..95 horizontal range
  // corresponds to 43.0..63.1 vertical at 16:9.
  nfs2015_chase::CameraState view;
  view.fov_y_degrees = 43.0;
  ExpectNear("43.0 vertical -> horizontal",
             view.HorizontalFovDegrees(16.0 / 9.0), 70.0, 0.1);
  view.fov_y_degrees = 63.1;
  ExpectNear("63.1 vertical -> horizontal",
             view.HorizontalFovDegrees(16.0 / 9.0), 95.0, 0.2);

  // The view detour interpolates the last two Game-tick snapshots. Yaw must
  // take the short way around +/-180 instead of sweeping through zero.
  {
    nfs2015_chase::ViewPose from;
    from.position = {-2.0, 4.0, 1.0};
    from.pitch_degrees = -5.0;
    from.yaw_degrees = 179.0;
    from.roll_degrees = 2.0;
    from.fov_degrees = 70.0;
    nfs2015_chase::ViewPose to;
    to.position = {8.0, 12.0, 3.0};
    to.pitch_degrees = 1.0;
    to.yaw_degrees = -179.0;
    to.roll_degrees = -2.0;
    to.fov_degrees = 90.0;
    const auto blended = nfs2015_chase::BlendViewPose(from, to, 0.5);
    ExpectNear("interpolate position x", blended.position.x, 3.0, 0.001);
    ExpectNear("interpolate position y", blended.position.y, 8.0, 0.001);
    ExpectNear("interpolate position z", blended.position.z, 2.0, 0.001);
    ExpectNear("interpolate pitch", blended.pitch_degrees, -2.0, 0.001);
    ExpectNear("interpolate roll", blended.roll_degrees, 0.0, 0.001);
    ExpectNear("interpolate fov", blended.fov_degrees, 80.0, 0.001);
    const double yaw_error = std::fabs(nfs2015_chase::WrapPi(
        (blended.yaw_degrees - 180.0) * nfs2015_chase::kRadiansPerDegree));
    ExpectNear("interpolate yaw across 180", yaw_error, 0.0, 0.001);

    constexpr std::int64_t kTick30 = 33333333;
    constexpr std::int64_t kTick60 = 16666667;
    ExpectNear("interpolation at publish", nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick30, kTick30, 0.05), 0.0, 0.001);
    ExpectNear("interpolation halfway", nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick30, kTick30 + kTick30 / 2, 0.05), 0.5, 0.001);
    ExpectNear("interpolation at next tick", nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick30, 2 * kTick30, 0.05), 1.0, 0.001);
    // A 100 ms hitch cannot make the camera trail by more than 50 ms.
    ExpectNear("interpolation hitch cap", nfs2015_chase::ViewInterpolationAlpha(
                   0, 100000000, 100000000, 0.05), 0.5, 0.001);
    ExpectNear("interpolation clamps before", nfs2015_chase::ViewInterpolationAlpha(
                   kTick60, 2 * kTick60, 0, 0.05), 0.0, 0.001);
    // The city can drop below 20 Hz. A 50 ms delay cap made alpha jump from
    // 24% to 100% at each publish there; the plugin now allows one full tick.
    constexpr std::int64_t kTick15 = 66666667;
    ExpectNear("interpolation at 15 Hz publish",
               nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick15, kTick15, 0.25),
               0.0, 0.001);
    ExpectNear("interpolation at 15 Hz halfway",
               nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick15, kTick15 + kTick15 / 2, 0.25),
               0.5, 0.001);
    ExpectNear("interpolation at 15 Hz next tick",
               nfs2015_chase::ViewInterpolationAlpha(
                   0, kTick15, 2 * kTick15, 0.25),
               1.0, 0.001);
  }

  // The speed/accel distance multiplier controls how far NTE's stronger
  // acceleration pushes the camera. 0 keeps the retail rest distance.
  {
    Config config = GamePreset();
    config.speed_distance_scale = 0.0;
    CarState car;
    car.position = {10.0, 20.0, 0.0};
    car.velocity = {20.0, 0.0, 0.0};
    car.speed = 20.0;
    ChaseCam camera(config);
    camera.Reset(car);
    const auto scaled_view = camera.Update(0.0, car);
    const Vec3 offset =
        nfs2015_chase::Sub(scaled_view.position, car.position);
    ExpectNear("speed distance scale 0", std::hypot(offset.x, offset.y),
               5.10, 0.05);
  }

  // Reset settles at the current operating point, so it keeps the speed
  // offset instead of jumping to the zero-speed rest and springing back.
  {
    CarState car;
    car.velocity = {25.0, 0.0, 0.0};
    car.speed = 25.0;
    ChaseCam camera(GamePreset());
    camera.Reset(car);
    const auto reset_view = camera.Update(0.0, car);
    const Vec3 offset =
        nfs2015_chase::Sub(reset_view.position, car.position);
    ExpectNear("reset keeps the speed offset", std::hypot(offset.x, offset.y),
               6.40, 0.10);
  }

  // CameraRoll is not drift-gated in the retail model; the optional setting
  // can gate it for players who want a level horizon in normal cornering.
  {
    CarState car;
    car.velocity = {20.0, 0.0, 0.0};
    car.speed = 20.0;
    car.yaw_rate = 1.0;
    car.acceleration = {0.0, 8.0, 0.0};
    car.slip_angle = 0.0;

    Config gated = GamePreset();
    gated.roll_only_in_drift = true;
    ChaseCam gated_camera(gated);
    gated_camera.Reset(car);
    const auto gated_view = gated_camera.Update(0.0, car);
    ExpectNear("normal-corner roll, drift gate on",
               gated_view.roll_degrees, 0.0, 0.01);

    Config ungated = GamePreset();
    ungated.roll_only_in_drift = false;
    ChaseCam ungated_camera(ungated);
    ungated_camera.Reset(car);
    const auto ungated_view = ungated_camera.Update(0.0, car);
    if (std::fabs(ungated_view.roll_degrees) < 0.5) {
      std::fprintf(stderr, "FAIL normal-corner roll without gate is %.3f\n",
                   ungated_view.roll_degrees);
      return 1;
    }
    std::printf("ok   %-34s %.3f\n", "normal-corner roll, gate off",
                ungated_view.roll_degrees);

    Config drift_gate = GamePreset();
    drift_gate.roll_only_in_drift = true;
    const Result drift_result = RunScenario(drift_gate, true);
    ExpectBetween("drift roll with drift gate", drift_result.roll_degrees,
                  -3.0, -0.05);
  }

  // Braking pitch is a separate NFS 2015 action-cam behaviour from the speed
  // dolly. The optional scale must be able to remove the nose dip without
  // touching the rest pose.
  {
    const auto pitch_after_braking = [](const double pitch_scale) {
      Config config = GamePreset();
      config.pitch_scale = pitch_scale;
      CarState car;
      car.velocity = {20.0, 0.0, 0.0};
      car.speed = 20.0;
      car.acceleration = {-8.0, 0.0, 0.0};
      ChaseCam camera(config);
      camera.Reset(car);
      return camera.Update(0.0, car).pitch_degrees;
    };
    const double no_pitch = pitch_after_braking(0.0);
    const double retail_pitch = pitch_after_braking(1.0);
    if (!(std::fabs(retail_pitch - no_pitch) > 1.0)) {
      std::fprintf(stderr,
                   "FAIL braking pitch scale is inactive: none=%.3f "
                   "retail=%.3f\n",
                   no_pitch, retail_pitch);
      return 1;
    }
    std::printf("ok   %-34s %.3f -> %.3f\n",
                "braking pitch scale", no_pitch, retail_pitch);

    const auto pitch_for_brake = [](const double brake) {
      CarState car;
      car.velocity = {20.0, 0.0, 0.0};
      car.speed = 20.0;
      car.acceleration = {-8.0, 0.0, 0.0};
      car.brake = brake;
      ChaseCam camera(GamePreset());
      camera.Reset(car);
      return camera.Update(0.0, car).pitch_degrees;
    };
    const double gated_pitch = pitch_for_brake(0.0);
    const double derived_pitch = pitch_for_brake(-1.0);
    if (!(std::fabs(derived_pitch - gated_pitch) > 1.0)) {
      std::fprintf(stderr,
                   "FAIL explicit brake gate is inactive: gated=%.3f "
                   "derived=%.3f\n",
                   gated_pitch, derived_pitch);
      return 1;
    }
    std::printf("ok   %-34s %.3f -> %.3f\n",
                "brake input overrides acceleration", gated_pitch,
                derived_pitch);
  }

  // The retail speed shake is a small camera-position noise. Its strength
  // must be independently adjustable so a player can remove the residual
  // size shimmer without changing the rest pose.
  {
    CarState car;
    car.velocity = {60.0, 0.0, 0.0};
    car.speed = 60.0;
    Config still_config = GamePreset();
    still_config.shake_scale = 0.0;
    Config shaking_config = GamePreset();
    shaking_config.shake_scale = 1.0;
    ChaseCam still(still_config);
    ChaseCam shaking(shaking_config);
    still.Reset(car);
    shaking.Reset(car);
    constexpr double kDt = 1.0 / 60.0;
    for (int frame = 0; frame < 60; ++frame) {
      static_cast<void>(still.Update(kDt, car));
      static_cast<void>(shaking.Update(kDt, car));
    }
    const auto still_view = still.Update(kDt, car);
    const auto shaking_view = shaking.Update(kDt, car);
    const double difference =
        nfs2015_chase::Length(nfs2015_chase::Sub(still_view.position,
                                                 shaking_view.position));
    if (!(difference > 0.001)) {
      std::fprintf(stderr, "FAIL shake scale is inactive: %.6f\n",
                   difference);
      return 1;
    }
    std::printf("ok   %-34s %.5f m\n", "shake scale displacement",
                difference);
  }

  // Reset must place a non-zero-yaw car's camera behind that car, not on the
  // world -X axis. At zero speed and dt = 0 the position spring does not move,
  // so this checks the local pivot orbit directly.
  {
    CarState car;
    car.yaw = 1.1;
    car.position = {100.0, -50.0, 3.0};
    car.velocity = {0.0, 0.0, 0.0};
    ChaseCam camera(GamePreset());
    camera.Reset(car);
    const auto at_reset = camera.Update(0.0, car);
    const Vec3 offset = nfs2015_chase::Sub(at_reset.position, car.position);
    const double behind = nfs2015_chase::Dot(offset, ForwardFromYaw(car.yaw));
    if (!(behind < -4.0)) {
      std::fprintf(stderr, "FAIL reset is not behind the car: %.3f\n", behind);
      return 1;
    }
    std::printf("ok   %-34s %.3f\n", "reset is behind the car", behind);
  }

  // A chase camera must follow car translation rigidly. An absolute-position
  // spring lags every move, which becomes a visible car-distance pulse when
  // the city tick drops; the offset spring must carry the full 10 m step.
  {
    CarState car;
    car.position = {0.0, 0.0, 0.0};
    car.velocity = {0.0, 0.0, 0.0};
    ChaseCam camera(GamePreset());
    camera.Reset(car);
    const auto before = camera.Update(0.0, car);
    car.position.x += 10.0;
    const auto after = camera.Update(1.0 / 15.0, car);
    ExpectNear("translation step is rigid", after.position.x - before.position.x,
               10.0, 0.05);
  }

  // --- T0 diagnostics: the rolling latency window -------------------------
  // The camera tick cost is judged by p99 and max, not by an average: a tick
  // that is usually 0.2 ms but sometimes 9 ms is a different bug from one that
  // is uniformly 4 ms. The histogram must separate the two.
  {
    using nfs2015_chase_cam::LatencyWindow;
    constexpr double kWidth = LatencyWindow::kBucketMilliseconds;
    LatencyWindow window;
    ExpectNear("empty latency average", window.Average(), 0.0, 1e-12);
    ExpectNear("empty latency p99", window.Percentile(0.99), 0.0, 1e-12);

    for (int sample = 0; sample < 95; ++sample) {
      window.Add(0.20);
    }
    for (int sample = 0; sample < 5; ++sample) {
      window.Add(9.0);
    }
    ExpectNear("latency average ignores rank", window.Average(),
               (0.20 * 95.0 + 9.0 * 5.0) / 100.0, 1e-9);
    ExpectNear("latency max is the peak", window.Maximum(),
               static_cast<double>(LatencyWindow::kBuckets) * kWidth, 1e-9);
    if (!(window.Percentile(0.99) > 1.0)) {
      std::fprintf(stderr, "FAIL latency p99 missed the peak: %.3f\n",
                   window.Percentile(0.99));
      return 1;
    }
    std::printf("ok   %-34s p99=%.2f max=%.2f ms\n", "latency p99 catches a spike",
                window.Percentile(0.99), window.Maximum());

    // Filling the window past its capacity must evict the spike entirely, not
    // leave it in the histogram forever: the numbers describe the last N
    // ticks, which is what makes them usable for a live diagnosis.
    for (int sample = 0; sample < static_cast<int>(LatencyWindow::kCapacity);
         ++sample) {
      window.Add(0.10);
    }
    ExpectNear("evicted latency average", window.Average(), 0.10, 1e-9);
    ExpectNear("evicted latency max", window.Maximum(), 3.0 * kWidth, 0.03);
    if (window.Percentile(0.99) > 1.0) {
      std::fprintf(stderr, "FAIL latency spike was not evicted: %.3f\n",
                   window.Percentile(0.99));
      return 1;
    }
    std::printf("ok   %-34s p99=%.2f ms\n", "latency window evicts spikes",
                window.Percentile(0.99));

    window.Reset();
    if (window.Count() != 0) {
      std::fprintf(stderr, "FAIL latency reset kept %zu samples\n",
                   window.Count());
      return 1;
    }
    std::printf("ok   %-34s count=%zu\n", "latency reset clears the window",
                window.Count());
  }

  std::printf("all chase-camera checks passed\n");
    {
    nfs2015_chase::CarSample a;
    a.time_ns = 1000;
    a.car.position = {0.0, 0.0, 0.0};
    a.car.yaw = 0.0;
    a.anchor_cm[0] = 0.0;
    nfs2015_chase::CarSample b;
    b.time_ns = 3000;
    b.car.position = {10.0, 20.0, 0.0};
    b.car.speed = 30.0;
    b.anchor_cm[0] = 10.0;
    nfs2015_chase::SampleRing ring;
    ring.Write(a);
    ring.Write(b);
    nfs2015_chase::CarSample mid;
    Expect("ring interpolates between samples", ring.At(2000, mid));
    ExpectNear("ring position midpoint x", mid.car.position.x, 5.0, 1e-9);
    ExpectNear("ring position midpoint y", mid.car.position.y, 10.0, 1e-9);
    ExpectNear("ring speed midpoint", mid.car.speed, 15.0, 1e-9);
    ExpectNear("ring anchor midpoint", mid.anchor_cm[0], 5.0, 1e-9);
    ExpectNear("ring time midpoint", static_cast<double>(mid.time_ns), 2000.0, 1.0);
  }
  {
    // The oldest sample carries the car past the +-180 degree seam; the blend
    // must go through 180, not back through 0.
    nfs2015_chase::CarSample a;
    a.time_ns = 0;
    a.car.yaw = 179.0 * nfs2015_chase::kRadiansPerDegree;
    nfs2015_chase::CarSample b;
    b.time_ns = 1000;
    b.car.yaw = -179.0 * nfs2015_chase::kRadiansPerDegree;
    nfs2015_chase::SampleRing ring;
    ring.Write(a);
    ring.Write(b);
    nfs2015_chase::CarSample mid;
    Expect("ring interpolates across the yaw seam", ring.At(500, mid));
    const double degrees =
        std::remainder(mid.car.yaw, 2.0 * nfs2015_chase::kPi) *
        nfs2015_chase::kDegreesPerRadian;
    ExpectNear("yaw seam stays at 180", std::fabs(degrees), 180.0, 0.5);
  }
  {
    nfs2015_chase::SampleRing ring;
    nfs2015_chase::CarSample out;
    Expect("empty ring reports no sample", !ring.At(0, out));
    nfs2015_chase::CarSample a;
    a.time_ns = 5000;
    a.car.position = {1.0, 2.0, 3.0};
    ring.Write(a);
    Expect("single sample answers any time", ring.At(0, out));
    ExpectNear("single sample clamps", out.car.position.z, 3.0, 1e-9);
    nfs2015_chase::CarSample b = a;
    b.time_ns = 6000;
    b.car.position = {2.0, 2.0, 3.0};
    ring.Write(b);
    Expect("time past the newest clamps", ring.At(999999, out));
    ExpectNear("past newest uses the newest", out.car.position.x, 2.0, 1e-9);
    ring.Clear();
    Expect("cleared ring reports no sample", !ring.At(5500, out));
  }
  {
    // Wrap-around: more writes than slots must keep the newest kCapacity.
    nfs2015_chase::SampleRing ring;
    for (int index = 0; index < 20; ++index) {
      nfs2015_chase::CarSample sample;
      sample.time_ns = index * 1000;
      sample.car.speed = static_cast<double>(index);
      ring.Write(sample);
    }
    nfs2015_chase::CarSample newest;
    Expect("ring latest after wrap", ring.Latest(newest));
    ExpectNear("ring latest is the last write", newest.car.speed, 19.0, 1e-9);
    nfs2015_chase::CarSample older;
    Expect("ring still holds the oldest retained slot", ring.At(15000, older));
    ExpectNear("ring wraps at capacity", older.car.speed, 15.0, 1e-9);
  }
  {
    // AtOneTickBehind: the blend must start at the previous sample when the
    // newest one lands, reach it one span later, and never clamp in between.
    nfs2015_chase::CarSample a;
    a.time_ns = 1000;
    a.car.position = {0.0, 0.0, 0.0};
    nfs2015_chase::CarSample b;
    b.time_ns = 3000;  // span 2000 ns
    b.car.position = {20.0, 0.0, 0.0};
    nfs2015_chase::SampleRing ring;
    ring.Write(a);
    ring.Write(b);
    nfs2015_chase::CarSample out;
    Expect("lag blend needs a sample", ring.AtOneTickBehind(3000, out));
    ExpectNear("lag blend starts at the previous sample", out.car.position.x,
               0.0, 1e-9);
    Expect("lag blend halfway", ring.AtOneTickBehind(4000, out));
    ExpectNear("lag blend is halfway through the span", out.car.position.x,
               10.0, 1e-9);
    Expect("lag blend at the end of the span", ring.AtOneTickBehind(5000, out));
    ExpectNear("lag blend reaches the newest sample", out.car.position.x, 20.0,
               1e-9);
    // A clock slightly behind the newest sample must not run backwards.
    Expect("lag blend with a clock behind the sample",
           ring.AtOneTickBehind(2500, out));
    ExpectNear("lag blend never goes before the previous sample",
               out.car.position.x, 0.0, 1e-9);
    // A stall: the next interval is longer than the span.
    Expect("lag blend during a stall", ring.AtOneTickBehind(9000, out));
    ExpectNear("lag blend holds at the cap", out.car.position.x, 40.0, 1e-9);
  }
  {
    // Continuity: the end of one bracket and the start of the next must agree.
    // This is the property At() with an absolute delay could not hold.
    nfs2015_chase::SampleRing ring;
    for (int index = 0; index < 3; ++index) {
      nfs2015_chase::CarSample sample;
      sample.time_ns = index * 1000;
      sample.car.position = {static_cast<double>(index) * 7.0, 0.0, 0.0};
      ring.Write(sample);
    }
    // Just before the next sample lands the blend is at the newest sample.
    nfs2015_chase::CarSample before;
    Expect("lag blend before the boundary", ring.AtOneTickBehind(2999, before));
    // The sample lands. One tick of lag means we go back to the previous one,
    // but the value must not step: alpha restarts from zero on the new pair,
    // which begins exactly where the old pair ended.
    nfs2015_chase::CarSample next;
    next.time_ns = 3000;
    next.car.position = {21.0, 0.0, 0.0};
    ring.Write(next);
    nfs2015_chase::CarSample after;
    Expect("lag blend after the boundary", ring.AtOneTickBehind(3001, after));
    ExpectNear("lag blend is continuous across a sample boundary",
               after.car.position.x - before.car.position.x, 0.0, 0.2);
  }
  {
    nfs2015_chase::SampleRing ring;
    nfs2015_chase::CarSample out;
    Expect("lag blend on an empty ring", !ring.AtOneTickBehind(0, out));
    nfs2015_chase::CarSample only;
    only.time_ns = 500;
    only.car.position = {3.0, 0.0, 0.0};
    ring.Write(only);
    Expect("lag blend with one sample", ring.AtOneTickBehind(900, out));
    ExpectNear("lag blend returns the only sample", out.car.position.x, 3.0,
               1e-9);
  }
  return 0;
}
