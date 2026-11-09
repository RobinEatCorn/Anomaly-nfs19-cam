// Acceptance tests for the Phase 1 pure burnout shot model.
//
// These tests do not load the game or the Anomaly runtime. They pin the state
// machine, the NFS 2015 timings, the horizontal-FOV conversion, the orbit blend
// and the ground/eye safety floor.

#include "../burnout_shot.hpp"
#include "../chase_cam.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase::BlendBurnoutCamera;
using nfs2015_chase::BurnoutSample;
using nfs2015_chase::BurnoutShot;
using nfs2015_chase::BurnoutShotConfig;
using nfs2015_chase::BurnoutShotResult;
using nfs2015_chase::BurnoutVerticalFovDegrees;
using nfs2015_chase::CameraState;
using nfs2015_chase::CarState;
using nfs2015_chase::ChaseCam;
using nfs2015_chase::GamePreset;
using nfs2015_chase::Length;
using nfs2015_chase::Sub;
using nfs2015_chase::Vec3;

constexpr double kDt = 1.0 / 60.0;

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
                const double tolerance) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    Fail(label, "outside tolerance", actual, expected);
  }
  std::printf("ok   %-42s %.6f (expected %.6f)\n", label, actual, expected);
}

[[nodiscard]] CarState MakeCar() noexcept {
  CarState car;
  car.position = {1.0, -2.0, 0.0};
  car.yaw = 0.3;
  car.pitch = 0.0;
  car.speed = 0.0;
  car.on_ground = true;
  return car;
}

[[nodiscard]] BurnoutSample MakeSample() noexcept {
  BurnoutSample sample;
  sample.active = true;
  sample.intensity = 1.0;
  sample.speed_mps = 0.0;
  sample.steering = 0.0;
  return sample;
}

[[nodiscard]] bool FiniteCamera(const CameraState& view) noexcept {
  return std::isfinite(view.position.x) && std::isfinite(view.position.y) &&
         std::isfinite(view.position.z) && std::isfinite(view.target.x) &&
         std::isfinite(view.target.y) && std::isfinite(view.target.z) &&
         std::isfinite(view.up.x) && std::isfinite(view.up.y) &&
         std::isfinite(view.up.z) && std::isfinite(view.fov_y_degrees) &&
         std::isfinite(view.roll) && std::isfinite(view.pitch_degrees) &&
         std::isfinite(view.yaw_degrees) &&
         std::isfinite(view.roll_degrees);
}

[[nodiscard]] bool SameCamera(const CameraState& a,
                              const CameraState& b) noexcept {
  return a.position.x == b.position.x && a.position.y == b.position.y &&
         a.position.z == b.position.z && a.target.x == b.target.x &&
         a.target.y == b.target.y && a.target.z == b.target.z &&
         a.up.x == b.up.x && a.up.y == b.up.y && a.up.z == b.up.z &&
         a.fov_y_degrees == b.fov_y_degrees && a.roll == b.roll &&
         a.pitch_degrees == b.pitch_degrees &&
         a.yaw_degrees == b.yaw_degrees &&
         a.roll_degrees == b.roll_degrees;
}

[[nodiscard]] BurnoutShotResult AdvanceUntilLatched(
    BurnoutShot& shot, const CarState& car, const BurnoutSample& sample,
    const int maximum_frames = 240) noexcept {
  BurnoutShotResult result;
  for (int frame = 0; frame < maximum_frames; ++frame) {
    result = shot.Update(kDt, car, sample);
    if (result.latched) {
      return result;
    }
  }
  return result;
}

[[nodiscard]] BurnoutShotResult AdvanceUntilFull(
    BurnoutShot& shot, const CarState& car, const BurnoutSample& sample,
    const int maximum_frames = 300) noexcept {
  BurnoutShotResult result;
  for (int frame = 0; frame < maximum_frames; ++frame) {
    result = shot.Update(kDt, car, sample);
    if (result.phase >= 1.0) {
      return result;
    }
  }
  return result;
}

void TestEntryDelayAndMonotonicRise() {
  BurnoutShot shot;
  const CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  sample.steering = -1.0;

  // 29 frames = 0.4833 s. The 0.5 s entry delay must still hold the shot off.
  for (int frame = 0; frame < 29; ++frame) {
    const auto result = shot.Update(kDt, car, sample);
    if (result.phase != 0.0 || result.latched) {
      Fail("entry delay", "shot started before 0.5 s", result.phase, 0.0);
    }
  }
  const auto before = shot.Update(0.0, car, sample);
  Expect("entry delay keeps phase at zero", before.phase == 0.0);
  Expect("entry delay keeps side unlatched", !before.latched);

  // The next tick crosses 0.5 s: latch and start the 1.0 s entry.
  auto result = shot.Update(kDt, car, sample);
  Expect("entry starts after 0.5 s", result.phase > 0.0);
  Expect("entry latches the side", result.latched);
  Expect("left steering latches left", result.side == -1);

  double previous_phase = result.phase;
  double previous_blend = result.blend;
  for (int frame = 0; frame < 120; ++frame) {
    result = shot.Update(kDt, car, sample);
    Expect("phase rises monotonically", result.phase >= previous_phase - 1e-12);
    Expect("blend rises monotonically", result.blend >= previous_blend - 1e-12);
    previous_phase = result.phase;
    previous_blend = result.blend;
  }
  ExpectNear("entry reaches full shot", result.phase, 1.0, 1e-12);
  ExpectNear("entry blend reaches one", result.blend, 1.0, 1e-12);
}

void TestExitDelayAndMonotonicFall() {
  BurnoutShot shot;
  const CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  auto result = AdvanceUntilFull(shot, car, sample);
  ExpectNear("setup: shot is full before exit", result.phase, 1.0, 1e-12);

  sample.active = false;
  // 11 frames = 0.1833 s, shorter than the 0.2 s exit delay.
  for (int frame = 0; frame < 11; ++frame) {
    result = shot.Update(kDt, car, sample);
    ExpectNear("exit delay keeps phase at one", result.phase, 1.0, 1e-12);
  }

  result = shot.Update(kDt, car, sample);
  Expect("exit starts after 0.2 s", result.phase < 1.0);
  double previous_phase = result.phase;
  for (int frame = 0; frame < 180; ++frame) {
    result = shot.Update(kDt, car, sample);
    Expect("phase falls monotonically", result.phase <= previous_phase + 1e-12);
    previous_phase = result.phase;
  }
  ExpectNear("exit reaches zero", result.phase, 0.0, 1e-12);
  Expect("full exit clears the latch", !result.latched);
  ExpectNear("full exit clears the blend", result.blend, 0.0, 1e-12);
}

void TestRetriggerDuringExit() {
  BurnoutShot shot;
  const CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  static_cast<void>(AdvanceUntilFull(shot, car, sample));

  sample.active = false;
  BurnoutShotResult result;
  for (int frame = 0; frame < 50; ++frame) {
    result = shot.Update(kDt, car, sample);
  }
  Expect("setup: shot is partially out", result.phase > 0.0 && result.phase < 1.0);
  Expect("setup: side stays latched while exiting", result.latched);
  const double phase_before = result.phase;

  sample.active = true;
  result = shot.Update(kDt, car, sample);
  Expect("retrigger does not reset phase", result.phase >= phase_before);
  Expect("retrigger rises from the current phase",
         result.phase - phase_before <= kDt + 1e-12);
  Expect("retrigger keeps the latch", result.latched);

  const double before = result.phase;
  result = AdvanceUntilFull(shot, car, sample);
  Expect("retrigger reaches full shot", result.phase >= before);
  ExpectNear("retrigger full phase", result.phase, 1.0, 1e-12);
}

void TestSideLatchAndReselection() {
  BurnoutShot shot;
  const CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  sample.steering = -1.0;
  auto result = AdvanceUntilLatched(shot, car, sample);
  Expect("entry selects left from steering", result.side == -1);

  // Once latched, steering and even the per-wheel flags must not swap the side.
  sample.steering = 1.0;
  sample.left = false;
  sample.right = true;
  for (int frame = 0; frame < 30; ++frame) {
    result = shot.Update(kDt, car, sample);
  }
  Expect("latched side ignores steering changes", result.side == -1);

  sample.active = false;
  for (int frame = 0; frame < 240; ++frame) {
    result = shot.Update(kDt, car, sample);
  }
  Expect("shot exits before reselection", result.phase == 0.0);
  Expect("exit clears the side latch", !result.latched);

  sample.active = true;
  sample.steering = 1.0;
  sample.left = false;
  sample.right = true;
  result = AdvanceUntilLatched(shot, car, sample);
  Expect("new shot can select the other side", result.side == 1);

  // A single per-wheel flag is stronger than steering, but only at selection.
  sample.active = false;
  for (int frame = 0; frame < 240; ++frame) {
    result = shot.Update(kDt, car, sample);
  }
  sample.active = true;
  sample.steering = 1.0;
  sample.left = true;
  sample.right = false;
  result = AdvanceUntilLatched(shot, car, sample);
  Expect("per-wheel left wins over right steering", result.side == -1);
}

void TestFastExitOnSpeed() {
  BurnoutShot shot;
  const CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  static_cast<void>(AdvanceUntilFull(shot, car, sample));

  sample.speed_mps = 4.0;  // above the 3.0 m/s hold ceiling
  BurnoutShotResult result;
  for (int frame = 0; frame < 24; ++frame) {  // 0.4 s
    result = shot.Update(kDt, car, sample);
  }
  Expect("speed limit forces an early exit", result.phase < 0.3);
  Expect("speed limit reports force_exit", result.force_exit);
  for (int frame = 0; frame < 20; ++frame) {  // past the 0.5 s fast exit
    result = shot.Update(kDt, car, sample);
  }
  ExpectNear("speed limit reaches chase in 0.5 s", result.phase, 0.0, 1e-12);
}

void TestBlendEndpointsAndOrbit() {
  CarState car = MakeCar();
  car.position = {10.0, -4.0, 2.0};
  car.yaw = 0.7;
  car.pitch = 8.0 * nfs2015_chase::kRadiansPerDegree;
  car.speed = 0.0;

  ChaseCam chase(GamePreset());
  chase.Reset(car);
  const CameraState chase_view = chase.Update(0.0, car);

  BurnoutShotConfig config;
  config.entry_delay = 0.0;
  config.entry_time = 0.1;
  config.shake_scale = 0.0;
  BurnoutShot shot(config);
  BurnoutSample sample = MakeSample();
  const auto result = AdvanceUntilFull(shot, car, sample);
  ExpectNear("setup: shot is full", result.phase, 1.0, 1e-12);

  const CameraState at_zero =
      BlendBurnoutCamera(chase_view, result.view, car, 0.0);
  const CameraState at_one =
      BlendBurnoutCamera(chase_view, result.view, car, 1.0);
  Expect("blend 0 is exactly chase", SameCamera(at_zero, chase_view));
  Expect("blend 1 is exactly shot", SameCamera(at_one, result.view));

  const CameraState middle =
      BlendBurnoutCamera(chase_view, result.view, car, 0.5);
  Expect("orbit blend is finite", FiniteCamera(middle));
  ExpectNear("orbit blend keeps a unit look target",
             Length(Sub(middle.target, middle.position)), 1.0, 1e-9);
  Expect("orbit blend keeps up near unit", std::fabs(Length(middle.up) - 1.0) <
                                              1e-9);

  // A straight lerp would put the eye at the midpoint of the two positions.
  // The orbit path has to differ, otherwise the rear-quarter arc is not active.
  const Vec3 linear_midpoint = nfs2015_chase::Mul(
      nfs2015_chase::Add(chase_view.position, result.view.position), 0.5);
  const double orbit_offset =
      Length(Sub(middle.position, linear_midpoint));
  Expect("orbit blend is not a straight line", orbit_offset > 0.01);
  std::printf("     orbit midpoint offset %.3f m\n", orbit_offset);
}

void TestHorizontalFov() {
  constexpr double kAspect = 16.0 / 9.0;
  const double vertical = BurnoutVerticalFovDegrees(45.0, kAspect);
  const double expected =
      2.0 * std::atan(std::tan(45.0 * nfs2015_chase::kRadiansPerDegree * 0.5) /
                      kAspect) *
      nfs2015_chase::kDegreesPerRadian;
  ExpectNear("45 horizontal -> vertical FOV", vertical, expected, 1e-9);
  Expect("45 horizontal at 16:9 is about 26.25 vertical",
         vertical > 26.0 && vertical < 26.6);

  CameraState view;
  view.fov_y_degrees = vertical;
  ExpectNear("vertical FOV round-trips to 45 horizontal",
             view.HorizontalFovDegrees(kAspect), 45.0, 1e-9);
}

void TestSafetyAndFiniteOutputs() {
  const double pitches[] = {-15.0, 0.0, 15.0};
  const int modes[] = {1, 2, 3};
  const double scales[] = {0.6, 1.0, 1.4};

  for (const double pitch_degrees : pitches) {
    for (const int mode : modes) {
      for (const double scale : scales) {
        CarState car = MakeCar();
        car.position = {-8.0, 3.0, 4.0};
        car.yaw = -1.2;
        car.pitch = pitch_degrees * nfs2015_chase::kRadiansPerDegree;
        BurnoutSample sample = MakeSample();
        sample.steering = mode == 1 ? -1.0 : (mode == 2 ? 1.0 : 0.0);

        BurnoutShotConfig config;
        config.entry_delay = 0.0;
        config.entry_time = 0.1;
        config.side_mode = mode;
        config.camera_scale = scale;
        BurnoutShot shot(config);
        const auto result = AdvanceUntilFull(shot, car, sample);

        Expect("safety shot is finite", FiniteCamera(result.view));
        Expect("safety shot reaches full", result.phase >= 1.0);
        const double target_distance =
            Length(Sub(result.view.target, result.view.position));
        Expect("safety target is not degenerate",
               target_distance > nfs2015_chase::kBurnoutMinTargetDistanceM);
        Expect("safety eye stays above the car origin",
               result.view.position.z >=
                   car.position.z +
                       nfs2015_chase::kBurnoutMinEyeZAboveCarM - 1e-9);
      }
    }
  }
}

void TestInvalidAspectRatio() {
  const double value = BurnoutVerticalFovDegrees(45.0, 0.0);
  Expect("zero aspect falls back to a finite FOV", std::isfinite(value));
  Expect("zero aspect keeps a sane vertical FOV", value > 1.0 && value < 89.0);

  BurnoutShot shot;
  CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  const auto result = shot.Update(kDt, car, sample, 0.0);
  Expect("invalid aspect still produces a finite view",
         FiniteCamera(result.view));
}

}  // namespace

int main() {
  TestEntryDelayAndMonotonicRise();
  TestExitDelayAndMonotonicFall();
  TestRetriggerDuringExit();
  TestSideLatchAndReselection();
  TestFastExitOnSpeed();
  TestBlendEndpointsAndOrbit();
  TestHorizontalFov();
  TestSafetyAndFiniteOutputs();
  TestInvalidAspectRatio();
  std::printf("all burnout shot tests passed\n");
  return 0;
}
