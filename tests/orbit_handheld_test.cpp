// Acceptance tests for the parked-orbit handheld noise (TG-8).
//
// These tests do not load the game or the Anomaly runtime. They pin the
// reference LCG value noise, the target-only offset, the local right/up
// decomposition, the strength/frequency behavior and the stage gating. The
// numbering matches PLAN-nfs2015-orbit-handheld-shake.md section 6.1.

#include "../orbit_shot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase::Add;
using nfs2015_chase::BurnoutSample;
using nfs2015_chase::CarBasis;
using nfs2015_chase::CarState;
using nfs2015_chase::Cross;
using nfs2015_chase::Dot;
using nfs2015_chase::kOrbitHandheldHorizontalM;
using nfs2015_chase::kOrbitHandheldRoughness;
using nfs2015_chase::kOrbitHandheldSeedX;
using nfs2015_chase::kOrbitHandheldSeedY;
using nfs2015_chase::kOrbitHandheldVerticalM;
using nfs2015_chase::kOrbitLookAtHeightM;
using nfs2015_chase::kOrbitPivotForwardM;
using nfs2015_chase::Length;
using nfs2015_chase::MakeCarBasis;
using nfs2015_chase::Mul;
using nfs2015_chase::Normalize;
using nfs2015_chase::OrbitPhase;
using nfs2015_chase::OrbitShot;
using nfs2015_chase::OrbitShotConfig;
using nfs2015_chase::OrbitShotResult;
using nfs2015_chase::Sub;
using nfs2015_chase::Vec3;

namespace noise = nfs2015_chase::orbit_shot_detail;

constexpr double kDt = 1.0 / 60.0;

void Fail(const char* label, const char* detail, const double actual,
          const double expected) {
  std::fprintf(stderr, "FAIL %s: %s, got %.12f (expected %.12f)\n", label,
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
  std::printf("ok   %-52s %.12f (expected %.12f)\n", label, actual, expected);
}

[[nodiscard]] CarState MakeCar() noexcept {
  CarState car;
  car.position = {2.0, -3.0, 0.5};
  car.yaw = 0.4;
  car.pitch = 0.0;
  car.speed = 0.0;
  car.on_ground = true;
  return car;
}

[[nodiscard]] Vec3 BehindCamera(const CarState& car) noexcept {
  const CarBasis basis = MakeCarBasis(car.yaw, car.pitch);
  return Add(car.position,
             Add(Mul(basis.forward, -6.0), Vec3{0.0, 0.0, 0.6}));
}

[[nodiscard]] Vec3 OrbitPivot(const CarState& car) noexcept {
  const CarBasis basis = MakeCarBasis(car.yaw, car.pitch);
  return Add(Add(car.position, Mul(basis.up, kOrbitLookAtHeightM)),
             Mul(basis.forward, kOrbitPivotForwardM));
}

[[nodiscard]] noise::ValueNoiseBand MakeBand(
    const std::uint32_t seed, const double period) noexcept {
  return {period, 0.0, 0.0, 0.0, seed};
}

struct Fixture final {
  OrbitShotConfig config{};
  OrbitShot shot{};
  CarState car{};
  BurnoutSample sample{};
  Vec3 camera{};
  OrbitShotResult last{};

  explicit Fixture(const double strength = 1.0,
                   const double frequency = 5.0,
                   const double entry_delay = 0.0,
                   const double entry_time = 1e-6) {
    config.handheld_strength = strength;
    config.handheld_frequency_hz = frequency;
    config.entry_delay = entry_delay;
    config.entry_time = entry_time;
    shot.SetConfig(config);
    car = MakeCar();
    camera = BehindCamera(car);
  }

  void Update(const double dt) noexcept {
    last = shot.Update(dt, car, sample, camera);
  }

  void AdvanceSeconds(const double seconds, const double dt = kDt) noexcept {
    const int frames = static_cast<int>(std::lround(seconds / dt));
    for (int frame = 0; frame < frames; ++frame) {
      Update(dt);
    }
  }

  // entry_delay = 0 and entry_time = 1e-6 make the state machine reach Orbit
  // on the second update. The first update latches without advancing noise.
  void EnterOrbitFast() noexcept {
    Update(0.0);
    Update(1.0e-3);
  }
};

constexpr double kExpectedX[8] = {
    -0.915875435, 0.916224957,  0.120165586, -0.671736002,
    -0.766323090, 0.908896208,  0.952592611, 0.825675011,
};

constexpr double kExpectedY[8] = {
    0.629856825, -0.805774927, 0.043700457, -0.875714779,
    -0.437681437, 0.899122477, 0.558944702, 0.203995228,
};

void TestReferenceSequence() {
  noise::ValueNoiseBand x =
      MakeBand(kOrbitHandheldSeedX, 0.2);
  noise::ValueNoiseBand y =
      MakeBand(kOrbitHandheldSeedY, 0.2);

  // The first period interpolates from the zero initial value; every sample
  // up to and including t = 0.2 s must be exactly zero.
  for (int step = 0; step < 4; ++step) {
    ExpectNear("6.1-1 first period x", noise::ValueNoiseStep(x, 0.05),
               0.0, 1e-12);
    ExpectNear("6.1-1 first period y", noise::ValueNoiseStep(y, 0.05),
               0.0, 1e-12);
  }

  // From t = 0.4 s on, the cosine interpolation lands exactly on a grid point
  // every four 0.05 s steps.
  for (int index = 0; index < 8; ++index) {
    double value_x = 0.0;
    double value_y = 0.0;
    for (int step = 0; step < 4; ++step) {
      value_x = noise::ValueNoiseStep(x, 0.05);
      value_y = noise::ValueNoiseStep(y, 0.05);
    }
    ExpectNear("6.1-3 x reference sequence",
               value_x, kExpectedX[index], 1e-6);
    ExpectNear("6.1-2 y reference sequence",
               value_y, kExpectedY[index], 1e-6);
  }
}

void TestBoundsAndSmoothness() {
  Fixture fixture(1.0, 5.0);
  fixture.EnterOrbitFast();

  double previous_x = 0.0;
  double previous_y = 0.0;
  bool have_previous = false;
  bool bounds_ok = true;
  bool smooth_ok = true;
  double maximum_x = 0.0;
  double maximum_y = 0.0;

  constexpr double dt = 1.0 / 240.0;
  for (int frame = 0; frame < 60 * 240; ++frame) {
    fixture.Update(dt);
    const double shake_x = fixture.last.state.shake_x_m;
    const double shake_y = fixture.last.state.shake_y_m;
    bounds_ok =
        bounds_ok &&
        std::fabs(shake_x) <=
            kOrbitHandheldHorizontalM * (1.0 + kOrbitHandheldRoughness) +
                1e-9 &&
        std::fabs(shake_y) <=
            kOrbitHandheldVerticalM * (1.0 + kOrbitHandheldRoughness) + 1e-9;
    maximum_x = (std::max)(maximum_x, std::fabs(shake_x));
    maximum_y = (std::max)(maximum_y, std::fabs(shake_y));
    if (have_previous) {
      smooth_ok = smooth_ok &&
                  std::fabs(shake_x - previous_x) < 0.012 &&
                  std::fabs(shake_y - previous_y) < 0.012;
    }
    previous_x = shake_x;
    previous_y = shake_y;
    have_previous = true;
  }
  Expect("6.1-4 x and y stay inside their amplitudes", bounds_ok);
  Expect("6.1-5 x and y stay smooth at 240 Hz", smooth_ok);
  Expect("6.1-4 x actually moves", maximum_x > 0.005);
  Expect("6.1-4 y actually moves", maximum_y > 0.04);
}

void TestDeterminismAndReset() {
  Fixture fixture(1.0, 5.0);
  fixture.EnterOrbitFast();

  constexpr int kFrames = 300;
  double first_x[kFrames]{};
  double first_y[kFrames]{};
  for (int frame = 0; frame < kFrames; ++frame) {
    fixture.Update(1.0 / 120.0);
    first_x[frame] = fixture.last.state.shake_x_m;
    first_y[frame] = fixture.last.state.shake_y_m;
  }

  fixture.shot.Reset();
  Expect("6.1-6 Reset clears the offsets",
         fixture.shot.state().shake_x_m == 0.0 &&
             fixture.shot.state().shake_y_m == 0.0);
  fixture.EnterOrbitFast();

  bool deterministic = true;
  for (int frame = 0; frame < kFrames; ++frame) {
    fixture.Update(1.0 / 120.0);
    if (fixture.last.state.shake_x_m != first_x[frame] ||
        fixture.last.state.shake_y_m != first_y[frame]) {
      deterministic = false;
    }
  }
  Expect("6.1-6 Reset makes the sequence repeat", deterministic);
}

void TestTargetOnly() {
  Fixture shaken(1.0, 5.0);
  Fixture still(0.0, 5.0);
  shaken.EnterOrbitFast();
  still.EnterOrbitFast();

  bool positions_match = true;
  bool up_roll_match = true;
  bool target_differs = false;
  for (int frame = 0; frame < 600; ++frame) {
    constexpr double dt = 1.0 / 120.0;
    shaken.Update(dt);
    still.Update(dt);
    positions_match =
        positions_match && shaken.last.view.position.x ==
                               still.last.view.position.x &&
        shaken.last.view.position.y == still.last.view.position.y &&
        shaken.last.view.position.z == still.last.view.position.z;
    up_roll_match =
        up_roll_match && shaken.last.view.up.x == still.last.view.up.x &&
        shaken.last.view.up.y == still.last.view.up.y &&
        shaken.last.view.up.z == still.last.view.up.z &&
        shaken.last.view.roll == still.last.view.roll &&
        shaken.last.view.roll_degrees == still.last.view.roll_degrees &&
        shaken.last.view.fov_y_degrees == still.last.view.fov_y_degrees;
    target_differs =
        target_differs ||
        Length(Sub(shaken.last.view.target, still.last.view.target)) > 1e-4;
  }
  Expect("6.1-7 noise never moves the eye", positions_match);
  Expect("6.1-7 noise never moves up/roll/fov", up_roll_match);
  Expect("6.1-7 noise moves the target", target_differs);
  Expect("6.1-7 strength 0 has no offset",
         still.last.state.shake_x_m == 0.0 &&
             still.last.state.shake_y_m == 0.0);
  Expect("6.1-7 strength 1 has an offset",
         shaken.last.state.shake_x_m != 0.0 ||
             shaken.last.state.shake_y_m != 0.0);
}

void TestLocalBasis() {
  Fixture fixture(1.0, 5.0);
  fixture.EnterOrbitFast();
  fixture.AdvanceSeconds(3.0, 1.0 / 120.0);

  const Vec3 pivot = OrbitPivot(fixture.car);
  const Vec3 offset = Sub(fixture.last.view.target, pivot);
  Vec3 look = Sub(pivot, fixture.last.view.position);
  Expect("6.1-8 target is not at the pivot", Length(offset) > 1e-6);
  look = Normalize(look);
  Vec3 right = Cross(Vec3{0.0, 0.0, 1.0}, look);
  Expect("6.1-8 right has a stable basis", Length(right) > 1e-9);
  right = Normalize(right);
  const Vec3 up_local = Normalize(Cross(look, right));

  const double along_look = Dot(offset, look);
  const double along_right = Dot(offset, right);
  const double along_up = Dot(offset, up_local);
  ExpectNear("6.1-8 offset has no along-look component", along_look, 0.0,
             1e-9);
  ExpectNear("6.1-8 offset equals shake_x on right", along_right,
             fixture.last.state.shake_x_m, 1e-9);
  ExpectNear("6.1-8 offset equals shake_y on local up", along_up,
             fixture.last.state.shake_y_m, 1e-9);
  Expect("6.1-8 horizontal stays inside its amplitude",
         std::fabs(along_right) <=
             kOrbitHandheldHorizontalM * (1.0 + kOrbitHandheldRoughness) +
                 1e-9);
  Expect("6.1-8 vertical stays inside its amplitude",
         std::fabs(along_up) <=
             kOrbitHandheldVerticalM * (1.0 + kOrbitHandheldRoughness) +
                 1e-9);
}

void TestStrengthLinearity() {
  Fixture one(1.0, 5.0);
  Fixture two(2.0, 5.0);
  one.EnterOrbitFast();
  two.EnterOrbitFast();

  bool linear = true;
  bool nonzero = false;
  for (int frame = 0; frame < 600; ++frame) {
    constexpr double dt = 1.0 / 120.0;
    one.Update(dt);
    two.Update(dt);
    linear = linear &&
             std::fabs(two.last.state.shake_x_m -
                       2.0 * one.last.state.shake_x_m) <= 1e-9 &&
             std::fabs(two.last.state.shake_y_m -
                       2.0 * one.last.state.shake_y_m) <= 1e-9;
    nonzero = nonzero || one.last.state.shake_x_m != 0.0 ||
              one.last.state.shake_y_m != 0.0;
  }
  Expect("6.1-9 strength 1 produces an offset", nonzero);
  Expect("6.1-9 strength 2 is exactly twice strength 1", linear);
}

void TestFrequency() {
  noise::ValueNoiseBand five =
      MakeBand(kOrbitHandheldSeedY, 0.2);
  noise::ValueNoiseBand ten =
      MakeBand(kOrbitHandheldSeedY, 0.1);
  double value_five = 0.0;
  double value_ten = 0.0;
  for (int step = 0; step < 8; ++step) {
    value_five = noise::ValueNoiseStep(five, 0.05);
    value_ten = noise::ValueNoiseStep(ten, 0.05);
  }
  // 5 Hz has two periods at t = 0.4 s and lands on r1; 10 Hz has four and
  // lands on r3.
  ExpectNear("6.1-10 5 Hz t=0.4 is r1", value_five, kExpectedY[0], 1e-6);
  ExpectNear("6.1-10 10 Hz t=0.4 is r3", value_ten, kExpectedY[2], 1e-6);
}

void TestFrequencyChangeContinuity() {
  Fixture fixture(1.0, 5.0);
  fixture.EnterOrbitFast();
  fixture.AdvanceSeconds(2.0, 1.0 / 120.0);
  const double before_x = fixture.last.state.shake_x_m;
  const double before_y = fixture.last.state.shake_y_m;
  Expect("6.1-10 frequency change starts from an offset",
         before_x != 0.0 || before_y != 0.0);

  fixture.config.handheld_frequency_hz = 10.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.Update(0.0);
  ExpectNear("6.1-10 frequency change keeps x continuous",
             fixture.last.state.shake_x_m, before_x, 1e-12);
  ExpectNear("6.1-10 frequency change keeps y continuous",
             fixture.last.state.shake_y_m, before_y, 1e-12);
}

void TestStageGating() {
  Fixture fixture(1.0, 5.0, 8.0, 2.0);
  fixture.AdvanceSeconds(7.9);
  Expect("6.1-11 idle before 8 s",
         fixture.last.state.mode == OrbitPhase::kIdle);
  Expect("6.1-11 idle has no offset",
         fixture.last.state.shake_x_m == 0.0 &&
             fixture.last.state.shake_y_m == 0.0);

  fixture.AdvanceSeconds(0.4);
  Expect("6.1-11 entered the transition",
         fixture.last.state.mode == OrbitPhase::kEnter);
  Expect("6.1-11 enter advances the noise",
         fixture.last.state.shake_x_m != 0.0 ||
             fixture.last.state.shake_y_m != 0.0);

  fixture.sample.input_active = true;
  fixture.AdvanceSeconds(1.1);
  Expect("6.1-11 input returns to idle",
         fixture.last.state.mode == OrbitPhase::kIdle);
  Expect("6.1-11 idle clears the offsets",
         fixture.last.state.shake_x_m == 0.0 &&
             fixture.last.state.shake_y_m == 0.0);
}

}  // namespace

int main() {
  TestReferenceSequence();
  TestBoundsAndSmoothness();
  TestDeterminismAndReset();
  TestTargetOnly();
  TestLocalBasis();
  TestStrengthLinearity();
  TestFrequency();
  TestFrequencyChangeContinuity();
  TestStageGating();
  std::printf("all parked orbit handheld tests passed\n");
  return 0;
}
