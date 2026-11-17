// Acceptance tests for the parked orbit shot model (TG2).
//
// These tests do not load the game or the Anomaly runtime. They pin the NFS
// 2015 parked-orbit geometry (radius, elevation, look-at height), the weave
// track, the 8 s / 2 s / 1 s state machine, the entry phase, and the shared
// shot blend. The numbering matches PLAN-nfs2015-parking-orbit-cam.md section
// 6.1 so a failure points straight at the contract it violated.

#include "../burnout_shot.hpp"
#include "../orbit_shot.hpp"
#include "../shot_blend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase::Add;
using nfs2015_chase::BlendBurnoutCamera;
using nfs2015_chase::BlendShotAroundCarPivot;
using nfs2015_chase::BurnoutSample;
using nfs2015_chase::CameraState;
using nfs2015_chase::CarBasis;
using nfs2015_chase::CarState;
using nfs2015_chase::Cross;
using nfs2015_chase::Dot;
using nfs2015_chase::kBurnoutOrbitPivotHeightM;
using nfs2015_chase::kDegreesPerRadian;
using nfs2015_chase::kOrbitLookAtHeightM;
using nfs2015_chase::kPi;
using nfs2015_chase::Length;
using nfs2015_chase::MakeCarBasis;
using nfs2015_chase::Mul;
using nfs2015_chase::Normalize;
using nfs2015_chase::OrbitPhase;
using nfs2015_chase::OrbitShot;
using nfs2015_chase::OrbitShotConfig;
using nfs2015_chase::OrbitShotResult;
using nfs2015_chase::OrbitWeave;
using nfs2015_chase::Sub;
using nfs2015_chase::Vec3;
using nfs2015_chase::WrapPi;

constexpr double kDt = 1.0 / 60.0;

void Fail(const char* label, const char* detail, const double actual,
          const double expected) {
  std::fprintf(stderr, "FAIL %s: %s, got %.9f (expected %.9f)\n", label,
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
  std::printf("ok   %-46s %.9f (expected %.9f)\n", label, actual, expected);
}

[[nodiscard]] CarState MakeCar(const double pitch = 0.0) noexcept {
  CarState car;
  car.position = {2.0, -3.0, 0.5};
  car.yaw = 0.4;
  car.pitch = pitch;
  car.speed = 0.0;
  car.on_ground = true;
  return car;
}

[[nodiscard]] BurnoutSample MakeSample() noexcept {
  BurnoutSample sample;
  sample.speed_mps = 0.0;
  return sample;
}

// A chase-like camera 6 m behind the car origin and 0.6 m above it. This is
// the pose the presenter would report at rest, so the entry phase should latch
// to "behind the car".
[[nodiscard]] Vec3 BehindCamera(const CarState& car,
                                const double distance = 6.0,
                                const double height = 0.6) noexcept {
  const CarBasis basis = MakeCarBasis(car.yaw, car.pitch);
  return Add(car.position,
             Add(Mul(basis.forward, -distance), Vec3{0.0, 0.0, height}));
}

[[nodiscard]] Vec3 OrbitPivot(const CarState& car,
                              const double look_height) noexcept {
  const CarBasis basis = MakeCarBasis(car.yaw, car.pitch);
  return Add(car.position, Mul(basis.up, look_height));
}

[[nodiscard]] Vec3 OrbitOffset(const CameraState& view, const CarState& car,
                               const double look_height) noexcept {
  return Sub(view.position, OrbitPivot(car, look_height));
}

// Azimuth of the camera around the orbit pivot, in the car body frame.
[[nodiscard]] double OrbitAzimuth(const CameraState& view,
                                  const CarState& car,
                                  const double look_height) noexcept {
  const CarBasis basis = MakeCarBasis(car.yaw, car.pitch);
  const Vec3 offset = OrbitOffset(view, car, look_height);
  return std::atan2(Dot(offset, basis.right), Dot(offset, basis.forward));
}

// Signed smallest angle from `from` to `to`, in degrees. Used for 30 s
// increments, where every step is well under 180 degrees.
[[nodiscard]] double SignedDegrees(const double from,
                                   const double to) noexcept {
  return WrapPi(to - from) * kDegreesPerRadian;
}

// Counter-clockwise angle from `from` to `to`, in degrees, 0..360. Used for the
// half/full revolution checks so +180 does not wrap to -180.
[[nodiscard]] double RevolvedDegrees(const double from,
                                     const double to) noexcept {
  double delta = std::fmod(to - from, 2.0 * kPi);
  if (delta < 0.0) {
    delta += 2.0 * kPi;
  }
  return delta * kDegreesPerRadian;
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

// One model plus the car/sample/camera it consumes. The default camera is the
// chase pose behind the car, so the entry phase is deterministic unless a test
// overrides it.
struct Fixture final {
  OrbitShotConfig config{};
  OrbitShot shot{config};
  CarState car = MakeCar();
  BurnoutSample sample = MakeSample();
  Vec3 camera{};
  OrbitShotResult last{};

  explicit Fixture(const double pitch = 0.0, const double weave = 0.0) noexcept
      : car(MakeCar(pitch)) {
    config.weave_amplitude_m = weave;
    shot.SetConfig(config);
    camera = BehindCamera(car);
  }

  void AdvanceFrames(const int frames, const double dt = kDt) noexcept {
    for (int frame = 0; frame < frames; ++frame) {
      last = shot.Update(dt, car, sample, camera);
    }
  }

  void AdvanceSeconds(const double seconds, const double dt = kDt) noexcept {
    AdvanceFrames(static_cast<int>(std::lround(seconds / dt)), dt);
  }

  // 8 s parked + exactly 2 s of entry: the returned frame is t = 0 of the orbit
  // and its clock must be zero.
  void EnterOrbit() noexcept {
    AdvanceSeconds(8.0);
    AdvanceSeconds(2.0);
  }
};

[[nodiscard]] CameraState SyntheticBaseView() noexcept {
  CameraState base;
  base.position = {-5.0, 0.0, 2.0};
  base.target = {0.0, 0.0, 1.0};
  base.up = {0.0, 0.0, 1.0};
  base.fov_y_degrees = 60.0;
  base.roll = 0.01;
  base.pitch_degrees = -5.0;
  base.yaw_degrees = 10.0;
  base.roll_degrees = 0.57;
  return base;
}

void TestEntryDelay() {
  Fixture fixture;
  fixture.AdvanceSeconds(7.9);
  ExpectNear("6.1-1 blend at 7.9 s", fixture.last.blend, 0.0, 0.0);
  Expect("6.1-1 still idle at 7.9 s",
         fixture.last.state.mode == OrbitPhase::kIdle);
  fixture.AdvanceFrames(6);  // exactly 8.0 s
  Expect("6.1-1 enters at 8.0 s",
         fixture.last.state.mode == OrbitPhase::kEnter);
  ExpectNear("6.1-1 latch frame keeps blend 0", fixture.last.blend, 0.0, 0.0);
  fixture.AdvanceFrames(1);
  Expect("6.1-1 blend rises after 8.0 s", fixture.last.blend > 0.0);
}

void TestEntryDuration() {
  Fixture fixture;
  fixture.AdvanceSeconds(8.0);
  fixture.AdvanceFrames(119);
  Expect("6.1-2 still entering one frame before 2.0 s",
         fixture.last.state.mode == OrbitPhase::kEnter &&
             fixture.last.blend < 1.0);
  fixture.AdvanceFrames(1);
  ExpectNear("6.1-2 blend after 2.0 s", fixture.last.blend, 1.0, 1e-12);
  Expect("6.1-2 reached orbit mode",
         fixture.last.state.mode == OrbitPhase::kOrbit);
  ExpectNear("6.1-2 orbit clock starts at 0", fixture.last.state.time, 0.0,
             0.0);
}

void TestGeometryAndAngles() {
  Fixture fixture;
  fixture.EnterOrbit();

  const double theta0 =
      OrbitAzimuth(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  const Vec3 offset0 =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 start_position = fixture.last.view.position;
  ExpectNear("6.1-3 |pos-pivot| at t=0", Length(offset0), 5.05, 0.01);
  ExpectNear("6.1-4 dot(pos-pivot, body up) at t=0", Dot(offset0, basis.up),
             0.598, 0.01);
  ExpectNear("entry phase matches the first pose", SignedDegrees(theta0,
                                                                fixture.last
                                                                    .state
                                                                    .phase0),
             0.0, 1e-6);
  ExpectNear("entry phase latches the chase azimuth", theta0, kPi, 1e-9);

  fixture.AdvanceSeconds(30.0);
  const double theta30 =
      OrbitAzimuth(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  const Vec3 offset30 =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  ExpectNear("6.1-3 |pos-pivot| at t=30", Length(offset30), 5.05, 0.01);
  ExpectNear("6.1-4 dot(pos-pivot, body up) at t=30", Dot(offset30, basis.up),
             0.598, 0.01);
  ExpectNear("6.1-5 revolved angle at t=30", RevolvedDegrees(theta0, theta30),
             180.0, 2.0);

  fixture.AdvanceSeconds(30.0);
  const Vec3 offset60 =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  const double theta60 =
      OrbitAzimuth(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  ExpectNear("6.1-3 |pos-pivot| at t=60", Length(offset60), 5.05, 0.01);
  ExpectNear("6.1-4 dot(pos-pivot, body up) at t=60", Dot(offset60, basis.up),
             0.598, 0.01);
  const double revolved60 = RevolvedDegrees(theta0, theta60);
  ExpectNear("6.1-5 revolved angle at t=60 (mod 360)",
             (std::min)(revolved60, 360.0 - revolved60), 0.0, 2.0);
  ExpectNear("6.1-5 pose returns at t=60",
             Length(Sub(fixture.last.view.position, start_position)), 0.0,
             0.01);
}

void TestOrbitHeight() {
  Fixture fixture;
  fixture.config.camera_height_m = 2.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 offset =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  const double vertical = Dot(offset, basis.up);
  ExpectNear("height override camera height above car origin",
             kOrbitLookAtHeightM + vertical, 2.0, 1e-9);
  ExpectNear("height override keeps the slant radius", Length(offset), 5.05,
             1e-9);
  ExpectNear("height override shrinks the horizontal radius",
             Length(Sub(offset, Mul(basis.up, vertical))),
             std::sqrt(5.05 * 5.05 - 0.77 * 0.77), 1e-9);
}

void TestConstantAngularRate() {
  Fixture fixture;
  fixture.EnterOrbit();
  const double theta0 =
      OrbitAzimuth(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  double previous = theta0;
  for (int step = 0; step < 12; ++step) {
    fixture.AdvanceSeconds(5.0);
    const double current =
        OrbitAzimuth(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
    // 6 deg/s * 5 s = 30 deg; 1e-8 rad is well below the visible threshold.
    ExpectNear("6.1-6 5 s angular step", SignedDegrees(previous, current), 30.0,
               5e-8);
    previous = current;
  }
  const double revolved = RevolvedDegrees(theta0, previous);
  ExpectNear("6.1-6 full revolution in 60 s",
             (std::min)(revolved, 360.0 - revolved), 0.0, 1e-6);
}

void TestWeaveTrack() {
  ExpectNear("6.1-8 weave(0)", OrbitWeave(0.0), 0.0, 1e-6);
  ExpectNear("6.1-8 weave(15)", OrbitWeave(15.0), 1.0, 1e-6);
  ExpectNear("6.1-8 weave(29.963)", OrbitWeave(29.963), 0.0, 1e-6);
  ExpectNear("6.1-8 weave(45)", OrbitWeave(45.0), 1.0, 1e-6);
  ExpectNear("6.1-8 weave(60)", OrbitWeave(60.0), 0.0, 1e-6);
  ExpectNear("6.1-8 weave repeats every 60 s", OrbitWeave(75.0), 1.0, 1e-6);
}

void TestWeaveDisplacement() {
  Fixture plain(0.0, 0.0);
  Fixture woven(0.0, 1.0);
  plain.EnterOrbit();
  woven.EnterOrbit();

  plain.AdvanceSeconds(15.0);
  woven.AdvanceSeconds(15.0);
  const Vec3 pivot = OrbitPivot(plain.car, kOrbitLookAtHeightM);
  const CarBasis basis = MakeCarBasis(plain.car.yaw, plain.car.pitch);
  const Vec3 radial = Normalize(Sub(plain.last.view.position, pivot));
  const Vec3 tangent = Normalize(Cross(basis.up, radial));
  Vec3 delta = Sub(woven.last.view.position, plain.last.view.position);
  ExpectNear("6.1-7 t=15 tangential displacement", Dot(delta, tangent), 1.0,
             0.05);
  ExpectNear("6.1-7 t=15 radial displacement", Dot(delta, radial), 0.0, 0.01);
  const double expected_radius =
      std::sqrt(5.05 * 5.05 *
                    std::cos(6.8 * nfs2015_chase::kRadiansPerDegree) *
                    std::cos(6.8 * nfs2015_chase::kRadiansPerDegree) +
                1.0 + 5.05 * 5.05 *
                          std::sin(6.8 * nfs2015_chase::kRadiansPerDegree) *
                          std::sin(6.8 * nfs2015_chase::kRadiansPerDegree));
  ExpectNear("6.1-9 |pos-pivot| at t=15", Length(Sub(woven.last.view.position,
                                                     pivot)),
             expected_radius, 0.01);

  plain.AdvanceSeconds(15.0);
  woven.AdvanceSeconds(15.0);
  delta = Sub(woven.last.view.position, plain.last.view.position);
  ExpectNear("6.1-7 t=30 weave returns to the orbit", Length(delta), 0.0,
             0.01);
  ExpectNear("6.1-7 t=30 radius is back to 5.05",
             Length(Sub(woven.last.view.position, pivot)), 5.05, 0.01);

  plain.AdvanceSeconds(15.0);
  woven.AdvanceSeconds(15.0);
  const Vec3 radial45 = Normalize(Sub(plain.last.view.position, pivot));
  const Vec3 tangent45 = Normalize(Cross(basis.up, radial45));
  delta = Sub(woven.last.view.position, plain.last.view.position);
  ExpectNear("6.1-7 t=45 tangential displacement", Dot(delta, tangent45), 1.0,
             0.05);
  ExpectNear("6.1-7 t=45 radial displacement", Dot(delta, radial45), 0.0, 0.01);
  ExpectNear("6.1-9 |pos-pivot| at t=45",
             Length(Sub(woven.last.view.position, pivot)), expected_radius,
             0.01);
}

void TestSpeedExit() {
  Fixture fixture;
  fixture.EnterOrbit();
  ExpectNear("6.1-10 full blend before the car moves", fixture.last.blend, 1.0,
             1e-12);

  fixture.car.speed = 0.2;
  fixture.AdvanceSeconds(1.0);
  ExpectNear("6.1-10 blend after 1.0 s at 0.2 m/s", fixture.last.blend, 0.0,
             0.0);
  Expect("6.1-10 returns to idle",
         fixture.last.state.mode == OrbitPhase::kIdle);

  CameraState base = SyntheticBaseView();
  base.position = {1.0, 2.0, 3.0};
  Expect("6.1-10 blend=0 returns the base bit-for-bit",
         SameCamera(BlendShotAroundCarPivot(base, fixture.last.view,
                                            fixture.car, 0.0,
                                            kOrbitLookAtHeightM),
                    base));
}

void TestInputExit() {
  Fixture fixture;
  fixture.EnterOrbit();
  fixture.sample.input_active = true;
  fixture.AdvanceSeconds(1.0);
  ExpectNear("6.1-11 blend after 1.0 s with input", fixture.last.blend, 0.0,
             0.0);
  Expect("6.1-11 input alone returns to idle",
         fixture.last.state.mode == OrbitPhase::kIdle);
}

void TestInputClearsHoldTimer() {
  Fixture fixture;
  fixture.AdvanceSeconds(7.9);
  fixture.sample.input_active = true;
  fixture.AdvanceFrames(1);
  ExpectNear("6.1-12 input clears the hold timer",
             fixture.last.state.hold_time, 0.0, 0.0);
  fixture.AdvanceSeconds(9.1);  // 17.0 s total
  ExpectNear("6.1-12 blend is still 0 at 17 s", fixture.last.blend, 0.0, 0.0);
  Expect("6.1-12 still idle at 17 s",
         fixture.last.state.mode == OrbitPhase::kIdle);

  fixture.sample.input_active = false;
  fixture.AdvanceSeconds(7.98);
  ExpectNear("6.1-12 no entry before a full new 8 s hold", fixture.last.blend,
             0.0, 0.0);
  fixture.AdvanceFrames(2);
  Expect("6.1-12 enters after the new 8 s hold",
         fixture.last.state.mode == OrbitPhase::kEnter);
}

void TestSlopeFollow() {
  Fixture fixture(/*pitch=*/0.14);
  fixture.EnterOrbit();
  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 offset =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  ExpectNear("6.1-13 slope |pos-pivot|", Length(offset), 5.05, 0.01);
  ExpectNear("6.1-13 dot(pos-pivot, body up)", Dot(offset, basis.up), 0.598,
             0.01);
  ExpectNear("6.1-13 slope keeps up world +Z (x)", fixture.last.view.up.x, 0.0,
             1e-3);
  ExpectNear("6.1-13 slope keeps up world +Z (y)", fixture.last.view.up.y, 0.0,
             1e-3);
  ExpectNear("6.1-13 slope keeps up world +Z (z)", fixture.last.view.up.z, 1.0,
             1e-3);
  ExpectNear("6.1-13 slope keeps roll at 0", fixture.last.view.roll, 0.0, 0.0);
}

void TestEntryPhase() {
  Fixture fixture;
  fixture.camera = BehindCamera(fixture.car, 6.0, 0.6);
  fixture.AdvanceSeconds(8.0);
  ExpectNear("6.1-14 phase0 lands behind the car",
             SignedDegrees(kPi, fixture.last.state.phase0), 0.0, 2.0);

  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 offset =
      OrbitOffset(fixture.last.view, fixture.car, kOrbitLookAtHeightM);
  ExpectNear("6.1-14 entry pose sits behind (forward component)",
             Dot(offset, basis.forward), -5.0144758, 0.01);

  Fixture degenerate;
  degenerate.camera = OrbitPivot(degenerate.car, kOrbitLookAtHeightM);
  degenerate.AdvanceSeconds(8.0);
  ExpectNear("6.1-14 camera at the pivot falls back to behind",
             degenerate.last.state.phase0, kPi, 1e-12);
}

void TestBlendEdges() {
  const CarState car = MakeCar();
  const CameraState base = SyntheticBaseView();
  CameraState shot = SyntheticBaseView();
  shot.position = {3.0, 4.0, 1.5};
  shot.target = {0.5, 0.0, 1.0};
  shot.up = {0.0, 0.0, 1.0};
  shot.fov_y_degrees = 43.0;
  shot.roll = 0.0;
  shot.pitch_degrees = -7.0;
  shot.yaw_degrees = -160.0;
  shot.roll_degrees = 0.0;

  Expect("6.1-15 blend=0 returns base bit-for-bit",
         SameCamera(BlendShotAroundCarPivot(base, shot, car, 0.0,
                                            kOrbitLookAtHeightM),
                    base));
  Expect("6.1-15 blend=1 returns shot bit-for-bit",
         SameCamera(BlendShotAroundCarPivot(base, shot, car, 1.0,
                                            kOrbitLookAtHeightM),
                    shot));
  Expect("blend clamps negative blend to base",
         SameCamera(BlendShotAroundCarPivot(base, shot, car, -1.0,
                                            kOrbitLookAtHeightM),
                    base));
  Expect("blend clamps blend > 1 to shot",
         SameCamera(BlendShotAroundCarPivot(base, shot, car, 2.0,
                                            kOrbitLookAtHeightM),
                    shot));
  Expect("BlendBurnoutCamera stays a wrapper over the shared helper",
         SameCamera(BlendBurnoutCamera(base, shot, car, 0.37),
                    BlendShotAroundCarPivot(base, shot, car, 0.37,
                                            kBurnoutOrbitPivotHeightM)));
}

void TestExitFreezesAndResumesClock() {
  Fixture fixture;
  fixture.EnterOrbit();
  fixture.AdvanceSeconds(5.0);
  const double parked_time = fixture.last.state.time;
  ExpectNear("orbit clock at 5 s", parked_time, 5.0, 1e-9);

  fixture.car.speed = 0.2;
  fixture.AdvanceSeconds(0.2);
  Expect("clock is frozen while exiting",
         fixture.last.state.mode == OrbitPhase::kExit);
  ExpectNear("clock does not move during exit", fixture.last.state.time,
             parked_time, 0.0);

  fixture.car.speed = 0.0;
  fixture.AdvanceSeconds(0.2);
  Expect("a still car re-enters without a new 8 s wait",
         fixture.last.state.mode == OrbitPhase::kOrbit);
  ExpectNear("clock resumes from the frozen value", fixture.last.state.time,
             parked_time, 0.0);
  fixture.AdvanceFrames(1);
  ExpectNear("clock advances again after re-entry", fixture.last.state.time,
             parked_time + kDt, 1e-9);
}

void TestReset() {
  Fixture fixture;
  fixture.EnterOrbit();
  fixture.AdvanceSeconds(5.0);
  fixture.shot.Reset();
  const auto& state = fixture.shot.state();
  Expect("Reset clears the shot",
         state.mode == OrbitPhase::kIdle && state.hold_time == 0.0 &&
             state.phase == 0.0 && state.time == 0.0 && !state.latched);
}

void TestFiniteOnBadInputs() {
  Fixture fixture;
  fixture.car.position = {std::nan(""), 0.0, 0.0};
  fixture.car.yaw = std::nan("");
  fixture.car.velocity = {std::nan(""), std::nan(""), std::nan("")};
  fixture.camera = {std::nan(""), std::nan(""), std::nan("")};
  fixture.config.weave_amplitude_m = std::nan("");
  fixture.config.period_seconds = std::nan("");
  fixture.config.radius_m = std::nan("");
  fixture.shot.SetConfig(fixture.config);
  fixture.AdvanceFrames(600);
  Expect("NaN inputs still produce a finite view",
         FiniteCamera(fixture.last.view));
  Expect("NaN inputs still produce finite state",
         std::isfinite(fixture.last.blend) &&
             std::isfinite(fixture.last.state.phase) &&
             std::isfinite(fixture.last.state.phase0) &&
             std::isfinite(fixture.last.state.time));
}

}  // namespace

int main() {
  TestEntryDelay();
  TestEntryDuration();
  TestGeometryAndAngles();
  TestOrbitHeight();
  TestConstantAngularRate();
  TestWeaveTrack();
  TestWeaveDisplacement();
  TestSpeedExit();
  TestInputExit();
  TestInputClearsHoldTimer();
  TestSlopeFollow();
  TestEntryPhase();
  TestBlendEdges();
  TestExitFreezesAndResumesClock();
  TestReset();
  TestFiniteOnBadInputs();
  std::printf("all parked orbit shot tests passed\n");
  return 0;
}
