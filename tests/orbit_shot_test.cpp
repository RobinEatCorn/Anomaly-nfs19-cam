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
using nfs2015_chase::kOrbitPivotForwardM;
using nfs2015_chase::kPi;
using nfs2015_chase::Length;
using nfs2015_chase::MakeCarBasis;
using nfs2015_chase::Mul;
using nfs2015_chase::Normalize;
using nfs2015_chase::OrbitPhase;
using nfs2015_chase::kOrbitTargetSwayRightM;
using nfs2015_chase::kOrbitTargetSwayUpM;
using nfs2015_chase::OrbitShot;
using nfs2015_chase::OrbitTargetSwayX;
using nfs2015_chase::OrbitTargetSwayY;
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
  return Add(Add(car.position, Mul(basis.up, look_height)),
             Mul(basis.forward, kOrbitPivotForwardM));
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
    // This fixture pins the geometry and the state machine, so both look-at
    // layers are off unless a test turns them on: the camera weave stays at 0,
    // and the low-frequency target sway (Instance279) is off as well. Keeping
    // them off means a geometry assertion never has to guess which layer moved
    // the target; the layers get their own explicit setups.
    config.target_sway = 0.0;
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

void TestPivotForwardOffset() {
  Fixture fixture;
  fixture.config.handheld_strength = 0.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 pivot = OrbitPivot(fixture.car, kOrbitLookAtHeightM);
  const Vec3 from_car = Sub(pivot, fixture.car.position);
  ExpectNear("6.1-7 pivot forward offset", Dot(from_car, basis.forward),
             kOrbitPivotForwardM, 1e-9);
  ExpectNear("6.1-7 pivot height", Dot(from_car, basis.up),
             kOrbitLookAtHeightM, 1e-9);
  ExpectNear("6.1-7 target is the shifted pivot",
             Length(Sub(fixture.last.view.target, pivot)), 0.0, 1e-9);
  ExpectNear("6.1-7 eye keeps the arm radius",
             Length(Sub(fixture.last.view.position, pivot)), 5.05, 0.01);
}

void TestLookAtHeight() {
  // The look-at height is an independent knob: it moves the pivot the camera
  // circles and aims at, while the eye height and the slant radius stay put.
  Fixture fixture;
  fixture.config.camera_height_m = 2.4;
  fixture.config.look_at_height_m = 0.6;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 offset = OrbitOffset(fixture.last.view, fixture.car, 0.6);
  const double vertical = Dot(offset, basis.up);
  ExpectNear("look-at override puts the pivot 0.6 m up",
             Dot(Sub(fixture.last.view.target, fixture.car.position),
                 basis.up),
             0.6, 0.05);
  ExpectNear("look-at override keeps the camera height above the origin",
             0.6 + vertical, 2.4, 1e-9);
  ExpectNear("look-at override keeps the slant radius", Length(offset), 5.05,
             1e-9);
  // asin((2.4 - 0.6) / 5.05) = 20.881 degrees: the two heights, not a stored
  // angle, decide the elevation.
  ExpectNear("look-at override sets the elevation from the two heights",
             std::asin(vertical / 5.05) * kDegreesPerRadian, 20.881457, 1e-6);
}

// The reference framing this plan adopts (Q1). These numbers are what the
// game's default parked orbit reproduces; every one of them is still a slider,
// so a failure here means the *default* drifted, not the model.
void TestReferenceDefaults() {
  const OrbitShotConfig defaults{};
  ExpectNear("default radius", defaults.radius_m, 5.05, 0.0);
  ExpectNear("default look-at height", defaults.look_at_height_m, 1.23, 0.0);
  ExpectNear("default pivot forward offset", defaults.pivot_forward_m, 0.772,
             0.0);
  ExpectNear("default vertical fov", defaults.vertical_fov_degrees, 28.0,
             0.0);
  ExpectNear("default look-at sway", defaults.target_sway, 1.0, 0.0);
  ExpectNear("default noise frequency", defaults.handheld_frequency_hz, 2.0,
             0.0);

  Fixture fixture;
  fixture.config.handheld_strength = 0.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  const CarBasis basis = MakeCarBasis(fixture.car.yaw, fixture.car.pitch);
  const Vec3 pivot = OrbitPivot(fixture.car, kOrbitLookAtHeightM);
  const Vec3 pivot_from_car = Sub(pivot, fixture.car.position);
  const Vec3 eye_from_car =
      Sub(fixture.last.view.position, fixture.car.position);
  const Vec3 offset = Sub(fixture.last.view.position, pivot);
  ExpectNear("reference |eye - pivot|", Length(offset), 5.05, 0.01);
  ExpectNear("reference pivot height above the car origin",
             Dot(pivot_from_car, basis.up), 1.23, 0.01);
  ExpectNear("reference pivot forward offset",
             Dot(pivot_from_car, basis.forward), 0.772, 0.01);
  // The camera height is what the presenter always overrides with: 1.23 m of
  // pivot plus the 0.6 m of arm that the geometry leaves after `asin((1.83 -
  // 1.23) / 5.05)`. The three numbers only work as a set.
  ExpectNear("reference camera height above the car origin",
             Dot(eye_from_car, basis.up), 1.83, 0.01);
  ExpectNear("reference elevation",
             std::asin(Dot(offset, basis.up) / Length(offset)) *
                 kDegreesPerRadian,
             6.8, 0.2);
  ExpectNear("reference vertical fov", fixture.last.view.fov_y_degrees, 28.0,
             1e-12);
}

// Instance192 / Instance188 land on these keys every 15 s. The published value
// goes through a 1 s first-order damper, so it lags the raw track by up to
// tau * slope; the check is therefore "close to the key and same sign", which
// is what distinguishes the asset curve from any other shape.
void TestTargetSwayKeys() {
  Fixture fixture;
  fixture.config.handheld_strength = 0.0;
  fixture.config.target_sway = 1.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  struct Key final {
    double t;
    double x;
    double y;
  };
  // Normalised track to metres: X * 1.0, Y * 0.3.
  const Key keys[] = {{0.0, 0.0, -0.3},    {15.0, 0.0, 0.0},
                      {22.5, 1.0, -0.15},  {30.0, 0.0, -0.3},
                      {37.5, -1.0, -0.15}, {45.0, 0.0, 0.0}};
  // The first orbit frame is seeded with the raw track instead of crawling in
  // from zero, so t = 0 is exact.
  ExpectNear("sway at t=0 is seeded with the raw key",
             fixture.last.state.target_sway_y_m, -0.3, 1e-12);
  ExpectNear("sway at t=0 keeps x at the raw key",
             fixture.last.state.target_sway_x_m, 0.0, 1e-12);

  // The asset track itself is exact: these two functions are the published
  // curves, so a mismatch here means the key table drifted.
  double done = 0.0;
  for (const Key& key : keys) {
    char label[64]{};
    std::snprintf(label, sizeof(label), "track X at t=%.1f s", key.t);
    ExpectNear(label, OrbitTargetSwayX(key.t) * kOrbitTargetSwayRightM, key.x,
               1e-12);
    std::snprintf(label, sizeof(label), "track Y at t=%.1f s", key.t);
    ExpectNear(label, OrbitTargetSwayY(key.t) * kOrbitTargetSwayUpM, key.y,
               1e-12);
  }

  // The published state follows that curve through the 1 s damper, so it lags
  // by at most tau * max|d/dt| = 1 s * 0.15 m/s on X and 1 s * 0.03 m/s on Y.
  // Those bounds are a known lag, not slack for a wrong shape: any other curve
  // would be off by whole metres at the +-1 m keys.
  for (const Key& key : keys) {
    fixture.AdvanceSeconds(key.t - done);
    done = key.t;
    char label[64]{};
    std::snprintf(label, sizeof(label), "damped sway X at t=%.1f s", key.t);
    ExpectNear(label, fixture.last.state.target_sway_x_m, key.x, 0.18);
    std::snprintf(label, sizeof(label), "damped sway Y at t=%.1f s", key.t);
    ExpectNear(label, fixture.last.state.target_sway_y_m, key.y, 0.05);
  }

  // A full 60 s loop returns to the t = 0 key, and the damped track never
  // swings outside the key envelope (Catmull-Rom on this table does not
  // overshoot, and the damper can only shrink the excursion). Start from a
  // fresh shot so the sample covers exactly one period.
  Fixture sweep;
  sweep.config.handheld_strength = 0.0;
  sweep.config.target_sway = 1.0;
  sweep.shot.SetConfig(sweep.config);
  sweep.EnterOrbit();
  constexpr double dt = 1.0 / 240.0;
  double min_x = 1.0e9;
  double max_x = -1.0e9;
  double min_y = 1.0e9;
  double max_y = -1.0e9;
  for (int frame = 0; frame < 60 * 240; ++frame) {
    sweep.AdvanceFrames(1, dt);
    min_x = (std::min)(min_x, sweep.last.state.target_sway_x_m);
    max_x = (std::max)(max_x, sweep.last.state.target_sway_x_m);
    min_y = (std::min)(min_y, sweep.last.state.target_sway_y_m);
    max_y = (std::max)(max_y, sweep.last.state.target_sway_y_m);
  }
  ExpectNear("sway wraps back to the t=0 key at 60 s",
             sweep.last.state.target_sway_y_m, -0.3, 0.05);
  ExpectNear("sway wraps back to a level right offset at 60 s",
             sweep.last.state.target_sway_x_m, 0.0, 0.05);
  Expect("sway X stays inside its +-1 m keys", min_x >= -1.0 && max_x <= 1.0);
  Expect("sway Y stays inside its 0..-0.3 m keys", min_y >= -0.3 && max_y <= 0.0);
  Expect("sway X uses both sides of the track", min_x < -0.8 && max_x > 0.8);
}

// With both look-at layers off the target is the pivot bit-for-bit: that is
// what keeps `target_sway == 0` a true off switch rather than a tiny offset.
void TestTargetSwayZero() {
  Fixture fixture;
  fixture.config.handheld_strength = 0.0;
  fixture.config.target_sway = 0.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();

  const Vec3 pivot = OrbitPivot(fixture.car, kOrbitLookAtHeightM);
  bool same_target = true;
  bool exact_state = true;
  for (int frame = 0; frame < 60 * 60; ++frame) {
    fixture.AdvanceFrames(1);
    same_target = same_target && fixture.last.view.target.x == pivot.x &&
                  fixture.last.view.target.y == pivot.y &&
                  fixture.last.view.target.z == pivot.z;
    exact_state = exact_state &&
                  fixture.last.state.target_sway_x_m == 0.0 &&
                  fixture.last.state.target_sway_y_m == 0.0;
  }
  Expect("sway 0: target is the pivot bit-for-bit", same_target);
  Expect("sway 0: the state stays at exact zero", exact_state);
}

// The sway layer only ever moves the look-at point. The eye, up and FOV must
// stay bit-for-bit identical, otherwise the orbit radius would wobble.
void TestTargetSwayDoesNotMoveEye() {
  Fixture off;
  off.config.handheld_strength = 0.0;
  off.config.target_sway = 0.0;
  off.shot.SetConfig(off.config);

  Fixture on;
  on.config.handheld_strength = 0.0;
  on.config.target_sway = 1.0;
  on.shot.SetConfig(on.config);

  off.EnterOrbit();
  on.EnterOrbit();
  bool same_position = true;
  bool same_up_fov = true;
  bool target_differs = false;
  for (int frame = 0; frame < 60 * 60; ++frame) {
    off.AdvanceFrames(1);
    on.AdvanceFrames(1);
    same_position = same_position &&
                    off.last.view.position.x == on.last.view.position.x &&
                    off.last.view.position.y == on.last.view.position.y &&
                    off.last.view.position.z == on.last.view.position.z;
    same_up_fov = same_up_fov && off.last.view.up.x == on.last.view.up.x &&
                  off.last.view.up.y == on.last.view.up.y &&
                  off.last.view.up.z == on.last.view.up.z &&
                  off.last.view.fov_y_degrees == on.last.view.fov_y_degrees;
    target_differs =
        target_differs ||
        Length(Sub(off.last.view.target, on.last.view.target)) > 1e-4;
  }
  Expect("sway leaves the eye bit-for-bit", same_position);
  Expect("sway leaves up and fov bit-for-bit", same_up_fov);
  Expect("sway moves only the target", target_differs);
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
  // The roll noise is the only thing that may tilt the horizon, so it is off
  // here: "slope adds no roll" has to hold exactly. The noise itself is pinned
  // by the handheld fixture.
  fixture.config.handheld_strength = 0.0;
  fixture.shot.SetConfig(fixture.config);
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
  // Turn the low-frequency sway on for this one: the reset must clear the new
  // layers, not just the state machine.
  fixture.config.target_sway = 1.0;
  fixture.shot.SetConfig(fixture.config);
  fixture.EnterOrbit();
  fixture.AdvanceSeconds(5.0);
  Expect("the layers are live before the reset",
         fixture.last.state.target_sway_y_m != 0.0 &&
             fixture.last.state.roll_noise_degrees != 0.0);
  fixture.shot.Reset();
  const auto& state = fixture.shot.state();
  Expect("Reset clears the shot",
         state.mode == OrbitPhase::kIdle && state.hold_time == 0.0 &&
             state.phase == 0.0 && state.time == 0.0 && !state.latched);
  // PublishOrbitReset() relies on this: after a reset the diagnostics and the
  // pose must show nothing left from the previous shot.
  Expect("Reset clears both look-at layers and the roll noise",
         state.shake_x_m == 0.0 && state.shake_y_m == 0.0 &&
             state.target_sway_x_m == 0.0 &&
             state.target_sway_y_m == 0.0 &&
             state.roll_noise_degrees == 0.0);
}

// The parked orbit no longer pins roll at zero: it carries the Instance16 ROLL
// noise and depends on BlendShotAroundCarPivot to fade it in and out. Pin the
// two ends (bit-for-bit) and the interior (the linear lerp the camera applies).
void TestRollNoiseFadesWithTheBlend() {
  Fixture fixture;
  fixture.EnterOrbit();
  bool have_noise = false;
  for (int frame = 0; frame < 240 && !have_noise; ++frame) {
    fixture.AdvanceFrames(1);
    have_noise = fixture.last.view.roll_degrees != 0.0;
  }
  Expect("the orbit carries the Instance16 roll noise", have_noise);
  const CameraState shot = fixture.last.view;
  Expect("the orbit's roll scalar matches its degrees",
         shot.roll == shot.roll_degrees * nfs2015_chase::kRadiansPerDegree);

  // Blending works in radians and recomputes roll_degrees from the result, so
  // the base needs a consistent pair. SyntheticBaseView deliberately carries a
  // slightly inconsistent one (0.01 rad vs 0.57 deg) to keep its other checks
  // readable; rebuild it here so the expectation below is about the lerp, not
  // about that 0.003 degree mismatch.
  CameraState base = SyntheticBaseView();
  base.roll_degrees = 0.57;
  base.roll = base.roll_degrees * nfs2015_chase::kRadiansPerDegree;
  const double blend = 0.5;
  const CameraState mid = BlendShotAroundCarPivot(base, shot, fixture.car,
                                                  blend, kOrbitLookAtHeightM);
  ExpectNear("the blend lerps the roll noise", mid.roll_degrees,
             base.roll_degrees +
                 (shot.roll_degrees - base.roll_degrees) * blend,
             1e-9);
  Expect("blend 0 drops the roll noise exactly",
         BlendShotAroundCarPivot(base, shot, fixture.car, 0.0,
                                 kOrbitLookAtHeightM)
                 .roll_degrees == base.roll_degrees);
  Expect("blend 1 keeps the roll noise exactly",
         BlendShotAroundCarPivot(base, shot, fixture.car, 1.0,
                                 kOrbitLookAtHeightM)
                 .roll_degrees == shot.roll_degrees);
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
  TestPivotForwardOffset();
  TestLookAtHeight();
  TestReferenceDefaults();
  TestTargetSwayKeys();
  TestTargetSwayZero();
  TestTargetSwayDoesNotMoveEye();
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
  TestRollNoiseFadesWithTheBlend();
  TestFiniteOnBadInputs();
  std::printf("all parked orbit shot tests passed\n");
  return 0;
}
