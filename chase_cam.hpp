#pragma once

// NFS 2015 "Action Camera" / drift chase camera, clean-room reconstruction.
//
// This is a C++ port of the `nfs2015ChaseCam.ts` reference from the NFS 2015
// camera analysis that accompanied the port. The parameter names
// (`CameraLocalRestPosition`, `DriftYawSpring`, ...) come from NFS 2015's
// Frostbite camera data; the implementation is a behavioural reconstruction,
// not Ghost Games' code.
//
// Conventions:
//   - UE axes: +X forward, +Y right, +Z up.
//   - metres, seconds, radians unless a field says "degrees".
//   - yaw 0 looks along +X; positive yaw turns toward +Y.
//
// The camera *position* orbits a pivot on a yaw spring that lags the car's
// nose. During a drift the spring is deliberately loose (`drift_yaw_spring`),
// so the camera hangs outside the car while the look-at point stays on the
// nose. The position yaw is looser than the look yaw, which is what makes the
// car body appear angled across the frame.

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace nfs2015_chase {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegreesPerRadian = 180.0 / kPi;
inline constexpr double kRadiansPerDegree = kPi / 180.0;
inline constexpr double kMphPerMetrePerSecond = 2.2369362920544;

struct Vec3 final {
  double x{};
  double y{};
  double z{};
};

[[nodiscard]] inline Vec3 Add(const Vec3 a, const Vec3 b) noexcept {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

[[nodiscard]] inline Vec3 Sub(const Vec3 a, const Vec3 b) noexcept {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] inline Vec3 Mul(const Vec3 a, const double scale) noexcept {
  return {a.x * scale, a.y * scale, a.z * scale};
}

[[nodiscard]] inline double Dot(const Vec3 a, const Vec3 b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] inline Vec3 Cross(const Vec3 a, const Vec3 b) noexcept {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

[[nodiscard]] inline double Length(const Vec3 a) noexcept {
  return std::sqrt(Dot(a, a));
}

[[nodiscard]] inline Vec3 Normalize(const Vec3 a) noexcept {
  const double length = Length(a);
  return length > 1e-9 ? Mul(a, 1.0 / length) : Vec3{1.0, 0.0, 0.0};
}

// Rotate around +Z by `angle` radians. This is UE's yaw rotation:
// forward (1,0,0) -> (cos, sin, 0), right (0,1,0) -> (-sin, cos, 0).
[[nodiscard]] inline Vec3 RotateYaw(const double angle, const Vec3 v) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

[[nodiscard]] inline Vec3 ForwardFromYaw(const double yaw) noexcept {
  return {std::cos(yaw), std::sin(yaw), 0.0};
}

[[nodiscard]] inline Vec3 RightFromYaw(const double yaw) noexcept {
  return {-std::sin(yaw), std::cos(yaw), 0.0};
}

// Rodrigues rotation of `v` around unit `axis`.
[[nodiscard]] inline Vec3 RotateAxis(const Vec3 v, const Vec3 axis,
                                     const double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return Add(Add(Mul(v, c), Mul(Cross(axis, v), s)),
             Mul(axis, Dot(axis, v) * (1.0 - c)));
}

[[nodiscard]] inline double Clamp(const double value, const double low,
                                  const double high) noexcept {
  return (std::min)(high, (std::max)(low, value));
}

[[nodiscard]] inline double Lerp(const double a, const double b,
                                 const double t) noexcept {
  return a + (b - a) * t;
}

[[nodiscard]] inline double SmoothStep(const double edge0, const double edge1,
                                       const double value) noexcept {
  if (edge0 == edge1) {
    return value < edge0 ? 0.0 : 1.0;
  }
  const double t = Clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

// Wrap to (-PI, PI].
[[nodiscard]] inline double WrapPi(double angle) noexcept {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

// First-order exponential smoothing. `rate` is 1/s.
[[nodiscard]] inline double Damp(const double current, const double target,
                                 const double rate, const double dt) noexcept {
  return target + (current - target) * std::exp(-rate * dt);
}

// Same, but takes the shortest path around the circle.
[[nodiscard]] inline double DampAngle(const double current, const double target,
                                      const double rate,
                                      const double dt) noexcept {
  return current + WrapPi(target - current) * (1.0 - std::exp(-rate * dt));
}

// Cheap 1D noise in roughly [-1, 1]; deterministic and dependency-free.
[[nodiscard]] inline double Noise1(const double t, const double seed) noexcept {
  return 0.55 * std::sin(t + seed * 12.9898) +
         0.30 * std::sin(t * 2.17 + seed * 78.233) +
         0.15 * std::sin(t * 4.31 + seed * 37.719);
}

struct Config final {
  // Local rest position at zero speed, relative to the car origin (metres).
  Vec3 rest{-5.2, 0.0, 1.55};
  // Local pivot the camera orbits around (metres).
  Vec3 pivot{0.2, 0.0, 0.65};
  // Base look-at height at zero speed (metres).
  double look_height{0.8};

  // Normal yaw spring rate (1/s). Higher = tighter.
  double yaw_spring{8.0};
  // Yaw spring rate while drifting (1/s). Lower = more side-swing.
  double drift_yaw_spring{2.2};
  // Yaw spring rate when the drift releases (1/s). Lower = looser reset.
  double yaw_return_spring{4.5};
  // How fast the look direction follows its target (1/s).
  double look_yaw_spring{6.0};

  // Extra outward yaw target per radian of slip (0..1).
  double position_slip_weight{0.25};
  // How much the look direction follows the velocity instead of the nose (0..1).
  double look_slip_weight{0.7};
  // Maximum camera position lag relative to the car nose, radians.
  double max_position_lag{55.0 * kRadiansPerDegree};
  // Maximum car angle away from the view axis, radians.
  double max_car_off_axis{42.0 * kRadiansPerDegree};

  // Drift close-up: metres the camera moves forward (toward the car).
  double drift_push_in{2.4};
  // Drift close-up: metres the camera drops.
  double drift_drop{0.62};
  // Drift close-up: metres the camera slides sideways.
  // Positive = outside (centres the car), negative = inside (more side-on).
  double drift_side{0.0};
  // Drift close-up: metres the look-at point drops.
  double drift_look_drop{0.15};

  // Slip angle where drift blend starts, radians.
  double drift_start{6.0 * kRadiansPerDegree};
  // Slip angle where drift blend is full, radians.
  double drift_full{28.0 * kRadiansPerDegree};
  // Drift blend attack rate (1/s).
  double drift_attack{9.0};
  // Drift blend release rate (1/s).
  double drift_release{3.0};

  // Extra distance at top speed, metres.
  double speed_pull_back{0.9};
  // Rest height at top speed, metres.
  double speed_rest_height{1.75};
  // Look-at height at top speed, metres.
  double speed_look_height{1.0};
  // Speed (m/s) where speed-based movement starts / ends.
  double speed_move_start{9.0};
  double speed_move_end{67.0};
  // Multiplier for the speed/acceleration fore-aft dolly. 1 = retail mapping.
  double speed_distance_scale{1.0};

  // Camera pull-back under forward acceleration, metres.
  double acceleration_back{0.55};
  // Camera push-forward under braking, metres.
  double brake_forward{0.45};
  // Camera rise / drop under acceleration, metres.
  double acceleration_up{0.08};
  double acceleration_down{0.06};
  // Normalisation for longitudinal acceleration, m/s^2.
  double acceleration_normalisation{10.0};

  // Position spring angular frequency (rad/s). Critical damping is used.
  double position_spring{7.5};
  // Pitch spring rate for braking pitch (1/s).
  double pitch_spring{6.0};
  // Roll spring rate (1/s).
  double roll_spring{9.0};
  // Maximum roll, degrees.
  double max_roll_degrees{12.0};
  // Braking-pitch multiplier. 1 is the retail value.
  double pitch_scale{1.0};
  // Roll from lateral acceleration, radians per g.
  double roll_lateral_gain{0.13};
  // Roll from yaw rate, radians per rad/s.
  double roll_yaw_gain{0.055};
  // Multiplier for the roll target, and an optional non-retail drift gate.
  double roll_scale{1.0};
  bool roll_only_in_drift{false};
  // Maximum braking pitch, degrees.
  double max_pitch_degrees{3.0};

  // Local look-at point X at low / high speed, metres ahead of the car origin.
  // NFS 2015 uses about 0.66-0.96 m here; it is not a far-away road target.
  double look_ahead_min{7.0};
  double look_ahead_max{17.0};

  // Vertical FOV, degrees. The game POV stores horizontal FOV; the consumer
  // converts with the viewport aspect ratio.
  double min_fov{56.0};
  double max_fov{70.0};
  double fov_start_mph{25.0};
  double fov_end_mph{150.0};
  double nitrous_fov{7.0};
  double drift_fov{2.5};
  double fov_spring{2.2};

  // Speed shake amplitude, metres.
  double speed_shake{0.012};
  // Multiplier for all camera shake. 1 is the retail value.
  double shake_scale{1.0};
  // Impact shake decay, 1/s.
  double impact_shake_decay{7.0};
  // Impact shake frequency, rad/s.
  double impact_shake_frequency{38.0};

  // Donut orbit speed, rad/s at donut_scale = 1.
  double donut_orbit_speed{0.55};
};

// The cinematic reconstruction from the analysis (`NFS2015_PRESET`).
[[nodiscard]] inline Config CinematicPreset() noexcept {
  Config cfg;
  cfg.rest = {-5.2, 0.0, 1.55};
  cfg.pivot = {0.2, 0.0, 0.65};
  cfg.look_height = 0.8;

  cfg.yaw_spring = 8.0;
  cfg.drift_yaw_spring = 2.2;
  cfg.yaw_return_spring = 4.5;
  cfg.look_yaw_spring = 6.0;

  cfg.position_slip_weight = 0.25;
  cfg.look_slip_weight = 0.7;
  cfg.max_position_lag = 55.0 * kRadiansPerDegree;
  cfg.max_car_off_axis = 42.0 * kRadiansPerDegree;

  cfg.drift_push_in = 2.4;
  cfg.drift_drop = 0.62;
  cfg.drift_side = 0.0;
  cfg.drift_look_drop = 0.15;

  cfg.drift_start = 6.0 * kRadiansPerDegree;
  cfg.drift_full = 28.0 * kRadiansPerDegree;
  cfg.drift_attack = 9.0;
  cfg.drift_release = 3.0;

  cfg.speed_pull_back = 0.9;
  cfg.speed_rest_height = 1.75;
  cfg.speed_look_height = 1.0;
  cfg.speed_move_start = 9.0;
  cfg.speed_move_end = 67.0;
  cfg.speed_distance_scale = 1.0;

  cfg.acceleration_back = 0.55;
  cfg.brake_forward = 0.45;
  cfg.acceleration_up = 0.08;
  cfg.acceleration_down = 0.06;
  cfg.acceleration_normalisation = 10.0;

  cfg.position_spring = 7.5;
  cfg.pitch_spring = 6.0;
  cfg.roll_spring = 9.0;
  cfg.max_roll_degrees = 12.0;
  cfg.pitch_scale = 1.0;
  cfg.roll_lateral_gain = 0.13;
  cfg.roll_yaw_gain = 0.055;
  cfg.roll_scale = 1.0;
  cfg.roll_only_in_drift = false;
  cfg.max_pitch_degrees = 3.0;

  cfg.look_ahead_min = 7.0;
  cfg.look_ahead_max = 17.0;

  cfg.min_fov = 56.0;
  cfg.max_fov = 70.0;
  cfg.fov_start_mph = 25.0;
  cfg.fov_end_mph = 150.0;
  cfg.nitrous_fov = 7.0;
  cfg.drift_fov = 2.5;
  cfg.fov_spring = 2.2;

  cfg.speed_shake = 0.012;
  cfg.shake_scale = 1.0;
  cfg.impact_shake_decay = 7.0;
  cfg.impact_shake_frequency = 38.0;
  cfg.donut_orbit_speed = 0.55;
  return cfg;
}

// Values extracted from the retail NFS 2015 files for the Honda Civic Type R
// 2000 (`NFS2015_GAME_PRESET`). The retail data has no explicit drift
// push-in/drop; those two offsets are the same reconstruction as the report.
[[nodiscard]] inline Config GamePreset() noexcept {
  Config cfg;
  cfg.rest = {-5.10, 0.0, 1.49};
  cfg.pivot = {0.786, 0.0, 0.0};
  cfg.look_height = 1.225;

  // 0.09 / 0.045 / 0.075 per-frame coefficients, converted at 60 Hz.
  cfg.yaw_spring = 5.4;
  cfg.drift_yaw_spring = 2.7;
  cfg.yaw_return_spring = 4.5;
  cfg.look_yaw_spring = 5.4;

  // The retail look-at is car-local, so no slip bias.
  cfg.position_slip_weight = 0.0;
  cfg.look_slip_weight = 0.0;
  cfg.max_position_lag = 55.0 * kRadiansPerDegree;
  cfg.max_car_off_axis = 60.0 * kRadiansPerDegree;

  cfg.drift_push_in = 2.0;
  cfg.drift_drop = 0.45;
  cfg.drift_side = 0.0;
  cfg.drift_look_drop = 0.0;

  cfg.drift_start = 6.0 * kRadiansPerDegree;
  cfg.drift_full = 28.0 * kRadiansPerDegree;
  cfg.drift_attack = 9.0;
  cfg.drift_release = 3.0;

  // SpeedBasedMovement: start 0 mph, top 50 mph, back -1.3 m,
  // rest height 1.79 m, look height 1.375 m.
  cfg.speed_pull_back = 1.3;
  cfg.speed_rest_height = 1.79;
  cfg.speed_look_height = 1.375;
  cfg.speed_move_start = 0.0;
  cfg.speed_move_end = 22.35;
  cfg.speed_distance_scale = 1.0;

  // AccelerationMovement limits/scalings, simplified.
  cfg.acceleration_back = 1.7;
  cfg.brake_forward = 0.5;
  cfg.acceleration_up = 0.2;
  cfg.acceleration_down = 0.1;
  cfg.acceleration_normalisation = 10.0;

  cfg.position_spring = 7.5;
  cfg.pitch_spring = 33.0;
  cfg.roll_spring = 60.0;
  cfg.max_roll_degrees = 4.7;
  cfg.pitch_scale = 1.0;
  cfg.roll_lateral_gain = 0.02;
  cfg.roll_yaw_gain = 0.01;
  cfg.roll_scale = 1.0;
  cfg.roll_only_in_drift = false;
  cfg.max_pitch_degrees = 4.5;

  cfg.look_ahead_min = 0.786;
  cfg.look_ahead_max = 0.786;

  // 70..95 horizontal at 16:9 -> 43.0..63.1 vertical.
  cfg.min_fov = 43.0;
  cfg.max_fov = 63.1;
  cfg.fov_start_mph = 0.0;
  cfg.fov_end_mph = 125.0;
  cfg.nitrous_fov = 0.0;
  cfg.drift_fov = 0.0;
  cfg.fov_spring = 2.2;

  cfg.speed_shake = 0.012;
  cfg.shake_scale = 1.0;
  cfg.impact_shake_decay = 7.0;
  cfg.impact_shake_frequency = 38.0;
  cfg.donut_orbit_speed = 0.55;
  return cfg;
}

struct CarState final {
  // World-space position of the car's origin (ground/centre, metres).
  Vec3 position{};
  // Car-body yaw, radians. 0 = +X, positive turns toward +Y.
  double yaw{};
  // World-space velocity, m/s.
  Vec3 velocity{};
  // Yaw rate, rad/s.
  double yaw_rate{};
  // World-space acceleration, m/s^2.
  Vec3 acceleration{};
  // Signed forward speed, m/s. Negative when reversing (used by callers only).
  double speed{};
  // Slip angle, radians: atan2(v . right, v . forward). Negative when the
  // velocity is to the left of the nose (a right-hand drift).
  double slip_angle{};
  // 0..1 drift amount from gameplay/physics. Optional; 0 lets the camera infer it.
  double drift_scale{};
  // 0..1 donut/burnout amount. Optional.
  double donut_scale{};
  // 0..1 nitrous. Optional.
  double nitrous{};
  // 0..1 throttle. Optional.
  double throttle{};
  // 0..1 filtered braking input. Negative derives braking pitch from
  // longitudinal acceleration; the reference tests use that default.
  double brake{-1.0};
  bool on_ground{true};
};

struct CameraState final {
  Vec3 position{};
  Vec3 target{};
  Vec3 up{};
  // Vertical FOV, degrees.
  double fov_y_degrees{};
  // Camera roll in radians (positive tilts the up vector toward the right).
  double roll{};
  // UE view rotation in degrees: pitch, yaw, roll.
  double pitch_degrees{};
  double yaw_degrees{};
  double roll_degrees{};

  // Convert the vertical lens to the horizontal lens used by UE's
  // FMinimalViewInfo::FOV.
  [[nodiscard]] double HorizontalFovDegrees(const double aspect_ratio) const noexcept {
    if (!(aspect_ratio > 0.0) || fov_y_degrees <= 1.0 ||
        fov_y_degrees >= 179.0) {
      return fov_y_degrees;
    }
    const double half =
        std::tan(fov_y_degrees * kRadiansPerDegree * 0.5) * aspect_ratio;
    return 2.0 * std::atan(half) * kDegreesPerRadian;
  }
};

class ChaseCam final {
 public:
  explicit ChaseCam(const Config& config = GamePreset()) noexcept : config_(config) {
    fov_ = config_.min_fov;
  }

  [[nodiscard]] const Config& config() const noexcept { return config_; }

  void SetConfig(const Config& config) noexcept {
    config_ = config;
    fov_ = config_.min_fov;
  }

  void SetSpeedDistanceScale(const double value) noexcept {
    config_.speed_distance_scale = Clamp(value, 0.0, 2.0);
  }

  void SetRollScale(const double value) noexcept {
    config_.roll_scale = Clamp(value, 0.0, 2.0);
  }

  void SetRollOnlyInDrift(const bool value) noexcept {
    config_.roll_only_in_drift = value;
  }

  void SetPitchScale(const double value) noexcept {
    config_.pitch_scale = Clamp(value, 0.0, 2.0);
  }

  void SetShakeScale(const double value) noexcept {
    config_.shake_scale = Clamp(value, 0.0, 2.0);
  }

  void Reset(const CarState& car) noexcept {
    const double slip = SafeSlip(car);
    position_azimuth_ = car.yaw;
    look_azimuth_ = car.yaw + slip * config_.look_slip_weight;
    position_ = DesiredPosition(car, car.yaw, config_.rest);
    position_offset_ = Sub(position_, car.position);
    position_velocity_ = {};
    drift_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    fov_ = config_.min_fov;
    smooth_longitudinal_acceleration_ = 0.0;
    impact_ = 0.0;
    impact_velocity_ = 0.0;
    time_ = 0.0;
    initialized_ = true;

    // Settle on the current operating point (speed, acceleration, drift) so a
    // reset recenters the camera without immediately springing back to the
    // old lag. The loop is cheap and only runs on start/reset/preset changes.
    for (int step = 0; step < 120; ++step) {
      static_cast<void>(Update(1.0 / 60.0, car));
    }
    // Restart the lens at its minimum so a reset does not pop the FOV to the
    // speed target; it springs there over the next second. Position, yaw,
    // roll and pitch are already settled above.
    fov_ = config_.min_fov;
    time_ = 0.0;
  }

  void AddImpact(const double strength) noexcept {
    impact_velocity_ += (std::max)(0.0, strength);
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] double drift() const noexcept { return drift_; }

  [[nodiscard]] CameraState Update(double dt, const CarState& car) noexcept {
    // The host can deliver a longer delta after a city hitch. The critically
    // damped spring is stable at 0.1 s; clamping at 1/30 only threw time away
    // and made the next frames visibly catch up.
    dt = Clamp(dt, 0.0, 0.1);
    if (!initialized_) {
      Reset(car);
    }
    time_ += dt;

    const Vec3 forward = ForwardFromYaw(car.yaw);
    const Vec3 right = RightFromYaw(car.yaw);

    const double speed =
        (std::max)(0.0, std::hypot(car.velocity.x, car.velocity.y));
    const double speed_mph = speed * kMphPerMetrePerSecond;
    const double slip = SafeSlip(car);
    const double absolute_slip = std::fabs(slip);

    // 1. Drift blend (`DriftScale` in NFS).
    const double speed_gate = SmoothStep(1.5, 7.0, speed);
    const double slip_blend =
        SmoothStep(config_.drift_start, config_.drift_full, absolute_slip);
    const double game_blend = Clamp(car.drift_scale, 0.0, 1.0);
    const double drift_target =
        (std::max)(slip_blend, game_blend) * speed_gate;
    const double drift_rate =
        drift_target > drift_ ? config_.drift_attack : config_.drift_release;
    drift_ = Damp(drift_, drift_target, drift_rate, dt);
    const double drift = drift_;

    // 2. Speed / acceleration offsets.
    const double speed01 =
        SmoothStep(config_.fov_start_mph, config_.fov_end_mph, speed_mph);
    const double move01 =
        SmoothStep(config_.speed_move_start, config_.speed_move_end, speed);

    const double longitudinal_raw = Dot(car.acceleration, forward);
    smooth_longitudinal_acceleration_ =
        Damp(smooth_longitudinal_acceleration_, longitudinal_raw, 10.0, dt);
    const double longitudinal =
        Clamp(smooth_longitudinal_acceleration_ /
                  config_.acceleration_normalisation,
              -1.0, 1.0);

    Vec3 rest = config_.rest;
    rest.z += (config_.speed_rest_height - config_.rest.z) * move01;
    const double distance_scale =
        Clamp(config_.speed_distance_scale, 0.0, 2.0);
    rest.x -= config_.speed_pull_back * move01 * distance_scale;

    // Acceleration / braking dolly (`AccelerationMovement`).
    if (longitudinal >= 0.0) {
      rest.x -= config_.acceleration_back * longitudinal * distance_scale;
      rest.z += config_.acceleration_up * longitudinal;
    } else {
      rest.x += config_.brake_forward * -longitudinal * distance_scale;
      rest.z -= config_.acceleration_down * -longitudinal;
    }

    // Drift close-up (`push-in`, `drop`, `side-swing`).
    rest.x += config_.drift_push_in * drift;
    rest.z -= config_.drift_drop * drift;
    if (slip > 0.0) {
      rest.y += config_.drift_side * drift;
    } else if (slip < 0.0) {
      rest.y -= config_.drift_side * drift;
    }

    // Donut / idle orbit (`DonutScale`, `CameraLocalDonutOffset`).
    const double donut = Clamp(car.donut_scale, 0.0, 1.0);
    if (donut > 0.0) {
      position_azimuth_ += config_.donut_orbit_speed * donut * dt;
      rest.y += 0.25 * donut;
      rest.x += 0.35 * donut;
    }

    // Never let the camera end up inside the car.
    rest.x = (std::min)(rest.x, -1.6);
    rest.z = (std::max)(rest.z, 0.45);

    Vec3 look{};
    look.y = 0.0;
    look.z = config_.look_height +
             (config_.speed_look_height - config_.look_height) * move01 -
             config_.drift_look_drop * drift;
    look.x = Lerp(config_.look_ahead_min, config_.look_ahead_max, speed01);

    // 3. Azimuth springs: position lags the nose, look follows its target.
    const double position_target =
        car.yaw + config_.position_slip_weight * slip * drift;
    const double look_target =
        car.yaw + config_.look_slip_weight * slip * drift;

    const bool entering = drift_target > drift;
    const double yaw_rate = entering
                                ? Lerp(config_.yaw_spring,
                                       config_.drift_yaw_spring, drift)
                                : config_.yaw_return_spring;

    position_azimuth_ =
        DampAngle(position_azimuth_, position_target, yaw_rate, dt);
    look_azimuth_ =
        DampAngle(look_azimuth_, look_target, config_.look_yaw_spring, dt);

    // Keep the camera from wrapping all the way around during a spin.
    position_azimuth_ =
        car.yaw + Clamp(WrapPi(position_azimuth_ - car.yaw),
                        -config_.max_position_lag, config_.max_position_lag);
    look_azimuth_ =
        car.yaw + Clamp(WrapPi(look_azimuth_ - car.yaw),
                        -config_.max_car_off_axis, config_.max_car_off_axis);

    // 4. Desired camera position and position spring.
    const Vec3 desired = DesiredPosition(car, position_azimuth_, rest);

    // Critically damped spring (`CenteringSprings`, `VelocityDrag`). Spring the
    // camera offset relative to the car, not its absolute world position: an
    // absolute spring lags every car translation, so at a low Game tick the
    // camera-to-car distance visibly pulses. Translation should be rigid; the
    // spring is for yaw/speed/drift offset changes.
    const double w = config_.position_spring;
    const double w2 = w * w;
    const double d = 2.0 * w;
    const Vec3 desired_offset = Sub(desired, car.position);
    position_velocity_.x +=
        ((desired_offset.x - position_offset_.x) * w2 -
         position_velocity_.x * d) * dt;
    position_velocity_.y +=
        ((desired_offset.y - position_offset_.y) * w2 -
         position_velocity_.y * d) * dt;
    position_velocity_.z +=
        ((desired_offset.z - position_offset_.z) * w2 -
         position_velocity_.z * d) * dt;
    position_offset_ = Add(position_offset_, Mul(position_velocity_, dt));
    position_ = Add(car.position, position_offset_);

    // 5. Roll / pitch / FOV.
    const double lateral_g = Dot(car.acceleration, right) / 9.81;

    double roll_target = -lateral_g * config_.roll_lateral_gain -
                         car.yaw_rate * config_.roll_yaw_gain;
    if (config_.roll_only_in_drift) {
      // Non-retail convenience: keep the horizon level in normal cornering.
      roll_target *= drift;
    }
    roll_target = Clamp(
        roll_target * Clamp(config_.roll_scale, 0.0, 2.0),
        -config_.max_roll_degrees * kRadiansPerDegree,
        config_.max_roll_degrees * kRadiansPerDegree);
    roll_ = Damp(roll_, roll_target, config_.roll_spring, dt);

    const double deceleration =
        car.brake >= 0.0 ? Clamp(car.brake, 0.0, 1.0)
                         : Clamp(-longitudinal, 0.0, 1.0);
    pitch_ = Damp(
        pitch_,
        deceleration * config_.max_pitch_degrees * kRadiansPerDegree *
            Clamp(config_.pitch_scale, 0.0, 2.0),
        config_.pitch_spring, dt);

    const double fov_target =
        Lerp(config_.min_fov, config_.max_fov, speed01) +
        config_.nitrous_fov * Clamp(car.nitrous, 0.0, 1.0) +
        config_.drift_fov * drift;
    fov_ = Damp(fov_, fov_target, config_.fov_spring, dt);

    // 6. Shake.
    // Impact is a damped oscillator (`ImpactShakeParams`).
    const double impact_acceleration =
        -impact_ * config_.impact_shake_frequency * config_.impact_shake_frequency;
    impact_velocity_ += impact_acceleration * dt;
    impact_velocity_ *= std::exp(-config_.impact_shake_decay * dt);
    impact_ += impact_velocity_ * dt;

    const double speed_shake = config_.speed_shake * speed01 * speed01;
    const double shake_amplitude =
        (speed_shake + (std::min)(0.25, std::fabs(impact_) * 0.08)) *
        Clamp(config_.shake_scale, 0.0, 2.0);
    const double t = time_;
    const Vec3 shake{Noise1(t * 12.7, 1.7), Noise1(t * 16.1, 4.2),
                     Noise1(t * 10.3, 8.9)};
    const Vec3 shake_position = Mul(shake, shake_amplitude);

    // 7. Build the camera frame.
    const Vec3 camera_position = Add(position_, shake_position);
    const Vec3 look_world =
        Add(car.position, RotateYaw(look_azimuth_, look));
    Vec3 view_forward = Normalize(Sub(look_world, camera_position));

    // Braking pitch: rotate the view around the camera's right axis.
    const Vec3 right0 = Normalize(Cross(Vec3{0.0, 0.0, 1.0}, view_forward));
    if (std::fabs(pitch_) > 1e-6) {
      view_forward = Normalize(RotateAxis(view_forward, right0, pitch_));
    }

    const Vec3 right1 = Normalize(Cross(Vec3{0.0, 0.0, 1.0}, view_forward));
    const Vec3 flat_up = Cross(view_forward, right1);
    const Vec3 up = Add(Mul(flat_up, std::cos(roll_)),
                        Mul(right1, std::sin(roll_)));

    CameraState result;
    result.position = camera_position;
    result.target = Add(camera_position, view_forward);
    result.up = up;
    result.fov_y_degrees = fov_;
    result.roll = roll_;
    result.pitch_degrees =
        std::asin(Clamp(view_forward.z, -1.0, 1.0)) * kDegreesPerRadian;
    result.yaw_degrees = std::atan2(view_forward.y, view_forward.x) *
                         kDegreesPerRadian;
    result.roll_degrees = roll_ * kDegreesPerRadian;
    return result;
  }

 private:
  [[nodiscard]] double SafeSlip(const CarState& car) const noexcept {
    const double speed = std::hypot(car.velocity.x, car.velocity.y);
    if (speed < 0.75) {
      return 0.0;
    }
    return Clamp(car.slip_angle, -kPi * 0.5, kPi * 0.5);
  }

  [[nodiscard]] Vec3 DesiredPosition(const CarState& car,
                                     const double position_azimuth,
                                     const Vec3& rest) const noexcept {
    const Vec3 pivot_world = Add(car.position, RotateYaw(car.yaw, config_.pivot));
    const Vec3 offset = Sub(rest, config_.pivot);
    // The offset from the pivot is car-local, so it turns with the camera's
    // *absolute* azimuth. Using only the lag here leaves the camera on the
    // world axis whenever the car is not facing +X.
    return Add(pivot_world, RotateYaw(position_azimuth, offset));
  }

  Config config_;
  bool initialized_{};
  double time_{};

  double position_azimuth_{};
  double look_azimuth_{};
  Vec3 position_{};
  Vec3 position_offset_{};
  Vec3 position_velocity_{};

  double drift_{};
  double roll_{};
  double pitch_{};
  double fov_{};

  double smooth_longitudinal_acceleration_{};
  double impact_{};
  double impact_velocity_{};
};

}  // namespace nfs2015_chase
