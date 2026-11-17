#pragma once

// Parked orbit camera model (NFS 2015 contact / idle camera).
//
// The shot activates when the car has been standing still with no vehicle input
// for a while, then circles the car on a 60 s revolution: radius 5.05 m,
// elevation 6.8 degrees (or a configurable camera height), looking at the car
// origin + 1.23 m, with a +-1 m tangential weave at t = 15 s and t = 45 s.
// NFS 2015 authors the orbit on the Frostbite world +Y axis; in this plugin's
// UE frame (+X forward, +Y right, +Z up) that is the car body's own vertical
// axis, so the orbit follows the body pitch and keeps the horizon level.
//
// This header is intentionally independent of the plugin's Game-tick and render
// plumbing (no SDK, no Windows, no clock). It consumes a `CarState`, the
// already-filtered `BurnoutSample` and the camera pose the caller would use
// if the orbit were off. The presenter owns the blend and the FOV fallback;
// nothing here reads the game or writes a camera pose.
//
// Deliberate deviations from the reference guide:
//   * No camera collision / sphere cast (the user asked for the shot first).
//   * Position damping exists but defaults to `tau = 0` (bypassed). The guide
//     leaves tau unsolved and the acceptance numbers require a stable 5.05 m
//     radius; enable it only if the real game shows anchor jitter.
//
// During the shot the eye stays smooth. A band-limited value-noise offset is
// applied only to the look-at target (vertical 0.05 m, horizontal 0.011 m at
// 5 Hz by default), matching the reference mode's handheld component. Its
// amplitude is scaled by the entry/exit blend through shot_blend.hpp.
//
// Weave keys are the asset's 60 s TranslationX track, independent of the
// configurable orbit period: `period_seconds` only changes the angular rate.
// Putting the weave on the radius instead of the tangent makes the camera
// breathe in and out, which is the classic implementation mistake.

#include "car_sample.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace nfs2015_chase {

// Reference geometry (guide section 3). The look-at point sits above the car
// origin, not on it: targeting the origin frames the tyres instead of the car.
inline constexpr double kOrbitLookAtHeightM = 1.23;
inline constexpr double kOrbitElevationDegrees = 6.8;

// One line to flip if the orbit turns the wrong way in the reference footage.
inline constexpr double kOrbitDirectionSign = 1.0;

// The weave is a separate 60 s track in the NFS asset. Its keys are not evenly
// spaced (29.963 s, not 30 s), and every tangent is zero, so a zero-slope
// smoothstep per segment reproduces it exactly.
inline constexpr double kOrbitWeavePeriodSeconds = 60.0;
inline constexpr double kOrbitWeaveFirstPeakSeconds = 15.0;
inline constexpr double kOrbitWeaveFirstZeroSeconds = 29.963;
inline constexpr double kOrbitWeaveSecondPeakSeconds = 45.0;

// Handheld look-at noise. The values are amplitudes, not peak-to-peak. The
// reference mode uses a 5 Hz band (0.2 s between LCG values) and independent
// LCG streams for the horizontal and vertical axes.
inline constexpr double kOrbitHandheldHorizontalM = 0.011;
inline constexpr double kOrbitHandheldVerticalM = 0.050;
inline constexpr double kOrbitHandheldFrequencyHz = 5.0;
inline constexpr std::uint32_t kOrbitHandheldSeedX = 0x1234u;
inline constexpr std::uint32_t kOrbitHandheldSeedY = 0x5678u;

struct OrbitShotConfig final {
  // 8.0 s of standing still, then 2.0 s into the orbit, 1.0 s back out.
  double entry_delay{8.0};
  double entry_time{2.0};
  double exit_time{1.0};

  // Orbit geometry. `period_seconds` is one full revolution.
  double period_seconds{60.0};
  double radius_m{5.05};
  // Absolute body-up camera height above the car origin. A positive value
  // overrides elevation_degrees while keeping the slant radius; zero keeps the
  // guide elevation for callers that only set the legacy geometry.
  double camera_height_m{0.0};
  double elevation_degrees{kOrbitElevationDegrees};
  double look_at_height_m{kOrbitLookAtHeightM};
  double weave_amplitude_m{1.0};
  double vertical_fov_degrees{45.0};

  // Handheld look-at noise. Strength 0 disables the effect; the frequency is
  // clamped to the guide's 0.5..30 Hz band inside Update().
  double handheld_strength{1.0};
  double handheld_frequency_hz{kOrbitHandheldFrequencyHz};

  // Entry/hold speed gates, m/s. NFS starts the idle timer when the car is
  // effectively stationary; the same threshold is a safe hold threshold.
  double entry_speed_mps{0.14};
  double hold_speed_mps{0.14};

  bool slope_follow{true};
  double direction{kOrbitDirectionSign};

  // 0 = disabled. See the header note.
  double position_tau{0.0};
};

enum class OrbitPhase : std::uint8_t {
  kIdle = 0,
  kEnter = 1,
  kOrbit = 2,
  kExit = 3,
};

// Stable labels for logs and the diagnostics panel. The numeric order is part
// of the published diagnostic snapshot, so do not renumber the enum.
[[nodiscard]] inline const char* OrbitPhaseName(
    const OrbitPhase phase) noexcept {
  switch (phase) {
    case OrbitPhase::kIdle:
      return "Idle";
    case OrbitPhase::kEnter:
      return "Enter";
    case OrbitPhase::kOrbit:
      return "Orbit";
    case OrbitPhase::kExit:
      return "Exit";
  }
  return "Idle";
}

struct OrbitShotState final {
  // `phase` is taken by the 0..1 blend weight below, so the state enum is
  // called `mode`. (The plan's API sketch cannot declare both as `phase`.)
  OrbitPhase mode{OrbitPhase::kIdle};
  double hold_time{};
  // Linear 0..1 transition weight; the result exposes its smoothstep.
  double phase{};
  // Orbit clock, seconds since entry completed. Frozen during Exit.
  double time{};
  // Entry azimuth in the car body frame, radians.
  double phase0{};
  bool latched{};
  double speed_mps{};
  bool input{};
  // Final look-at offset after strength scaling, in metres. Kept in the state
  // so diagnostics and tests can read the exact values used for the target.
  double shake_x_m{};
  double shake_y_m{};
};

struct OrbitShotResult final {
  CameraState view{};
  double blend{};
  OrbitShotState state{};
};

namespace orbit_shot_detail {

[[nodiscard]] inline double Finite(const double value,
                                   const double fallback) noexcept {
  return std::isfinite(value) ? value : fallback;
}

[[nodiscard]] inline Vec3 SafeVec(const Vec3 value,
                                  const Vec3 fallback = {}) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
                 std::isfinite(value.z)
             ? value
             : fallback;
}

// Zero-slope Hermite / smoothstep on one 0..1 interval.
[[nodiscard]] inline double Smooth01(double u) noexcept {
  u = Clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

// One axis of the guide's single-octave value-noise band. `prev`/`cur` start
// at zero, so the first period always produces zero and every entry is
// reproducible. The LCG and float bit pattern are part of the reference
// algorithm; do not substitute a different RNG.
struct ValueNoiseBand final {
  double period{0.2};
  double t{};
  double prev{};
  double cur{};
  std::uint32_t seed{};
};

[[nodiscard]] inline double ValueNoiseSigned(
    const std::uint32_t seed) noexcept {
  static_assert(sizeof(float) == sizeof(std::uint32_t),
                "handheld noise expects a 32-bit float");
  const std::uint32_t bits = (seed >> 9) | 0x3F800000u;
  float unit = 0.0F;
  std::memcpy(&unit, &bits, sizeof(unit));
  return (static_cast<double>(unit) - 1.0) * 2.0 - 1.0;
}

[[nodiscard]] inline double ValueNoiseStep(ValueNoiseBand& band,
                                           const double dt) noexcept {
  const double period = (std::max)(1e-6, Finite(band.period, 0.2));
  band.t += (std::max)(0.0, Finite(dt, 0.0));
  while (band.t >= period) {
    band.prev = band.cur;
    band.seed = band.seed * 1664525u + 1013904223u;
    band.cur = ValueNoiseSigned(band.seed);
    band.t -= period;
  }
  const double s = 0.5 - 0.5 * std::cos(kPi * band.t / period);
  return (1.0 - s) * band.prev + s * band.cur;
}

[[nodiscard]] inline CarBasis OrbitBasis(const CarState& car,
                                         const bool slope_follow) noexcept {
  constexpr double kPitchLimit = 89.0 * kRadiansPerDegree;
  const double yaw = Finite(car.yaw, 0.0);
  const double pitch =
      slope_follow ? Clamp(Finite(car.pitch, 0.0), -kPitchLimit, kPitchLimit)
                   : 0.0;
  return MakeCarBasis(yaw, pitch);
}

}  // namespace orbit_shot_detail

// The NFS 2015 TranslationX track: keys 0 / 15 / 29.963 / 45 / 60 s map to
// 0 / +1 / 0 / +1 / 0 with zero slope at every key. Returns a value in [0, 1].
[[nodiscard]] inline double OrbitWeave(
    const double time_seconds) noexcept {
  const double period = kOrbitWeavePeriodSeconds;
  double u = std::fmod(
      orbit_shot_detail::Finite(time_seconds, 0.0), period);
  if (u < 0.0) {
    u += period;
  }
  if (u < kOrbitWeaveFirstPeakSeconds) {
    return orbit_shot_detail::Smooth01(u / kOrbitWeaveFirstPeakSeconds);
  }
  if (u < kOrbitWeaveFirstZeroSeconds) {
    return 1.0 - orbit_shot_detail::Smooth01(
                     (u - kOrbitWeaveFirstPeakSeconds) /
                     (kOrbitWeaveFirstZeroSeconds -
                      kOrbitWeaveFirstPeakSeconds));
  }
  if (u < kOrbitWeaveSecondPeakSeconds) {
    return orbit_shot_detail::Smooth01(
        (u - kOrbitWeaveFirstZeroSeconds) /
        (kOrbitWeaveSecondPeakSeconds - kOrbitWeaveFirstZeroSeconds));
  }
  return 1.0 - orbit_shot_detail::Smooth01(
                   (u - kOrbitWeaveSecondPeakSeconds) /
                   (period - kOrbitWeaveSecondPeakSeconds));
}

// Pure state machine plus pose. The class owns no clock and no locks: `Update`
// receives dt from the render clock and the current (orbit-off) camera, and
// returns the shot pose plus the smoothstep blend the presenter should use.
class OrbitShot final {
 public:
  explicit OrbitShot(OrbitShotConfig config = {}) noexcept
      : config_(config) {
    ResetNoise();
  }

  void Reset() noexcept {
    state_ = {};
    damped_position_ = {};
    have_damped_position_ = false;
    ResetNoise();
  }

  // Changing settings does not reset an in-flight shot, so the presenter can
  // retune radius/FOV without a hard cut. Call Reset() on epoch/world changes
  // or when the view is re-activated after a long time elsewhere.
  void SetConfig(const OrbitShotConfig& config) noexcept { config_ = config; }

  [[nodiscard]] const OrbitShotConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] const OrbitShotState& state() const noexcept { return state_; }

  [[nodiscard]] OrbitShotResult Update(
      const double dt_seconds, const CarState& car,
      const BurnoutSample& sample, const Vec3& current_camera,
      const double aspect_ratio = 16.0 / 9.0) noexcept {
    using namespace orbit_shot_detail;
    // The configured lens is vertical; the aspect ratio does not enter the
    // pose. It stays in the signature for symmetry with the other shot models.
    (void)aspect_ratio;
    const double dt = Clamp(Finite(dt_seconds, 0.0), 0.0, 0.1);

    const double entry_delay =
        (std::max)(0.0, Finite(config_.entry_delay, 8.0));
    const double entry_time =
        (std::max)(1e-6, Finite(config_.entry_time, 2.0));
    const double exit_time =
        (std::max)(1e-6, Finite(config_.exit_time, 1.0));
    const double entry_speed =
        (std::max)(0.0, Finite(config_.entry_speed_mps, 0.14));
    const double hold_speed =
        (std::max)(0.0, Finite(config_.hold_speed_mps, 0.14));

    // Horizontal speed is what matters: a car sliding sideways is moving even
    // when its forward speed is zero. `velocity` is a damped position
    // differential and can legitimately be zero while a caller still publishes
    // a forward speed, so a vanishing differential falls back to the published
    // speed signals rather than freezing the exit gate.
    const Vec3 velocity = SafeVec(car.velocity);
    double speed = std::hypot(velocity.x, velocity.y);
    if (!std::isfinite(speed) || speed < 1e-6) {
      speed = (std::max)(std::fabs(Finite(sample.speed_mps, 0.0)),
                         std::fabs(Finite(car.speed, 0.0)));
    }
    speed = (std::max)(0.0, Finite(speed, 0.0));

    const bool input = sample.input_active;
    const bool can_enter = speed <= entry_speed && !input;
    const bool can_hold = speed <= hold_speed && !input;

    state_.speed_mps = speed;
    state_.input = input;
    if (can_enter) {
      state_.hold_time += dt;
    } else {
      state_.hold_time = 0.0;
    }

    const CarBasis basis = OrbitBasis(car, config_.slope_follow);

    // Exiting on the frame the trigger appears keeps the pull-out exactly
    // `exit_time` long instead of one frame longer. While still parked it
    // rises again with the same duration (no 8 s wait).
    const auto step_exit = [&]() {
      if (can_enter) {
        state_.phase += dt / exit_time;
        if (state_.phase + 1e-12 >= 1.0) {
          state_.phase = 1.0;
          state_.mode = OrbitPhase::kOrbit;
        }
      } else {
        state_.phase -= dt / exit_time;
        if (state_.phase <= 1e-9) {
          state_.phase = 0.0;
          state_.mode = OrbitPhase::kIdle;
          state_.latched = false;
          state_.hold_time = 0.0;
          state_.time = 0.0;
          have_damped_position_ = false;
          ResetNoise();
        }
      }
    };

    switch (state_.mode) {
      case OrbitPhase::kIdle:
        // Idle -> Enter. Latch the entry azimuth once, from the camera the
        // caller would show if the orbit were off, so the blend starts from
        // the chase pose instead of snapping to the front of the car.
        state_.latched = false;
        if (can_enter && state_.hold_time + 1e-12 >= entry_delay) {
          state_.latched = true;
          state_.phase = 0.0;
          state_.time = 0.0;
          state_.phase0 =
              EntryPhase0(car, basis, current_camera,
                          (std::max)(0.0, Finite(config_.look_at_height_m,
                                                 kOrbitLookAtHeightM)));
          state_.mode = OrbitPhase::kEnter;
          have_damped_position_ = false;
          ResetNoise();
        }
        break;
      case OrbitPhase::kEnter:
        if (!can_enter) {
          state_.mode = OrbitPhase::kExit;
          step_exit();
        } else {
          state_.phase += dt / entry_time;
          if (state_.phase + 1e-12 >= 1.0) {
            state_.phase = 1.0;
            state_.mode = OrbitPhase::kOrbit;
            // The orbit clock starts at zero on the frame the entry completes.
            state_.time = 0.0;
          }
        }
        break;
      case OrbitPhase::kOrbit:
        if (can_hold) {
          state_.time += dt;
        } else {
          state_.mode = OrbitPhase::kExit;
          step_exit();
        }
        break;
      case OrbitPhase::kExit:
        step_exit();
        break;
    }

    UpdateHandheldNoise(dt);

    // Orbit pose. `dir`/`tangent` live in the car body's forward-right plane,
    // so on a slope the whole circle follows the body pitch (guide Q5-B).
    const double look_at_height =
        (std::max)(0.0, Finite(config_.look_at_height_m,
                              kOrbitLookAtHeightM));
    const Vec3 pivot =
        Add(SafeVec(car.position), Mul(basis.up, look_at_height));
    const double radius =
        (std::max)(0.0, Finite(config_.radius_m, 5.05));
    const double camera_height = Finite(config_.camera_height_m, 0.0);
    double horizontal_radius;
    double vertical_offset;
    if (camera_height > 0.0) {
      // The height is measured along the body up axis from the car origin.
      // Clamp to the sphere when a small radius cannot reach the requested
      // height; the UI range is chosen to avoid this in normal use.
      vertical_offset =
          Clamp(camera_height - look_at_height, -radius, radius);
      horizontal_radius = std::sqrt(
          (std::max)(0.0, radius * radius - vertical_offset * vertical_offset));
    } else {
      const double elevation =
          Clamp(Finite(config_.elevation_degrees, kOrbitElevationDegrees),
                -89.0, 89.0) *
          kRadiansPerDegree;
      horizontal_radius = radius * std::cos(elevation);
      vertical_offset = radius * std::sin(elevation);
    }
    const double period =
        (std::max)(1e-3, Finite(config_.period_seconds, 60.0));
    const double omega = 2.0 * kPi / period;
    const double direction =
        Finite(config_.direction, kOrbitDirectionSign);
    const double weave_amplitude =
        Finite(config_.weave_amplitude_m, 1.0);
    const double theta =
        state_.phase0 + direction * omega * state_.time;
    const double cosine = std::cos(theta);
    const double sine = std::sin(theta);
    const Vec3 radial = Add(Mul(basis.forward, cosine),
                            Mul(basis.right, sine));
    const Vec3 tangent = Add(Mul(basis.forward, -sine),
                             Mul(basis.right, cosine));

    Vec3 orbit_position =
        Add(Add(pivot, Mul(radial, horizontal_radius)),
            Mul(basis.up, vertical_offset));
    orbit_position = Add(
        orbit_position,
        Mul(tangent, weave_amplitude * OrbitWeave(state_.time)));

    Vec3 eye = orbit_position;
    const double tau = (std::max)(0.0, Finite(config_.position_tau, 0.0));
    if (tau > 1e-6) {
      if (!have_damped_position_) {
        damped_position_ = orbit_position;
        have_damped_position_ = true;
      }
      const double alpha =
          1.0 - std::exp(-dt / (std::max)(tau, 1e-4));
      damped_position_ = Add(
          damped_position_, Mul(Sub(orbit_position, damped_position_), alpha));
      eye = damped_position_;
    }

    // Look at the pivot; up stays world +Z so the horizon is level even on a
    // slope, and roll is exactly zero (NFS has no RotationZ animation).
    Vec3 look = Sub(pivot, eye);
    if (Length(look) < 1e-9) {
      look = basis.forward;
    }
    look = Normalize(look);

    // Handheld noise changes only the look-at point. The eye, up and roll are
    // untouched, so the orbit radius stays exactly the same. Branches on zero
    // keep `strength == 0` bit-for-bit identical to the unshaken shot.
    Vec3 target = pivot;
    if (state_.shake_x_m != 0.0 || state_.shake_y_m != 0.0) {
      Vec3 right = Cross(Vec3{0.0, 0.0, 1.0}, look);
      if (Length(right) < 1e-9) {
        right = basis.right;
      }
      right = Normalize(right);
      const Vec3 up_local = Normalize(Cross(look, right));
      target = Add(pivot, Add(Mul(right, state_.shake_x_m),
                              Mul(up_local, state_.shake_y_m)));
      look = Normalize(Sub(target, eye));
    }

    CameraState view;
    view.position = eye;
    view.target = target;
    view.up = {0.0, 0.0, 1.0};
    view.fov_y_degrees =
        Clamp(Finite(config_.vertical_fov_degrees, 45.0), 1.0, 179.0);
    view.roll = 0.0;
    view.roll_degrees = 0.0;
    view.pitch_degrees =
        std::asin(Clamp(look.z, -1.0, 1.0)) * kDegreesPerRadian;
    view.yaw_degrees =
        std::atan2(look.y, look.x) * kDegreesPerRadian;

    OrbitShotResult result;
    result.view = view;
    result.blend = SmoothStep(0.0, 1.0, state_.phase);
    result.state = state_;
    return result;
  }

 private:
  void ResetNoise() noexcept {
    noise_x_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_, 0.0, 0.0, 0.0, kOrbitHandheldSeedX};
    noise_y_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_, 0.0, 0.0, 0.0, kOrbitHandheldSeedY};
    state_.shake_x_m = 0.0;
    state_.shake_y_m = 0.0;
  }

  void UpdateHandheldPeriod(const double frequency_hz) noexcept {
    const double frequency = Clamp(
        orbit_shot_detail::Finite(frequency_hz, kOrbitHandheldFrequencyHz),
        0.5, 30.0);
    const double period = 1.0 / frequency;
    if (period == noise_period_) {
      return;
    }
    const double scale = period / noise_period_;
    noise_x_.t *= scale;
    noise_y_.t *= scale;
    noise_x_.period = period;
    noise_y_.period = period;
    noise_period_ = period;
  }

  void UpdateHandheldNoise(const double dt) noexcept {
    UpdateHandheldPeriod(config_.handheld_frequency_hz);
    if (state_.mode == OrbitPhase::kIdle) {
      state_.shake_x_m = 0.0;
      state_.shake_y_m = 0.0;
      return;
    }
    const double strength = (std::max)(
        0.0, orbit_shot_detail::Finite(config_.handheld_strength, 1.0));
    // Advance even when strength is zero; only the returned offset is zero.
    const double noise_x = orbit_shot_detail::ValueNoiseStep(noise_x_, dt);
    const double noise_y = orbit_shot_detail::ValueNoiseStep(noise_y_, dt);
    state_.shake_x_m = kOrbitHandheldHorizontalM * strength * noise_x;
    state_.shake_y_m = kOrbitHandheldVerticalM * strength * noise_y;
  }

  [[nodiscard]] static double EntryPhase0(
      const CarState& car, const CarBasis& basis, const Vec3& current_camera,
      const double look_at_height_m) noexcept {
    using namespace orbit_shot_detail;
    const Vec3 pivot = Add(SafeVec(car.position),
                           Mul(basis.up, look_at_height_m));
    const Vec3 offset = Sub(SafeVec(current_camera), pivot);
    const double x = Dot(offset, basis.forward);
    const double y = Dot(offset, basis.right);
    // Too close to the pivot: the azimuth is meaningless. Start behind the car.
    if (std::hypot(x, y) < 0.5) {
      return kPi;
    }
    return std::atan2(y, x);
  }

  OrbitShotConfig config_{};
  OrbitShotState state_{};
  Vec3 damped_position_{};
  bool have_damped_position_{};
  double noise_period_{1.0 / kOrbitHandheldFrequencyHz};
  orbit_shot_detail::ValueNoiseBand noise_x_{};
  orbit_shot_detail::ValueNoiseBand noise_y_{};
};

}  // namespace nfs2015_chase
