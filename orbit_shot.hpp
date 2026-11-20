#pragma once

// Parked orbit camera model (NFS 2015 contact / idle camera).
//
// The shot activates when the car has been standing still with no vehicle input
// for a while, then circles the car on a 60 s revolution: radius 5.05 m,
// elevation 6.8 degrees (or a configurable camera height), looking at a pivot
// 1.23 m above and 0.772 m ahead of the car origin (the asset's arm offset
// s = (0, 1.2258, 0.7720)), with a +-1 m tangential weave at t = 15 s and
// t = 45 s.
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
// During the shot the eye stays smooth. Three look-at layers ride on top of the
// geometry above, each with its own evidence level:
//
//   * Target noise (A-grade). A band-limited value-noise offset applied only to
//     the look-at point. Runtime write-breakpoint evidence from the retail
//     orbit (2026-11-18, pose 0xE504B390, write 0x143FF13BC) shows the active
//     controller is Instance18: horizontal 0.020 m, vertical 0.070 m, 2 Hz
//     base, two octaves, persistence 0.5. Instance14 (0.011/0.05 m at 5 Hz,
//     one octave) is present in the asset but its live scale is 0 and it never
//     writes the orbit target, so do not copy it.
//   * Target low-frequency sway (B-grade). `Instance192` (right, +-1) and
//     `Instance188` (up, 0..-0.3) drive `Instance279`, and `Instance279` is the
//     only input of `Instance25`'s TARGET reaction, which adds onto pose+0x10 -
//     the same field the noise above writes. The tracks and the 1.0 s damper
//     (`Instance127`, DampTime 1.0) are read off the asset; the amplitude
//     `Instance279` finally emits was never measured, so the metres below are
//     the asset keys rather than a runtime output, and the layer is a slider.
//   * Roll noise (A-grade). `Instance16` is a ROLL NoiseController: magnitude
//     0.7 degrees, frequency 2 Hz, two octaves, roughness 0.1. The decompiled
//     controller adds `noise1d * magnitude.x` into pose+0x64 and a 30 s retail
//     capture lands in -0.71..+0.68 degrees, so this one is closed. It shares
//     the handheld strength slider, so strength 0 keeps the horizon level.
//
// The camera's tangential weave (`weave_amplitude_m`) stays as it was: the
// asset's `Instance74` is a TransformPartPropertyTrackData TranslationX in the
// 60 s sequence, the asset graph shows it never reaches `Instance25`'s target
// reaction, and whether it moves the camera arm is still open (B/C-grade). It
// is neither proven nor disproven, so it remains a separately adjustable
// approximation.
//
// The asset's 30 Hz layer is deliberately not implemented. A 60 Hz capture of
// the retail orbit shows no 0.1 m signature - the band is buried at about
// 0.013 m - so adding it would invent motion.
//
// Every layer is scaled by the entry/exit blend through shot_blend.hpp.
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
// The reference arm is `B = T + M * (0, 1.2258, 0.7720)` (report 6B.19):
// the pivot is 1.2258 m up and 0.7720 m along the car's forward axis from the
// base T. The height is folded into kOrbitLookAtHeightM; this constant is the
// forward part that the old guide geometry left out.
inline constexpr double kOrbitPivotForwardM = 0.772;
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

// Handheld look-at noise. These are the Instance18 amplitudes, not
// peak-to-peak: 0.02 m horizontal, 0.07 m vertical, 2 Hz base band, two
// octaves, persistence (roughness) 0.5. The second octave runs at 4 Hz and is
// weighted 0.5; the two bands use independent LCG streams per axis.
inline constexpr double kOrbitHandheldHorizontalM = 0.020;
inline constexpr double kOrbitHandheldVerticalM = 0.070;
inline constexpr double kOrbitHandheldFrequencyHz = 2.0;
inline constexpr double kOrbitHandheldRoughness = 0.5;
inline constexpr std::uint32_t kOrbitHandheldSeedX = 0x1234u;
inline constexpr std::uint32_t kOrbitHandheldSeedY = 0x5678u;
inline constexpr std::uint32_t kOrbitHandheldSeedX2 = 0x9E3779B9u;
inline constexpr std::uint32_t kOrbitHandheldSeedY2 = 0x85EBCA6Bu;

// Look-at low-frequency sway (`Instance279`, B-grade: the tracks and the damper
// are read off the asset, the output amplitude was never measured). The key
// tables below are stored normalised - Instance192 in +-1, Instance188 in -1..0
// - and the two amplitude constants turn them into metres. 60 s period, so the
// sway rides the same clock as the camera weave.
inline constexpr double kOrbitTargetSwayDampSeconds = 1.0;
inline constexpr double kOrbitTargetSwayPeriodSeconds = 60.0;
inline constexpr double kOrbitTargetSwayRightM = 1.0;
inline constexpr double kOrbitTargetSwayUpM = 0.3;

// Instance192 right track, normalised: 0 / 15 / 22.5 / 30 / 37.5 / 45 s map to
// 0 / 0 / +1 / 0 / -1 / 0, and the track wraps back to 0 at 60 s.
inline constexpr int kOrbitTargetSwayXKeyCount = 6;
inline constexpr double kOrbitTargetSwayXTimes[kOrbitTargetSwayXKeyCount] = {
    0.0, 15.0, 22.5, 30.0, 37.5, 45.0};
inline constexpr double kOrbitTargetSwayXValues[kOrbitTargetSwayXKeyCount] = {
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0};

// Instance188 up track, normalised: 0 / 15 / 30 / 45 s map to -1 / 0 / -1 / 0,
// and the track wraps back to -1 at 60 s.
inline constexpr int kOrbitTargetSwayYKeyCount = 4;
inline constexpr double kOrbitTargetSwayYTimes[kOrbitTargetSwayYKeyCount] = {
    0.0, 15.0, 30.0, 45.0};
inline constexpr double kOrbitTargetSwayYValues[kOrbitTargetSwayYKeyCount] = {
    -1.0, 0.0, -1.0, 0.0};

// Vertical FOV of the parked orbit. The chase preset authors 43 degrees and the
// orbit controller subtracts 15, so the runtime value is 28.
inline constexpr double kOrbitVerticalFovDegrees = 28.0;

// Roll noise (`Instance16`, A-grade): magnitude 0.7 degrees, 2 Hz, two octaves,
// roughness 0.1. The second band runs at 4 Hz and is weighted 0.1. The band
// frequency follows the handheld slider, so the two constants must agree.
inline constexpr double kOrbitRollNoiseDegrees = 0.7;
inline constexpr double kOrbitRollNoiseFrequencyHz = 2.0;
inline constexpr double kOrbitRollNoiseRoughness = 0.1;
inline constexpr std::uint32_t kOrbitRollSeed = 0xA5A5A5A5u;
inline constexpr std::uint32_t kOrbitRollSeed2 = 0x5A5A5A5Au;
static_assert(kOrbitRollNoiseFrequencyHz == kOrbitHandheldFrequencyHz,
              "the roll noise shares the handheld frequency slider");

struct OrbitShotConfig final {
  // 8.0 s of standing still, then 2.0 s into the orbit, 1.0 s back out.
  double entry_delay{8.0};
  double entry_time{2.0};
  double exit_time{1.0};

// Orbit geometry. `period_seconds` is one full revolution.
  //
  // The numbers below are the reference guide's geometry, kept as the model's
  // fallbacks so this header stays a faithful copy of the analysed asset. The
  // plugin does not ship them: every run sets the radius, the camera height and
  // the look-at height from the settings, whose defaults live in
  // src/constants.hpp (kDefaultOrbitRadius / kDefaultOrbitHeight /
  // kDefaultOrbitLookAtHeight).
  double period_seconds{60.0};
  double radius_m{5.05};
  // Absolute body-up camera height above the car origin. A positive value
  // overrides elevation_degrees while keeping the slant radius; zero keeps the
  // guide elevation for callers that only set the legacy geometry.
  double camera_height_m{0.0};
  double elevation_degrees{kOrbitElevationDegrees};
  double look_at_height_m{kOrbitLookAtHeightM};
  // Forward offset of the orbit pivot from the car origin, in the car body
  // frame. The asset arm is `s = (right=0, up=1.2258, forward=0.7720)`; the
  // old guide only carried the up component. Set to 0 for the legacy
  // car-origin-centred orbit.
  double pivot_forward_m{kOrbitPivotForwardM};
  double weave_amplitude_m{1.0};
  // Look-at low-frequency sway scale, 0 disables the layer. 1.0 keeps the
  // asset keys (+-1 m along the camera's right, 0..-0.3 m along its up) as
  // they are; the settings slider allows 0..2. B-grade: see OrbitTargetSwayX
  // and OrbitTargetSwayY for what is known and what is not.
  double target_sway{1.0};
  // Vertical FOV. 28 degrees is the runtime value (43 authored, minus the
  // controller's 15); the settings slider can still pick anything between 20
  // and 70.
  double vertical_fov_degrees{kOrbitVerticalFovDegrees};

  // Handheld look-at noise. Strength 0 disables the effect; the frequency is
  // clamped to the guide's 0.5..30 Hz band inside Update().
  double handheld_strength{1.0};
  double handheld_frequency_hz{kOrbitHandheldFrequencyHz};
  double handheld_roughness{kOrbitHandheldRoughness};

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
  // Low-frequency target sway after the strength scale and the 1 s damper, in
  // metres along the camera's local right / up. Deliberately separate from the
  // noise above: the two layers have independent sliders.
  double target_sway_x_m{};
  double target_sway_y_m{};
  // Roll noise for this frame in degrees, after the shared strength scale.
  double roll_noise_degrees{};
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

// Periodic Catmull-Rom over a keyed track. `times` must be strictly increasing
// and live in [0, period); the segment after the last key wraps to key 0 shifted
// by +period, so every key has a left and a right neighbour. Tangents are
// central differences divided by the real key spacing (non-uniform Catmull-Rom),
// which keeps the asset's mixed 7.5 s / 15 s spacing C1 without the extra
// overshoot the uniform form produces. The result is clamped because the asset
// keys are the extremes and an interpolating spline can still leave them.
[[nodiscard]] inline double SamplePeriodicKeyedTrack(
    const double* times, const double* values, const int count,
    const double period, const double time_seconds, const double low,
    const double high) noexcept {
  if (times == nullptr || values == nullptr || count <= 0) {
    return 0.0;
  }
  if (count == 1) {
    return Clamp(Finite(values[0], 0.0), low, high);
  }
  const double span = (std::max)(1e-6, Finite(period, 60.0));
  double t = std::fmod(Finite(time_seconds, 0.0), span);
  if (t < 0.0) {
    t += span;
  }
  // Largest key index whose time is <= t; everything past the last key is the
  // wrap segment that ends at key 0 plus one period.
  int i = 0;
  for (int k = 1; k < count; ++k) {
    if (t < times[k]) {
      break;
    }
    i = k;
  }
  const int j = (i + 1) % count;
  const int p = (i + count - 1) % count;
  const int n = (j + 1) % count;
  // Put every neighbour on the same unwrapped timeline as key i. `times` only
  // covers [0, period), so a neighbour that sits on the other side of the wrap
  // has to be shifted by whole periods until it is on the right side of its
  // reference key. Doing it with loops instead of index tests keeps the
  // three-key and two-key tracks correct as well.
  const double t_i = times[i];
  double t_j = times[j];
  while (t_j <= t_i) {
    t_j += span;
  }
  double t_p = times[p];
  while (t_p >= t_i) {
    t_p -= span;
  }
  double t_n = times[n];
  while (t_n <= t_j) {
    t_n += span;
  }
  const double v_i = Finite(values[i], 0.0);
  const double v_j = Finite(values[j], 0.0);
  const double v_p = Finite(values[p], 0.0);
  const double v_n = Finite(values[n], 0.0);
  const double h = (std::max)(1e-9, t_j - t_i);
  const double u = Clamp((t - t_i) / h, 0.0, 1.0);
  const double m_i = (v_j - v_p) / (std::max)(1e-9, t_j - t_p);
  const double m_j = (v_n - v_i) / (std::max)(1e-9, t_n - t_i);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double value = (2.0 * u3 - 3.0 * u2 + 1.0) * v_i +
                       (u3 - 2.0 * u2 + u) * h * m_i +
                       (-2.0 * u3 + 3.0 * u2) * v_j +
                       (u3 - u2) * h * m_j;
  return Clamp(Finite(value, v_i), low, high);
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

// `Instance192` right track, normalised to [-1, +1]. This knows nothing about
// metres: kOrbitTargetSwayRightM turns the value into metres. The asset keys
// are 0 / 0 / +1 / 0 / -1 / 0 at 0 / 15 / 22.5 / 30 / 37.5 / 45 s, wrapping to
// 0 at 60 s; the spline adds a small (~0.1) negative lobe near t = 10 s and a
// matching positive one near t = 50 s that fall out of the C1 tangents.
[[nodiscard]] inline double OrbitTargetSwayX(
    const double time_seconds) noexcept {
  return orbit_shot_detail::SamplePeriodicKeyedTrack(
      kOrbitTargetSwayXTimes, kOrbitTargetSwayXValues,
      kOrbitTargetSwayXKeyCount, kOrbitTargetSwayPeriodSeconds, time_seconds,
      -1.0, 1.0);
}

// `Instance188` up track, normalised to [-1, 0]: -1 / 0 / -1 / 0 at 0 / 15 /
// 30 / 45 s, wrapping back to -1 at 60 s. This one lands exactly on zero-slope
// smoothsteps, so it never needs the clamp.
[[nodiscard]] inline double OrbitTargetSwayY(
    const double time_seconds) noexcept {
  return orbit_shot_detail::SamplePeriodicKeyedTrack(
      kOrbitTargetSwayYTimes, kOrbitTargetSwayYValues,
      kOrbitTargetSwayYKeyCount, kOrbitTargetSwayPeriodSeconds, time_seconds,
      -1.0, 0.0);
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
                                                 kOrbitLookAtHeightM)),
                          Finite(config_.pivot_forward_m,
                                 kOrbitPivotForwardM));
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
    UpdateTargetSway(dt);

    // Orbit pose. `dir`/`tangent` live in the car body's forward-right plane,
    // so on a slope the whole circle follows the body pitch (guide Q5-B).
    const double look_at_height =
        (std::max)(0.0, Finite(config_.look_at_height_m,
                              kOrbitLookAtHeightM));
    const double pivot_forward =
        Finite(config_.pivot_forward_m, kOrbitPivotForwardM);
    const Vec3 pivot =
        Add(Add(SafeVec(car.position), Mul(basis.up, look_at_height)),
            Mul(basis.forward, pivot_forward));
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
    // slope. Any roll comes from the noise layer above, never from `up`.
    Vec3 look = Sub(pivot, eye);
    if (Length(look) < 1e-9) {
      look = basis.forward;
    }
    look = Normalize(look);

    // Handheld noise and the low-frequency sway change only the look-at point.
    // The eye, up and the roll basis stay untouched, so the orbit radius stays
    // exactly the same. Branches on zero keep both `strength == 0` and
    // `target_sway == 0` bit-for-bit identical to the shot without them.
    Vec3 target = pivot;
    if (state_.target_sway_x_m != 0.0 || state_.target_sway_y_m != 0.0 ||
        state_.shake_x_m != 0.0 || state_.shake_y_m != 0.0) {
      Vec3 right = Cross(Vec3{0.0, 0.0, 1.0}, look);
      if (Length(right) < 1e-9) {
        right = basis.right;
      }
      right = Normalize(right);
      const Vec3 up_local = Normalize(Cross(look, right));
      // Sway and noise are added on the same target point because the runtime
      // does exactly that: Instance279 (reaction) and Instance18 (noise) both
      // write pose+0x10. `0.0 + shake` stays exact when the sway is off.
      target = Add(pivot, Add(Mul(right, state_.target_sway_x_m +
                                            state_.shake_x_m),
                              Mul(up_local, state_.target_sway_y_m +
                                                state_.shake_y_m)));
      look = Normalize(Sub(target, eye));
    }

    CameraState view;
    view.position = eye;
    view.target = target;
    view.up = {0.0, 0.0, 1.0};
    view.fov_y_degrees =
        Clamp(Finite(config_.vertical_fov_degrees, kOrbitVerticalFovDegrees),
              1.0, 179.0);
    // Roll is the inherited ROLL noise controller (Instance16), not a keyframed
    // RotationZ and not a tilted up vector: pose+0x64 is a scalar the camera
    // applies around the view axis, so `up` stays world +Z here.
    view.roll_degrees = state_.roll_noise_degrees;
    view.roll = view.roll_degrees * kRadiansPerDegree;
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
    noise_x_oct2_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_oct2_, 0.0, 0.0, 0.0, kOrbitHandheldSeedX2};
    noise_y_oct2_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_oct2_, 0.0, 0.0, 0.0, kOrbitHandheldSeedY2};
    roll_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_, 0.0, 0.0, 0.0, kOrbitRollSeed};
    roll_oct2_ = orbit_shot_detail::ValueNoiseBand{
        noise_period_oct2_, 0.0, 0.0, 0.0, kOrbitRollSeed2};
    state_.shake_x_m = 0.0;
    state_.shake_y_m = 0.0;
    state_.roll_noise_degrees = 0.0;
    state_.target_sway_x_m = 0.0;
    state_.target_sway_y_m = 0.0;
    damped_sway_x_ = 0.0;
    damped_sway_y_ = 0.0;
    have_damped_sway_ = false;
  }

  void UpdateHandheldPeriod(const double frequency_hz) noexcept {
    const double frequency = Clamp(
        orbit_shot_detail::Finite(frequency_hz, kOrbitHandheldFrequencyHz),
        0.5, 30.0);
    const double period = 1.0 / frequency;
    if (period == noise_period_) {
      return;
    }
    const double period2 = 0.5 * period;
    const double scale = period / noise_period_;
    const double scale2 = period2 / noise_period_oct2_;
    noise_x_.t *= scale;
    noise_y_.t *= scale;
    noise_x_.period = period;
    noise_y_.period = period;
    noise_x_oct2_.t *= scale2;
    noise_y_oct2_.t *= scale2;
    noise_x_oct2_.period = period2;
    noise_y_oct2_.period = period2;
    // The roll bands share the frequency, so they have to be rescaled too or a
    // slider move would jump the roll phase.
    roll_.t *= scale;
    roll_oct2_.t *= scale2;
    roll_.period = period;
    roll_oct2_.period = period2;
    noise_period_ = period;
    noise_period_oct2_ = period2;
  }

  void UpdateHandheldNoise(const double dt) noexcept {
    UpdateHandheldPeriod(config_.handheld_frequency_hz);
    if (state_.mode == OrbitPhase::kIdle) {
      state_.shake_x_m = 0.0;
      state_.shake_y_m = 0.0;
      state_.roll_noise_degrees = 0.0;
      return;
    }
    const double strength = (std::max)(
        0.0, orbit_shot_detail::Finite(config_.handheld_strength, 1.0));
    const double roughness = Clamp(
        orbit_shot_detail::Finite(config_.handheld_roughness,
                                  kOrbitHandheldRoughness),
        0.0, 1.0);
    // Advance every band even when strength is zero; only the returned offset
    // is zero. Instance18 sums octave 1 + roughness * octave 2; the peak can
    // therefore exceed the magnitude by (1 + roughness).
    const double noise_x =
        orbit_shot_detail::ValueNoiseStep(noise_x_, dt) +
        roughness * orbit_shot_detail::ValueNoiseStep(noise_x_oct2_, dt);
    const double noise_y =
        orbit_shot_detail::ValueNoiseStep(noise_y_, dt) +
        roughness * orbit_shot_detail::ValueNoiseStep(noise_y_oct2_, dt);
    state_.shake_x_m = kOrbitHandheldHorizontalM * strength * noise_x;
    state_.shake_y_m = kOrbitHandheldVerticalM * strength * noise_y;
    // Roll is a separate controller (Instance16) with its own roughness 0.1 and
    // its own LCG streams; it shares only the strength and frequency sliders.
    // `strength == 0` multiplies to exactly zero, so the horizon is level.
    const double roll_noise =
        orbit_shot_detail::ValueNoiseStep(roll_, dt) +
        kOrbitRollNoiseRoughness *
            orbit_shot_detail::ValueNoiseStep(roll_oct2_, dt);
    state_.roll_noise_degrees =
        kOrbitRollNoiseDegrees * strength * roll_noise;
  }

  // Instance279 reaction: the two asset tracks scaled by the sway slider and
  // smoothed by Instance127's 1.0 s damper. The damped value is seeded with the
  // raw track on the first frame of a shot, so the layer does not crawl in from
  // zero during the 2 s entry. Zero strength publishes exact zeros, which is
  // what keeps the whole layer bit-for-bit inert when it is turned off.
  void UpdateTargetSway(const double dt) noexcept {
    const double strength = Clamp(
        orbit_shot_detail::Finite(config_.target_sway, 1.0), 0.0, 2.0);
    if (strength <= 0.0 || state_.mode == OrbitPhase::kIdle) {
      damped_sway_x_ = 0.0;
      damped_sway_y_ = 0.0;
      have_damped_sway_ = false;
      state_.target_sway_x_m = 0.0;
      state_.target_sway_y_m = 0.0;
      return;
    }
    const double raw_x =
        strength * kOrbitTargetSwayRightM * OrbitTargetSwayX(state_.time);
    const double raw_y =
        strength * kOrbitTargetSwayUpM * OrbitTargetSwayY(state_.time);
    if (!have_damped_sway_) {
      damped_sway_x_ = raw_x;
      damped_sway_y_ = raw_y;
      have_damped_sway_ = true;
    } else {
      const double alpha =
          1.0 - std::exp(-(std::max)(0.0, dt) /
                         kOrbitTargetSwayDampSeconds);
      damped_sway_x_ += (raw_x - damped_sway_x_) * alpha;
      damped_sway_y_ += (raw_y - damped_sway_y_) * alpha;
    }
    state_.target_sway_x_m = damped_sway_x_;
    state_.target_sway_y_m = damped_sway_y_;
  }

  [[nodiscard]] static double EntryPhase0(
      const CarState& car, const CarBasis& basis, const Vec3& current_camera,
      const double look_at_height_m, const double pivot_forward_m) noexcept {
    using namespace orbit_shot_detail;
    const Vec3 pivot =
        Add(Add(SafeVec(car.position), Mul(basis.up, look_at_height_m)),
            Mul(basis.forward, pivot_forward_m));
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
  double noise_period_oct2_{0.5 / kOrbitHandheldFrequencyHz};
  orbit_shot_detail::ValueNoiseBand noise_x_{};
  orbit_shot_detail::ValueNoiseBand noise_y_{};
  orbit_shot_detail::ValueNoiseBand noise_x_oct2_{};
  orbit_shot_detail::ValueNoiseBand noise_y_oct2_{};
  orbit_shot_detail::ValueNoiseBand roll_{};
  orbit_shot_detail::ValueNoiseBand roll_oct2_{};
  double damped_sway_x_{};
  double damped_sway_y_{};
  bool have_damped_sway_{};
};

}  // namespace nfs2015_chase
