#pragma once

// Pure burnout close-up camera model (NFS 2015 WheelSpin / Action Camera).
//
// This header is intentionally independent of the plugin's Game-tick and render
// plumbing. It consumes a `CarState` plus the already-filtered `BurnoutSample`
// and owns only the shot state machine, the wheel close-up pose, the shake and
// the orbit blend back to the chase camera. Phase 3 passes the result through
// the existing route B publish path; nothing here reads the game or writes a
// camera pose.
//
// NFS 2015 reference values (see PLAN-burnout-camera.md), tuned for NTE:
//
//   eye in car space  = (-2.8 m, side * 2.0 m, 0.55 m)
//   look target       = (-1.45 m, side * 1.0 m, 0.35 m)
//
// The target is the outboard face of the rear wheel, not a point inside the
// body: an inboard target makes the fender occlude the wheel during the orbit.
// NTE's car is wider than the NFS reference, so the eye is pulled outboard and
// the target is moved to the wheel face.
//   horizontal FOV    = 45 degrees
//   enter             = 0.5 s delay + 1.0 s orbit blend
//   exit              = 0.2 s delay + 2.5 s orbit blend
//
// UE axes: +X forward, +Y right, +Z up. The shot uses the car body frame when
// slope following is enabled, so an uphill/downhill car keeps the eye above the
// body instead of rotating the shot into the ground.

#include "car_sample.hpp"
#include "shot_blend.hpp"

#include <algorithm>
#include <cmath>

namespace nfs2015_chase {

// Pivot used by the chase -> shot orbit. It sits above the car origin so the
// eye arcs around the rear quarter instead of cutting through the boot.
inline constexpr double kBurnoutOrbitPivotHeightM = 0.8;

// The shot model never returns a degenerate look vector. A small floor also
// keeps the blend safe when a caller passes a very small `camera_scale`.
inline constexpr double kBurnoutMinTargetDistanceM = 0.5;
inline constexpr double kBurnoutMinEyeZAboveCarM = 0.25;

struct BurnoutShotConfig final {
  // NFS 2015 Cinebot transition timings.
  double entry_delay{0.5};
  double entry_time{1.0};
  double exit_delay{0.2};
  double exit_time{2.5};

  // Safety exit: an airborne or reversing car must not drag the close-up for
  // the full cinematic 2.5 s.
  double fast_exit_time{0.5};

  // Launch hand-off. NFS 2015's WheelSpin OutTransition is 0.2 s delay +
  // 2.5 s CINEBOT_BLEND_EULER_ORBIT (CinebotTransitionData Instance140).
  // NFS starts that timer when WheelSpin mode exits; our speed gate fires
  // after the car is already rolling, so the full 2.5 s reads as too slow.
  // Keep the NFS delay/orbit but use the measured-good 1.8 s pull-out.
  double launch_exit_time{1.8};

  // Horizontal FOV is authoritative; the vertical value is derived from the
  // current viewport aspect ratio. NFS stores 45 degrees horizontally.
  double horizontal_fov_degrees{45.0};

  // 0 = auto, 1 = forced left, 2 = forced right, 3 = centred.
  int side_mode{0};

  // Entry is stricter than hold, so a car that creeps during the burnout can
  // keep the shot, while a normal launch cannot trigger it.
  double entry_speed_mps{1.0};
  double hold_speed_mps{3.0};

  bool slope_follow{true};

  // The existing presenter camera-scale transform is applied around the model
  // origin. Keep a conservative local floor so a small scale cannot put the
  // close-up inside the car.
  double camera_scale{1.0};
  double min_eye_radius_m{1.3};

  // Car-local shot geometry, metres.
  double back_m{2.8};
  double outboard_m{2.0};
  double height_m{0.55};
  double target_x_m{-1.45};
  double target_y_m{1.0};
  double target_z_m{0.35};

  // Shot-only ground protection. `car.position` is treated as the ground/body
  // origin, which is the convention used by the existing chase model.
  double min_eye_z_above_car_m{kBurnoutMinEyeZAboveCarM};
  double min_target_distance_m{kBurnoutMinTargetDistanceM};

  // Shake starts only after the camera has moved in, matching the NFS
  // "zoom first, then shake" feel. The caps keep a runaway intensity from
  // making the view unusable.
  double shake_scale{1.0};
  double shake_start_blend{0.55};
  double shake_end_blend{1.0};
  double shake_position_m{0.010};
  double shake_rotation_degrees{0.20};
  double shake_max_position_m{0.03};
  double shake_max_rotation_degrees{0.5};
};

struct BurnoutShotState final {
  // Linear 0..1 progress. `blend` is the eased value used by the presenter.
  double phase{};
  double blend{};
  double hold_time{};
  double inactive_time{};
  bool latched{};
  bool force_exit{};
  // +1 right rear, -1 left rear, 0 centred.
  int side{1};
};

struct BurnoutShotResult final {
  CameraState view{};
  double phase{};
  double blend{};
  int side{1};
  bool latched{};
  bool force_exit{};
};

// Convert a horizontal lens to the vertical lens stored in CameraState.
[[nodiscard]] inline double BurnoutVerticalFovDegrees(
    const double horizontal_degrees, const double aspect_ratio) noexcept {
  const double horizontal =
      Clamp(std::isfinite(horizontal_degrees) ? horizontal_degrees : 45.0,
            1.0, 179.0);
  const double aspect =
      std::isfinite(aspect_ratio) && aspect_ratio > 0.0
          ? Clamp(aspect_ratio, 0.1, 10.0)
          : 16.0 / 9.0;
  return 2.0 * std::atan(
                   std::tan(horizontal * kRadiansPerDegree * 0.5) / aspect) *
         kDegreesPerRadian;
}

// The inverse is useful to diagnostics/tests; the model itself always stores
// the vertical form.
[[nodiscard]] inline double BurnoutHorizontalFovDegrees(
    const double vertical_degrees, const double aspect_ratio) noexcept {
  const double vertical =
      Clamp(std::isfinite(vertical_degrees) ? vertical_degrees : 26.25, 1.0,
            179.0);
  const double aspect =
      std::isfinite(aspect_ratio) && aspect_ratio > 0.0
          ? Clamp(aspect_ratio, 0.1, 10.0)
          : 16.0 / 9.0;
  return 2.0 * std::atan(
                   std::tan(vertical * kRadiansPerDegree * 0.5) * aspect) *
         kDegreesPerRadian;
}

// The shot-pose helpers now live in shot_blend.hpp so the parked orbit can
// share them. The using-declarations keep every existing call site (and the
// burnout tests) working unchanged.
namespace burnout_shot_detail {

using shot_blend_detail::Finite;
using shot_blend_detail::OrthonormalUp;
using shot_blend_detail::SafeVec;

}  // namespace burnout_shot_detail

// Pure state machine plus shot pose. The class owns no clock and no locks:
// `Update` receives dt from the render clock and returns both the shot view and
// the blend the presenter should use.
class BurnoutShot final {
 public:
  explicit BurnoutShot(BurnoutShotConfig config = {}) noexcept
      : config_(config) {}

  [[nodiscard]] const BurnoutShotConfig& config() const noexcept {
    return config_;
  }

  // Changing settings does not reset an in-flight shot: the presenter can apply
  // a new FOV or shake scale without a hard cut. Call Reset() on epoch/world
  // changes.
  void SetConfig(const BurnoutShotConfig& config) noexcept { config_ = config; }

  void Reset() noexcept {
    state_ = {};
    time_ = 0.0;
  }

  [[nodiscard]] const BurnoutShotState& state() const noexcept { return state_; }

  [[nodiscard]] BurnoutShotResult Update(
      double dt, const CarState& car, const BurnoutSample& sample,
      const double aspect_ratio = 16.0 / 9.0) noexcept {
    dt = Clamp(burnout_shot_detail::Finite(dt, 0.0), 0.0, 0.1);
    time_ += dt;

    const double speed =
        burnout_shot_detail::Finite(sample.speed_mps,
                                    burnout_shot_detail::Finite(car.speed, 0.0));
    const double absolute_speed = std::fabs(speed);
    const bool on_ground = car.on_ground;
    const bool direction_ok = speed > -0.5;
    const bool speed_exit =
        absolute_speed >= (std::max)(0.0, config_.hold_speed_mps);
    const bool unsafe_exit = !on_ground || !direction_ok;
    const bool force_exit = unsafe_exit || speed_exit;

    const bool can_hold =
        sample.active && on_ground && direction_ok &&
        absolute_speed < (std::max)(0.0, config_.hold_speed_mps);
    const bool can_enter =
        can_hold && absolute_speed <
                       (std::max)(0.0, config_.entry_speed_mps);
    const bool holding =
        !force_exit && can_hold && (state_.latched || can_enter);

    if (holding) {
      state_.inactive_time = 0.0;
      state_.force_exit = false;
      state_.hold_time += dt;
      const double entry_delay = (std::max)(0.0, config_.entry_delay);
      if (!state_.latched && state_.hold_time + 1e-12 >= entry_delay) {
        state_.latched = true;
        state_.side = SelectSide(sample);
      }
      if (state_.latched) {
        SetPhaseToward(1.0, dt, config_.entry_time);
      }
    } else {
      state_.hold_time = 0.0;
      if (state_.phase > 0.0) {
        state_.force_exit = force_exit;
        state_.inactive_time += dt;
        const double exit_delay =
            unsafe_exit ? 0.0 : (std::max)(0.0, config_.exit_delay);
        if (state_.inactive_time + 1e-12 >= exit_delay) {
          // A launch keeps the NFS OutTransition delay and Euler orbit, but
          // uses the 1.8 s pull-out that matches the later speed-gate trigger.
          // Airborne/reverse still use the short safety exit.
          const double duration =
              unsafe_exit ? config_.fast_exit_time
                          : (speed_exit ? config_.launch_exit_time
                                        : config_.exit_time);
          SetPhaseToward(0.0, dt, duration);
        }
      } else {
        state_.phase = 0.0;
        state_.latched = false;
        state_.inactive_time = 0.0;
        state_.force_exit = force_exit;
      }
    }

    if (state_.phase <= 0.0) {
      state_.phase = 0.0;
      state_.latched = false;
    }
    state_.blend = SmoothStep(0.0, 1.0, state_.phase);

    BurnoutShotResult result;
    result.view = BuildShotView(car, sample, aspect_ratio);
    result.phase = state_.phase;
    result.blend = state_.blend;
    result.side = state_.side;
    result.latched = state_.latched;
    result.force_exit = state_.force_exit;
    return result;
  }

 private:
  [[nodiscard]] int SelectSide(const BurnoutSample& sample) const noexcept {
    if (config_.side_mode == 1) {
      return -1;
    }
    if (config_.side_mode == 2) {
      return 1;
    }
    if (config_.side_mode == 3) {
      return 0;
    }
    // Per-wheel flags are stronger evidence than steering. On this car both
    // rear wheels can be true at once, so fall through to steering then the
    // documented right-side default.
    if (sample.left != sample.right) {
      return sample.left ? -1 : 1;
    }
    const double steering =
        burnout_shot_detail::Finite(sample.steering, 0.0);
    if (steering < -0.05) {
      return -1;
    }
    if (steering > 0.05) {
      return 1;
    }
    return 1;
  }

  void SetPhaseToward(const double target, const double dt,
                      const double duration) noexcept {
    if (!(duration > 1e-9)) {
      state_.phase = target;
      return;
    }
    const double step = dt / duration;
    if (target > state_.phase) {
      state_.phase = (std::min)(target, state_.phase + step);
    } else {
      state_.phase = (std::max)(target, state_.phase - step);
    }
  }

  [[nodiscard]] CameraState BuildShotView(
      const CarState& car, const BurnoutSample& sample,
      const double aspect_ratio) const noexcept {
    using namespace burnout_shot_detail;

    const Vec3 car_position = SafeVec(car.position);
    const double yaw = Finite(car.yaw, 0.0);
    const double raw_pitch = Finite(car.pitch, 0.0);
    constexpr double kPitchLimit = 89.0 * kRadiansPerDegree;
    const double pitch =
        config_.slope_follow ? Clamp(raw_pitch, -kPitchLimit, kPitchLimit)
                             : 0.0;
    const CarBasis basis = MakeCarBasis(yaw, pitch);

    const double scale =
        Clamp(Finite(config_.camera_scale, 1.0), 0.1, 4.0);
    const double min_radius = (std::max)(0.0, config_.min_eye_radius_m);
    const double back =
        (std::max)(min_radius, (std::max)(0.0, config_.back_m) * scale);
    const double outboard =
        (std::max)(min_radius, (std::max)(0.0, config_.outboard_m) * scale);
    const double height = (std::max)(0.1, config_.height_m * scale);
    const int side = state_.side < 0 ? -1 : (state_.side > 0 ? 1 : 0);

    const Vec3 eye_local{-back, side * outboard, height};
    const Vec3 target_local{config_.target_x_m, side * config_.target_y_m,
                            config_.target_z_m};
    Vec3 eye = Add(car_position, CarLocalToWorld(eye_local, basis));
    const Vec3 look_target =
        Add(car_position, CarLocalToWorld(target_local, basis));

    const double min_eye_z =
        car_position.z + (std::max)(0.0, config_.min_eye_z_above_car_m);
    eye.z = (std::max)(eye.z, min_eye_z);

    const double min_target_distance =
        (std::max)(kBurnoutMinTargetDistanceM, config_.min_target_distance_m);
    Vec3 forward = Sub(look_target, eye);
    if (Length(forward) < min_target_distance) {
      forward = basis.forward;
    }
    forward = Normalize(forward);

    Vec3 up = OrthonormalUp(basis.up, forward, basis.up);

    // Position shake lives on the eye; the look target stays put so the wheel
    // remains in frame. Recompute the view direction after the eye moves, then
    // apply the much smaller angular shake.
    const double intensity =
        Clamp(Finite(sample.intensity, 0.0), 0.0, 1.0);
    const double shake_gate =
        SmoothStep(config_.shake_start_blend, config_.shake_end_blend,
                   state_.blend);
    const double shake_scale =
        Clamp(Finite(config_.shake_scale, 1.0), 0.0, 2.0);
    const double position_amplitude =
        (std::min)((std::max)(0.0, config_.shake_max_position_m),
                   config_.shake_position_m * intensity * shake_gate *
                       shake_scale);
    const double rotation_amplitude =
        (std::min)((std::max)(0.0, config_.shake_max_rotation_degrees),
                   config_.shake_rotation_degrees * intensity * shake_gate *
                       shake_scale) *
        kRadiansPerDegree;

    if (position_amplitude > 0.0) {
      const Vec3 shake{Noise1(time_ * 94.2, 1.7), Noise1(time_ * 125.7, 4.2),
                       Noise1(time_ * 75.4, 8.9)};
      eye = Add(eye, Mul(shake, position_amplitude));
      // The floor is a hard safety constraint, not a soft target.
      eye.z = (std::max)(eye.z, min_eye_z);

      forward = Sub(look_target, eye);
      if (Length(forward) < min_target_distance) {
        forward = basis.forward;
      }
      forward = Normalize(forward);
      up = OrthonormalUp(up, forward, basis.up);
    }

    if (rotation_amplitude > 0.0) {
      Vec3 right = Cross(up, forward);
      if (Length(right) < 1e-9) {
        right = basis.right;
      }
      right = Normalize(right);
      forward = RotateAxis(forward, up,
                           Noise1(time_ * 94.2 + 3.1, 2.3) *
                               rotation_amplitude);
      forward = RotateAxis(forward, right,
                           Noise1(time_ * 125.7 + 1.7, 5.1) *
                               rotation_amplitude);
      forward = Normalize(forward);
      up = OrthonormalUp(up, forward, basis.up);
    }

    CameraState view;
    view.position = eye;
    view.target = Add(eye, forward);
    view.up = up;
    view.fov_y_degrees = BurnoutVerticalFovDegrees(
        config_.horizontal_fov_degrees, aspect_ratio);
    // NFS WheelSpin has Roll 0.0 and no DUTCH controller. The low-angle tilt
    // is pitch, and the orbit blend recovers it; there is no separate roll
    // spring to emulate here.
    view.roll = 0.0;
    view.pitch_degrees =
        std::asin(Clamp(forward.z, -1.0, 1.0)) * kDegreesPerRadian;
    view.yaw_degrees =
        std::atan2(forward.y, forward.x) * kDegreesPerRadian;
    view.roll_degrees = 0.0;
    return view;
  }

  BurnoutShotConfig config_{};
  BurnoutShotState state_{};
  double time_{};
};

// Blend the chase view and the burnout shot around the car-local pivot. The
// EULER_ORBIT implementation is shared with the parked orbit (shot_blend.hpp);
// this wrapper only supplies the burnout pivot height so the existing call
// sites and the burnout acceptance tests keep their exact behaviour.
//
// `blend == 0` returns `chase` bit-for-bit (no normalisation), and `blend == 1`
// returns `shot` bit-for-bit; only the interior takes the orbit path.
[[nodiscard]] inline CameraState BlendBurnoutCamera(
    const CameraState& chase, const CameraState& shot, const CarState& car,
    const double blend, const double pivot_height_m =
                             kBurnoutOrbitPivotHeightM) noexcept {
  return BlendShotAroundCarPivot(chase, shot, car, blend, pivot_height_m);
}

}  // namespace nfs2015_chase
