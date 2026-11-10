#include "wheel_steer.hpp"

#include <cmath>
#include <cstdio>

namespace {

using nfs2015_chase::AxisAngle;
using nfs2015_chase::ComputeSteeredRelativeRotation;
using nfs2015_chase::ComputeVisualWheelRelativeRotation;
using nfs2015_chase::ComputeWheelSteerAngleFromWorld;
using nfs2015_chase::ComputeWheelSteerTravel;
using nfs2015_chase::AdvanceWheelSteerTarget;
using nfs2015_chase::Cross;
using nfs2015_chase::Dot;
using nfs2015_chase::kDegreesPerRadian;
using nfs2015_chase::kRadiansPerDegree;
using nfs2015_chase::Quat;
using nfs2015_chase::QuatConjugate;
using nfs2015_chase::QuatFromRotatorDegrees;
using nfs2015_chase::QuatMul;
using nfs2015_chase::QuatNormalize;
using nfs2015_chase::QuatRotate;
using nfs2015_chase::RemoveWheelSpinTwist;
using nfs2015_chase::RotatorDegreesFromQuat;
using nfs2015_chase::Vec3;
using nfs2015_chase::WheelSteerTarget;
using nfs2015_chase::WheelSteerConfig;
using nfs2015_chase::WheelSteerDriftState;
using nfs2015_chase::WheelSteerPhase;

int failures = 0;

void Check(const bool condition, const char *message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

double WrappedDegrees(const double radians) {
  return radians * kDegreesPerRadian;
}

WheelSteerTarget StepTravel(WheelSteerDriftState& state,
                          const WheelSteerConfig& config,
                          const double front_slip_degrees,
                          const double rear_slip_degrees,
                          const double speed_mps,
                          const double forward_speed_mps,
                          const double steering = 0.0,
                          const bool steering_known = true,
                          const double dt = 1.0 / 60.0) {
  return AdvanceWheelSteerTarget(
      state, front_slip_degrees * kRadiansPerDegree,
      rear_slip_degrees * kRadiansPerDegree, speed_mps, forward_speed_mps,
      steering, steering_known, false, dt, config);
}

WheelSteerTarget Step(WheelSteerDriftState& state,
                      const WheelSteerConfig& config,
                      const double slip_degrees, const double speed_mps,
                      const double steering = 0.0,
                      const bool steering_known = true,
                      const double dt = 1.0 / 60.0) {
  const double slip = slip_degrees * kRadiansPerDegree;
  return StepTravel(state, config, slip_degrees, slip_degrees, speed_mps,
                    speed_mps * std::cos(slip), steering, steering_known, dt);
}

}  // namespace

int main() {
  WheelSteerConfig config;
  config.max_angle_radians = 40.0 * kRadiansPerDegree;
  config.gain = 1.0;
  config.enter_slip_radians = 6.5 * kRadiansPerDegree;
  config.exit_slip_radians = 3.0 * kRadiansPerDegree;
  config.full_slip_radians = 40.0 * kRadiansPerDegree;
  config.rate_radians_per_second = 120.0 * kRadiansPerDegree;
  config.entry_speed_mps = 40.0 / 3.6;
  config.hold_seconds = 0.5;
  config.exit_hold_seconds = 0.25;
  config.blend_in_seconds = 0.20;
  config.blend_out_seconds = 0.30;
  config.stock_max_angle_radians = 35.0 * kRadiansPerDegree;
  config.counter_authority_radians = 5.0 * kRadiansPerDegree;
  config.turn_in_seconds = 0.18;
  config.turn_in_min_seconds = 0.10;
  config.turn_in_max_seconds = 0.30;
  config.turn_in_release_slip_radians = 18.0 * kRadiansPerDegree;
  config.reentry_cooldown_seconds = 0.5;
  config.standstill_speed_mps = 0.5;
  config.standstill_grace_seconds = 0.5;

  // NFS-style low-speed gate: a 30 degree body slip at 5 m/s is a pivot or a
  // normal low-speed turn, not a drift. It must leave the stock wheel alone.
  {
    WheelSteerDriftState state;
    const auto target = Step(state, config, 30.0, 5.0);
    Check(!target.active, "low-speed slip must not engage");
    Check(!target.engaged, "low-speed slip must not enter drift");
    Check(target.phase == WheelSteerPhase::grip,
          "low-speed slip must stay in grip");
  }

  // The NFS entry threshold is 6.5 degrees, with 3 degrees to exit. A target
  // below the countersteer authority stays on the stock animation; otherwise a
  // normal high-speed turn would park the wheel without producing a visible
  // countersteer.
  {
    WheelSteerDriftState state;
    const auto below = Step(state, config, 6.0, 20.0);
    Check(!below.active, "6.0 degree slip is below the entry threshold");
    const auto entering = Step(state, config, 20.0, 20.0);
    Check(entering.engaged, "20 degree slip should enter the drift state");
    Check(entering.phase == WheelSteerPhase::entering,
          "neutral steering should start in the entering phase");
    Check(entering.active,
          "a meaningful countersteer target should take over the wheel");
  }

  // The entry blend starts at the stock angle, not at zero.
  {
    WheelSteerDriftState state;
    const auto target = Step(state, config, 20.0, 20.0, 0.5);
    Check(target.engaged, "steering handoff must have entered the drift");
    Check(std::fabs(WrappedDegrees(target.stock_angle_radians) - 17.5) <
              0.01,
          "positive steering should produce a positive stock angle");
    Check(std::fabs(target.angle_radians - target.stock_angle_radians) <
              2.0 * kRadiansPerDegree,
          "the first entry target must stay close to the stock angle");
  }

  // A zero countersteer target must preserve the stock steering, not center
  // the wheel while the driver is still holding a normal turn.
  {
    WheelSteerDriftState state;
    WheelSteerTarget target{};
    for (int tick = 0; tick < 20; ++tick) {
      target = Step(state, config, 7.0, 20.0, 0.5, true, 0.05);
    }
    Check(!target.active,
          "zero countersteer must not take over the stock animation");
    Check(target.phase == WheelSteerPhase::grip,
          "zero countersteer must not enter the drift state");
    Check(std::fabs(target.angle_radians - target.stock_angle_radians) <
              1.0 * kRadiansPerDegree,
          "zero countersteer must preserve the stock angle");
  }

  // A small countersteer target is not enough to park the stock animation.
  {
    WheelSteerDriftState state;
    WheelSteerTarget target{};
    for (int tick = 0; tick < 20; ++tick) {
      target = Step(state, config, 12.0, 20.0, 0.5, true, 0.05);
    }
    Check(!target.active,
          "small countersteer target must not take over the stock animation");
    Check(std::fabs(target.angle_radians - target.stock_angle_radians) <
              5.0 * kRadiansPerDegree,
          "small countersteer target must stay close to the stock angle");
  }

  // A high slip angle ramps the countersteer target to the visual clamp.
  {
    WheelSteerDriftState state;
    WheelSteerTarget target{};
    for (int tick = 0; tick < 12; ++tick) {
      target = Step(state, config, 60.0, 20.0, 0.0, true, 0.05);
    }
    Check(target.active, "60 degree slip should engage");
    Check(std::fabs(WrappedDegrees(target.angle_radians) - 40.0) < 0.05,
          "60 degree slip should clamp to 40 degrees");
    Check(std::fabs(target.blend - 1.0) < 1e-9,
          "60 degree slip should reach full blend");
  }

  // 25 degrees is past entry but below the 40 degree full-countersteer angle;
  // the target should be partial, not instantly 25 or 40 degrees.
  {
    WheelSteerDriftState state;
    WheelSteerTarget target{};
    for (int tick = 0; tick < 12; ++tick) {
      target = Step(state, config, 25.0, 20.0, 0.0, true, 0.05);
    }
    const double degrees = WrappedDegrees(target.angle_radians);
    Check(degrees > 10.0 && degrees < 18.0,
          "25 degree slip should produce a partial countersteer target");
  }

  // With the wheel pointing into the slide, hold the turn-in angle first and
  // only then blend toward countersteer. This is the high-speed U-turn feel.
  {
    WheelSteerDriftState state;
    const auto first =
        Step(state, config, 20.0, 20.0, -0.5, true, 0.05);
    Check(first.phase == WheelSteerPhase::turn_in,
          "steering into the slide should start turn_in");
    Check(first.active, "turn_in must own the wheel immediately");
    Check(std::fabs(WrappedDegrees(first.angle_radians) + 17.5) < 0.01,
          "turn_in must hold the inside steering angle, not countersteer");

    WheelSteerTarget later = first;
    for (int tick = 0; tick < 5; ++tick) {
      later = Step(state, config, 20.0, 20.0, -0.5, true, 0.05);
    }
    Check(later.phase == WheelSteerPhase::entering ||
              later.phase == WheelSteerPhase::drifting,
          "turn_in should hand over to the countersteer blend");
    Check(WrappedDegrees(later.angle_radians) >
              WrappedDegrees(first.angle_radians),
          "the wheel must move toward countersteer after turn_in");

  }

  // A sharp entry can already have a large rear slip. Still show a short
  // inside-steering phase; do not skip turn_in entirely on the first tick.
  {
    WheelSteerDriftState state;
    const auto first =
        Step(state, config, 30.0, 20.0, -0.5, true, 0.05);
    Check(first.phase == WheelSteerPhase::turn_in,
          "high-slip entry must still start turn_in");
    Step(state, config, 30.0, 20.0, -0.5, true, 0.05);
    const auto after_min =
        Step(state, config, 30.0, 20.0, -0.5, true, 0.05);
    Check(after_min.phase == WheelSteerPhase::entering ||
              after_min.phase == WheelSteerPhase::drifting,
          "high-slip turn_in should hand over after its minimum hold");
  }

  // If the driver is already countersteering, skip turn_in.
  {
    WheelSteerDriftState state;
    const auto target =
        Step(state, config, 20.0, 20.0, 0.5, true, 0.05);
    Check(target.phase == WheelSteerPhase::entering,
          "already-countersteering input should skip turn_in");
  }

  // The drift cannot exit during the 0.5 second lock even if the slip drops.
  {
    WheelSteerDriftState state;
    Step(state, config, 20.0, 20.0, 0.0, true, 0.05);
    const auto early = Step(state, config, 2.0, 20.0, 0.0, true, 0.1);
    Check(early.engaged, "drift must hold through the lock window");
    Check(early.phase == WheelSteerPhase::entering ||
              early.phase == WheelSteerPhase::drifting,
          "lock window should stay in a drift phase");
    const auto locked = Step(state, config, 2.0, 20.0, 0.0, true, 0.25);
    Check(locked.engaged, "drift must still be locked at 0.4 seconds");
    const auto later = Step(state, config, 2.0, 20.0, 0.0, true, 0.25);
    Check(later.phase == WheelSteerPhase::releasing,
          "slip below 3 degrees should release after the lock");
    Check(later.active, "release still owns the wheel for the handoff");
  }

  // Between the exit and entry thresholds the drift stays engaged.
  {
    WheelSteerDriftState state;
    Step(state, config, 20.0, 20.0, 0.0, true, 0.6);
    const auto between = Step(state, config, 4.0, 20.0, 0.0, true, 0.3);
    Check(between.engaged,
          "4 degree slip is above the exit threshold and must stay engaged");
  }

  // The release phase must keep the last countersteer target while the blend
  // returns to stock. Recomputing from the zeroed travel vector would first
  // point the wheel straight and then back to stock.
  {
    WheelSteerDriftState state;
    WheelSteerTarget engaged{};
    for (int tick = 0; tick < 12; ++tick) {
      engaged = Step(state, config, 25.0, 20.0, 0.0, true, 0.05);
    }
    const auto release = Step(state, config, 2.0, 20.0, 0.0, true, 0.25);
    Check(release.phase == WheelSteerPhase::releasing,
          "a collapsed slip angle should release after the lock");
    Check(std::fabs(release.angle_radians - engaged.angle_radians) <
              1e-9,
          "release must start from the last countersteer target");
    Check(std::fabs(WrappedDegrees(release.angle_radians)) > 10.0,
          "release must not zero the countersteer in one tick");
  }

  // Speed is not an exit condition. A low-speed drift with the rear still
  // sliding must keep countersteering, even if entry speed would reject it.
  {
    WheelSteerDriftState state;
    for (int tick = 0; tick < 12; ++tick) {
      Step(state, config, 20.0, 20.0, 0.0, true, 0.05);
    }
    const auto low_speed = Step(state, config, 20.0, 2.0, 0.0, true, 0.05);
    Check(low_speed.engaged, "low speed must not release an engaged drift");
    Check(low_speed.phase == WheelSteerPhase::drifting,
          "low speed drift should stay in the drifting phase");
    Check(low_speed.active, "low speed drift should keep the wheel override");
    Check(std::fabs(WrappedDegrees(low_speed.angle_radians)) > 5.0,
          "low speed drift should keep the countersteer angle");
  }

  // A large drift angle can make the forward component zero while the car is
  // still moving sideways. That must not release the drift.
  {
    WheelSteerDriftState state;
    for (int tick = 0; tick < 12; ++tick) {
      Step(state, config, 20.0, 20.0, 0.0, true, 0.05);
    }
    const auto near_zero_forward =
        StepTravel(state, config, 30.0, 30.0, 20.0, 0.0, 0.0, true, 0.05);
    Check(near_zero_forward.engaged,
          "zero forward speed during a sideways slide must stay engaged");
    Check(near_zero_forward.phase == WheelSteerPhase::drifting,
          "near-zero forward drift should stay in drifting");
    Check(std::fabs(WrappedDegrees(near_zero_forward.angle_radians)) > 5.0,
          "near-zero forward drift must keep countersteering");
  }

  // An entry that immediately collapses must keep the entry counter target
  // for the release blend instead of zeroing it on the below-exit tick.
  {
    WheelSteerDriftState state;
    const auto first = Step(state, config, 28.0, 20.0, 0.0, true, 0.05);
    Check(first.engaged, "28 degree entry should engage");
    const double entry_counter = first.counter_angle_radians;
    const auto collapsed = Step(state, config, 0.2, 20.0, 0.0, true, 0.05);
    Check(collapsed.phase == WheelSteerPhase::entering ||
              collapsed.phase == WheelSteerPhase::drifting,
          "entry collapse should stay in a drift phase before release");
    Check(std::fabs(collapsed.counter_angle_radians - entry_counter) < 1e-9,
          "entry collapse must preserve the last counter target");
  }

  // A >90 degree slide can make the signed forward speed negative. That is
  // not reverse gear and must not release an already engaged drift.
  {
    WheelSteerDriftState state;
    for (int tick = 0; tick < 12; ++tick) {
      Step(state, config, 30.0, 20.0, 0.0, true, 0.05);
    }
    const auto spun = AdvanceWheelSteerTarget(
        state, 30.0 * kRadiansPerDegree, 30.0 * kRadiansPerDegree, 20.0,
        -8.0, 0.0, true, true, 0.05, config);
    Check(spun.engaged,
          "negative forward speed during a big slide must stay engaged");
    Check(std::fabs(WrappedDegrees(spun.angle_radians)) > 5.0,
          "big slide must keep countersteering even if forward speed is negative");
  }

  // Only a truly degenerate velocity vector uses the standstill grace. It
  // holds the last target briefly and then releases; it is not a speed floor.
  {
    WheelSteerDriftState state;
    for (int tick = 0; tick < 12; ++tick) {
      Step(state, config, 20.0, 20.0, 0.0, true, 0.05);
    }
    const auto held = Step(state, config, 20.0, 0.3, 0.0, true, 0.1);
    Check(held.engaged, "standstill grace should hold the drift briefly");
    Step(state, config, 20.0, 0.3, 0.0, true, 0.25);
    const auto released =
        Step(state, config, 20.0, 0.3, 0.0, true, 0.25);
    Check(released.phase == WheelSteerPhase::releasing,
          "standstill grace should release after 0.5 seconds");
  }

  // Once the release blend is zero, the pure target no longer requests the
  // override; the plugin keeps writing until applied reaches the stock angle.
  {
    WheelSteerDriftState state;
    Step(state, config, 20.0, 20.0, 0.0, true, 0.6);
    Step(state, config, 2.0, 20.0, 0.0, true, 0.5);
    WheelSteerTarget finished{};
    for (int tick = 0; tick < 3; ++tick) {
      finished = Step(state, config, 2.0, 20.0, 0.0, true, 0.25);
    }
    Check(!finished.active, "finished release should return to grip");
    Check(finished.phase == WheelSteerPhase::grip,
          "finished release should be in grip");
  }

  // Reverse and unknown steering must not produce a positive-angle handoff.
  {
    WheelSteerDriftState state;
    const auto reverse = AdvanceWheelSteerTarget(
        state, 20.0 * kRadiansPerDegree, 20.0 * kRadiansPerDegree, 20.0,
        -5.0, 0.5, true, true, 0.05, config);
    Check(!reverse.active, "reverse must not engage");
    WheelSteerDriftState unknown_state;
    const auto unknown = Step(unknown_state, config, 7.0, 20.0, 0.5, false);
    Check(unknown.stock_angle_radians == 0.0,
          "unknown steering must use a zero stock estimate");
  }

  // The production defaults follow the NFS 2015 steering scale: a 40 degree
  // drift-angle reference and a conservative 32 degree visual countersteer.
  {
    const WheelSteerConfig defaults;
    Check(std::fabs(WrappedDegrees(defaults.max_angle_radians) - 32.0) <
              0.01,
          "default visual countersteer should be 32 degrees");
    Check(std::fabs(WrappedDegrees(defaults.full_slip_radians) - 40.0) <
              0.01,
          "default full countersteer slip should be 40 degrees");
    Check(std::fabs(defaults.entry_speed_mps - 40.0 / 3.6) < 1e-9,
          "default entry speed should be 40 km/h");
  }

  // Front-axle travel differs from body-centre travel by omega x r. With a
  // positive yaw rate, the front axle has a positive lateral component and the
  // rear axle the opposite; the countersteer target must use the front angle.
  {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    const Vec3 velocity{20.0, 0.0, 0.0};
    const auto travel = ComputeWheelSteerTravel(
        forward, up, velocity, 0.5, {1.3, 0.0, 0.0}, {-1.3, 0.0, 0.0});
    Check(travel.front_valid && travel.rear_valid,
          "front and rear axle travel should be valid");
    Check(std::fabs(WrappedDegrees(travel.front_angle_radians) - 1.86) <
              0.1,
          "front axle travel should lead the body centre");
    Check(std::fabs(WrappedDegrees(travel.rear_angle_radians) + 1.86) <
              0.1,
          "rear axle travel should lag the body centre");

    WheelSteerDriftState center_state;
    WheelSteerDriftState front_state;
    const auto center = StepTravel(center_state, config, 20.0, 20.0, 20.0,
                                   19.0, 0.0, true, 0.05);
    const auto front = StepTravel(front_state, config, 10.0, 20.0, 20.0,
                                  19.0, 0.0, true, 0.05);
    Check(std::fabs(front.counter_angle_radians) <
              std::fabs(center.counter_angle_radians),
          "countersteer target must use the front-axle travel angle");
  }

  // The final applied-angle clamp is still a pure rate limit.
  {
    const double current = -20.0 * kRadiansPerDegree;
    const double target = 20.0 * kRadiansPerDegree;
    const double step = nfs2015_chase::MoveTowards(
        current, target, 120.0 * kRadiansPerDegree / 60.0);
    Check(std::fabs(WrappedDegrees(step - current) - 2.0) < 1e-9,
          "rate limit should cap the per-tick angle change");
  }

  // Stock-angle measurement is used before parking the animation. The axle
  // is local Y, and a mirrored wheel's 180-degree socket must still read as
  // zero stock steer.
  {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    double angle = 0.0;
    const Quat right =
        AxisAngle(up, 10.0 * kRadiansPerDegree);
    Check(ComputeWheelSteerAngleFromWorld(forward, up, right, angle),
          "right stock wheel angle should be measurable");
    Check(std::fabs(WrappedDegrees(angle) - 10.0) < 0.01,
          "right stock wheel angle should be positive");
    const Quat left =
        AxisAngle(up, -10.0 * kRadiansPerDegree);
    Check(ComputeWheelSteerAngleFromWorld(forward, up, left, angle),
          "left stock wheel angle should be measurable");
    Check(std::fabs(WrappedDegrees(angle) + 10.0) < 0.01,
          "left stock wheel angle should be negative");
    const Quat mirrored =
        AxisAngle(up, 180.0 * kRadiansPerDegree);
    Check(ComputeWheelSteerAngleFromWorld(forward, up, mirrored, angle),
          "mirrored stock wheel angle should be measurable");
    Check(std::fabs(WrappedDegrees(angle)) < 0.01,
          "mirrored wheel should read zero stock steer");
  }

  // Feedback loop: UE applies the relative quaternion directly:
  // wheel_world = socket * relative. This checks the fixed point after ticks.
  {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    const double socket_rotator[3] = {-2.2655664633830535,
                                      -21.760354335697016,
                                      -11.747082664022344};
    const Quat socket =
        QuatNormalize(QuatFromRotatorDegrees(socket_rotator));
    const double target_angle = 25.0 * kRadiansPerDegree;
    const Vec3 target_forward =
        QuatRotate(AxisAngle(up, target_angle), forward);
    Quat last{};
    Quat wheel_world = socket;
    for (int tick = 0; tick < 6; ++tick) {
      Quat relative_new{};
      const bool ok = ComputeSteeredRelativeRotation(
          forward, up, target_forward, wheel_world, last, relative_new);
      Check(ok, "steered relative rotation should be finite");
      if (!ok) {
        break;
      }
      // UE applies the relative quaternion directly.
      wheel_world = QuatNormalize(QuatMul(socket, relative_new));
      last = relative_new;
      const Vec3 axle = QuatRotate(wheel_world, {0.0, 1.0, 0.0});
      const Vec3 wheel_forward = nfs2015_chase::Normalize(Cross(axle, up));
      const double psi = std::atan2(Dot(up, Cross(forward, wheel_forward)),
                                    Dot(forward, wheel_forward));
      Check(std::fabs(WrappedDegrees(psi) - 25.0) < 0.05,
            "feedback loop should hold the 25 degree visual angle");
    }
  }

  // Parked-wheel-animation composition: steer and roll are written together.
  // The real fix is not another feedback loop; it is that the rolling spin must
  // be removed from the game's wheel controller while the visual angle is
  // authored, otherwise any relative yaw precesses with the roll.
  {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    const double socket_rotator[3] = {-2.2655664633830535,
                                      -21.760354335697016,
                                      -11.747082664022344};
    const Quat socket =
        QuatNormalize(QuatFromRotatorDegrees(socket_rotator));
    const double target_angle = 25.0 * kRadiansPerDegree;
    const Vec3 target_forward =
        QuatRotate(AxisAngle(up, target_angle), forward);
    for (const double roll_degrees : {0.0, 45.0, 90.0, 180.0, 270.0}) {
      const double roll = roll_degrees * kRadiansPerDegree;
      Quat relative{};
      const bool ok = ComputeVisualWheelRelativeRotation(
          forward, up, target_forward, socket, roll, relative);
      Check(ok, "visual wheel relative rotation should be finite");
      if (!ok) {
        continue;
      }
      // UE applies the relative quaternion directly.
      const Quat wheel_world =
          QuatNormalize(QuatMul(socket, relative));
      const Vec3 axle = QuatRotate(wheel_world, {0.0, 1.0, 0.0});
      const Vec3 wheel_forward = nfs2015_chase::Normalize(Cross(axle, up));
      const double psi = std::atan2(Dot(up, Cross(forward, wheel_forward)),
                                    Dot(forward, wheel_forward));
      Check(std::fabs(WrappedDegrees(psi) - 25.0) < 0.05,
            "parked wheel animation should hold the steer angle while rolling");
      // The plugin writes an FRotator, so the round trip through the game's
      // FRotator->Quat conversion must preserve the orientation. The old
      // generic Euler inverse failed exactly here for some roll phases.
      double rotator[3]{};
      RotatorDegreesFromQuat(relative, rotator);
      const Quat written = QuatNormalize(QuatFromRotatorDegrees(rotator));
      const double dot = std::fabs(relative.x * written.x +
                                   relative.y * written.y +
                                   relative.z * written.z +
                                   relative.w * written.w);
      Check(dot > 0.99999, "rotator round trip must preserve the wheel quat");
      const Quat written_world =
          QuatNormalize(QuatMul(socket, written));
      const Vec3 written_axle = QuatRotate(written_world, {0.0, 1.0, 0.0});
      const Vec3 written_forward =
          nfs2015_chase::Normalize(Cross(written_axle, up));
      const double written_psi =
          std::atan2(Dot(up, Cross(forward, written_forward)),
                     Dot(forward, written_forward));
      Check(std::fabs(WrappedDegrees(written_psi) - 25.0) < 0.05,
            "written rotator must keep the axle on target while rolling");
    }
  }


  // A parked socket can still carry the wheel's current spin phase. Steering
  // around car-up with that phase is exactly the fore-aft "universal joint"
  // failure, so the captured socket must have its local-Y twist removed.
  {
    for (const double spin_degrees : {0.0, 45.0, 90.0, 180.0, 270.0}) {
      const Quat spin =
          AxisAngle({0.0, 1.0, 0.0}, spin_degrees * kRadiansPerDegree);
      const Quat cleaned = RemoveWheelSpinTwist(spin);
      const double dot = std::fabs(cleaned.w);
      Check(dot > 0.999999, "pure wheel spin must be removed from the socket");
    }
    const Quat steer =
        AxisAngle({0.0, 0.0, 1.0}, 20.0 * kRadiansPerDegree);
    const Quat spin =
        AxisAngle({0.0, 1.0, 0.0}, 90.0 * kRadiansPerDegree);
    const Quat combined = QuatNormalize(QuatMul(steer, spin));
    const Quat cleaned = RemoveWheelSpinTwist(combined);
    const double dot = std::fabs(cleaned.x * steer.x + cleaned.y * steer.y +
                                 cleaned.z * steer.z + cleaned.w * steer.w);
    Check(dot > 0.999999, "spin removal must preserve the socket steer");
  }

  if (failures == 0) {
    std::printf("wheel_steer_test: ok\n");
  }
  return failures == 0 ? 0 : 1;
}
