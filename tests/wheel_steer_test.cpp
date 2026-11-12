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
using nfs2015_chase::AdvanceWheelSteerAppliedAngle;
using nfs2015_chase::ComposeWheelSteerStockAngle;
using nfs2015_chase::ComposeWheelSteerTargetAngle;
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
  config.entry_speed_mps = 54.0 / 3.6;
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

  // The production gate defaults to 54 km/h and is entry-only. Just below
  // the gate the stock animation must stay in control; just above it the
  // same slip and steering can engage.
  {
    WheelSteerDriftState below_state;
    const auto below = Step(below_state, config, 20.0, 53.0 / 3.6);
    Check(!below.active, "53 km/h must stay below the default entry gate");

    WheelSteerDriftState above_state;
    const auto above = Step(above_state, config, 20.0, 54.5 / 3.6);
    Check(above.active, "54.5 km/h should pass the default entry gate");
    Check(above.engaged, "the default entry gate must engage the drift state");

    WheelSteerConfig lowered = config;
    lowered.entry_speed_mps = 40.0 / 3.6;
    WheelSteerDriftState lowered_state;
    const auto lowered_target =
        Step(lowered_state, lowered, 20.0, 45.0 / 3.6);
    Check(lowered_target.active,
          "the configurable entry speed should lower the gate");
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
    Check(first.quick_turn,
          "holding into the slide at entry must trigger the quick turn");
    Check(std::fabs(WrappedDegrees(first.angle_radians) + 40.0) < 0.01,
          "the quick turn must target the drift-side limit, not countersteer");

    WheelSteerTarget later = first;
    for (int tick = 0; tick < 5; ++tick) {
      later = Step(state, config, 20.0, 20.0, -0.5, true, 0.05);
    }
    Check(later.phase == WheelSteerPhase::entering ||
              later.phase == WheelSteerPhase::drifting,
          "turn_in should hand over to the countersteer blend");
    Check(later.quick_turn,
          "the quick turn must stay latched while the key is held");
    Check(std::fabs(WrappedDegrees(later.angle_radians) + 40.0) < 0.01,
          "the held quick turn must stay at the drift-side limit");

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

  // The stock turn-in pose still scales with rear slip: a small slide skips
  // the full turn-in, a medium slide gets a partial stock angle, and a
  // high-slip U-turn gets the full stock angle. When the drift-side key is
  // held, the one-shot quick turn overrides the visual target with the
  // drift-side limit.
  {
    WheelSteerConfig scaled = config;
    scaled.counter_authority_radians = 0.5 * kRadiansPerDegree;

    WheelSteerDriftState small_state;
    const auto small = StepTravel(small_state, scaled, 40.0, 9.0, 20.0,
                                  19.0, -0.5, true, 0.05);
    Check(small.phase == WheelSteerPhase::entering,
          "small rear slip should skip the full turn_in phase");
    Check(small.turn_in_alpha < scaled.turn_in_min_alpha,
          "small rear slip should keep turn_in_alpha below the threshold");
    const double small_stock = WrappedDegrees(
        ComposeWheelSteerStockAngle(-5.0 * kRadiansPerDegree, small));
    Check(std::fabs(small_stock + 5.2) < 1.0,
          "small drift should keep the measured entry stock pose");
    Check(small.quick_turn,
          "a held drift-side key must quick-turn even on a small drift");

    WheelSteerDriftState medium_state;
    const auto medium = StepTravel(medium_state, scaled, 40.0, 15.0, 20.0,
                                   19.0, -1.0, true, 0.05);
    Check(medium.phase == WheelSteerPhase::turn_in,
          "medium rear slip should enter turn_in");
    Check(medium.turn_in_alpha > 0.3 &&
              medium.turn_in_alpha < 0.7,
          "medium rear slip should produce a partial turn-in scale");
    Check(std::fabs(medium.turn_in_duration_seconds - 0.10) < 1e-9,
          "medium turn-in should use the minimum visible duration");
    const double medium_stock = WrappedDegrees(
        ComposeWheelSteerStockAngle(-5.0 * kRadiansPerDegree, medium));
    Check(medium_stock < -15.0 && medium_stock > -25.0,
          "medium drift should keep a partial stock angle");
    Check(medium.quick_turn,
          "a held drift-side key must quick-turn on a medium drift");

    WheelSteerDriftState high_state;
    const auto high = StepTravel(high_state, scaled, 30.0, 25.0, 20.0,
                                 19.0, -1.0, true, 0.05);
    Check(high.phase == WheelSteerPhase::turn_in,
          "high rear slip should enter turn_in");
    Check(high.turn_in_alpha > 0.99,
          "high rear slip should use the full turn-in scale");
    Check(std::fabs(high.turn_in_duration_seconds - 0.18) < 1e-9,
          "high-slip turn-in should use the full duration");
    const double high_stock = WrappedDegrees(
        ComposeWheelSteerStockAngle(-5.0 * kRadiansPerDegree, high));
    Check(std::fabs(high_stock + 35.0) < 0.1,
          "high-slip U-turn should keep the full stock angle");
    Check(high.quick_turn,
          "a held drift-side key must quick-turn on a high-slip U-turn");
  }

  // The composition helper is the visual contract between the state machine
  // and the measured wheel pose: alpha scales the stock target, the drift
  // blend then moves that target toward countersteer, and release returns to
  // the entry pose instead of snapping to the raw steering angle.
  {
    WheelSteerTarget target{};
    target.phase = WheelSteerPhase::turn_in;
    target.stock_angle_radians = -35.0 * kRadiansPerDegree;
    target.counter_angle_radians = 20.0 * kRadiansPerDegree;
    target.angle_blend = 0.0;
    target.turn_in_alpha = 0.0;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) + 5.0) < 0.01,
          "zero alpha must keep the measured entry pose");
    target.turn_in_alpha = 0.5;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerStockAngle(
                        -5.0 * kRadiansPerDegree, target)) + 20.0) < 0.01,
          "half alpha must produce a halfway stock target");
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) + 20.0) < 0.01,
          "half alpha must blend halfway to the raw stock angle");
    target.turn_in_alpha = 1.0;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) + 35.0) < 0.01,
          "full alpha must reach the raw stock angle");

    target.phase = WheelSteerPhase::entering;
    target.angle_blend = 1.0;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) - 20.0) < 0.01,
          "full drift blend must reach countersteer");

    target.phase = WheelSteerPhase::releasing;
    target.angle_blend = 0.0;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) + 5.0) < 0.01,
          "release must return to the measured entry pose");

    target.phase = WheelSteerPhase::grip;
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, target)) + 5.0) < 0.01,
          "grip handoff must hold the measured entry pose");
  }

  // The applied-angle helper is the actual visual sweep. Starting at the
  // measured 5 degrees, a high-slip turn_in must reach the full stock target
  // within turn_in_min_seconds, not stay frozen at the measured pose.
  {
    WheelSteerTarget target{};
    target.phase = WheelSteerPhase::turn_in;
    target.angle_radians = -35.0 * kRadiansPerDegree;
    target.turn_in_duration_seconds = 0.18;
    target.turn_in_alpha = 1.0;
    double applied = -5.0 * kRadiansPerDegree;
    for (int tick = 0; tick < 6; ++tick) {
      applied = AdvanceWheelSteerAppliedAngle(
          applied, target, 1.0 / 60.0, config);
    }
    Check(std::fabs(WrappedDegrees(applied) + 35.0) < 0.5,
          "high-slip turn_in must reach full stock before handoff");

    target.angle_radians = -20.0 * kRadiansPerDegree;
    target.turn_in_duration_seconds = 0.10;
    target.turn_in_alpha = 0.5;
    applied = -5.0 * kRadiansPerDegree;
    for (int tick = 0; tick < 6; ++tick) {
      applied = AdvanceWheelSteerAppliedAngle(
          applied, target, 1.0 / 60.0, config);
    }
    Check(std::fabs(WrappedDegrees(applied) + 20.0) < 0.5,
          "medium turn_in must reach its partial stock target");
  }

  // If the driver is already countersteering, skip turn_in.
  {
    WheelSteerDriftState state;
    const auto target =
        Step(state, config, 20.0, 20.0, 0.5, true, 0.05);
    Check(target.phase == WheelSteerPhase::entering,
          "already-countersteering input should skip turn_in");
  }

  // The first drift-side press (including a key already held when the drift
  // starts) turns the wheel toward the drift at the fast turn-in rate.
  // Releasing the key returns to the countersteer target; later presses in the
  // same drift are ordinary countersteer input.
  {
    WheelSteerConfig quick_config = config;
    quick_config.drift_key_threshold = 0.1;

    WheelSteerDriftState state;
    // Positive rear slip is a left-hand drift; a negative input presses into
    // the drift (the countersteer target is positive).
    const auto first = Step(state, quick_config, 20.0, 20.0, -1.0, true,
                            1.0 / 60.0);
    Check(first.phase == WheelSteerPhase::turn_in,
          "an into-drift entry must start turn_in");
    Check(first.quick_turn,
          "a key already held at drift entry must trigger the quick turn");
    Check(std::fabs(WrappedDegrees(ComposeWheelSteerTargetAngle(
                        -5.0 * kRadiansPerDegree, first)) + 40.0) < 0.01,
          "the quick turn must target the full angle into the drift");

    // Keep holding: the latch owns the wheel.
    auto held = first;
    for (int tick = 0; tick < 10; ++tick) {
      held = Step(state, quick_config, 20.0, 20.0, -1.0, true,
                  1.0 / 60.0);
    }
    Check(held.quick_turn,
          "holding the drift-side key must keep the quick turn active");

    // Release: back to the normal target; the applied angle stays rate-limited.
    const auto released =
        Step(state, quick_config, 20.0, 20.0, 0.0, true, 1.0 / 60.0);
    Check(!released.quick_turn,
          "releasing the drift-side key must drop the quick turn");

    // A second press in the same drift is ordinary countersteer input.
    const auto again =
        Step(state, quick_config, 20.0, 20.0, -1.0, true, 1.0 / 60.0);
    Check(!again.quick_turn,
          "a second drift-side press in the same drift must not re-trigger");

    // The visual sweep is rate-limited: 120 deg/s is 2 deg per 60 Hz tick.
    WheelSteerTarget quick_target{};
    quick_target.phase = WheelSteerPhase::turn_in;
    quick_target.turn_in_duration_seconds = 0.18;
    quick_target.quick_turn = true;
    quick_target.quick_turn_angle_radians = -40.0 * kRadiansPerDegree;
    quick_target.angle_radians = quick_target.quick_turn_angle_radians;
    double applied = 0.0;
    applied = AdvanceWheelSteerAppliedAngle(
        applied, quick_target, 1.0 / 60.0, quick_config);
    Check(std::fabs(WrappedDegrees(applied) + 2.0) < 1e-6,
          "the quick turn must move at the 120 deg/s front-wheel rate");

    // Entering while countersteering does not arm the latch; the first
    // drift-side press after entry does.
    WheelSteerDriftState late_state;
    const auto counter_entry =
        Step(late_state, quick_config, 20.0, 20.0, 1.0, true, 1.0 / 60.0);
    Check(!counter_entry.quick_turn,
          "countersteering entry must not trigger the quick turn");
    const auto pressed =
        Step(late_state, quick_config, 20.0, 20.0, -1.0, true,
             1.0 / 60.0);
    Check(pressed.quick_turn,
          "the first drift-side press after entry must trigger the quick turn");

    // Stick noise below the threshold is not a press.
    WheelSteerDriftState small_state;
    const auto small = Step(small_state, quick_config, 20.0, 20.0, -0.05,
                            true, 1.0 / 60.0);
    Check(!small.quick_turn,
          "analog input below the drift-key threshold must not trigger");

    // The next drift re-arms the one-shot.
    for (int tick = 0; tick < 12; ++tick) {
      static_cast<void>(Step(late_state, quick_config, 1.0, 20.0, 0.0,
                             true, 0.3));
    }
    Check(late_state.phase == WheelSteerPhase::grip,
          "the test drift must release before the next entry");
    const auto next =
        Step(late_state, quick_config, 20.0, 20.0, -1.0, true,
             1.0 / 60.0);
    Check(next.quick_turn,
          "a new drift must re-arm the quick turn");
  }

  // A continuous drift can change direction without returning to grip. A new
  // direction segment gets its own one-shot quick turn, including when the new
  // side key is already held through the change.
  {
    WheelSteerConfig quick_config = config;
    quick_config.drift_key_threshold = 0.1;
    WheelSteerDriftState state;
    for (int tick = 0; tick < 30; ++tick) {
      static_cast<void>(Step(state, quick_config, 20.0, 20.0, -1.0, true,
                             1.0 / 60.0));
    }
    Check(state.phase == WheelSteerPhase::drifting,
          "continuous-drift test must reach drifting");
    Check(state.quick_turn_active,
          "the left drift must engage the quick turn");
    Check(state.drift_direction_sign == 1,
          "positive rear slip must record a left drift");

    const auto flipped = Step(state, quick_config, -20.0, 20.0, 1.0, true,
                              1.0 / 60.0);
    Check(flipped.phase == WheelSteerPhase::drifting,
          "a direction flip must stay in the same continuous drift");
    Check(state.drift_direction_sign == -1,
          "the drift direction sign must follow the opposite rear slip");
    Check(flipped.quick_turn,
          "a key held through the direction change must trigger the new "
          "quick turn");
    Check(std::fabs(WrappedDegrees(flipped.quick_turn_angle_radians) - 40.0) <
              0.01,
          "the new quick turn must target the opposite full angle");

    const double previous_applied = -40.0 * kRadiansPerDegree;
    const double applied = AdvanceWheelSteerAppliedAngle(
        previous_applied, flipped, 1.0 / 60.0, quick_config);
    Check(std::fabs(WrappedDegrees(applied) + 38.0) < 1e-6,
          "a direction flip must sweep at 120 deg/s, not jump");

    const auto released = Step(state, quick_config, -20.0, 20.0, 0.0, true,
                               1.0 / 60.0);
    Check(!released.quick_turn,
          "releasing after the direction change must drop the quick turn");
    const auto again = Step(state, quick_config, -20.0, 20.0, 1.0, true,
                            1.0 / 60.0);
    Check(!again.quick_turn,
          "a second press in the new direction segment must not re-trigger");

    const auto back = Step(state, quick_config, 20.0, 20.0, -1.0, true,
                           1.0 / 60.0);
    Check(back.quick_turn,
          "a second direction change must re-arm the quick turn again");
    Check(state.drift_direction_sign == 1,
          "the drift direction sign must follow the return flip");
  }

  // A sign flip below the entry slip threshold is noise, not a new segment.
  {
    WheelSteerConfig quick_config = config;
    quick_config.drift_key_threshold = 0.1;
    WheelSteerDriftState state;
    for (int tick = 0; tick < 30; ++tick) {
      static_cast<void>(Step(state, quick_config, 20.0, 20.0, -1.0, true,
                             1.0 / 60.0));
    }
    static_cast<void>(Step(state, quick_config, 20.0, 20.0, 0.0, true,
                           1.0 / 60.0));
    const auto shallow = Step(state, quick_config, -5.0, 20.0, 0.0, true,
                              1.0 / 60.0);
    Check(!shallow.quick_turn,
          "a shallow sign flip must not trigger the quick turn");
    Check(state.drift_direction_sign == 1,
          "a shallow sign flip must not change the drift direction");
    const auto shallow_press =
        Step(state, quick_config, -5.0, 20.0, 1.0, true, 1.0 / 60.0);
    Check(!shallow_press.quick_turn,
          "a shallow opposite key must not re-arm the quick turn");
    const auto deep = Step(state, quick_config, -7.0, 20.0, 1.0, true,
                           1.0 / 60.0);
    Check(deep.quick_turn,
          "the opposite drift must re-arm once it reaches the entry "
          "threshold");
    Check(state.drift_direction_sign == -1,
          "the drift direction sign must follow the established flip");
  }

  // A new direction is not armed until it reaches the entry threshold, even
  // when the previous direction never spent its quick turn. This keeps the
  // one-shot latch tied to the established direction segment.
  {
    WheelSteerConfig quick_config = config;
    quick_config.drift_key_threshold = 0.1;
    WheelSteerDriftState state;
    // Enter a left drift by countersteering, so the left segment never uses
    // its quick turn.
    for (int tick = 0; tick < 30; ++tick) {
      static_cast<void>(Step(state, quick_config, 20.0, 20.0, 1.0, true,
                             1.0 / 60.0));
    }
    Check(state.phase == WheelSteerPhase::drifting,
          "the unused-latch test must reach drifting");
    Check(!state.quick_turn_used,
          "a countersteering entry must leave the quick-turn latch unused");
    const auto early = Step(state, quick_config, -5.0, 20.0, 1.0, true,
                            1.0 / 60.0);
    Check(!early.quick_turn,
          "a new direction below 6.5 degrees must not trigger the quick "
          "turn");
    const auto established = Step(state, quick_config, -7.0, 20.0, 1.0,
                                  true, 1.0 / 60.0);
    Check(established.quick_turn,
          "the held key must trigger once the new direction reaches 6.5 "
          "degrees");
  }

  // A brief dip through releasing still counts as the same continuous drift:
  // an opposite direction established there re-arms the latch for the new
  // segment, even though the release phase itself never quick-turns.
  {
    WheelSteerConfig quick_config = config;
    quick_config.drift_key_threshold = 0.1;
    WheelSteerDriftState state;
    for (int tick = 0; tick < 30; ++tick) {
      static_cast<void>(Step(state, quick_config, 20.0, 20.0, -1.0, true,
                             1.0 / 60.0));
    }
    for (int tick = 0; tick < 20 && state.phase != WheelSteerPhase::releasing;
         ++tick) {
      static_cast<void>(Step(state, quick_config, 1.0, 20.0, 0.0, true, 0.1));
    }
    Check(state.phase == WheelSteerPhase::releasing,
          "the test drift must reach the releasing phase");
    Check(state.quick_turn_used,
          "the release phase must keep the spent quick-turn latch");
    const auto flip = Step(state, quick_config, -20.0, 20.0, 0.0, true, 0.1);
    Check(!flip.quick_turn,
          "the release phase itself must not engage a quick turn");
    Check(state.drift_direction_sign == -1,
          "an opposite drift established during release must change the "
          "sign");
    Check(!state.quick_turn_used,
          "an opposite drift established during release must re-arm the "
          "latch");
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
  // drift-angle reference, a conservative 32 degree visual countersteer, and
  // the tunable 54 km/h entry gate.
  {
    const WheelSteerConfig defaults;
    Check(std::fabs(WrappedDegrees(defaults.max_angle_radians) - 32.0) <
              0.01,
          "default visual countersteer should be 32 degrees");
    Check(std::fabs(WrappedDegrees(defaults.full_slip_radians) - 40.0) <
              0.01,
          "default full countersteer slip should be 40 degrees");
    Check(std::fabs(defaults.entry_speed_mps - 54.0 / 3.6) < 1e-9,
          "default entry speed should be 54 km/h");
    Check(defaults.counter_sample_fraction == 0.0,
          "default countersteer sample point should be the axle centre");
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

  // The countersteer sample point moves along the front axle as a fraction of
  // half track. Positive is the inside of the drift, so a right-hand drift
  // (positive yaw, negative rear slip) samples the car's right side and a
  // left-hand drift mirrors it. Zero must keep the historical centre angle.
  {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    const Vec3 velocity{20.0, 0.0, 0.0};
    const Vec3 front{1.3, 0.0, 0.0};
    const Vec3 rear{-1.3, 0.0, 0.0};
    const double half_track = 0.8;

    const auto centre_travel =
        ComputeWheelSteerTravel(forward, up, velocity, 0.5, front, rear);
    const auto inside_travel = ComputeWheelSteerTravel(
        forward, up, velocity, 0.5, front, rear, 1.0, half_track);
    const auto outside_travel = ComputeWheelSteerTravel(
        forward, up, velocity, 0.5, front, rear, -1.0, half_track);
    Check(std::fabs(inside_travel.front_center_angle_radians -
                    centre_travel.front_angle_radians) < 1e-9,
          "the front-centre angle must not move with the sample point");
    Check(std::fabs(inside_travel.front_sample_offset_meters - half_track) <
              1e-9,
          "positive sample fraction must move toward the drift inside");
    Check(inside_travel.front_angle_radians >
              centre_travel.front_angle_radians,
          "the inside sample must enlarge a positive front travel angle");
    Check(outside_travel.front_angle_radians <
              centre_travel.front_angle_radians,
          "the outside sample must shrink a positive front travel angle");

    const auto mirrored_centre =
        ComputeWheelSteerTravel(forward, up, velocity, -0.5, front, rear);
    const auto mirrored_inside = ComputeWheelSteerTravel(
        forward, up, velocity, -0.5, front, rear, 1.0, half_track);
    Check(mirrored_inside.front_sample_offset_meters < 0.0,
          "a left-hand drift must mirror the sample to the car's left side");
    Check(mirrored_inside.front_angle_radians <
              mirrored_centre.front_angle_radians,
          "the mirrored inside sample must enlarge a negative front angle");

    const auto no_track = ComputeWheelSteerTravel(
        forward, up, velocity, 0.5, front, rear, 1.0, 0.0);
    Check(std::fabs(no_track.front_angle_radians -
                    no_track.front_center_angle_radians) < 1e-12,
          "an unknown front track must fall back to the axle centre");
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
