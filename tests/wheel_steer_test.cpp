#include "wheel_steer.hpp"

#include <cmath>
#include <cstdio>

namespace {

using nfs2015_chase::AxisAngle;
using nfs2015_chase::ComputeSteeredRelativeRotation;
using nfs2015_chase::ComputeVisualWheelRelativeRotation;
using nfs2015_chase::ComputeWheelSteerTarget;
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
using nfs2015_chase::WheelSteerConfig;

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

}  // namespace

int main() {
  WheelSteerConfig config;
  config.max_angle_radians = 40.0 * kRadiansPerDegree;
  config.gain = 1.0;
  config.enter_slip_radians = 8.0 * kRadiansPerDegree;
  config.full_slip_radians = 25.0 * kRadiansPerDegree;
  config.rate_radians_per_second = 360.0 * kRadiansPerDegree;
  config.minimum_speed_mps = 3.0;

  // NFS-style target: at full blend the wheel points along actual travel.
  {
    const double slip = 25.0 * kRadiansPerDegree;
    const Vec3 velocity{std::cos(slip), std::sin(slip), 0.0};
    const auto target = ComputeWheelSteerTarget(
        {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, velocity, 20.0, 20.0, false,
        config);
    Check(target.active, "25 degree slip should engage");
    Check(std::fabs(WrappedDegrees(target.angle_radians) - 25.0) < 0.05,
          "25 degree slip should target 25 degrees");
    Check(std::fabs(target.blend - 1.0) < 1e-9,
          "25 degree slip should be full blend");
  }

  // The explicit visual clamp still applies above the configured limit.
  {
    const double slip = 60.0 * kRadiansPerDegree;
    const Vec3 velocity{std::cos(slip), std::sin(slip), 0.0};
    const auto target = ComputeWheelSteerTarget(
        {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, velocity, 20.0, 20.0, false,
        config);
    Check(target.active, "60 degree slip should engage");
    Check(std::fabs(WrappedDegrees(target.angle_radians) - 40.0) < 0.05,
          "60 degree slip should clamp to 40 degrees");
  }

  // Below the entry threshold the feature stays out of the stock steering.
  {
    const double slip = 4.0 * kRadiansPerDegree;
    const Vec3 velocity{std::cos(slip), std::sin(slip), 0.0};
    const auto target = ComputeWheelSteerTarget(
        {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, velocity, 20.0, 20.0, false,
        config);
    Check(!target.active, "4 degree slip should stay inactive");
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
