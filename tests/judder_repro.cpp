// Offline reproduction of the chase-camera judder.
//
// The complaint is that the car shakes in frame while driving. Everything about
// that can be reproduced without the game: the car's true path is a line I
// choose, the Game tick that samples it is a clock I choose, and the render call
// that consumes the ring is another clock I choose. Feed those through the same
// model the plugin uses and measure where the car lands on screen.
//
// "On screen" here is the angle between the camera's look axis and the true car
// position, which is what the eye reads. If the camera were perfect that angle
// would be constant. Its frame-to-frame wobble is the judder.
//
//   g++ -std=c++20 -O2 -Wall -Wextra -o /tmp/repro tests/judder_repro.cpp
//   /tmp/repro

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../car_sample.hpp"
#include "../chase_cam.hpp"

namespace {

using nfs2015_chase::CarSample;
using nfs2015_chase::CarState;
using nfs2015_chase::ChaseCam;
using nfs2015_chase::SampleRing;

// A small deterministic noise source, so runs are comparable between builds.
struct Random final {
  std::uint64_t state = 0x2545F4914F6CDD1DULL;

  double Next() noexcept {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return static_cast<double>(state >> 11) / 9007199254740992.0;
  }
};

// The true car path. Straight line at a chosen speed, so any camera wobble is
// the camera's own doing and not the car's.
struct Path final {
  double speed_mps = 30.0;
  // Lateral excursion, so the path is not perfectly degenerate.
  double sway_metres = 0.0;
  double sway_hz = 0.5;

  double Along(double t) const noexcept { return speed_mps * t; }

  double Across(double t) const noexcept {
    if (sway_metres <= 0.0) {
      return 0.0;
    }
    return sway_metres * std::sin(2.0 * 3.14159265358979 * sway_hz * t);
  }

  double AcrossRate(double t) const noexcept {
    if (sway_metres <= 0.0) {
      return 0.0;
    }
    return sway_metres * 2.0 * 3.14159265358979 * sway_hz *
           std::cos(2.0 * 3.14159265358979 * sway_hz * t);
  }

  CarState At(double t) const noexcept {
    CarState car{};
    car.position.x = Along(t);
    car.position.y = Across(t);
    car.position.z = 0.0;
    car.velocity.x = speed_mps;
    car.velocity.y = AcrossRate(t);
    car.speed = speed_mps;
    // Straight line: nose along the path.
    car.yaw = std::atan2(AcrossRate(t), speed_mps);
    car.yaw_rate = 0.0;
    car.acceleration = {};
    car.slip_angle = 0.0;
    return car;
  }
};

struct Clock final {
  // Mean interval and jitter, in seconds.
  double mean = 0.0169;
  double jitter = 0.0015;
  // Every `hitch_period` ticks, the interval is multiplied by `hitch_factor`.
  int hitch_period = 0;
  double hitch_factor = 3.0;

  double Next(Random &random, int index) const noexcept {
    double interval = mean + (random.Next() - 0.5) * 2.0 * jitter;
    if (hitch_period > 0 && index % hitch_period == hitch_period - 1) {
      interval *= hitch_factor;
    }
    return interval;
  }
};

struct Result final {
  double peak_degrees = 0.0;
  double sum_degrees = 0.0;
  std::uint64_t count = 0;
  // The camera's own speed, in cm/s, and how much that speed jumps from frame to
  // frame. This is the background's flow rate: a camera whose speed steps makes
  // the whole world step, and the car in the middle of the frame does not move a
  // pixel while it happens.
  double speed_peak_cmps = 0.0;
  double speed_sum_cmps = 0.0;
  std::uint64_t speed_count = 0;
  double residual = 0.0;
  double residual_sum = 0.0;
  std::uint64_t residual_count = 0;

  [[nodiscard]] double SpeedJumpAverage() const noexcept {
    return speed_count == 0 ? 0.0 : speed_sum_cmps / static_cast<double>(speed_count);
  }
  [[nodiscard]] double Average() const noexcept {
    return count == 0 ? 0.0 : sum_degrees / static_cast<double>(count);
  }
  [[nodiscard]] double ResidualAverage() const noexcept {
    return residual_count == 0
               ? 0.0
               : residual_sum / static_cast<double>(residual_count);
  }
};

// The angle, in degrees, between the camera's look axis and the true car
// position. Its wobble is what the eye reads as the car shaking.
double ScreenAngleDegrees(const nfs2015_chase::CameraState &view,
                          const CarState &truth) noexcept {
  const double forward[3] = {view.target.x - view.position.x,
                             view.target.y - view.position.y,
                             view.target.z - view.position.z};
  const double to_car[3] = {truth.position.x - view.position.x,
                            truth.position.y - view.position.y,
                            truth.position.z - view.position.z};
  const double forward2 = forward[0] * forward[0] + forward[1] * forward[1] +
                          forward[2] * forward[2];
  const double car2 = to_car[0] * to_car[0] + to_car[1] * to_car[1] +
                      to_car[2] * to_car[2];
  if (!(forward2 > 1e-9) || !(car2 > 1e-9)) {
    return 0.0;
  }
  double dot = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    dot += forward[axis] * to_car[axis];
  }
  const double cosine = dot / std::sqrt(forward2 * car2);
  const double angle = std::acos(std::fmax(-1.0, std::fmin(1.0, cosine)));
  return angle * nfs2015_chase::kDegreesPerRadian;
}

// Runs one scenario.
//   lag 0 = zero: the newest sample, a staircase
//   lag 1 = a fixed delay of one mean tick interval
//   lag 2 = the last measured interval, used raw
//   lag 3 = one and a half mean intervals, so the sample point always sits
//           strictly inside a bracket
Result Run(const Path &path, const Clock &tick_clock, const Clock &frame_clock,
           int lag, bool verbose) {
  Random tick_random;
  Random frame_random;

  SampleRing ring;
  ChaseCam chase;
  Result result;

  auto make_sample = [&path](double t) {
    CarSample sample{};
    sample.time_ns = static_cast<std::int64_t>(std::llround(t * 1e9));
    sample.car = path.At(t);
    for (int axis = 0; axis < 3; ++axis) {
      const double value = axis == 0 ? sample.car.position.x
                                     : (axis == 1 ? sample.car.position.y
                                                  : sample.car.position.z);
      sample.anchor_cm[axis] = value * 100.0;
      sample.origin_cm[axis] = value * 100.0;
    }
    sample.driving = true;
    sample.epoch = 1;
    sample.camera_scale = 1.0;
    sample.aspect_ratio = 16.0 / 9.0;
    sample.fov_enabled = true;
    return sample;
  };

  // Seed a few ticks so the ring holds a bracket, and so the delay modes have
  // something older than the newest to look at.
  double tick_time = 0.0;
  double last_interval = tick_clock.mean;
  double mean_interval = tick_clock.mean;
  for (int index = 0; index < 6; ++index) {
    tick_time += tick_clock.Next(tick_random, index);
    last_interval = tick_time - (index == 0 ? 0.0 : tick_time - last_interval);
    ring.Write(make_sample(tick_time));
  }
  {
    // Recompute the mean from the seeded intervals.
    last_interval = tick_time / 6.0;
    mean_interval = last_interval;
  }
  double next_tick = tick_time;
  int tick_index = 6;
  CarSample newest{};
  CarSample second_newest{};
  ring.Latest(newest);
  ring.At(newest.time_ns - 1, second_newest);
  if (second_newest.time_ns <= 0) {
    second_newest = newest;
  }

  bool initialized = false;
  chase.Reset(path.At(tick_time));
  initialized = true;

  constexpr double kRenderCallSeconds = 1.0 / 1400.0;
  double now = tick_time;
  double next_frame = tick_time;
  double previous_angle = 0.0;
  double previous_rate = 0.0;
  bool have_previous = false;
  double previous_eye[3]{};
  double previous_eye_speed[3]{};
  double previous_eye_time = 0.0;
  const double horizon = 6.0;

  while (now < horizon) {
    now += kRenderCallSeconds;

    while (now >= next_tick) {
      const double interval = tick_clock.Next(tick_random, tick_index++);
      next_tick += interval;
      last_interval = interval;
      // A slow running average, as the plugin's smoothed delay would have.
      mean_interval += (interval - mean_interval) * 0.02;
      const CarSample fresh = make_sample(next_tick - interval);
      ring.Write(fresh);
      second_newest = newest;
      newest = fresh;
    }

    CarSample sample{};
    const std::int64_t now_ns = static_cast<std::int64_t>(std::llround(now * 1e9));
    bool got = false;
    switch (lag) {
      case 0:
        got = ring.Latest(sample);
        break;
      case 1:
        got = ring.At(now_ns - static_cast<std::int64_t>(mean_interval * 1e9),
                      sample);
        break;
      case 2:
        got = ring.At(now_ns - static_cast<std::int64_t>(last_interval * 1e9),
                      sample);
        break;
      case 3:
        got = ring.At(now_ns - static_cast<std::int64_t>(mean_interval * 1.5e9),
                      sample);
        break;
      case 5:
        got = ring.AtSmooth(
            now_ns - static_cast<std::int64_t>(mean_interval * 1.5e9), sample);
        break;
      default: {
        // One and a half mean intervals back, but when that lands past the
        // newest sample, carry on along the last segment instead of clamping.
        // A stall is a late sample, not a stopped car.
        const double target =
            static_cast<double>(now_ns) * 1e-9 - mean_interval * 1.5;
        const double newest_time = static_cast<double>(newest.time_ns) * 1e-9;
        if (target <= newest_time) {
          got = ring.At(now_ns - static_cast<std::int64_t>(mean_interval * 1.5e9),
                        sample);
        } else {
          const double span =
              static_cast<double>(newest.time_ns - second_newest.time_ns) * 1e-9;
          if (span > 1e-6) {
            double ahead = (target - newest_time) / span;
            if (ahead > 1.0) {
              ahead = 1.0;
            }
            sample = nfs2015_chase::BlendCarSample(
                newest, second_newest, -ahead);
            sample.time_ns = now_ns;
            got = true;
          } else {
            got = ring.Latest(sample);
          }
        }
        break;
      }
    }
    if (!got || !sample.driving) {
      continue;
    }
    if (!initialized) {
      chase.Reset(sample.car);
      initialized = true;
    }
    const nfs2015_chase::CameraState view =
        chase.Update(kRenderCallSeconds, sample.car);
    const double eye[3] = {view.position.x, view.position.y, view.position.z};
    const double eye_now = static_cast<double>(now_ns) * 1e-9;
    if (have_previous) {
      const double dt_eye = eye_now - previous_eye_time;
      if (dt_eye > 1e-6) {
        double jump2 = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
          const double speed = (eye[axis] - previous_eye[axis]) / dt_eye;
          const double jump = speed - previous_eye_speed[axis];
          jump2 += jump * jump;
          previous_eye_speed[axis] = speed;
        }
        const double jump = std::sqrt(jump2);
        if (std::isfinite(jump) && jump < 5000.0) {
          if (jump > result.speed_peak_cmps) {
            result.speed_peak_cmps = jump;
          }
          result.speed_sum_cmps += jump;
          ++result.speed_count;
        }
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      previous_eye[axis] = eye[axis];
    }
    previous_eye_time = eye_now;
    have_previous = true;

    if (now >= next_frame) {
      // The frame the player sees, at the frame clock's own time, against the
      // true car position at that instant.
      const double angle = ScreenAngleDegrees(view, path.At(now));
      const double frame_seconds = frame_clock.Next(frame_random, 0);
      if (have_previous && frame_seconds > 0.002 && now > 1.5) {
        const double rate = (angle - previous_angle) / frame_seconds;
        const double residual = (rate - previous_rate) * frame_seconds;
        if (std::isfinite(residual) && std::fabs(residual) < 180.0) {
          const double magnitude = std::fabs(residual);
          if (magnitude > result.peak_degrees) {
            result.peak_degrees = magnitude;
          }
          result.sum_degrees += magnitude;
          ++result.count;
          if (verbose && magnitude > 0.02) {
            std::printf("    t=%.4f  residual=%.4f deg\n", now, magnitude);
          }
        }
      }
      previous_rate = (angle - previous_angle) / frame_seconds;
      previous_angle = angle;
      have_previous = true;
      next_frame = now + frame_seconds;
    }
  }
  return result;
}

void Report(const char *name, const Result &result) {
  std::printf("%-34s car %.3f/%.3f deg   eye speed jump %8.2f/%.2f cm/s\n",
              name, result.Average(), result.peak_degrees,
              result.SpeedJumpAverage(), result.speed_peak_cmps);
}

}  // namespace

int main() {
  Clock tick_clock;
  tick_clock.mean = 0.0169;  // ~59 Hz, the rate reported idle
  tick_clock.jitter = 0.0015;
  tick_clock.hitch_period = 0;
  tick_clock.hitch_factor = 1.0;

  Clock frame_clock;
  frame_clock.mean = 0.0169;
  frame_clock.jitter = 0.0010;

  Path path;
  path.speed_mps = 30.0;

  const char *names[6] = {"zero lag (staircase)", "fixed mean delay",
                          "raw last interval", "1.5x mean delay",
                          "1.5x + continue on a stall", "1.5x + curved"};
  const int kModes = 6;

  std::printf("straight 30 m/s, steady tick, steady frames\n");
  {
    Clock steady;
    steady.mean = 0.0169;
    steady.jitter = 0.0;
    for (int lag = 0; lag < kModes; ++lag) {
      Report(names[lag], Run(path, steady, frame_clock, lag, false));
    }
  }

  std::printf("straight 30 m/s, tick jitter +-1.5 ms\n");
  for (int lag = 0; lag < kModes; ++lag) {
    Report(names[lag], Run(path, tick_clock, frame_clock, lag, false));
  }

  std::printf("straight 30 m/s, tick jitter, a 3x hitch every 20 ticks\n");
  {
    Clock hitchy = tick_clock;
    hitchy.hitch_period = 20;
    hitchy.hitch_factor = 3.0;
    for (int lag = 0; lag < kModes; ++lag) {
      Report(names[lag], Run(path, hitchy, frame_clock, lag, false));
    }
  }

  std::printf("city 45 Hz tick against 59 Hz frames, jitter, hitches\n");
  {
    Clock city = tick_clock;
    city.mean = 0.0222;
    city.jitter = 0.0040;
    city.hitch_period = 20;
    city.hitch_factor = 2.2;
    for (int lag = 0; lag < kModes; ++lag) {
      Report(names[lag], Run(path, city, frame_clock, lag, false));
    }
  }

  std::printf("swaying path, 30 m/s, 0.5 Hz, 0.3 m across\n");
  {
    Path sway;
    sway.speed_mps = 30.0;
    sway.sway_metres = 0.3;
    sway.sway_hz = 0.5;
    for (int lag = 0; lag < kModes; ++lag) {
      Report(names[lag], Run(sway, tick_clock, frame_clock, lag, false));
    }
  }
  return 0;
}
