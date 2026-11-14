// Route B: build the camera pose from whichever render-side source is freshest
// (render proxy transform, model pose, presenter, or the car directly) and
// write it into the renderer's view init before visibility runs.
#include "internal.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace nfs2015_chase_cam::detail {

// Route B's initializer, matching FMinimalInitializer in the 2.6.0 build.
// Every offset here is pinned by static_assert: the render-thread constructor
// reads these fields directly, so a layout drift must fail at compile time
// rather than move the camera from a misread matrix.
struct alignas(16) RouteBViewInit final {
  double view_rotation[4][4]{};
  double projection[4][4]{};
  double view_origin[3]{};
  std::int32_t constrained_rect[4]{};
  std::int32_t stereo_pass{};
  bool use_faux_ortho{};
};
static_assert(offsetof(RouteBViewInit, projection) == 0x80);
static_assert(offsetof(RouteBViewInit, view_origin) == 0x100);
static_assert(offsetof(RouteBViewInit, constrained_rect) == 0x118);
static_assert(offsetof(RouteBViewInit, stereo_pass) == 0x128);
static_assert(offsetof(RouteBViewInit, use_faux_ortho) == 0x12C);
static_assert(sizeof(RouteBViewInit) == 0x130);

void Multiply4x4(const double a[4][4], const double b[4][4],
                 double out[4][4]) noexcept {
  double result[4][4]{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      double sum = 0.0;
      for (std::size_t inner = 0; inner < 4; ++inner) {
        sum += a[row][inner] * b[inner][column];
      }
      result[row][column] = sum;
    }
  }
  std::memcpy(out, result, sizeof(result));
}

// FInverseRotationMatrix(ViewRotation) * ViewPlanesMatrix, exactly the
// composition FViewMatrices uses for a view rotation (degrees).
void MakeViewRotationMatrix(const double pitch_degrees,
                            const double yaw_degrees,
                            const double roll_degrees,
                            double out[4][4]) noexcept {
  const double yaw = yaw_degrees * kDegreesToRadians;
  const double pitch = pitch_degrees * kDegreesToRadians;
  const double roll = roll_degrees * kDegreesToRadians;
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);

  const double yaw_matrix[4][4] = {
      {cy, -sy, 0.0, 0.0},
      {sy, cy, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double pitch_matrix[4][4] = {
      {cp, 0.0, -sp, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {sp, 0.0, cp, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double roll_matrix[4][4] = {
      {1.0, 0.0, 0.0, 0.0},
      {0.0, cr, sr, 0.0},
      {0.0, -sr, cr, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };
  const double planes[4][4] = {
      {0.0, 0.0, 1.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  };

  double yaw_pitch[4][4]{};
  double yaw_pitch_roll[4][4]{};
  Multiply4x4(yaw_matrix, pitch_matrix, yaw_pitch);
  Multiply4x4(yaw_pitch, roll_matrix, yaw_pitch_roll);
  Multiply4x4(yaw_pitch_roll, planes, out);
}

bool ReadRouteBSnapshot(const RenderXProbe &probe, double car_cm[3],
                        double forward[3], std::uintptr_t &proxy,
                        std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint64_t first =
        probe.transform_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0) {
      continue;
    }
    const std::uintptr_t watched =
        probe.watched_proxy.load(std::memory_order_relaxed);
    const std::uintptr_t current_proxy =
        probe.transform_proxy.load(std::memory_order_relaxed);
    const bool valid =
        probe.transform_valid.load(std::memory_order_relaxed);
    const bool orientation_valid =
        probe.transform_orientation_valid.load(std::memory_order_relaxed);
    double current_car[3]{};
    double current_forward[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      current_car[axis] =
          probe.transform_cm[axis].load(std::memory_order_relaxed);
      current_forward[axis] =
          probe.transform_forward[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t current_timestamp =
        probe.transform_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.transform_sequence.load(std::memory_order_acquire);
    if (first != second || !valid || !orientation_valid ||
        watched == 0 || current_proxy != watched ||
        current_timestamp == 0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = current_car[axis];
      forward[axis] = current_forward[axis];
    }
    proxy = current_proxy;
    timestamp_ns = current_timestamp;
    return std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
           std::isfinite(car_cm[2]) && std::isfinite(forward[0]) &&
           std::isfinite(forward[1]) && std::isfinite(forward[2]);
  }
  return false;
}

// The camera model's pitch source. Same seqlock and proxy checks as route B,
// but only the body pitch is needed. `atan2` handles both the old horizontally
// normalised vector and the new 3D one.
bool ReadRenderPitch(const RenderXProbe &probe, double &pitch,
                     std::uintptr_t &proxy,
                     std::int64_t &timestamp_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, snapshot_proxy,
                          snapshot_ns)) {
    return false;
  }
  if (!BodyPitchFromForward(forward, pitch)) {
    return false;
  }
  proxy = snapshot_proxy;
  timestamp_ns = snapshot_ns;
  return true;
}

// Update the cached body pitch from the render-thread snapshot and return the
// value for this Game tick. Fresh snapshots win; while moving, a short render
// gap holds the last good value and then falls back to flat. A parked car keeps
// the last orientation because the transform hook may not fire again until it
// moves. The snapshot must also be newer than the last reset/teleport floor, so
// an old slope cannot leak into the new frame.
double UpdateRenderPitch(Context &context) noexcept {
  double pitch = 0.0;
  std::uintptr_t proxy = 0;
  std::int64_t snapshot_ns = 0;
  const bool read = ReadRenderPitch(context.render_x, pitch, proxy,
                                    snapshot_ns);
  const std::uintptr_t watched =
      context.render_x.watched_proxy.load(std::memory_order_relaxed);
  // A different watched proxy is a different car. Drop the cached attitude
  // before trying the new one; otherwise a parked car switch could keep the
  // old car's slope indefinitely.
  if (context.render_pitch_valid && watched != 0 &&
      watched != context.render_pitch_proxy) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
  }
  const std::int64_t now_ns = NowNanoseconds();
  if (read && proxy == watched &&
      snapshot_ns >= context.render_pitch_floor_ns) {
    const std::int64_t age = now_ns - snapshot_ns;
    if (age >= 0 && age <= kVehicleRenderTransformMaxAgeNs) {
      context.render_pitch_valid = true;
      context.render_pitch_radians = pitch;
      context.render_pitch_ns = snapshot_ns;
      context.render_pitch_proxy = proxy;
    }
  }
  if (!context.render_pitch_valid) {
    return 0.0;
  }
  const std::int64_t held = now_ns - context.render_pitch_ns;
  const double speed_cmps =
      std::hypot(context.velocity_cmps[0], context.velocity_cmps[1]);
  const bool parked = !(speed_cmps >= kRenderPitchStationarySpeedCmps);
  if (held < 0 || (!parked && held > kRenderPitchHoldNs)) {
    context.render_pitch_valid = false;
    context.render_pitch_radians = 0.0;
    context.render_pitch_ns = 0;
    context.render_pitch_proxy = 0;
    return 0.0;
  }
  return context.render_pitch_radians;
}

// The route B fallback is the only path that applies body pitch outside the
// chase model. It must obey the same master switch and flat-pose diagnostic so
// disabling slope follow also disables the fallback tilt.
[[nodiscard]] bool RouteBFollowPitch(const Context &context) noexcept {
  return context.slope_follow.load(std::memory_order_acquire) &&
         !context.diagnostic_flat_pose.load(std::memory_order_acquire);
}

bool BuildRouteBPoseFromCar(const double car_cm[3],
                            const double forward[3],
                            const bool follow_pitch, double eye[3],
                            double rotation[3]) noexcept {
  // The fallback uses the same body basis as the chase model's position frame.
  // `forward` may be fully normalised or only horizontally normalised; the
  // helper handles both. The master switch and the flat-pose diagnostic keep
  // the fallback yaw-only too.
  double body_pitch = 0.0;
  if (!BodyPitchFromForward(forward, body_pitch)) {
    return false;
  }
  if (!follow_pitch) {
    body_pitch = 0.0;
  }
  const double car_yaw = std::atan2(forward[1], forward[0]);
  // FViewMatrices and FViewInfo store world positions in Unreal units
  // (centimetres), unlike the chase model's metres. Keep this conversion
  // explicit: a metre-valued ViewOrigin puts the camera near the world origin
  // and leaves only the skybox visible.
  const nfs2015_chase::Vec3 car{
      car_cm[0],
      car_cm[1],
      car_cm[2] + kVehicleAnchorCenterHeightCm,
  };
  const nfs2015_chase::Vec3 rest{kRouteBRestXM * 100.0, 0.0,
                                 kRouteBRestZM * 100.0};
  const nfs2015_chase::Vec3 look_offset{kRouteBLookAheadM * 100.0, 0.0,
                                        kRouteBLookHeightM * 100.0};
  // `rest` is in centimetres, matching `car_cm`; the basis is unitless, so
  // CarLocalToWorld keeps the centimetre scale.
  const nfs2015_chase::CarBasis basis =
      nfs2015_chase::MakeCarBasis(car_yaw, body_pitch);
  const nfs2015_chase::Vec3 eye_vec =
      nfs2015_chase::Add(car, nfs2015_chase::CarLocalToWorld(rest, basis));
  // The look target stays in the world-horizontal nose frame: the rig climbs
  // with the body, but the horizon must not tilt with the slope.
  const nfs2015_chase::Vec3 look_vec =
      nfs2015_chase::Add(car, nfs2015_chase::RotateYaw(car_yaw, look_offset));
  const nfs2015_chase::Vec3 delta =
      nfs2015_chase::Sub(look_vec, eye_vec);
  const double delta_horizontal =
      std::sqrt(delta.x * delta.x + delta.y * delta.y);
  if (!(delta_horizontal > 1e-4)) {
    return false;
  }
  eye[0] = eye_vec.x;
  eye[1] = eye_vec.y;
  eye[2] = eye_vec.z;
  rotation[0] = std::atan2(delta.z, delta_horizontal) * kRadiansToDegrees;
  rotation[1] = std::atan2(delta.y, delta.x) * kRadiansToDegrees;
  rotation[2] = 0.0;
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

bool ReadRouteBModelPose(const RenderXProbe &probe, double car_cm[3],
                         double eye_cm[3], double rotation[3],
                         std::int64_t &timestamp_ns) noexcept {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t first =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if ((first & 1U) != 0 || !probe.route_b_model_valid.load(
                                     std::memory_order_relaxed)) {
      continue;
    }
    double local_car[3]{};
    double local_eye[3]{};
    double local_rotation[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      local_car[axis] =
          probe.route_b_model_car_cm[axis].load(std::memory_order_relaxed);
      local_eye[axis] =
          probe.route_b_model_eye_cm[axis].load(std::memory_order_relaxed);
      local_rotation[axis] =
          probe.route_b_model_rotation[axis].load(std::memory_order_relaxed);
    }
    const std::int64_t timestamp =
        probe.route_b_model_ns.load(std::memory_order_relaxed);
    const std::uint64_t second =
        probe.route_b_model_sequence.load(std::memory_order_acquire);
    if (first != second || timestamp == 0 || !std::isfinite(local_car[0]) ||
        !std::isfinite(local_car[1]) || !std::isfinite(local_car[2]) ||
        !std::isfinite(local_eye[0]) || !std::isfinite(local_eye[1]) ||
        !std::isfinite(local_eye[2]) || !std::isfinite(local_rotation[0]) ||
        !std::isfinite(local_rotation[1]) ||
        !std::isfinite(local_rotation[2])) {
      continue;
    }
    const double dx = local_eye[0] - local_car[0];
    const double dy = local_eye[1] - local_car[1];
    const double dz = local_eye[2] - local_car[2];
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
        dx * dx + dy * dy + dz * dz > 5000.0 * 5000.0) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = local_car[axis];
      eye_cm[axis] = local_eye[axis];
      rotation[axis] = local_rotation[axis];
    }
    timestamp_ns = timestamp;
    return true;
  }
  return false;
}

// The NFS camera is still the model's camera: the render transform only
// supplies this frame's car translation. Moving the model pose by that delta
// keeps the NFS offset/attitude (drift swing, speed dolly, roll) while making
// the camera and the drawn car share one render frame. If the model has not
// produced a pose yet, the old rigid mount is the safe fallback.
bool BuildRouteBPoseFromModel(const RenderXProbe &probe,
                              const double car_cm[3],
                              const double forward[3], double eye[3],
                              double rotation[3]) noexcept {
  double model_car[3]{};
  double model_eye[3]{};
  double model_rotation[3]{};
  std::int64_t model_ns = 0;
  if (!ReadRouteBModelPose(probe, model_car, model_eye, model_rotation,
                           model_ns)) {
    return false;
  }
  const std::int64_t age = NowNanoseconds() - model_ns;
  if (age < 0 || age > 100'000'000) {
    return false;
  }
  // `model_eye` already contains the body-frame slope from the chase model.
  // Only its translation moves to this render frame; applying the pitch again
  // here would double the slope. The raw proxy translation is the ground, and
  // the model's car origin is the fixed ground too, so no +70/-70 conversion
  // is needed here.
  const double fresh_car[3] = {car_cm[0], car_cm[1], car_cm[2]};
  double delta[3]{};
  double distance_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    delta[axis] = fresh_car[axis] - model_car[axis];
    distance_squared += delta[axis] * delta[axis];
  }
  if (!std::isfinite(distance_squared) ||
      distance_squared > kVehicleProxyMaxOffsetCm * kVehicleProxyMaxOffsetCm) {
    return false;
  }
  const double distance_scale = nfs2015_chase::Clamp(
      probe.route_b_model_distance_scale.load(std::memory_order_relaxed),
      0.05, 10.0);
  const double height_scale = nfs2015_chase::Clamp(
      probe.route_b_model_height_scale.load(std::memory_order_relaxed),
      0.05, 10.0);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double model_offset = model_eye[axis] - model_car[axis];
    const double factor = axis == 2 ? height_scale : distance_scale;
    eye[axis] = fresh_car[axis] + model_offset * factor;
    rotation[axis] = model_rotation[axis];
  }
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]) &&
         std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
         std::isfinite(forward[2]);
}

bool BuildRouteBPose(const RenderXProbe &probe, const bool follow_pitch,
                     double eye[3], double rotation[3],
                     std::int64_t &age_ns) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  std::uintptr_t proxy{};
  std::int64_t transform_ns{};
  if (!ReadRouteBSnapshot(probe, car_cm, forward, proxy, transform_ns)) {
    return false;
  }
  const std::int64_t now = NowNanoseconds();
  age_ns = now - transform_ns;
  if (age_ns < 0 || age_ns > kVehicleRenderTransformMaxAgeNs) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

// The pre-visibility pass can run before the render transform notification
// publishes this frame's proxy matrix. The proxy itself, however, is already
// the object the renderer draws the car from. Read its current LocalToWorld
// directly (not as a prediction): if the matrix has already advanced, this
// keeps route B on the exact same car frame as the draw; if it has not, the
// snapshot fallback is used.
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept;

// The Game thread already published the exact NFS camera pose it is using
// (`CameraViewPointDetour`). Applying that pose before visibility avoids
// rebuilding a camera from a car transform that the render thread has not
// received yet: at this point the transform snapshot is often one frame old,
// while the presenter pose is the camera the game itself would draw with.
//
// The presenter pose is translated by the difference between the car it was
// built from and the car the renderer is about to draw. That is a same-frame
// anchor, not a speed/prediction compensation: no extrapolation, no latency
// estimate, only the transform read from the watched proxy on this frame.
bool BuildRouteBPoseFromPresenter(Context &context, const RenderXProbe &probe,
                                  double eye[3],
                                  double rotation[3]) noexcept {
  double presenter_car[3]{};
  float lens = 0.0F;
  bool fov_enabled = false;
  if (!ReadPresenterPose(context, eye, rotation, &lens, &fov_enabled,
                         presenter_car)) {
    return false;
  }
  double fresh_car[3]{};
  double forward[3]{};
  if (ReadRouteBDirectCar(probe, fresh_car, forward)) {
    // Move the whole presenter pose onto this frame's VehicleMesh transform.
    // Aligning only X/Y left the camera on the Game tick's Z, so on a slope
    // the drawn car moved vertically under a camera that did not: visible
    // up/down bobbing. Z must come from the same ComponentToWorld as X/Y.
    for (std::size_t axis = 0; axis < 3; ++axis) {
      eye[axis] += fresh_car[axis] - presenter_car[axis];
    }
    // Both poses are now built around the same fixed ground origin; the old
    // origin-offset trim has no user-facing counterpart to re-apply here.
  }
  return std::isfinite(eye[0]) && std::isfinite(eye[1]) &&
         std::isfinite(eye[2]) && std::isfinite(rotation[0]) &&
         std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
}

// Read the car transform the renderer is using. The VehicleMesh component
// transform is the authoritative value at the Game camera point: the probe
// showed parent-final = 0.00 cm while the watched proxy was still one frame
// behind. Prefer that component; the proxy remains the fallback for frames
// where the parent pointer is not available. Returns the raw translation
// (without the anchor centre height).
bool ReadRouteBDirectCar(const RenderXProbe &probe, double car_cm[3],
                         double forward[3]) noexcept {
  const std::uintptr_t parent =
      probe.vehicle_parent.load(std::memory_order_relaxed);
  if (parent != 0) {
    const auto *translation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformTranslationOffset);
    const auto *orientation = reinterpret_cast<const double *>(
        parent + kSceneComponentTransformOffset);
    const double qx = orientation[0];
    const double qy = orientation[1];
    const double qz = orientation[2];
    const double qw = orientation[3];
    const double norm2 = qx * qx + qy * qy + qz * qz + qw * qw;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      car_cm[axis] = translation[axis];
    }
    if (std::isfinite(car_cm[0]) && std::isfinite(car_cm[1]) &&
        std::isfinite(car_cm[2]) && norm2 > 0.81 && norm2 < 1.21) {
      const double inverse_norm = 1.0 / std::sqrt(norm2);
      const double x = qx * inverse_norm;
      const double y = qy * inverse_norm;
      const double z = qz * inverse_norm;
      const double w = qw * inverse_norm;
      // Rotate the local +X (nose) axis by the ComponentToWorld quaternion.
      const double tx = 0.0;
      const double ty = 2.0 * z;
      const double tz = -2.0 * y;
      forward[0] = 1.0 + w * tx + (y * tz - z * ty);
      forward[1] = w * ty + (z * tx - x * tz);
      forward[2] = w * tz + (x * ty - y * tx);
      const double length_squared = forward[0] * forward[0] +
                                    forward[1] * forward[1] +
                                    forward[2] * forward[2];
      if (std::isfinite(forward[0]) && std::isfinite(forward[1]) &&
          std::isfinite(forward[2]) && length_squared > 1e-8) {
        const double inverse_length = 1.0 / std::sqrt(length_squared);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          forward[axis] *= inverse_length;
        }
        return true;
      }
    }
  }

  double snapshot_car[3]{};
  double snapshot_forward[3]{};
  std::uintptr_t snapshot_proxy{};
  std::int64_t snapshot_ns{};
  const bool have_snapshot =
      ReadRouteBSnapshot(probe, snapshot_car, snapshot_forward,
                         snapshot_proxy, snapshot_ns);
  const std::uintptr_t watched =
      probe.watched_proxy.load(std::memory_order_relaxed);
  if (watched == 0) {
    return false;
  }
  const bool snapshot_matches = have_snapshot && snapshot_proxy == watched;

  const auto *matrix = reinterpret_cast<const double *>(watched + 0xE0);
  const auto *translation = reinterpret_cast<const double *>(
      watched + kVehicleProxyTranslationOffset);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    car_cm[axis] = translation[axis];
    forward[axis] = matrix[axis];
  }
  if (!std::isfinite(car_cm[0]) || !std::isfinite(car_cm[1]) ||
      !std::isfinite(car_cm[2]) || !std::isfinite(forward[0]) ||
      !std::isfinite(forward[1]) || !std::isfinite(forward[2])) {
    return false;
  }
  const double length_squared = forward[0] * forward[0] +
                                forward[1] * forward[1] +
                                forward[2] * forward[2];
  if (!(length_squared > 1e-8)) {
    return false;
  }
  const double inverse_length = 1.0 / std::sqrt(length_squared);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    forward[axis] *= inverse_length;
  }

  double reference_cm[3]{};
  bool have_reference = false;
  double limit_cm = 0.0;
  if (snapshot_matches) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] = snapshot_car[axis];
    }
    have_reference = true;
    limit_cm = kVehicleProxyMaxOffsetCm;
  } else if (probe.vehicle_anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.vehicle_anchor_cm[axis].load(std::memory_order_relaxed);
    }
    reference_cm[2] -= kVehicleAnchorCenterHeightCm;
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  } else if (probe.anchor_valid.load(std::memory_order_acquire)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_cm[axis] =
          probe.anchor_cm[axis].load(std::memory_order_relaxed);
    }
    have_reference = true;
    limit_cm = kViewHistorySnapDistanceCm;
  }
  if (have_reference) {
    double distance_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = car_cm[axis] - reference_cm[axis];
      distance_squared += delta * delta;
    }
    if (!std::isfinite(distance_squared) ||
        distance_squared > limit_cm * limit_cm) {
      return false;
    }
  }
  return true;
}

bool BuildRouteBPoseDirect(const RenderXProbe &probe, const bool follow_pitch,
                           double eye[3],
                           double rotation[3]) noexcept {
  double car_cm[3]{};
  double forward[3]{};
  if (!ReadRouteBDirectCar(probe, car_cm, forward)) {
    return false;
  }
  return BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) ||
         BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation);
}

bool IsRouteBFullScreenView(void *view) noexcept {
  if (view == nullptr ||
      reinterpret_cast<std::uintptr_t>(view) < 0x10000) {
    return false;
  }
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(
                  reinterpret_cast<std::uintptr_t>(view) +
                  kViewInfoRectOffset),
              sizeof(rect));
  return rect[2] >= kRouteBMinViewWidth &&
         rect[3] >= kRouteBMinViewHeight;
}

double Determinant4x4(const double matrix[16]) noexcept {
  return matrix[0] *
             (matrix[5] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) +
              matrix[7] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13])) -
         matrix[1] *
             (matrix[4] * (matrix[10] * matrix[15] -
                           matrix[11] * matrix[14]) -
              matrix[6] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12])) +
         matrix[2] *
             (matrix[4] * (matrix[9] * matrix[15] -
                           matrix[11] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[15] -
                           matrix[11] * matrix[12]) +
              matrix[7] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12])) -
         matrix[3] *
             (matrix[4] * (matrix[9] * matrix[14] -
                           matrix[10] * matrix[13]) -
              matrix[5] * (matrix[8] * matrix[14] -
                           matrix[10] * matrix[12]) +
              matrix[6] * (matrix[8] * matrix[13] -
                           matrix[9] * matrix[12]));
}

// A hash match proves the offsets were verified against this profile; it does
// not prove that a particular FViewInfo is still a live full-screen view. Check
// the fields route B is about to read and write before touching them, so a
// changed view layout fails closed instead of writing a camera pose into an
// unrelated object.
bool IsRouteBViewWritable(void *view) noexcept {
  if (!IsRouteBFullScreenView(view)) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);
  double location[3]{};
  double rotation[3]{};
  double projection[16]{};
  float fov = 0.0F;
  std::memcpy(location,
              reinterpret_cast<const void *>(address + kViewLocationOffset),
              sizeof(location));
  std::memcpy(rotation,
              reinterpret_cast<const void *>(address + kViewRotationOffset),
              sizeof(rotation));
  std::memcpy(projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(projection));
  std::memcpy(&fov, reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(fov));
  for (double value : location) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : rotation) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (double value : projection) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (!(std::fabs(Determinant4x4(projection)) > 1e-12) ||
      !(fov > 1.0F) || !(fov < 179.0F)) {
    return false;
  }
  return true;
}

// Write the final route B pose into one view, before visibility and before
// PrepareViewStateForVisibility snapshots the temporal history. This is the
// only place the camera may be moved: a post-Prepare write would leave the
// culling frustum, motion vectors and TAA history describing a different
// camera, which is visible as a one-frame ghost/flash.
//
// The FViewMatrices constructor is used instead of scaling the stored
// projection in place. In-place scaling leaves InvProjectionMatrix,
// ProjectionNoAAMatrix and ScreenToClipMatrix describing the game's lens; TAA,
// SSR and the depth reconstruction read those on the same frame. The
// constructor refreshes every derived matrix from the projection and rotation
// the renderer actually uses.
bool ApplyRouteBPoseToView(void *view, const double eye[3],
                           const double rotation[3],
                           const RenderXProbe &probe) noexcept {
  if (view == nullptr || !IsRouteBFullScreenView(view) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return false;
  }
  ViewMatricesCtorFn ctor =
      g_view_matrices_ctor.load(std::memory_order_acquire);
  if (ctor == nullptr) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(view);

  RouteBViewInit init{};
  std::memcpy(init.projection,
              reinterpret_cast<const void *>(address + kViewMatricesOffset),
              sizeof(init.projection));

  // The model lens replaces the game's lens for the same frame. Keep the
  // depth range and TAA jitter terms untouched; only the X/Y scale changes.
  const float desired =
      probe.route_b_model_lens.load(std::memory_order_relaxed);
  float current = 0.0F;
  std::memcpy(&current,
              reinterpret_cast<const void *>(address + kViewFovOffset),
              sizeof(current));
  bool lens_changed = false;
  if (std::isfinite(current) && current > 1.0F && current < 179.0F &&
      desired > 1.0F && desired < 179.0F &&
      std::fabs(current - desired) >= 0.01F) {
    const double current_radians =
        static_cast<double>(current) * kDegreesToRadians;
    const double desired_radians =
        static_cast<double>(desired) * kDegreesToRadians;
    const double denominator = std::tan(desired_radians * 0.5);
    if (std::fabs(denominator) > 1e-6) {
      const double factor = std::tan(current_radians * 0.5) / denominator;
      if (std::isfinite(factor) && factor > 0.0) {
        init.projection[0][0] *= factor;
        init.projection[1][1] *= factor;
        lens_changed = true;
      }
    }
  }

  MakeViewRotationMatrix(rotation[0], rotation[1], rotation[2],
                         init.view_rotation);
  init.view_origin[0] = eye[0];
  init.view_origin[1] = eye[1];
  init.view_origin[2] = eye[2];
  std::int32_t rect[4]{};
  std::memcpy(rect,
              reinterpret_cast<const void *>(address + kViewInfoRectOffset),
              sizeof(rect));
  std::memcpy(init.constrained_rect, rect, sizeof(init.constrained_rect));
  init.stereo_pass = 0;
  init.use_faux_ortho = false;

  try {
    ctor(reinterpret_cast<void *>(address + kViewMatricesOffset), &init);
  } catch (...) {
    return false;
  }

  if (lens_changed) {
    std::memcpy(reinterpret_cast<void *>(address + kViewFovOffset), &desired,
                sizeof(desired));
    std::memcpy(reinterpret_cast<void *>(address + kViewDesiredFovOffset),
                &desired, sizeof(desired));
  }
  std::memcpy(reinterpret_cast<void *>(address + kViewLocationOffset), eye,
              sizeof(eye[0]) * 3);
  std::memcpy(reinterpret_cast<void *>(address + kViewRotationOffset),
              rotation, sizeof(rotation[0]) * 3);

  // Keep ShadowViewMatrices and ViewFrustum in sync; the constructor only
  // rebuilt the view matrices. SetupViewFrustum then refreshes both the culling
  // frustum and its origin for the non-stereo visibility path.
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update != nullptr) {
    update(view);
  }
  SceneViewSetupFrustumFn setup_frustum =
      g_scene_view_setup_frustum.load(std::memory_order_acquire);
  if (setup_frustum != nullptr) {
    setup_frustum(view);
  }
  return true;
}

// Apply route B before FSceneRenderer::PrepareViewStateForVisibility. At this
// point the view matrices are still the game's, but visibility tasks have not
// been launched and the temporal history has not been advanced. Rebuilding the
// matrices here makes the frustum and the TAA history describe the same chase
// camera the renderer will later use.
bool ApplyRouteBEarly(Context &context, void *renderer) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!probe.version_gate.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // A failed attempt this frame must not leave an older frame's view list
  // looking fresh to the diagnostics.
  probe.route_b_early_views_valid.store(false, std::memory_order_release);
  probe.route_b_early_view_count.store(0, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(0, std::memory_order_relaxed);
  for (auto &state : probe.route_b_early_view_states) {
    state.store(0, std::memory_order_relaxed);
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    probe.prepare_thread_mismatch.fetch_add(1, std::memory_order_relaxed);
  }
  if (renderer == nullptr) {
    return false;
  }
  if (!probe.anchor_enabled.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Only route B while the driving camera owns the view. After the player
  // exits the vehicle `PublishInactive` clears this flag, but the presenter
  // pose remains cached; without this gate the early hook kept writing that
  // last vehicle pose, freezing the on-foot camera at the parked car.
  if (!context.view_active.load(std::memory_order_acquire)) {
    probe.prepare_anchor_off.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  SceneViewUpdateMatrixFn update =
      g_scene_view_update_matrix.load(std::memory_order_acquire);
  if (update == nullptr) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  const auto renderer_address = reinterpret_cast<std::uintptr_t>(renderer);
  void *views = nullptr;
  std::int32_t count = 0;
  std::memcpy(&views,
              reinterpret_cast<const void *>(renderer_address +
                                             kSceneRendererAllViewsOffset),
              sizeof(views));
  std::memcpy(&count,
              reinterpret_cast<const void *>(
                  renderer_address + kSceneRendererAllViewCountOffset),
              sizeof(count));
  if (views == nullptr || count <= 0 || count > kRouteBMaxEarlyViews) {
    probe.prepare_no_views.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  double eye[3]{};
  double rotation[3]{};
  std::int64_t age_ns = 0;
  const std::int64_t now_ns = NowNanoseconds();
  const bool follow_pitch = RouteBFollowPitch(context);
  // The pre-visibility call can run one render frame before the transform
  // notification is published; the 2.7 log showed a 23 ms snapshot age here.
  // Using that snapshot makes the camera ride the previous car transform and
  // the car visibly shakes. The proxy itself is already the object the
  // renderer draws this frame, so read it directly first. The notification
  // snapshot is only a fallback when the proxy pointer is not available.
  // The presenter pose is the artifact-free path: it is the NFS camera the
  // Game thread already produced, and it is aligned horizontally here with
  // the fresh VehicleMesh transform. Its Z carries the model height and the
  // origin-offset slider. The model-pose path remains a fallback.
  bool have_pose =
      BuildRouteBPoseFromPresenter(context, probe, eye, rotation);
  if (have_pose) {
    probe.route_b_early_presenter.fetch_add(1, std::memory_order_relaxed);
  } else if (BuildRouteBPoseDirect(probe, follow_pitch, eye, rotation)) {
    probe.route_b_early_direct.fetch_add(1, std::memory_order_relaxed);
    have_pose = true;
  } else if (BuildRouteBPose(probe, follow_pitch, eye, rotation, age_ns)) {
    have_pose = true;
  }
  if (!have_pose) {
    const std::int64_t held_age = now_ns - probe.route_b_held_ns;
    if (probe.route_b_held_pose_valid && held_age >= 0 &&
        held_age <= kRouteBHeldPoseNs) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        eye[axis] = probe.route_b_held_eye_cm[axis];
        rotation[axis] = probe.route_b_held_rotation[axis];
      }
      probe.route_b_early_held.fetch_add(1, std::memory_order_relaxed);
      have_pose = true;
    }
  }
  if (!have_pose) {
    probe.prepare_pose_fail.fetch_add(1, std::memory_order_relaxed);
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    if (context.presenter_burnout_blend.load(std::memory_order_relaxed) >
        0.001) {
      context.presenter_burnout_fallbacks.fetch_add(1,
                                                    std::memory_order_relaxed);
    }
    return false;
  }
  // Keep the last applied pose for the one-frame hold above. The pose is in
  // world centimetres, like the view fields it is copied into.
  probe.route_b_held_pose_valid = true;
  probe.route_b_held_ns = now_ns;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    probe.route_b_held_eye_cm[axis] = eye[axis];
    probe.route_b_held_rotation[axis] = rotation[axis];
  }

  auto *view_array = static_cast<void *const *>(views);
  std::uint64_t applied = 0;
  for (std::int32_t index = 0; index < count; ++index) {
    void *view = view_array[index];
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    // One authoritative write, before visibility: pose, lens and every derived
    // matrix (including InvProjection) are rebuilt together.
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    if (applied <
        sizeof(probe.route_b_early_views) /
            sizeof(probe.route_b_early_views[0])) {
      probe.route_b_early_views[applied].store(
          reinterpret_cast<std::uintptr_t>(view), std::memory_order_relaxed);
      std::uintptr_t state = 0;
      std::memcpy(&state,
                  reinterpret_cast<const void *>(view_address +
                                                 kViewStateOffset),
                  sizeof(state));
      probe.route_b_early_view_states[applied].store(
          state, std::memory_order_relaxed);
    }
    ++applied;
  }

  probe.prepare_fullscreen_views.fetch_add(applied, std::memory_order_relaxed);
  if (applied == 0) {
    probe.route_b_early_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  probe.route_b_early_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_early_last_age_us.store(
      static_cast<std::uint64_t>(age_ns / 1000), std::memory_order_relaxed);
  const std::int64_t prepared_ns = NowNanoseconds();
  probe.route_b_early_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_early_view_count.store(
      static_cast<std::uint32_t>(applied), std::memory_order_release);
  probe.route_b_prepare_ns.store(prepared_ns, std::memory_order_relaxed);
  probe.route_b_prepare_renderer.store(
      reinterpret_cast<std::uintptr_t>(renderer), std::memory_order_release);
  probe.route_b_prepare_generation.fetch_add(1, std::memory_order_acq_rel);
  probe.route_b_early_views_valid.store(true, std::memory_order_release);

  // TEMP read-only timing diagnostics: snapshot the car/camera sources at the
  // exact point where the early camera is written. No writes to game state.
  {
    probe.timing_frame.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_count.fetch_add(1, std::memory_order_relaxed);
    probe.timing_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
    probe.timing_late_seen.store(false, std::memory_order_release);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      probe.timing_early_view[axis].store(eye[axis],
                                          std::memory_order_relaxed);
    }
    double car_cm[3]{};
    double forward[3]{};
    if (ReadRouteBDirectCar(probe, car_cm, forward)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_early_car[axis].store(car_cm[axis],
                                           std::memory_order_relaxed);
      }
    }
    double presenter_car[3]{};
    double presenter_eye[3]{};
    double presenter_rotation[3]{};
    float presenter_lens = 0.0F;
    bool presenter_fov = false;
    if (ReadPresenterPose(context, presenter_eye, presenter_rotation,
                          &presenter_lens, &presenter_fov,
                          presenter_car)) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        probe.timing_presenter_car[axis].store(
            presenter_car[axis], std::memory_order_relaxed);
      }
    }
  }
  return true;
}

// Experimental: apply the fresh render proxy transform after the car's
// OnTransformChanged has updated LocalToWorld. This is deliberately a
// separate writer from ApplyRouteBEarly; it is used to test whether the
// transform notification is still before the visibility task data consumes
// the view.
// Experimental transform-hook writer. It is intentionally guarded by the
// view list captured by ApplyRouteBEarly and by the prepare generation, so it
// writes at most once per prepared frame. This is not a late uniform fallback.
void ApplyRouteBAtRenderTransform(Context &context, const double car_cm[3],
                                  const double forward[3]) noexcept {
  RenderXProbe &probe = context.render_x;
  if (!context.started.load(std::memory_order_acquire) ||
      !probe.version_gate.load(std::memory_order_acquire)) {
    return;
  }
  const std::uint32_t render_thread =
      probe.render_thread_id.load(std::memory_order_acquire);
  if (render_thread != 0 &&
      static_cast<std::uint32_t>(GetCurrentThreadId()) != render_thread) {
    return;
  }
  if (!probe.route_b_early_views_valid.load(std::memory_order_acquire)) {
    return;
  }
  const std::uintptr_t prepared_renderer =
      probe.route_b_prepare_renderer.load(std::memory_order_relaxed);
  const std::uintptr_t active_renderer =
      probe.route_b_active_renderer.load(std::memory_order_acquire);
  if (prepared_renderer == 0 ||
      (active_renderer != 0 && active_renderer == prepared_renderer)) {
    return;
  }
  const std::uint64_t generation =
      probe.route_b_prepare_generation.load(std::memory_order_acquire);
  const std::uint64_t last_generation =
      probe.route_b_transform_generation.load(std::memory_order_acquire);
  if (generation == 0 || generation == last_generation) {
    return;
  }
  const std::int64_t prepared_ns =
      probe.route_b_prepare_ns.load(std::memory_order_relaxed);
  const std::int64_t now_ns = NowNanoseconds();
  if (prepared_ns == 0 || now_ns < prepared_ns ||
      now_ns - prepared_ns > 100'000'000) {
    return;
  }
  const std::uint32_t count =
      probe.route_b_early_view_count.load(std::memory_order_acquire);
  if (count == 0 ||
      count > sizeof(probe.route_b_early_views) /
                  sizeof(probe.route_b_early_views[0])) {
    return;
  }
  double eye[3]{};
  double rotation[3]{};
  const bool follow_pitch = RouteBFollowPitch(context);
  if (!BuildRouteBPoseFromModel(probe, car_cm, forward, eye, rotation) &&
      !BuildRouteBPoseFromCar(car_cm, forward, follow_pitch, eye, rotation)) {
    return;
  }
  std::uint64_t applied = 0;
  for (std::uint32_t index = 0; index < count; ++index) {
    void *view = reinterpret_cast<void *>(
        probe.route_b_early_views[index].load(std::memory_order_relaxed));
    if (!IsRouteBFullScreenView(view)) {
      continue;
    }
    const auto view_address = reinterpret_cast<std::uintptr_t>(view);
    std::uintptr_t state = 0;
    std::memcpy(&state,
                reinterpret_cast<const void *>(view_address +
                                               kViewStateOffset),
                sizeof(state));
    if (state == 0 ||
        state != probe.route_b_early_view_states[index].load(
                     std::memory_order_relaxed)) {
      continue;
    }
    if (!ApplyRouteBPoseToView(view, eye, rotation, probe)) {
      continue;
    }
    ++applied;
  }
  if (applied == 0) {
    return;
  }
  probe.route_b_render_applied.fetch_add(applied, std::memory_order_relaxed);
  probe.route_b_transform_generation.store(generation,
                                            std::memory_order_release);
  probe.route_b_early_last_age_us.store(0, std::memory_order_relaxed);
  probe.route_b_early_ns.store(NowNanoseconds(), std::memory_order_relaxed);
}

}  // namespace nfs2015_chase_cam::detail
