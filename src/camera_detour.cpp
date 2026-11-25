// The Game thread's camera manager detour: replace the view point the game
// camera produced with the snapshot the chase-camera model published, and set
// the optional lens on the same out-parameter. No allocation, no lock of our
// own, no service call: the detour only copies what Update already published.
#include "internal.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace nfs2015_chase_cam::detail {

// ---------------------------------------------------------------------------
// Camera view-point detour
// ---------------------------------------------------------------------------

void ANOMALY_CALL CameraViewPointDetour(void *object, double *location,
                                        double *rotation) noexcept {
  Context *context = g_active.load(std::memory_order_acquire);
  if (context != nullptr &&
      !context->started.load(std::memory_order_acquire)) {
    context = nullptr;
  }
  if (context != nullptr) {
    context->diag.detour_calls.fetch_add(1, std::memory_order_relaxed);
    context->diag.detour_calls_window.fetch_add(1,
                                               std::memory_order_relaxed);
    NoteDetourThread(*context);
  }
  AnomalyGenerationHandleV1 lease{};
  bool leased = false;
  CameraViewPointFn original =
      g_original.load(std::memory_order_acquire);
  try {
    const std::uint64_t hook_id =
        context == nullptr ? 0
                           : context->view_hook_id.load(std::memory_order_acquire);
    if (context != nullptr && hook_id != 0 && HookReady(context->hook)) {
      const AnomalyGenerationHandleV1 handle{
          hook_id, context->view_hook_generation.load(
                       std::memory_order_acquire)};
      leased = context->hook
                   ->begin_callback(context->hook->user, handle, &lease)
                   .code == ANOMALY_STATUS_V1_OK;
      if (!leased) {
        context->diag.lease_failures.fetch_add(1,
                                               std::memory_order_relaxed);
      }
    }
  } catch (...) {
    leased = false;
  }

  bool original_called = false;
  try {
    if (original != nullptr) {
      original(object, location, rotation);
      original_called = true;
    }
  } catch (...) {
  }

  try {
    if (context != nullptr && location != nullptr && rotation != nullptr &&
        object != nullptr) {
      const std::uintptr_t manager = reinterpret_cast<std::uintptr_t>(object);
      context->observed_manager.store(manager, std::memory_order_release);
#if defined(NFS2015_VIEWMODE_TRACE)
      {
        // Acceptance probe (plan TG-4): log the manager address once so the
        // view-target offset can be located from outside without guessing.
        static std::atomic_bool manager_logged{false};
        if (!manager_logged.load(std::memory_order_relaxed)) {
          manager_logged.store(true, std::memory_order_relaxed);
          char manager_line[96]{};
          std::snprintf(manager_line, sizeof(manager_line),
                        "camera manager: 0x%llX",
                        static_cast<unsigned long long>(manager));
          Log(*context, ANOMALY_CORE_LOG_LEVEL_V1_INFO,
              std::string_view(manager_line));
        }
      }
#endif
      // Task group 0 probe: the render thread's own read of the host's player
      // chain. Deliberately before the camera work, and independent of it.
      SampleRenderXPhase(*context);
      const bool pose_finite =
          std::isfinite(location[0]) && std::isfinite(location[1]) &&
          std::isfinite(location[2]) && std::isfinite(rotation[0]) &&
          std::isfinite(rotation[1]) && std::isfinite(rotation[2]);
      if (pose_finite) {
        const std::uint64_t observed_sequence =
            context->observed_sequence.load(std::memory_order_relaxed);
        context->observed_sequence.store(observed_sequence + 1U,
                                         std::memory_order_release);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          context->observed_location[axis].store(
              location[axis], std::memory_order_relaxed);
          context->observed_rotation[axis].store(
              rotation[axis], std::memory_order_relaxed);
        }
        context->observed_valid.store(true, std::memory_order_relaxed);
        context->observed_sequence.store(observed_sequence + 2U,
                                         std::memory_order_release);
      }

      const std::uint64_t sequence =
          context->view_sequence.load(std::memory_order_acquire);
      bool active = (sequence & 1U) == 0 &&
                    context->view_active.load(std::memory_order_relaxed);
      if (active) {
        // Plan SS2.4: the game's own tag always wins over the plugin's flag.
        // `view_active` is only cleared on the next Game tick, and a tick is a
        // frame of the chase pose over a first-person (or cutscene) camera. The
        // read is live and happens here, on the thread that is about to draw.
        if (!ChaseCameraGuardAllows(*context)) {
          context->diag.tag_veto_frames.fetch_add(1,
                                                 std::memory_order_relaxed);
          active = false;
        }
      }
      if (active && !leased) {
        // The game camera shows through for this call. Counted separately from
        // the torn read below: this one is the lease, not the snapshot.
        context->diag.detour_no_publish.fetch_add(1,
                                                 std::memory_order_relaxed);
      }
      if (leased && active) {
        context->view_output_frames.fetch_add(1, std::memory_order_relaxed);
        const std::int64_t render_now_ns = NowNanoseconds();
        double present_location[3]{};
        double present_rotation[3]{};
        float present_lens = 0.0F;
        bool present_fov_enabled = false;
        // The Game's own position is still in `location` here: the original
        // ran before us and we have not written yet.
        const double game_eye_cm[3] = {location[0], location[1],
                                       location[2]};
        bool advanced = false;
        if (AdvancePresenter(*context, render_now_ns, game_eye_cm,
                             present_location, present_rotation,
                             &present_lens, &present_fov_enabled, &advanced)) {
          SamplePresenterMotion(*context, render_now_ns, present_location,
                                present_rotation, advanced);
          {
            const bool skip_position =
                context->diagnostic_skip_position.load(
                    std::memory_order_relaxed);
            const bool skip_rotation =
                context->diagnostic_skip_rotation.load(
                    std::memory_order_relaxed);
            // `AdvancePresenter` applies the Game-camera borrow inside its
            // lock while producing the pose; when it returned a cached pose
            // (same frame, or another thread won the lock) the borrow is
            // already reflected there.
            for (std::size_t axis = 0; axis < 3; ++axis) {
              if (!skip_position) {
                location[axis] = static_cast<float>(present_location[axis]);
              }
              if (!skip_rotation) {
                rotation[axis] = static_cast<float>(present_rotation[axis]);
              }
            }
          }

          const std::uintptr_t pov =
              context->camera_pov_manager.load(std::memory_order_acquire) ==
                      manager
                  ? context->camera_pov.load(std::memory_order_acquire)
                  : 0;
          if (pov != 0 && present_fov_enabled) {
            if (std::isfinite(present_lens) && present_lens > 1.0F &&
                present_lens < 179.0F) {
              std::uintptr_t address{};
              if (AddAddress(pov, kCameraPovFovOffset, address)) {
                *reinterpret_cast<float *>(address) = present_lens;
              }
            }
          } else if (context->fov_restore.exchange(
                         false, std::memory_order_acq_rel)) {
            const std::uintptr_t restore_pov =
                context->camera_pov_manager.load(std::memory_order_acquire) ==
                        manager
                    ? context->camera_pov.load(std::memory_order_acquire)
                    : 0;
            const float game_lens =
                context->game_fov.load(std::memory_order_acquire);
            std::uintptr_t address{};
            if (restore_pov != 0 && std::isfinite(game_lens) &&
                game_lens > 1.0F && game_lens < 179.0F &&
                AddAddress(restore_pov, kCameraPovFovOffset, address)) {
              *reinterpret_cast<float *>(address) = game_lens;
            }
          }
        }
      } else if (context->fov_restore.exchange(false,
                                               std::memory_order_acq_rel)) {
        const std::uintptr_t pov =
            context->camera_pov_manager.load(std::memory_order_acquire) ==
                    manager
                ? context->camera_pov.load(std::memory_order_acquire)
                : 0;
        const float game_lens =
            context->game_fov.load(std::memory_order_acquire);
        std::uintptr_t address{};
        if (pov != 0 && std::isfinite(game_lens) && game_lens > 1.0F &&
            game_lens < 179.0F &&
            AddAddress(pov, kCameraPovFovOffset, address)) {
          *reinterpret_cast<float *>(address) = game_lens;
        }
      }
    }
  } catch (...) {
  }

  // Same rule as the transform detour: release a lease that was actually
  // acquired, even while the host is already stopping the hook service.
  if (leased && context != nullptr && context->hook != nullptr &&
      context->hook->end_callback != nullptr) {
    static_cast<void>(
        context->hook->end_callback(context->hook->user, lease));
  }
}

}  // namespace nfs2015_chase_cam::detail
