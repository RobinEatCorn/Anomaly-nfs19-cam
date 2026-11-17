#pragma once

// Parked-orbit main-UI hide state machine.
//
// This header is intentionally pure: no Anomaly SDK, no Windows, no clock and
// no game calls. The Game-thread backend feeds it the presenter's published
// orbit state, the current visibility readback, and the result of the action it
// requested on the previous tick. `Update` returns at most one command:
//
//   kHide     -> call W_OverallUILayout_C::HideAllLayer
//   kRestore  -> call W_OverallUILayout_C::RevertLayerShow
//
// The backend executes that command and reports success through the next
// `invoke_ok` input. The state machine is deliberately conservative: if the
// readback is unavailable it still hides on entry, but always restores on exit
// (Q2=C), so a read failure cannot leave the HUD hidden.
//
// Q2=A semantics:
//   * Enter while already hidden: remember `was_hidden`, do not touch the UI,
//     and do not restore on exit.
//   * Enter while visible: hide once, then poll the readback. If the player
//     opens the UI while orbiting, mark `intervened`, stop forcing, and leave
//     the player's choice alone on exit.
//   * Exit while still hidden by us: restore, but only while the readback still
//     says hidden (or is unavailable). If the player already opened it, do not
//     restore.
//
// The state machine is intentionally independent of `OrbitShot`'s geometry;
// only its phase enum is used. Enter and Orbit keep the HUD hidden. Exit is the
// camera pull-out transition, so the HUD is restored on its first tick and is
// already visible while the exit blend plays.

#include "orbit_shot.hpp"

#include <cmath>
#include <cstdint>

namespace nfs2015_chase {

// Readback polling interval while the scene is hidden by us. The UI function
// itself is cheap, but this keeps the game-thread work off the camera path and
// gives a manual toggle time to settle.
inline constexpr double kOrbitUiHidePollSeconds = 0.25;

enum class OrbitUiHideAction : std::uint8_t {
  kNone = 0,
  kHide = 1,
  kRestore = 2,
};

// Internal phase, also useful in diagnostics/logs. It is not the same as the
// Q2 diagnostic enum in the plan; the runtime can map it when publishing state.
enum class OrbitUiHidePhase : std::uint8_t {
  kIdle = 0,
  kHidePending = 1,
  kHidden = 2,
  kRestorePending = 3,
  kIntervened = 4,
  kAlreadyHidden = 5,
  kFailed = 6,
};

[[nodiscard]] inline const char* OrbitUiHidePhaseName(
    const OrbitUiHidePhase phase) noexcept {
  switch (phase) {
    case OrbitUiHidePhase::kIdle:
      return "Idle";
    case OrbitUiHidePhase::kHidePending:
      return "HidePending";
    case OrbitUiHidePhase::kHidden:
      return "Hidden";
    case OrbitUiHidePhase::kRestorePending:
      return "RestorePending";
    case OrbitUiHidePhase::kIntervened:
      return "Intervened";
    case OrbitUiHidePhase::kAlreadyHidden:
      return "AlreadyHidden";
    case OrbitUiHidePhase::kFailed:
      return "Failed";
  }
  return "Idle";
}

// True while the orbit still owns the gameplay HUD. The exit transition is
// deliberately excluded: the restore starts as soon as `OrbitShot` enters
// `kExit`, so the HUD fades back in during the pull-out rather than after it.
[[nodiscard]] inline bool OrbitPhaseHidesHud(const OrbitPhase phase) noexcept {
  switch (phase) {
    case OrbitPhase::kEnter:
    case OrbitPhase::kOrbit:
      return true;
    case OrbitPhase::kIdle:
    case OrbitPhase::kExit:
      return false;
  }
  return false;
}

struct OrbitUiHideInput final {
  // Presenter-published orbit state. Only Enter and Orbit keep the UI hidden.
  OrbitPhase orbit_mode{OrbitPhase::kIdle};

  bool orbit_enabled{true};
  bool hide_ui_enabled{true};
  // The orbit may be latched while the player is walking away; the backend
  // passes the presenter's `in_vehicle` answer here.
  bool driving{true};
  // Z can leave the chase slot for the game's third/first-person camera while
  // the orbit model is still winding down. In that state the HUD belongs to
  // the normal game view and must be restored.
  bool chase_view{true};

  // Result of the visibility probe. `readback_valid=false` means the probe
  // failed; `readback_visible` is then ignored.
  bool readback_visible{false};
  bool readback_valid{false};

  // Result of the action returned by the previous Update. Ignored when no
  // action is pending. Defaults to true so a caller that has no error path can
  // treat the command as accepted.
  bool invoke_ok{true};

  double now_seconds{0.0};
};

struct OrbitUiHideState final {
  // Entry/exit decisions.
  bool was_hidden{false};
  bool hidden_by_us{false};
  bool intervened{false};

  // Readback support and the last successful sample.
  bool readback_supported{false};
  bool last_visible{false};
  double next_poll{0.0};

  // Phase for logs/diagnostics. `hidden_by_us` remains the authority for
  // whether an exit must restore; phase is descriptive.
  OrbitUiHidePhase phase{OrbitUiHidePhase::kIdle};

  // Diagnostics counters; the backend publishes these in TG-15.
  bool hide_failed{false};
  std::uint32_t hide_requests{0};
  std::uint32_t restore_requests{0};
  std::uint32_t errors{0};
};

class OrbitUiHide final {
 public:
  explicit OrbitUiHide(
      const double poll_seconds = kOrbitUiHidePollSeconds) noexcept
      : poll_seconds_(std::isfinite(poll_seconds) && poll_seconds > 0.0
                          ? poll_seconds
                          : kOrbitUiHidePollSeconds) {}

  // Clear the episode state and return a restore command if the UI may still
  // be hidden by us. Callers use this for Reset/epoch changes. The command is
  // idempotent; executing it when nothing was hidden is harmless.
  [[nodiscard]] OrbitUiHideAction Reset() noexcept {
    const bool may_be_hidden =
        state_.hidden_by_us || pending_ == OrbitUiHideAction::kHide;
    state_ = {};
    pending_ = OrbitUiHideAction::kNone;
    next_restore_ = 0.0;
    entered_ = false;
    return may_be_hidden ? OrbitUiHideAction::kRestore
                         : OrbitUiHideAction::kNone;
  }

  [[nodiscard]] OrbitUiHideAction Update(
      const OrbitUiHideInput& input) noexcept {
    const double now =
        std::isfinite(input.now_seconds) ? input.now_seconds : 0.0;
    const bool want_hide = input.driving && input.chase_view &&
                           input.orbit_enabled && input.hide_ui_enabled &&
                           OrbitPhaseHidesHud(input.orbit_mode);

    if (input.readback_valid) {
      state_.readback_supported = true;
      state_.last_visible = input.readback_visible;
    }

    // Consume the result of the command returned on the previous tick. The
    // hide is treated as ours until a failure is reported, so an exit that
    // happens before the next update still asks for a restore (fail-safe).
    if (pending_ == OrbitUiHideAction::kHide) {
      pending_ = OrbitUiHideAction::kNone;
      if (input.invoke_ok) {
        state_.hidden_by_us = true;
        state_.phase = OrbitUiHidePhase::kHidden;
        state_.last_visible = false;
        state_.next_poll = now + poll_seconds_;
      } else {
        state_.hidden_by_us = false;
        state_.hide_failed = true;
        state_.phase = OrbitUiHidePhase::kFailed;
        ++state_.errors;
      }
    } else if (pending_ == OrbitUiHideAction::kRestore) {
      pending_ = OrbitUiHideAction::kNone;
      if (input.invoke_ok) {
        state_.hidden_by_us = false;
        state_.last_visible = true;
        state_.phase = state_.intervened
                           ? OrbitUiHidePhase::kIntervened
                           : OrbitUiHidePhase::kIdle;
      } else {
        // Keep ownership so a later tick retries the restore. Back off, but
        // never forget that the UI may still be hidden by us.
        state_.phase = OrbitUiHidePhase::kHidden;
        ++state_.errors;
        next_restore_ = now + 1.0;
      }
    }

    if (!want_hide) {
      if (state_.hidden_by_us) {
        // The player may have opened the UI while we were exiting. A valid
        // visible sample means the player already owns the state.
        if (input.readback_valid && input.readback_visible) {
          state_.intervened = true;
          state_.hidden_by_us = false;
          state_.phase = OrbitUiHidePhase::kIntervened;
          entered_ = false;
          return OrbitUiHideAction::kNone;
        }
        if (now + 1e-12 < next_restore_) {
          return OrbitUiHideAction::kNone;
        }
        pending_ = OrbitUiHideAction::kRestore;
        state_.phase = OrbitUiHidePhase::kRestorePending;
        ++state_.restore_requests;
        return OrbitUiHideAction::kRestore;
      }

      // No command is pending and nothing is hidden by us. Keep the entry
      // flags for diagnostics until the next active episode.
      if (entered_) {
        entered_ = false;
      }
      if (state_.phase != OrbitUiHidePhase::kIntervened &&
          state_.phase != OrbitUiHidePhase::kAlreadyHidden &&
          state_.phase != OrbitUiHidePhase::kFailed) {
        state_.phase = OrbitUiHidePhase::kIdle;
      }
      return OrbitUiHideAction::kNone;
    }

    if (!entered_) {
      entered_ = true;
      state_.was_hidden =
          input.readback_valid && !input.readback_visible;
      state_.intervened = false;
      state_.hide_failed = false;
      state_.next_poll = now;
      next_restore_ = 0.0;

      if (state_.was_hidden) {
        state_.hidden_by_us = false;
        state_.phase = OrbitUiHidePhase::kAlreadyHidden;
        return OrbitUiHideAction::kNone;
      }

      // If readback failed, assume visible and hide; exit will restore
      // unconditionally because readback_supported stays false.
      state_.hidden_by_us = true;
      state_.phase = OrbitUiHidePhase::kHidePending;
      pending_ = OrbitUiHideAction::kHide;
      ++state_.hide_requests;
      return OrbitUiHideAction::kHide;
    }

    // Entry hide was acknowledged as a failure; stay visible for this episode
    // instead of retrying every tick.
    if (state_.phase == OrbitUiHidePhase::kFailed) {
      return OrbitUiHideAction::kNone;
    }

    // We are in an active episode but did not hide (already hidden, player
    // intervened, or a failed request). Do not force the UI.
    if (!state_.hidden_by_us) {
      return OrbitUiHideAction::kNone;
    }

    // Poll while we own the hidden state. A visible sample means a manual
    // intervention; stop owning it and never restore over the player.
    if (now + 1e-12 >= state_.next_poll) {
      state_.next_poll = now + poll_seconds_;
      if (input.readback_valid && input.readback_visible) {
        state_.intervened = true;
        state_.hidden_by_us = false;
        state_.phase = OrbitUiHidePhase::kIntervened;
        return OrbitUiHideAction::kNone;
      }
    }
    state_.phase = OrbitUiHidePhase::kHidden;
    return OrbitUiHideAction::kNone;
  }

  // The player opened the game UI (ESC menu/phone) while we own the hidden
  // state. Keep ownership so exit/stop still restore the HUD, but mark the
  // episode intervened so no further force-hide happens. The backend reveals
  // only the menu layer; restoring the HUD here would let the game's own ESC
  // hide animation replay over an already hidden HUD.
  [[nodiscard]] bool MenuOpened() noexcept {
    if (!state_.hidden_by_us || state_.intervened) {
      return false;
    }
    pending_ = OrbitUiHideAction::kNone;
    state_.intervened = true;
    state_.phase = OrbitUiHidePhase::kIntervened;
    return true;
  }

  // True until an episode has seen a non-orbit Update (or Reset). The backend
  // uses this to keep feeding the model while idle so an ESC intervention does
  // not leave `entered_` latched into the next orbit.
  [[nodiscard]] bool EpisodeOpen() const noexcept { return entered_; }

  [[nodiscard]] const OrbitUiHideState& state() const noexcept {
    return state_;
  }

 private:
  OrbitUiHideState state_{};
  double poll_seconds_{kOrbitUiHidePollSeconds};
  double next_restore_{0.0};
  OrbitUiHideAction pending_{OrbitUiHideAction::kNone};
  bool entered_{false};
};

}  // namespace nfs2015_chase
