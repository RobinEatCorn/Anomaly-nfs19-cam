// TG-13 acceptance tests for the parked-orbit UI-hide state machine.
//
// Pure model tests: no game, no Anomaly SDK, no clock. The fixture feeds the
// same inputs the Game-thread backend will provide in TG-14 and acknowledges
// each returned command on the next Update.

#include "../orbit_ui_hide.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

using nfs2015_chase::OrbitPhase;
using nfs2015_chase::OrbitUiHide;
using nfs2015_chase::OrbitUiHideAction;
using nfs2015_chase::OrbitUiHideInput;
using nfs2015_chase::OrbitUiHidePhase;

void Expect(const char* label, const bool condition) {
  if (!condition) {
    std::fprintf(stderr, "FAIL %s\n", label);
    std::exit(1);
  }
  std::printf("ok   %s\n", label);
}

void ExpectEqual(const char* label, const int actual, const int expected) {
  if (actual != expected) {
    std::fprintf(stderr, "FAIL %s: got %d, expected %d\n", label, actual,
                 expected);
    std::exit(1);
  }
  std::printf("ok   %-56s %d\n", label, actual);
}

struct Fixture final {
  OrbitUiHide model;
  OrbitUiHideInput input;
  double now{0.0};

  Fixture() {
    input.orbit_mode = OrbitPhase::kEnter;
    input.orbit_enabled = true;
    input.hide_ui_enabled = true;
    input.readback_valid = true;
    input.readback_visible = true;
    input.now_seconds = now;
  }

  // Advance time and deliver one Update. `invoke_ok` acknowledges the command
  // returned by the previous Update; the default is success.
  OrbitUiHideAction Step(const bool invoke_ok = true) {
    input.now_seconds = now;
    input.invoke_ok = invoke_ok;
    return model.Update(input);
  }

  void Advance(const double seconds) { now += seconds; }

  void EnterAndHide() {
    ExpectEqual("enter requests hide", static_cast<int>(Step()), 1);
    Expect("hide is optimistic until acknowledged", model.state().hidden_by_us);
    ExpectEqual("hide acknowledgement has no new action",
                static_cast<int>(Step()), 0);
    ExpectEqual("phase is Hidden after acknowledgement",
                static_cast<int>(model.state().phase), 2);
  }

  void ExitToIdle() {
    input.orbit_mode = OrbitPhase::kIdle;
  }
};

void TestDrivingGate() {
  Fixture f;
  f.input.driving = false;
  ExpectEqual("walking entry does not hide", static_cast<int>(f.Step()), 0);
  Expect("walking entry is not owned", !f.model.state().hidden_by_us);

  f.input.driving = true;
  f.EnterAndHide();
  f.input.driving = false;
  f.input.readback_visible = false;
  ExpectEqual("leaving the vehicle restores", static_cast<int>(f.Step()), 2);
  ExpectEqual("driving-gate restore ack", static_cast<int>(f.Step()), 0);
  Expect("driving-gate restore clears ownership",
         !f.model.state().hidden_by_us);
}

void TestThirdPersonRestoresHiddenHud() {
  Fixture f;
  f.EnterAndHide();

  f.input.chase_view = false;
  f.input.readback_visible = false;
  ExpectEqual("third-person restores hidden HUD",
              static_cast<int>(f.Step()), 2);
  ExpectEqual("third-person restore ack", static_cast<int>(f.Step()), 0);
  Expect("third-person restore clears ownership",
         !f.model.state().hidden_by_us);

  f.input.chase_view = true;
  f.input.readback_visible = true;
  ExpectEqual("back to chase can hide again", static_cast<int>(f.Step()), 1);
  ExpectEqual("back to chase hide ack", static_cast<int>(f.Step()), 0);
  Expect("back to chase owns HUD again", f.model.state().hidden_by_us);
}

void TestMenuOpenedKeepsHudOwnership() {
  Fixture f;
  f.EnterAndHide();
  Expect("menu open is accepted", f.model.MenuOpened());
  Expect("menu open marks intervention", f.model.state().intervened);
  Expect("menu open keeps HUD ownership", f.model.state().hidden_by_us);
  ExpectEqual("menu open phase", static_cast<int>(f.model.state().phase), 4);
  Expect("menu open is idempotent", !f.model.MenuOpened());

  // HUD-only mode feeds readback_valid=false while an orbit episode owns the
  // HUD, so a menu-visible frame cannot be mistaken for a player restore.
  f.input.readback_valid = false;
  f.input.readback_visible = false;
  f.ExitToIdle();
  ExpectEqual("menu exit still restores", static_cast<int>(f.Step()), 2);
  ExpectEqual("menu restore ack", static_cast<int>(f.Step()), 0);
  Expect("menu exit clears ownership", !f.model.state().hidden_by_us);
}

void TestVisibleHideAndRestore() {
  Fixture f;
  f.EnterAndHide();
  Expect("visible entry is not was_hidden", !f.model.state().was_hidden);
  ExpectEqual("hidden poll has no action", static_cast<int>(f.Step()), 0);

  f.ExitToIdle();
  f.input.readback_visible = false;
  ExpectEqual("exit requests restore", static_cast<int>(f.Step()), 2);
  ExpectEqual("restore acknowledgement has no new action",
              static_cast<int>(f.Step()), 0);
  Expect("restore clears ownership", !f.model.state().hidden_by_us);
  Expect("restore marks readback visible", f.model.state().last_visible);
  ExpectEqual("idle after restore has no action", static_cast<int>(f.Step()), 0);
  ExpectEqual("hide was requested once", static_cast<int>(f.model.state().hide_requests), 1);
  ExpectEqual("restore was requested once",
              static_cast<int>(f.model.state().restore_requests), 1);
}

void TestAlreadyHiddenIsLeftAlone() {
  Fixture f;
  f.input.readback_visible = false;
  ExpectEqual("already hidden enter has no action", static_cast<int>(f.Step()), 0);
  Expect("already hidden is remembered", f.model.state().was_hidden);
  Expect("already hidden is not owned by us", !f.model.state().hidden_by_us);
  ExpectEqual("already hidden phase", static_cast<int>(f.model.state().phase), 5);

  f.ExitToIdle();
  ExpectEqual("already hidden exit does not restore", static_cast<int>(f.Step()), 0);
  ExpectEqual("reset after already hidden has no restore",
              static_cast<int>(f.model.Reset()), 0);
}

void TestPlayerInterventionIsRespected() {
  Fixture f;
  f.EnterAndHide();

  f.Advance(0.26);
  f.input.readback_visible = true;
  ExpectEqual("manual UI open has no action", static_cast<int>(f.Step()), 0);
  Expect("manual UI open marks intervention", f.model.state().intervened);
  Expect("manual UI open clears ownership", !f.model.state().hidden_by_us);
  ExpectEqual("intervened phase", static_cast<int>(f.model.state().phase), 4);

  f.ExitToIdle();
  ExpectEqual("intervened exit does not restore", static_cast<int>(f.Step()), 0);
  Expect("intervention survives exit for diagnostics",
         f.model.state().intervened);
}

void TestDisableWhileHiddenRestores() {
  Fixture f;
  f.EnterAndHide();
  f.input.hide_ui_enabled = false;
  f.input.readback_visible = false;
  ExpectEqual("disabling hide setting restores", static_cast<int>(f.Step()), 2);
  ExpectEqual("disable restore ack", static_cast<int>(f.Step()), 0);
  Expect("disable clears ownership", !f.model.state().hidden_by_us);

  Fixture g;
  g.EnterAndHide();
  g.input.orbit_enabled = false;
  g.input.readback_visible = false;
  ExpectEqual("disabling orbit restores", static_cast<int>(g.Step()), 2);
  ExpectEqual("orbit disable restore ack", static_cast<int>(g.Step()), 0);
  Expect("orbit disable clears ownership", !g.model.state().hidden_by_us);
}

void TestResetRestores() {
  Fixture f;
  f.EnterAndHide();
  ExpectEqual("reset returns restore", static_cast<int>(f.model.Reset()), 2);
  Expect("reset clears ownership", !f.model.state().hidden_by_us);
  ExpectEqual("reset phase", static_cast<int>(f.model.state().phase), 0);
  f.ExitToIdle();
  ExpectEqual("no action after reset", static_cast<int>(f.Step()), 0);
}

void TestUnavailableReadbackFallsBackToC() {
  Fixture f;
  f.input.readback_valid = false;
  f.input.readback_visible = false;
  ExpectEqual("unavailable readback still hides", static_cast<int>(f.Step()), 1);
  ExpectEqual("unavailable hide ack", static_cast<int>(f.Step()), 0);
  Expect("unavailable readback stays hidden", f.model.state().hidden_by_us);
  Expect("readback support stays false", !f.model.state().readback_supported);

  f.ExitToIdle();
  ExpectEqual("unavailable readback restores on exit",
              static_cast<int>(f.Step()), 2);
  ExpectEqual("unavailable restore ack", static_cast<int>(f.Step()), 0);
  Expect("unavailable readback clears ownership",
         !f.model.state().hidden_by_us);
}

void TestHideFailureKeepsStateClean() {
  Fixture f;
  ExpectEqual("hide request before failure", static_cast<int>(f.Step()), 1);
  ExpectEqual("failed hide is acknowledged as no action",
              static_cast<int>(f.Step(false)), 0);
  Expect("failed hide is not owned", !f.model.state().hidden_by_us);
  Expect("failed hide is marked", f.model.state().hide_failed);
  ExpectEqual("failed hide phase", static_cast<int>(f.model.state().phase), 6);
  ExpectEqual("failed hide error count", static_cast<int>(f.model.state().errors), 1);

  ExpectEqual("failed hide does not retry while active",
              static_cast<int>(f.Step()), 0);
  f.ExitToIdle();
  ExpectEqual("failed hide does not restore on exit",
              static_cast<int>(f.Step()), 0);
  ExpectEqual("failed hide reset has no restore",
              static_cast<int>(f.model.Reset()), 0);
}

void TestRestoreFailureRetriesWithoutForgettingOwnership() {
  Fixture f;
  f.EnterAndHide();
  f.ExitToIdle();
  f.input.readback_visible = false;
  ExpectEqual("exit asks for restore", static_cast<int>(f.Step()), 2);
  ExpectEqual("failed restore is acknowledged as no action",
              static_cast<int>(f.Step(false)), 0);
  Expect("failed restore keeps ownership", f.model.state().hidden_by_us);
  ExpectEqual("failed restore error count",
              static_cast<int>(f.model.state().errors), 1);

  f.Advance(0.5);
  ExpectEqual("failed restore is backed off", static_cast<int>(f.Step()), 0);
  f.Advance(0.6);
  ExpectEqual("failed restore retries", static_cast<int>(f.Step()), 2);
  ExpectEqual("retry restore ack", static_cast<int>(f.Step()), 0);
  Expect("retry restore clears ownership", !f.model.state().hidden_by_us);
}

void TestExitPhaseRestoresBeforeExitFade() {
  Fixture f;
  f.EnterAndHide();

  f.input.orbit_mode = OrbitPhase::kOrbit;
  f.input.readback_visible = false;
  ExpectEqual("orbit phase keeps HUD hidden", static_cast<int>(f.Step()), 0);
  Expect("orbit phase keeps ownership", f.model.state().hidden_by_us);

  // The pull-out begins in kExit while the render blend is still near one.
  // The HUD must start restoring on this tick, not when the blend reaches 0.
  f.input.orbit_mode = OrbitPhase::kExit;
  ExpectEqual("exit phase starts restore", static_cast<int>(f.Step()), 2);
  ExpectEqual("exit restore ack", static_cast<int>(f.Step()), 0);
  Expect("exit restore clears ownership", !f.model.state().hidden_by_us);
}

void TestRepeatedUpdatesAreIdempotent() {
  Fixture f;
  f.EnterAndHide();
  ExpectEqual("second hidden tick has no hide", static_cast<int>(f.Step()), 0);
  ExpectEqual("third hidden tick has no hide", static_cast<int>(f.Step()), 0);
  ExpectEqual("only one hide request", static_cast<int>(f.model.state().hide_requests), 1);

  f.ExitToIdle();
  f.input.readback_visible = false;
  ExpectEqual("first exit restores", static_cast<int>(f.Step()), 2);
  ExpectEqual("second exit call is a no-op", static_cast<int>(f.Step()), 0);
  Expect("exit clears ownership", !f.model.state().hidden_by_us);
  ExpectEqual("only one restore request",
              static_cast<int>(f.model.state().restore_requests), 1);
}

}  // namespace

int main() {
  TestVisibleHideAndRestore();
  TestDrivingGate();
  TestThirdPersonRestoresHiddenHud();
  TestMenuOpenedKeepsHudOwnership();
  TestAlreadyHiddenIsLeftAlone();
  TestPlayerInterventionIsRespected();
  TestDisableWhileHiddenRestores();
  TestResetRestores();
  TestUnavailableReadbackFallsBackToC();
  TestHideFailureKeepsStateClean();
  TestRestoreFailureRetriesWithoutForgettingOwnership();
  TestExitPhaseRestoresBeforeExitFade();
  TestRepeatedUpdatesAreIdempotent();
  std::printf("all orbit UI-hide tests passed\n");
  return 0;
}
