// Acceptance tests for the grouped settings panel
// (PLAN-settings-ui-groups.md task group 3).
//
// The panel is a pure function of the DisplayState snapshot and the UI service,
// so a recording fake service can drive the real Draw(): every frame the test
// inspects which widgets were drawn, and answers one widget per frame with a
// click or a slider move to play the user. That pins the properties the eye
// would otherwise have to check in game:
//   - the default panel draws no sliders at all,
//   - each group offers exactly the modes it can express,
//   - only Custom opens detail controls,
//   - every interactive widget has a unique id,
//   - the springs disappear under the Cinematic preset but keep their mode,
//   - the advanced block is closed by default,
//   - an edit lands in the custom block, then in the effective value,
//   - restore defaults clears modes and custom numbers.

#include "../src/internal.hpp"

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace {

using nfs2015_chase_cam::detail::ApplyGroup;
using nfs2015_chase_cam::detail::Context;
using nfs2015_chase_cam::detail::CustomSettings;
using nfs2015_chase_cam::detail::Draw;
using nfs2015_chase_cam::detail::GroupMode;
using nfs2015_chase_cam::detail::kDefaultCameraDistanceM;
using nfs2015_chase_cam::detail::kDefaultCameraHeightM;
using nfs2015_chase_cam::detail::kSettingsGroupCount;
using nfs2015_chase_cam::detail::NormalizeGroupMode;
using nfs2015_chase_cam::detail::SettingsGroup;

void Fail(const char *label, const char *detail) {
  std::fprintf(stderr, "FAIL %s: %s\n", label, detail);
  std::exit(1);
}

void Expect(const char *label, const bool condition) {
  if (!condition) Fail(label, "condition was false");
  std::printf("ok   %s\n", label);
}

void ExpectNear(const char *label, const double actual, const double expected,
                const double tolerance = 1e-5) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::fprintf(stderr, "FAIL %s: got %.9g (expected %.9g)\n", label, actual,
                 expected);
    std::exit(1);
  }
  std::printf("ok   %-52s %.6f\n", label, actual);
}

// ImGui derives a widget's identity from its "###" suffix when it has one, and
// from the visible label otherwise. Same rule here, so a duplicate reported by
// the test is a duplicate the real panel would have.
[[nodiscard]] std::string VisiblePart(const std::string &label) {
  const std::size_t hash = label.find("###");
  return hash == std::string::npos ? label : label.substr(0, hash);
}

[[nodiscard]] std::string WidgetId(const std::string &label) {
  const std::size_t hash = label.find("###");
  return hash == std::string::npos ? label : label.substr(hash + 3);
}

struct Widget final {
  std::string kind;
  std::string label;
  float value{};
};

class FakeUi final {
public:
  AnomalyUiServiceV1 service{};
  std::vector<Widget> widgets;
  std::vector<std::string> texts;
  std::string click;  // Widget id to click in the next frame, consumed once.
  std::string edit;   // Slider id to move in the next frame, consumed once.
  float edit_value{};

  FakeUi() {
    service.struct_size = sizeof(service);
    service.service_version = 1;
    service.user = this;
    service.set_next_window_size = &SetNextWindowSize;
    service.begin_window = &BeginWindow;
    service.end_window = &EndWindow;
    service.text = &Text;
    service.button = &Button;
    service.checkbox = &Checkbox;
    service.slider_float = &Slider;
    service.separator = &Separator;
    service.same_line = &SameLine;
  }

  void BeginFrame() {
    widgets.clear();
    texts.clear();
  }

  [[nodiscard]] const Widget *Find(const std::string &id) const {
    for (const Widget &widget : widgets) {
      if (WidgetId(widget.label) == id) return &widget;
    }
    return nullptr;
  }

  [[nodiscard]] const Widget *FindVisible(const std::string &visible) const {
    for (const Widget &widget : widgets) {
      if (VisiblePart(widget.label) == visible) return &widget;
    }
    return nullptr;
  }

  [[nodiscard]] std::size_t Count(const char *kind) const {
    std::size_t total = 0;
    for (const Widget &widget : widgets) {
      if (widget.kind == kind) ++total;
    }
    return total;
  }

  [[nodiscard]] bool HasText(const std::string &needle) const {
    for (const std::string &text : texts) {
      if (text.find(needle) != std::string::npos) return true;
    }
    return false;
  }

private:
  static void ANOMALY_CALL SetNextWindowSize(void *, float, float,
                                             std::uint32_t) {}
  static int ANOMALY_CALL BeginWindow(void *, AnomalyStringViewV1, int *open,
                                      std::uint32_t) {
    if (open != nullptr) *open = 1;
    return 1;
  }
  static void ANOMALY_CALL EndWindow(void *) {}
  static void ANOMALY_CALL Text(void *user, AnomalyStringViewV1 text) {
    auto *ui = static_cast<FakeUi *>(user);
    ui->texts.emplace_back(text.data, text.size);
  }
  static int ANOMALY_CALL Button(void *user, AnomalyStringViewV1 label, float,
                                 float) {
    auto *ui = static_cast<FakeUi *>(user);
    const std::string text(label.data, label.size);
    ui->widgets.push_back({"button", text, 0.0F});
    if (ui->click == WidgetId(text)) {
      ui->click.clear();
      return 1;
    }
    return 0;
  }
  static int ANOMALY_CALL Checkbox(void *user, AnomalyStringViewV1 label,
                                   int *value) {
    auto *ui = static_cast<FakeUi *>(user);
    const std::string text(label.data, label.size);
    const float shown =
        value != nullptr ? static_cast<float>(*value) : 0.0F;
    ui->widgets.push_back({"checkbox", text, shown});
    if (value != nullptr && ui->click == WidgetId(text)) {
      ui->click.clear();
      *value = *value != 0 ? 0 : 1;
      return 1;
    }
    return 0;
  }
  static int ANOMALY_CALL Slider(void *user, AnomalyStringViewV1 label,
                                 float *value, float minimum, float maximum) {
    auto *ui = static_cast<FakeUi *>(user);
    const std::string text(label.data, label.size);
    ui->widgets.push_back(
        {"slider", text, value != nullptr ? *value : 0.0F});
    if (value == nullptr || ui->edit != WidgetId(text)) return 0;
    ui->edit.clear();
    float next = ui->edit_value;
    if (next < minimum) next = minimum;
    if (next > maximum) next = maximum;
    *value = next;
    ui->widgets.back().value = next;
    return 1;
  }
  static void ANOMALY_CALL Separator(void *) {}
  static void ANOMALY_CALL SameLine(void *, float, float) {}
};

struct Harness final {
  FakeUi ui;
  Context context;

  Harness() { context.display.enabled = true; }

  // The tick copies the modes and the preset into the snapshot the panel reads.
  void Sync() {
    for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
      const auto group = static_cast<SettingsGroup>(index);
      context.display.group_modes[index] = NormalizeGroupMode(
          group,
          context.group_modes[index].load(std::memory_order_acquire));
    }
    context.display.preset = context.preset.load(std::memory_order_acquire);
  }

  void Frame() {
    Sync();
    ui.BeginFrame();
    Draw(&context, &ui.service);
  }

  void Click(const char *id) {
    ui.click = id;
    Frame();
    if (!ui.click.empty()) {
      std::fprintf(stderr, "FAIL the panel never drew %s\n", id);
      std::exit(1);
    }
    std::printf("ok   clicked %s\n", id);
  }

  void MoveSlider(const char *id, const float value) {
    ui.edit = id;
    ui.edit_value = value;
    Frame();
    if (!ui.edit.empty()) {
      std::fprintf(stderr, "FAIL the panel never drew the slider %s\n", id);
      std::exit(1);
    }
    std::printf("ok   moved %s to %.3f\n", id, static_cast<double>(value));
  }

  [[nodiscard]] CustomSettings Custom() {
    std::scoped_lock lock(context.custom_settings_mutex);
    return context.custom_settings;
  }

  [[nodiscard]] GroupMode Mode(const SettingsGroup group) const {
    return static_cast<GroupMode>(
        context.group_modes[static_cast<std::size_t>(group)].load(
            std::memory_order_acquire));
  }
};

void ExpectUniqueWidgetIds(const FakeUi &ui) {
  std::map<std::string, std::size_t> seen;
  for (const Widget &widget : ui.widgets) {
    if (++seen[WidgetId(widget.label)] > 1) {
      std::fprintf(stderr, "FAIL duplicate widget id: %s\n",
                   WidgetId(widget.label).c_str());
      std::exit(1);
    }
  }
  std::printf("ok   every interactive widget id is unique (%zu widgets)\n",
              ui.widgets.size());
}

// How many of these exact widget ids were drawn this frame. Counting ids, not
// sliders, keeps each check about one group while other groups are open.
std::size_t CountIds(const FakeUi &ui,
                     const std::initializer_list<const char *> ids) {
  std::size_t total = 0;
  for (const char *id : ids) {
    if (ui.Find(id) != nullptr) ++total;
  }
  return total;
}

void ExpectModeButton(const FakeUi &ui, const char *id, const bool present) {
  const bool drawn = ui.Find(id) != nullptr;
  if (drawn != present) {
    std::fprintf(stderr, "FAIL %s should %s\n", id,
                 present ? "be drawn" : "be absent");
    std::exit(1);
  }
}

void CheckDefaultPanel(Harness &harness) {
  harness.Frame();
  Expect("the default panel draws no sliders", harness.ui.Count("slider") == 0);
  Expect("the default panel draws the group titles",
         harness.ui.HasText("NFS lens (FOV)") &&
             harness.ui.HasText("Camera geometry") &&
             harness.ui.HasText("Camera springs") &&
             harness.ui.HasText("Camera dynamics") &&
             harness.ui.HasText("Slope following") &&
             harness.ui.HasText("Drift front wheels") &&
             harness.ui.HasText("Burnout close-up"));

  // Only the modes a group can express are offered: the lens has no numbers to
  // tune, the geometry and the springs have no "off".
  ExpectModeButton(harness.ui, "group-nfs-lens-mode-0", true);
  ExpectModeButton(harness.ui, "group-nfs-lens-mode-1", false);
  ExpectModeButton(harness.ui, "group-nfs-lens-mode-2", true);
  ExpectModeButton(harness.ui, "group-rig-mode-0", true);
  ExpectModeButton(harness.ui, "group-rig-mode-1", true);
  ExpectModeButton(harness.ui, "group-rig-mode-2", false);
  ExpectModeButton(harness.ui, "group-springs-mode-0", true);
  ExpectModeButton(harness.ui, "group-springs-mode-1", true);
  ExpectModeButton(harness.ui, "group-springs-mode-2", false);
  for (const char *id : {"group-dynamics-mode-0", "group-dynamics-mode-1",
                         "group-dynamics-mode-2", "group-slope-mode-0",
                         "group-slope-mode-1", "group-slope-mode-2",
                         "group-wheel-steer-mode-0", "group-wheel-steer-mode-1",
                         "group-wheel-steer-mode-2", "group-burnout-mode-0",
                         "group-burnout-mode-1", "group-burnout-mode-2"}) {
    ExpectModeButton(harness.ui, id, true);
  }

  const Widget *selected = harness.ui.Find("group-rig-mode-0");
  Expect("the selected mode carries the marker",
         selected != nullptr && selected->label.rfind("> ", 0) == 0);
  const Widget *burnout_on = harness.ui.Find("group-burnout-mode-2");
  Expect("the burnout third state reads On",
         burnout_on != nullptr &&
             VisiblePart(burnout_on->label).find("On") != std::string::npos &&
             VisiblePart(burnout_on->label).find("Off") == std::string::npos);

  Expect("the advanced block starts closed",
         harness.ui.Find("advanced-open") != nullptr &&
             harness.ui.Find("chase-borrow") == nullptr &&
             harness.ui.Find("anchor-0") == nullptr);
  Expect("a closed group draws no summary line",
         !harness.ui.HasText("Distance, height"));
  ExpectUniqueWidgetIds(harness.ui);
}

void CheckRigDetails(Harness &harness) {
  harness.Click("group-rig-mode-1");
  harness.Frame();
  Expect("the rig opens four sliders", harness.ui.Count("slider") == 4);
  Expect("the rig opens its summary line",
         harness.ui.HasText("Distance, height"));
  for (const char *id : {"chase-camera-distance", "chase-camera-height",
                         "chase-speed-distance", "chase-aspect"}) {
    ExpectModeButton(harness.ui, id, true);
  }
  ExpectUniqueWidgetIds(harness.ui);

  harness.MoveSlider("chase-camera-distance", 6.4F);
  harness.Frame();
  ExpectNear("the distance edit lands in the custom block",
             harness.Custom().camera_distance_m, 6.4);
  ExpectNear("the distance edit lands in the effective value",
             harness.context.camera_distance_m.load(), 6.4);
  ExpectNear("the distance edit leaves the static height alone",
             harness.context.camera_height_m.load(), kDefaultCameraHeightM);
  Expect("the group stays on Custom",
         harness.Mode(SettingsGroup::Rig) == GroupMode::Custom);

  harness.MoveSlider("chase-camera-height", 2.2F);
  harness.Frame();
  ExpectNear("the height edit lands in the custom block",
             harness.Custom().camera_height_m, 2.2);
  ExpectNear("the height edit lands in the effective value",
             harness.context.camera_height_m.load(), 2.2);
  ExpectNear("the height edit leaves the static distance alone",
             harness.context.camera_distance_m.load(), 6.4);

  const Widget *slider = harness.ui.Find("chase-camera-distance");
  ExpectNear("the distance slider shows the custom value",
             slider != nullptr ? static_cast<double>(slider->value) : 0.0, 6.4);

  harness.Click("group-rig-mode-0");
  harness.Frame();
  Expect("Default closes the rig details", harness.ui.Count("slider") == 0);
  ExpectNear("Default restores the built-in distance",
             harness.context.camera_distance_m.load(),
             kDefaultCameraDistanceM);
  ExpectNear("Default restores the built-in height",
             harness.context.camera_height_m.load(),
             kDefaultCameraHeightM);
  ExpectNear("Default keeps the custom distance as the custom value",
             harness.Custom().camera_distance_m, 6.4);
  ExpectNear("Default keeps the custom height as the custom value",
             harness.Custom().camera_height_m, 2.2);

  harness.Click("group-rig-mode-1");
  harness.Frame();
  slider = harness.ui.Find("chase-camera-distance");
  ExpectNear("switching back restores the tuned distance slider",
             slider != nullptr ? static_cast<double>(slider->value) : 0.0, 6.4);
  slider = harness.ui.Find("chase-camera-height");
  ExpectNear("switching back restores the tuned height slider",
             slider != nullptr ? static_cast<double>(slider->value) : 0.0, 2.2);
}

void CheckOppositeStates(Harness &harness) {
  harness.Click("group-dynamics-mode-2");
  harness.Frame();
  Expect("Dynamics/Off draws no detail controls",
         harness.ui.Find("chase-roll-scale") == nullptr);
  ExpectNear("Dynamics/Off zeroes the roll response",
             harness.context.roll_scale.load(), 0.0);
  ExpectNear("Dynamics/Off zeroes the pitch response",
             harness.context.pitch_scale.load(), 0.0);
  ExpectNear("Dynamics/Off zeroes the shake",
             harness.context.shake_scale.load(), 0.0);

  harness.Click("group-slope-mode-2");
  harness.Frame();
  Expect("Slope/Off stops following the slope",
         !harness.context.slope_follow.load());
  Expect("Slope/Off draws no detail controls",
         harness.ui.Find("chase-slope-strength") == nullptr);

  harness.Click("group-wheel-steer-mode-2");
  harness.Frame();
  Expect("Wheel/Off disables the steer",
         !harness.context.wheel_steer_enabled.load());
  Expect("Wheel/Off draws no detail controls",
         harness.ui.Find("chase-wheel-steer-max") == nullptr);

  harness.Click("group-nfs-lens-mode-2");
  harness.Frame();
  Expect("Lens/Off turns the FOV off", !harness.context.fov_enabled.load());
  harness.Click("group-nfs-lens-mode-0");
  harness.Frame();
  Expect("Lens/Default turns the FOV on", harness.context.fov_enabled.load());
}

void CheckBurnoutDetails(Harness &harness) {
  harness.Click("group-burnout-mode-1");
  harness.Frame();
  Expect("Burnout/Custom opens nine sliders",
         CountIds(harness.ui,
                  {"chase-burnout-fov", "chase-burnout-shake",
                   "chase-burnout-entry-delay", "chase-burnout-entry-time",
                   "chase-burnout-exit-delay", "chase-burnout-exit-time",
                   "chase-burnout-launch-exit-time",
                   "chase-burnout-entry-speed",
                   "chase-burnout-hold-speed"}) == 9);
  for (const char *id : {"burnout-side-0", "burnout-side-1", "burnout-side-2",
                         "burnout-side-3"}) {
    ExpectModeButton(harness.ui, id, true);
  }
  Expect("Burnout/Custom enables the close-up",
         harness.context.burnout_enabled.load());
  ExpectUniqueWidgetIds(harness.ui);

  harness.MoveSlider("chase-burnout-entry-speed", 8.0F);
  harness.Frame();
  ExpectNear("the entry speed reaches the custom block",
             harness.Custom().burnout_entry_speed, 8.0);
  Expect("the hold speed is lifted with the entry speed",
         harness.Custom().burnout_hold_speed >= 8.0);
  Expect("the effective hold speed stays at or above the entry speed",
         harness.context.burnout_hold_speed.load() >= 8.0);

  harness.Click("burnout-side-2");
  harness.Frame();
  Expect("the side button stores the right rear",
         harness.Custom().burnout_side_mode == 2);
  Expect("the side button reaches the effective value",
         harness.context.burnout_side_mode.load() == 2);

  harness.Click("group-burnout-mode-0");
  harness.Frame();
  Expect("Burnout/Default switches the close-up off",
         !harness.context.burnout_enabled.load());
  Expect("Burnout/Default draws no detail controls",
         harness.ui.Find("chase-burnout-fov") == nullptr &&
             harness.ui.Find("burnout-side-0") == nullptr);
}

void CheckPresetHidesSprings(Harness &harness) {
  harness.Click("group-springs-mode-1");
  harness.Frame();
  Expect("Springs/Custom opens seven sliders",
         CountIds(harness.ui,
                  {"chase-spring-position", "chase-spring-yaw",
                   "chase-spring-drift-yaw", "chase-look-yaw-spring",
                   "chase-spring-yaw-return", "chase-spring-pitch",
                   "chase-spring-roll"}) == 7);

  harness.Click("Cinematic reconstruction");
  harness.Frame();
  ExpectModeButton(harness.ui, "group-springs-mode-1", false);
  Expect("Cinematic hides the springs",
         !harness.ui.HasText("Camera springs"));
  Expect("Cinematic keeps the springs mode",
         harness.Mode(SettingsGroup::Springs) == GroupMode::Custom);
  ExpectNear("Cinematic keeps the springs custom value",
             harness.Custom().roll_spring,
             nfs2015_chase_cam::detail::kDefaultRollSpring);

  harness.Click("NFS 2015 (retail data)");
  harness.Frame();
  ExpectModeButton(harness.ui, "group-springs-mode-1", true);
  Expect("the NFS preset brings the springs back",
         harness.ui.Find("chase-spring-pitch") != nullptr);
}

void CheckAdvancedBlock(Harness &harness) {
  harness.Click("advanced-open");
  harness.Frame();
  Expect("the advanced block draws the anchors",
         harness.ui.Find("anchor-0") != nullptr &&
             harness.ui.Find("anchor-3") != nullptr);
  Expect("the advanced block draws the lock slider",
         harness.ui.Find("chase-borrow") != nullptr);
  for (const char *id : {"chase-flat-pose", "chase-zero-lag",
                         "chase-skip-position", "chase-skip-rotation",
                         "chase-diagnostics"}) {
    ExpectModeButton(harness.ui, id, true);
  }
  Expect("the diagnostics panel starts closed",
         !harness.ui.HasText("Speed source"));
  ExpectUniqueWidgetIds(harness.ui);

  harness.Click("chase-diagnostics");
  harness.Frame();
  Expect("the diagnostics panel opens", harness.ui.HasText("Speed source"));
}

void CheckRestoreDefaults(Harness &harness) {
  harness.Frame();
  Expect("two groups are open before restoring",
         harness.ui.Find("chase-camera-distance") != nullptr &&
             harness.ui.Find("chase-spring-pitch") != nullptr);
  harness.Click("Restore defaults");
  harness.Frame();
  Expect("restore defaults closes every group",
         harness.ui.Find("chase-camera-distance") == nullptr &&
             harness.ui.Find("chase-spring-pitch") == nullptr &&
             harness.ui.Find("chase-roll-scale") == nullptr);
  for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
    Expect("restore defaults returns every group to Default",
           harness.context.group_modes[index].load() == 0);
  }
  ExpectNear("restore defaults returns the distance",
             harness.context.camera_distance_m.load(),
             kDefaultCameraDistanceM);
  ExpectNear("restore defaults returns the height",
             harness.context.camera_height_m.load(),
             kDefaultCameraHeightM);
  ExpectNear("restore defaults clears the custom distance",
             harness.Custom().camera_distance_m, kDefaultCameraDistanceM);
  ExpectNear("restore defaults clears the custom height",
             harness.Custom().camera_height_m, kDefaultCameraHeightM);
}

}  // namespace

int main() {
  Harness harness;
  CheckDefaultPanel(harness);
  CheckRigDetails(harness);
  CheckOppositeStates(harness);
  CheckBurnoutDetails(harness);
  CheckPresetHidesSprings(harness);
  CheckAdvancedBlock(harness);
  CheckRestoreDefaults(harness);
  std::printf("\nall settings panel checks passed\n");
  return 0;
}
