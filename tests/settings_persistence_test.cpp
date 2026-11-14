// Acceptance tests for the settings document
// (PLAN-settings-ui-groups.md task group 2).
//
// The test supplies a fake config service and a fake JSON service, so the real
// PersistSettings()/LoadSettings() run against an in-memory document exactly as
// they do against the host. It pins:
//   - the seven mode keys travel through the document,
//   - a round trip keeps modes, custom values and effective values,
//   - a document without mode keys is inferred back to the same behavior,
//   - a switched-off group does not come back on with its old numbers,
//   - an unreadable or impossible mode falls back safely.

#include "../src/internal.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

namespace {

using nfs2015_chase_cam::detail::ApplyGroup;
using nfs2015_chase_cam::detail::Context;
using nfs2015_chase_cam::detail::CustomSettings;
using nfs2015_chase_cam::detail::GroupMode;
using nfs2015_chase_cam::detail::kMaximumSettingsBytes;
using nfs2015_chase_cam::detail::kSettingsGroupCount;
using nfs2015_chase_cam::detail::LoadSettings;
using nfs2015_chase_cam::detail::PersistSettings;
using nfs2015_chase_cam::detail::SetDefaults;
using nfs2015_chase_cam::detail::SettingsGroup;

const char *const kModeKeys[kSettingsGroupCount] = {
    "nfsLensMode", "rigMode",        "springsMode", "dynamicsMode",
    "slopeMode",   "wheelSteerMode", "burnoutMode"};

void Fail(const char *label, const char *detail) {
  std::fprintf(stderr, "FAIL %s: %s\n", label, detail);
  std::exit(1);
}

void Expect(const char *label, const bool condition) {
  if (!condition) Fail(label, "condition was false");
  std::printf("ok   %s\n", label);
}

void ExpectNear(const char *label, const double actual, const double expected,
                const double tolerance = 1e-9) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::fprintf(stderr, "FAIL %s: got %.17g (expected %.17g)\n", label, actual,
                 expected);
    std::exit(1);
  }
  std::printf("ok   %-48s %.6f\n", label, actual);
}

AnomalyStatusV1 Status(const std::uint32_t code) noexcept {
  return {code, 0, {nullptr, 0}};
}

// --- a minimal JSON document model ------------------------------------------
//
// The settings document is one flat object of numbers and booleans, so the
// fixture's parser needs nothing else. The same model backs the fake config
// service's stored bytes and the JSON service the loader decodes through.

struct JsonNode final {
  std::uint32_t kind{ANOMALY_JSON_V1_OBJECT};
  double number{};
  bool boolean{};
  std::map<std::string, std::uint64_t, std::less<>> children;
};

class FakeHost final {
public:
  AnomalyConfigServiceV1 config{};
  AnomalyJsonServiceV1 json{};
  std::map<std::uint64_t, JsonNode> nodes;
  std::uint64_t next_handle{1};
  std::uint64_t root{};
  std::string document;
  bool has_document{false};

  FakeHost() {
    config.struct_size = sizeof(config);
    config.service_version = 1;
    config.user = this;
    config.register_schema = &RegisterSchema;
    config.unregister_schema = &UnregisterSchema;
    config.read = &Read;
    config.write_atomic = &WriteAtomic;
    config.migrate = nullptr;
    json.struct_size = sizeof(json);
    json.service_version = 1;
    json.user = this;
    json.parse = &Parse;
    json.release = &Release;
    json.kind = &Kind;
    json.boolean_value = &BooleanValue;
    json.number_value = &NumberValue;
    json.string_value = nullptr;
    json.array_size = nullptr;
    json.array_item = nullptr;
    json.object_size = &ObjectSize;
    json.object_key_at = nullptr;
    json.object_find = &ObjectFind;
    json.serialize = &Serialize;
  }

  // Parses `text` and makes it the document the editor works on.
  bool ParseDocument(const std::string &text) {
    const char *cursor = text.c_str();
    std::uint64_t parsed = 0;
    if (!ParseObject(*this, cursor, parsed)) return false;
    root = parsed;
    document = text;
    has_document = true;
    return true;
  }

  [[nodiscard]] std::uint64_t Child(const std::string &key) const {
    const auto parent = nodes.find(root);
    if (parent == nodes.end()) return 0;
    const auto child = parent->second.children.find(key);
    return child == parent->second.children.end() ? 0 : child->second;
  }

  bool SetNumber(const std::string &key, const double value) {
    const auto child = Child(key);
    if (child == 0) return false;
    JsonNode &node = nodes.at(child);
    node.kind = ANOMALY_JSON_V1_NUMBER;
    node.number = value;
    return true;
  }

  bool SetBoolean(const std::string &key, const bool value) {
    const auto child = Child(key);
    if (child == 0) return false;
    JsonNode &node = nodes.at(child);
    node.kind = ANOMALY_JSON_V1_BOOLEAN;
    node.boolean = value;
    return true;
  }

  bool RemoveKey(const std::string &key) {
    const auto parent = nodes.find(root);
    if (parent == nodes.end()) return false;
    return parent->second.children.erase(key) != 0;
  }

  [[nodiscard]] std::string SerializeDocument() const {
    const auto parent = nodes.find(root);
    if (parent == nodes.end()) return {};
    std::string out = "{";
    bool first = true;
    for (const auto &[key, handle] : parent->second.children) {
      const auto node = nodes.find(handle);
      if (node == nodes.end()) return {};
      if (!first) out.push_back(',');
      first = false;
      out.push_back('"');
      out += key;
      out += "\":";
      if (node->second.kind == ANOMALY_JSON_V1_BOOLEAN) {
        out += node->second.boolean ? "true" : "false";
      } else {
        char number[40]{};
        std::snprintf(number, sizeof(number), "%.17g", node->second.number);
        out += number;
      }
    }
    out.push_back('}');
    return out;
  }

  std::uint64_t AddNode(
      const std::uint32_t kind, const double number, const bool boolean,
      std::map<std::string, std::uint64_t, std::less<>> children = {}) {
    const std::uint64_t handle = next_handle++;
    JsonNode node;
    node.kind = kind;
    node.number = number;
    node.boolean = boolean;
    node.children = std::move(children);
    nodes.emplace(handle, std::move(node));
    return handle;
  }

private:
  static AnomalyStatusV1 ANOMALY_CALL RegisterSchema(
      void *, AnomalyStringViewV1, std::uint32_t, AnomalyByteSpanV1,
      AnomalyGenerationHandleV1 *handle) {
    if (handle != nullptr) *handle = {1, 1};
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL UnregisterSchema(
      void *, AnomalyGenerationHandleV1) {
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL Read(
      void *user, AnomalyStringViewV1, std::uint32_t *version,
      AnomalyMutableByteSpanV1 destination, std::size_t *inout_size) {
    auto *host = static_cast<FakeHost *>(user);
    if (version == nullptr || inout_size == nullptr) {
      return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
    }
    if (!host->has_document) return Status(ANOMALY_STATUS_V1_NOT_FOUND);
    *version = 1;
    const std::size_t required = host->document.size();
    if (destination.data == nullptr) {
      *inout_size = required;
      return Status(ANOMALY_STATUS_V1_OK);
    }
    if (*inout_size < required || destination.size < required) {
      *inout_size = required;
      return Status(ANOMALY_STATUS_V1_BUFFER_TOO_SMALL);
    }
    std::memcpy(destination.data, host->document.data(), required);
    *inout_size = required;
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL WriteAtomic(
      void *user, AnomalyStringViewV1, std::uint32_t,
      AnomalyByteSpanV1 document) {
    auto *host = static_cast<FakeHost *>(user);
    host->document.assign(reinterpret_cast<const char *>(document.data),
                          document.size);
    host->has_document = true;
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL Parse(
      void *user, AnomalyStringViewV1 document,
      AnomalyGenerationHandleV1 *handle) {
    auto *host = static_cast<FakeHost *>(user);
    if (handle == nullptr) return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
    const std::string text(document.data, document.size);
    const char *cursor = text.c_str();
    std::uint64_t parsed = 0;
    if (!ParseObject(*host, cursor, parsed)) {
      return Status(ANOMALY_STATUS_V1_FAILED);
    }
    *handle = {parsed, 1};
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL Release(void *,
                                              AnomalyGenerationHandleV1) {
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL Kind(
      void *user, const AnomalyGenerationHandleV1 handle,
      std::uint32_t *kind) {
    auto *host = static_cast<FakeHost *>(user);
    const auto node = host->nodes.find(handle.id);
    if (node == host->nodes.end() || kind == nullptr) {
      return Status(ANOMALY_STATUS_V1_NOT_FOUND);
    }
    *kind = node->second.kind;
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL BooleanValue(
      void *user, const AnomalyGenerationHandleV1 handle, std::int32_t *value) {
    auto *host = static_cast<FakeHost *>(user);
    const auto node = host->nodes.find(handle.id);
    if (node == host->nodes.end() || value == nullptr ||
        node->second.kind != ANOMALY_JSON_V1_BOOLEAN) {
      return Status(ANOMALY_STATUS_V1_FAILED);
    }
    *value = node->second.boolean ? 1 : 0;
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL NumberValue(
      void *user, const AnomalyGenerationHandleV1 handle, double *value) {
    auto *host = static_cast<FakeHost *>(user);
    const auto node = host->nodes.find(handle.id);
    if (node == host->nodes.end() || value == nullptr ||
        node->second.kind != ANOMALY_JSON_V1_NUMBER) {
      return Status(ANOMALY_STATUS_V1_FAILED);
    }
    *value = node->second.number;
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL ObjectSize(
      void *user, const AnomalyGenerationHandleV1 handle, std::size_t *size) {
    auto *host = static_cast<FakeHost *>(user);
    const auto node = host->nodes.find(handle.id);
    if (node == host->nodes.end() || size == nullptr ||
        node->second.kind != ANOMALY_JSON_V1_OBJECT) {
      return Status(ANOMALY_STATUS_V1_FAILED);
    }
    *size = node->second.children.size();
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL ObjectFind(
      void *user, const AnomalyGenerationHandleV1 handle,
      AnomalyStringViewV1 key, AnomalyGenerationHandleV1 *child) {
    auto *host = static_cast<FakeHost *>(user);
    const auto node = host->nodes.find(handle.id);
    if (node == host->nodes.end() || child == nullptr ||
        node->second.kind != ANOMALY_JSON_V1_OBJECT) {
      return Status(ANOMALY_STATUS_V1_FAILED);
    }
    const std::string name(key.data, key.size);
    const auto found = node->second.children.find(name);
    if (found == node->second.children.end()) {
      return Status(ANOMALY_STATUS_V1_NOT_FOUND);
    }
    *child = {found->second, 1};
    return Status(ANOMALY_STATUS_V1_OK);
  }

  static AnomalyStatusV1 ANOMALY_CALL Serialize(
      void *, AnomalyGenerationHandleV1, char *, std::size_t *) {
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE);
  }

  static bool SkipSpace(const char *&cursor) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' ||
           *cursor == '\r') {
      ++cursor;
    }
    return true;
  }

  static bool ParseString(const char *&cursor, std::string &out) {
    if (*cursor != '"') return false;
    ++cursor;
    out.clear();
    while (*cursor != '"') {
      if (*cursor == '\0') return false;
      out.push_back(*cursor++);
    }
    ++cursor;
    return true;
  }

  static bool ParseValue(FakeHost &host, const char *&cursor,
                         std::uint64_t &handle) {
    SkipSpace(cursor);
    // The settings document has no string values.
    if (*cursor == '"') return false;
    if (std::strncmp(cursor, "true", 4) == 0) {
      cursor += 4;
      handle = host.AddNode(ANOMALY_JSON_V1_BOOLEAN, 0.0, true);
      return true;
    }
    if (std::strncmp(cursor, "false", 5) == 0) {
      cursor += 5;
      handle = host.AddNode(ANOMALY_JSON_V1_BOOLEAN, 0.0, false);
      return true;
    }
    char *end = nullptr;
    const double value = std::strtod(cursor, &end);
    if (end == cursor) return false;
    cursor = end;
    handle = host.AddNode(ANOMALY_JSON_V1_NUMBER, value, false);
    return true;
  }

  static bool ParseObject(FakeHost &host, const char *&cursor,
                          std::uint64_t &handle) {
    SkipSpace(cursor);
    if (*cursor != '{') return false;
    ++cursor;
    std::map<std::string, std::uint64_t, std::less<>> children;
    SkipSpace(cursor);
    if (*cursor == '}') {
      ++cursor;
    } else {
      while (true) {
        SkipSpace(cursor);
        std::string key;
        if (!ParseString(cursor, key)) return false;
        SkipSpace(cursor);
        if (*cursor != ':') return false;
        ++cursor;
        std::uint64_t child = 0;
        if (!ParseValue(host, cursor, child)) return false;
        children.emplace(std::move(key), child);
        SkipSpace(cursor);
        if (*cursor == ',') {
          ++cursor;
          continue;
        }
        if (*cursor == '}') {
          ++cursor;
          break;
        }
        return false;
      }
    }
    handle = host.next_handle++;
    JsonNode node;
    node.kind = ANOMALY_JSON_V1_OBJECT;
    node.children = std::move(children);
    host.nodes.emplace(handle, std::move(node));
    return true;
  }
};

// --- helpers -----------------------------------------------------------------

void Bind(Context &context, FakeHost &host) {
  context.config = &host.config;
  context.json = &host.json;
}

void SetMode(Context &context, const SettingsGroup group,
             const GroupMode mode) {
  context.group_modes[static_cast<std::size_t>(group)].store(
      static_cast<std::uint32_t>(mode), std::memory_order_release);
  ApplyGroup(context, group);
}

void EditCustom(Context &context, void (*edit)(CustomSettings &)) {
  std::scoped_lock lock(context.custom_settings_mutex);
  edit(context.custom_settings);
}

[[nodiscard]] CustomSettings CustomCopy(Context &context) {
  std::scoped_lock lock(context.custom_settings_mutex);
  return context.custom_settings;
}

[[nodiscard]] bool GroupIs(const Context &context, const SettingsGroup group,
                           const GroupMode mode) {
  return context.group_modes[static_cast<std::size_t>(group)].load(
             std::memory_order_acquire) ==
         static_cast<std::uint32_t>(mode);
}

[[nodiscard]] std::string WithoutModeKeys(const std::string &document) {
  FakeHost editor;
  if (!editor.ParseDocument(document)) return {};
  for (const char *key : kModeKeys) {
    if (!editor.RemoveKey(key)) return {};
  }
  return editor.SerializeDocument();
}

// --- tests -------------------------------------------------------------------

void CheckDefaultsDocument(FakeHost &host) {
  Context context;
  Bind(context, host);
  SetDefaults(context);
  Expect("persist a default document", PersistSettings(context));
  Expect("the document is written",
         host.has_document && !host.document.empty());
  Expect("the document fits the settings buffer",
         host.document.size() < kMaximumSettingsBytes);
  for (const char *key : kModeKeys) {
    const std::string label = std::string("the document stores ") + key +
                              " as Default";
    const std::string needle = std::string(",\"") + key + "\":0";
    Expect(label.c_str(), host.document.find(needle) != std::string::npos);
  }

  Context loaded;
  Bind(loaded, host);
  Expect("load the default document", LoadSettings(loaded));
  for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
    Expect("a default document loads every group as Default",
           loaded.group_modes[index].load() == 0);
  }
  ExpectNear("loaded defaults keep the camera scale",
             loaded.camera_scale.load(),
             nfs2015_chase_cam::detail::kDefaultCameraScale);
  Expect("loaded defaults keep the burnout off",
         !loaded.burnout_enabled.load());
}

void CheckRoundTrip(FakeHost &host) {
  Context context;
  Bind(context, host);
  SetDefaults(context);
  EditCustom(context, [](CustomSettings &custom) {
    custom.camera_scale = 1.25;
    custom.position_spring = 12.5;
    custom.roll_spring = 80.0;
    custom.roll_scale = 0.5;
    custom.shake_scale = 1.5;
    custom.slope_strength = 1.4;
    custom.wheel_steer_max_degrees = 44.0;
    custom.wheel_steer_sample_percent = -25.0;
    custom.burnout_fov = 62.0;
    custom.burnout_side_mode = 2;
  });
  SetMode(context, SettingsGroup::NfsLens, GroupMode::Opposite);
  SetMode(context, SettingsGroup::Rig, GroupMode::Custom);
  SetMode(context, SettingsGroup::Springs, GroupMode::Custom);
  SetMode(context, SettingsGroup::Dynamics, GroupMode::Custom);
  SetMode(context, SettingsGroup::Slope, GroupMode::Custom);
  SetMode(context, SettingsGroup::WheelSteer, GroupMode::Custom);
  SetMode(context, SettingsGroup::Burnout, GroupMode::Custom);
  Expect("persist a tuned document", PersistSettings(context));

  Context loaded;
  Bind(loaded, host);
  Expect("load the tuned document", LoadSettings(loaded));
  Expect("the lens mode survives",
         GroupIs(loaded, SettingsGroup::NfsLens, GroupMode::Opposite));
  Expect("the rig mode survives",
         GroupIs(loaded, SettingsGroup::Rig, GroupMode::Custom));
  Expect("the springs mode survives",
         GroupIs(loaded, SettingsGroup::Springs, GroupMode::Custom));
  Expect("the dynamics mode survives",
         GroupIs(loaded, SettingsGroup::Dynamics, GroupMode::Custom));
  Expect("the slope mode survives",
         GroupIs(loaded, SettingsGroup::Slope, GroupMode::Custom));
  Expect("the wheel steer mode survives",
         GroupIs(loaded, SettingsGroup::WheelSteer, GroupMode::Custom));
  Expect("the burnout mode survives",
         GroupIs(loaded, SettingsGroup::Burnout, GroupMode::Custom));

  const CustomSettings custom = CustomCopy(loaded);
  ExpectNear("the custom camera scale survives", custom.camera_scale, 1.25);
  ExpectNear("the custom position spring survives",
             custom.position_spring, 12.5);
  ExpectNear("the custom roll spring survives", custom.roll_spring, 80.0);
  ExpectNear("the custom roll scale survives", custom.roll_scale, 0.5);
  ExpectNear("the custom shake scale survives", custom.shake_scale, 1.5);
  ExpectNear("the custom slope strength survives", custom.slope_strength, 1.4);
  ExpectNear("the custom wheel steer angle survives",
             custom.wheel_steer_max_degrees, 44.0);
  ExpectNear("the custom wheel steer sample survives",
             custom.wheel_steer_sample_percent, -25.0);
  ExpectNear("the custom burnout FOV survives", custom.burnout_fov, 62.0);
  Expect("the custom burnout side survives", custom.burnout_side_mode == 2);

  ExpectNear("the effective camera scale follows the custom value",
             loaded.camera_scale.load(), 1.25);
  ExpectNear("the effective roll spring follows the custom value",
             loaded.roll_spring.load(), 80.0);
  ExpectNear("the effective slope strength follows the custom value",
             loaded.slope_strength.load(), 1.4);
  Expect("the effective FOV stays off", !loaded.fov_enabled.load());
  Expect("the effective burnout stays on", loaded.burnout_enabled.load());
  ExpectNear("the effective burnout FOV follows the custom value",
             loaded.burnout_fov.load(), 62.0);
}

void CheckLegacyInference(FakeHost &host) {
  // `host` still holds the tuned document. Strip the mode keys: the numbers and
  // the switches alone must reproduce the same state.
  Context reference;
  Bind(reference, host);
  Expect("load the tuned document again", LoadSettings(reference));

  FakeHost legacy_host;
  legacy_host.document = WithoutModeKeys(host.document);
  Expect("the legacy copy is not empty", !legacy_host.document.empty());
  {
    bool no_modes = true;
    for (const char *key : kModeKeys) {
      no_modes =
          no_modes && legacy_host.document.find(std::string("\"") + key +
                                                "\"") ==
                          std::string::npos;
    }
    Expect("the legacy copy has no mode keys", no_modes);
  }
  legacy_host.has_document = true;

  Context legacy;
  Bind(legacy, legacy_host);
  Expect("load a document without mode keys", LoadSettings(legacy));
  ExpectNear("legacy camera scale matches the tuned document",
             legacy.camera_scale.load(), reference.camera_scale.load());
  ExpectNear("legacy roll spring matches the tuned document",
             legacy.roll_spring.load(), reference.roll_spring.load());
  ExpectNear("legacy roll scale matches the tuned document",
             legacy.roll_scale.load(), reference.roll_scale.load());
  ExpectNear("legacy slope strength matches the tuned document",
             legacy.slope_strength.load(), reference.slope_strength.load());
  ExpectNear("legacy wheel steer angle matches the tuned document",
             legacy.wheel_steer_max_degrees.load(),
             reference.wheel_steer_max_degrees.load());
  ExpectNear("legacy burnout FOV matches the tuned document",
             legacy.burnout_fov.load(), reference.burnout_fov.load());
  Expect("legacy FOV switch matches the tuned document",
         legacy.fov_enabled.load() == reference.fov_enabled.load());
  Expect("legacy burnout switch matches the tuned document",
         legacy.burnout_enabled.load() == reference.burnout_enabled.load());
  for (std::size_t index = 0; index < kSettingsGroupCount; ++index) {
    Expect("the inferred mode matches the tuned document",
           legacy.group_modes[index].load() ==
               reference.group_modes[index].load());
  }
}

void CheckOffWins() {
  // A legacy document can carry tuned numbers for a group that was switched
  // off. The upgrade must keep the numbers but not turn the feature back on.
  FakeHost defaults_host;
  {
    Context defaults;
    Bind(defaults, defaults_host);
    SetDefaults(defaults);
    Expect("persist the default document",
           PersistSettings(defaults));
  }

  FakeHost legacy_host;
  // Strip the mode keys first: this document has to look like one written
  // before the groups existed.
  Expect("parse the default document",
         legacy_host.ParseDocument(WithoutModeKeys(defaults_host.document)));
  Expect("tune the burnout FOV", legacy_host.SetNumber("burnoutFov", 62.0));
  Expect("keep the burnout off",
         legacy_host.SetBoolean("burnoutEnabled", false));
  Expect("tune the wheel steer angle",
         legacy_host.SetNumber("wheelSteerMaxDegrees", 44.0));
  Expect("keep the wheel steer off",
         legacy_host.SetBoolean("wheelSteerEnabled", false));
  Expect("tune the slope strength",
         legacy_host.SetNumber("slopeStrength", 1.4));
  Expect("keep the slope off", legacy_host.SetBoolean("slopeFollow", false));
  Expect("tune the roll scale", legacy_host.SetNumber("rollScale", 0.5));
  legacy_host.document = legacy_host.SerializeDocument();
  legacy_host.has_document = true;

  Context loaded;
  Bind(loaded, legacy_host);
  Expect("load the switched-off document", LoadSettings(loaded));
  Expect("the burnout stays on Default",
         GroupIs(loaded, SettingsGroup::Burnout, GroupMode::Default));
  Expect("the burnout stays off", !loaded.burnout_enabled.load());
  ExpectNear("the burnout keeps its default FOV", loaded.burnout_fov.load(),
             nfs2015_chase_cam::detail::kDefaultBurnoutFov);
  ExpectNear("the burnout keeps the tuned FOV as custom",
             CustomCopy(loaded).burnout_fov, 62.0);

  Expect("the wheel steer infers Opposite",
         GroupIs(loaded, SettingsGroup::WheelSteer, GroupMode::Opposite));
  Expect("the wheel steer stays off", !loaded.wheel_steer_enabled.load());
  ExpectNear("the wheel steer keeps its default angle",
             loaded.wheel_steer_max_degrees.load(),
             nfs2015_chase_cam::detail::kDefaultWheelSteerMaxDegrees);
  ExpectNear("the wheel steer keeps the tuned angle as custom",
             CustomCopy(loaded).wheel_steer_max_degrees, 44.0);

  Expect("the slope infers Opposite",
         GroupIs(loaded, SettingsGroup::Slope, GroupMode::Opposite));
  Expect("the slope stops following", !loaded.slope_follow.load());
  ExpectNear("the slope keeps its default strength",
             loaded.slope_strength.load(),
             nfs2015_chase_cam::detail::kDefaultSlopeStrength);
  ExpectNear("the slope keeps the tuned strength as custom",
             CustomCopy(loaded).slope_strength, 1.4);

  Expect("the dynamics infer Custom",
         GroupIs(loaded, SettingsGroup::Dynamics, GroupMode::Custom));
  ExpectNear("the dynamics keep the tuned roll scale",
             loaded.roll_scale.load(), 0.5);
}

void CheckBadModes(FakeHost &tuned_host) {
  // An out-of-range mode is treated as missing: the numbers still describe the
  // intent, so the geometry comes back as Custom.
  FakeHost broken;
  Expect("parse the tuned document", broken.ParseDocument(tuned_host.document));
  Expect("break the rig mode", broken.SetNumber("rigMode", 7.0));
  Expect("tune the camera scale", broken.SetNumber("cameraScale", 1.3));
  broken.document = broken.SerializeDocument();
  broken.has_document = true;
  Context loaded;
  Bind(loaded, broken);
  Expect("a broken mode still loads", LoadSettings(loaded));
  Expect("a broken rig mode infers Custom",
         GroupIs(loaded, SettingsGroup::Rig, GroupMode::Custom));
  ExpectNear("a broken rig mode keeps the geometry",
             loaded.camera_scale.load(), 1.3);

  // A readable but impossible mode (the rig cannot be Opposite) normalizes to
  // Default instead of leaving a state the panel cannot draw.
  FakeHost impossible;
  Expect("parse the tuned document again",
         impossible.ParseDocument(tuned_host.document));
  Expect("write an impossible rig mode", impossible.SetNumber("rigMode", 2.0));
  impossible.document = impossible.SerializeDocument();
  impossible.has_document = true;
  Context normalized;
  Bind(normalized, impossible);
  Expect("an impossible mode still loads", LoadSettings(normalized));
  Expect("an impossible rig mode becomes Default",
         GroupIs(normalized, SettingsGroup::Rig, GroupMode::Default));
  ExpectNear("an impossible mode keeps the custom geometry",
             CustomCopy(normalized).camera_scale, 1.25);
}

}  // namespace

int main() {
  FakeHost host;
  CheckDefaultsDocument(host);
  CheckRoundTrip(host);
  CheckLegacyInference(host);
  CheckOffWins();
  CheckBadModes(host);
  std::printf("\nall settings persistence checks passed\n");
  return 0;
}
