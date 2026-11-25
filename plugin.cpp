// NFS 2015 Chase Cam — third-party Anomaly plugin (ABI v1).
//
// This package is built against the installed Anomaly SDK only: it includes no
// host-internal headers and carries no JSON library of its own. Settings are
// persisted through `anomaly.config` and decoded through `anomaly.json`.
//
// Feeds the clean-room NFS 2015 chase-camera model in `chase_cam.hpp` from
// Anomaly's NTE snapshots and the vehicle's ComponentToWorld. The model advances
// once at the entry of ULocalPlayer::CalcSceneViewInitOptions, and the
// GetViewPoint detour copies that pose into the engine's FMinimalViewInfo.
// Anomaly 2.7.0 removed `anomaly.nte.vehicle`, so vehicle presence is recovered
// by calling the same reflected getters that service used, through
// `ue5.objects` + `ue5.process-event`; the player capsule's AttachParent is the
// presence fallback and the vehicle transform source. Speed comes from
// differentiating the player position and is signed against the resolved car
// yaw, with hysteresis for reverse. No GObjects scan.
//
// The `Probe` block below is read-only AHUD diagnostics. It never feeds the
// camera, and every result is shown in the diagnostics panel.
//
// Thread domains:
//   on_update = Game. Reads NTE services, publishes the latest car sample,
//               clears the frame gate, and persists settings.
//   GetViewPoint / CalcSceneViewInitOptions = Game. The scene phase advances
//               the model; every view-point call copies the published pose.
//   on_draw   = Render. Copies the display snapshot under a short lock and
//               calls UI outside it. No camera or game object is written here.

#include "localization.hpp"

#include "anomaly/sdk/cpp.hpp"
#include "anomaly/sdk/services/core.h"
#include "anomaly/sdk/services/interop.h"
#include "anomaly/sdk/services/json.h"
#include "anomaly/sdk/services/nte.h"
#include "anomaly/sdk/services/ui.h"
#include "anomaly/sdk/services/ui_resources.h"
#include "anomaly/sdk/services/ue5.h"

#include "chase_cam.hpp"
#include "wheel_steer.hpp"
#include "latency_window.hpp"
#include "car_sample.hpp"
#include "burnout_shot.hpp"
#include "view_blend.hpp"

#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <thread>
#include <system_error>

#include "src/constants.hpp"
#include "src/internal.hpp"
#include "src/sdk.hpp"
#include "src/state.hpp"

ANOMALY_SDK_EXPORT AnomalyStatusV1 ANOMALY_CALL AnomalyPluginEntryV1(
    AnomalyPluginDescriptorV1 *descriptor) {
  if (descriptor == nullptr ||
      descriptor->struct_size < sizeof(*descriptor)) {
    return {ANOMALY_STATUS_V1_INVALID_ARGUMENT, 0, {}};
  }
  using namespace nfs2015_chase_cam::detail;
  *descriptor = {
      sizeof(*descriptor),
      ANOMALY_PLUGIN_API_V1_MAJOR,
      ANOMALY_PLUGIN_API_V1_MINOR,
      StringView(kPluginId),
      StringView("NFS 2015 Chase Cam"),
      StringView("RobinEatCorn"),
      StringView("2.1.0"),
      Load,
      Start,
      Stop,
      Unload,
      Update,
      Draw};
  return anomaly::sdk::Ok();
}
