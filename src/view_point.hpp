// The game-thread view-point adapter (2.1.0). It is the only place the chase
// model writes the engine's view data: the first GetViewPoint call inside
// CalcSceneViewInitOptions advances the model, later calls in the same frame
// copy the same pose, and every non-view query is left on the game camera.
#pragma once

#include "constants.hpp"

#include <cstddef>
#include <cstdint>

namespace nfs2015_chase_cam::detail {

struct Context;

// A typed view over the prefix of the engine's FMinimalViewInfo. The engine
// object is 0xC70 bytes and is owned by the caller; the plugin only touches
// these four fields, all confirmed by the game's own UStruct reflection.
struct MinimalViewInfo final {
  double location[3];    // +0x00
  double rotation[3];    // +0x18
  float fov;             // +0x30
  float desired_fov;     // +0x34
};

static_assert(offsetof(MinimalViewInfo, location) ==
              kMinimalViewInfoLocationOffset);
static_assert(offsetof(MinimalViewInfo, rotation) ==
              kMinimalViewInfoRotationOffset);
static_assert(offsetof(MinimalViewInfo, fov) == kMinimalViewInfoFovOffset);
static_assert(offsetof(MinimalViewInfo, desired_fov) ==
              kMinimalViewInfoDesiredFovOffset);
static_assert(sizeof(MinimalViewInfo) == 0x38);

// Applies the chase pose to `view` when the D27 frame gate allows it. Returns
// true when a pose was written. The first GetViewPoint call in a frame advances
// the model, even when it comes from the UI projection before CalcSceneView;
// later calls copy that pose. If the advance fails, the frame stays on the game
// camera instead of retrying the model for every consumer.
[[nodiscard]] bool ApplyViewPoint(Context &context,
                                  MinimalViewInfo &view) noexcept;

// Detours installed only when the temporary A/B switch selects the game-thread
// path. Both call the original first and are fail-open on a missing trampoline.
void ANOMALY_CALL LocalPlayerViewPointDetour(const void *self,
                                             void *out) noexcept;
bool ANOMALY_CALL CalcSceneViewInitOptionsDetour(
    void *self, void *out, void *viewport, void *drawer,
    std::int32_t stereo) noexcept;

}  // namespace nfs2015_chase_cam::detail
