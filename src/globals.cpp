// The detour originals and the active-instance pointer, published as atomics so
// the hook threads can reach them without holding a lock.
#include "sdk.hpp"

namespace nfs2015_chase_cam::detail {

std::atomic<Context *> g_active{};
std::atomic<LocalPlayerGetViewPointFn> g_local_player_get_view_point_original{};
std::atomic<CalcSceneViewInitOptionsFn> g_calc_scene_view_init_options_original{};
std::atomic<ToggleViewModeFn> g_toggle_view_mode_original{};
std::atomic<Context *> g_view_mode_context{};

}  // namespace nfs2015_chase_cam::detail
