// The detour originals and the active-instance pointer, published as atomics so
// the hook threads can reach them without holding a lock.
#include "sdk.hpp"

namespace nfs2015_chase_cam::detail {

std::atomic<Context *> g_active{};
std::atomic<CameraViewPointFn> g_original{};
std::atomic<OnTransformChangedFn> g_on_transform_changed_original{};
std::atomic<SetupUniformBufferFn> g_setup_uniform_buffer_original{};
std::atomic<ViewMatricesCtorFn> g_view_matrices_ctor{};
std::atomic<PrepareViewStateFn> g_prepare_view_state_original{};
std::atomic<SceneViewUpdateMatrixFn> g_scene_view_update_matrix{};
std::atomic<SceneViewSetupFrustumFn> g_scene_view_setup_frustum{};

}  // namespace nfs2015_chase_cam::detail
