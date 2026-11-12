// The ABI v1 surface the host calls: Load, Start, Stop, Unload, Update. Every one
// of these runs on a host thread, so the bodies only sequence work the other
// translation units implement.
#include "internal.hpp"

#include <cmath>
#include <cstdint>
#include <new>
#include <string>
#include <thread>

namespace nfs2015_chase_cam::detail {

// ---------------------------------------------------------------------------
// Plugin lifecycle
// ---------------------------------------------------------------------------

AnomalyStatusV1 ANOMALY_CALL Load(const AnomalyHostApiV1 *host,
                                  void **plugin_context) {
  if (host == nullptr || plugin_context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  *plugin_context = nullptr;
  auto *context = new (std::nothrow) Context();
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_FAILED);
  }
  context->localizer = nfs2015_chase_cam::Localizer(host);
  context->core = Query<AnomalyCoreServiceV1>(
      host, ANOMALY_CORE_SERVICE_V1_ID, ANOMALY_CORE_SERVICE_V1_VERSION);
  context->config = Query<AnomalyConfigServiceV1>(
      host, ANOMALY_CONFIG_SERVICE_V1_ID, ANOMALY_CONFIG_SERVICE_V1_VERSION);
  context->json = Query<AnomalyJsonServiceV1>(
      host, ANOMALY_JSON_SERVICE_V1_ID, ANOMALY_JSON_SERVICE_V1_VERSION);
  context->input = Query<AnomalyInputServiceV1>(
      host, ANOMALY_INPUT_SERVICE_V1_ID, ANOMALY_INPUT_SERVICE_V1_VERSION);
  context->ui = Query<AnomalyUiServiceV1>(
      host, ANOMALY_UI_SERVICE_V1_ID, ANOMALY_UI_SERVICE_V1_VERSION);
  // 2.6.0 managed windows. Optional: 2.5.0 has no `anomaly.window`, in which
  // case Draw falls back to the direct `anomaly.ui` begin/end pair.
  context->window = Query<AnomalyWindowServiceV1>(
      host, ANOMALY_WINDOW_SERVICE_V1_ID, ANOMALY_WINDOW_SERVICE_V1_VERSION);
  context->signature = Query<AnomalySignatureServiceV1>(
      host, ANOMALY_SIGNATURE_SERVICE_V1_ID,
      ANOMALY_SIGNATURE_SERVICE_V1_VERSION);
  context->hook = Query<AnomalyHookServiceV1>(
      host, ANOMALY_HOOK_SERVICE_V1_ID, ANOMALY_HOOK_SERVICE_V1_VERSION);
  context->session = Query<AnomalyNteSessionServiceV1>(
      host, ANOMALY_NTE_SESSION_SERVICE_V1_ID,
      ANOMALY_NTE_SESSION_SERVICE_V1_VERSION);
  context->player = Query<AnomalyNtePlayerServiceV1>(
      host, ANOMALY_NTE_PLAYER_SERVICE_V1_ID,
      ANOMALY_NTE_PLAYER_SERVICE_V1_VERSION);
  // 2.6's anomaly.nte.vehicle service is gone. The getters it called are still
  // ordinary reflected UFunctions, so resolve them through the generic UE5
  // services instead of guessing from motion.
  context->objects = Query<AnomalyUe5ObjectsServiceV1>(
      host, ANOMALY_UE5_OBJECTS_SERVICE_V1_ID,
      ANOMALY_UE5_OBJECTS_SERVICE_V1_VERSION);
  context->events = Query<AnomalyUe5ProcessEventServiceV1>(
      host, ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_ID,
      ANOMALY_UE5_PROCESS_EVENT_SERVICE_V1_VERSION);
  // Optional engine access for the probe. Declared optional in the manifest, so
  // a host without `ue5.ahud` simply leaves this null and the probe reports
  // "off" instead of failing the load.
  context->ahud = Query<AnomalyUe5AhudServiceV1>(
      host, ANOMALY_UE5_AHUD_SERVICE_V1_ID, ANOMALY_UE5_AHUD_SERVICE_V1_VERSION);
  // Version insurance for the X anchor. Optional: a host without the build
  // service leaves the X path disabled and the borrow estimate in charge.
  context->build = Query<AnomalyUe5BuildServiceV1>(
      host, ANOMALY_UE5_BUILD_SERVICE_V1_ID,
      ANOMALY_UE5_BUILD_SERVICE_V1_VERSION);
  // Optional name pool access. The Z cycle names its two view modes by text at
  // runtime instead of carrying FName ids, which the game reassigns per run.
  // Only `resolve_utf8` (the v1 entry, an id -> text lookup) is used; the v2
  // `find_utf8` does not find gameplay tags registered at startup, so asking
  // for v2 would exclude hosts for a function the plugin does not need.
  context->names = Query<AnomalyUe5NamesServiceV1>(
      host, ANOMALY_UE5_NAMES_SERVICE_V1_ID, 1);
  // The entities/actors services are deliberately not leased. Every one of
  // their reflection calls (`component_bounds`, `frame`, `page`) runs a
  // name-keyed walk inside the host on every call, and the camera needs none
  // of them: the Pawn root is the vehicle actor while driving, and its
  // position and bounds arrive through the player service's cached snapshots.
  if (!CoreReady(context->core) || !ConfigReady(context->config) ||
      !UiReady(context->ui) || !SignatureReady(context->signature) ||
      !HookReady(context->hook)) {
    delete context;
    return Status(ANOMALY_STATUS_V1_UNAVAILABLE,
                  "required camera services are unavailable");
  }
  const auto schema_status = context->config->register_schema(
      context->config->user, StringView(kSettingsSchemaId),
      kSettingsSchemaVersion, Bytes(kSettingsSchema), &context->settings_schema);
  if (schema_status.code != ANOMALY_STATUS_V1_OK ||
      context->settings_schema.id == 0 || !LoadSettings(*context)) {
    ReleaseSettingsSchema(*context);
    delete context;
    return Status(ANOMALY_STATUS_V1_FAILED, "camera settings are invalid");
  }
  *plugin_context = context;
  return anomaly::sdk::Ok();
}

void RegisterSettingsWindow(Context &context) noexcept {
  if (!WindowReady(context.window) || context.window_handle.id != 0) {
    return;
  }
  try {
    AnomalyWindowSpecV1 spec{};
    spec.struct_size = sizeof(spec);
    spec.flags = 0;
    spec.id = StringView(kSettingsWindowId);
    const std::string title =
        context.localizer.Text("window.title", "NFS 2015 Chase Cam");
    spec.title = StringView(title);
    spec.initial_width = 430.0F;
    spec.initial_height = 720.0F;
    spec.minimum_width = 320.0F;
    spec.minimum_height = 240.0F;
    spec.maximum_width = 900.0F;
    spec.maximum_height = 1200.0F;
    spec.default_open = 1;
    const AnomalyStatusV1 status = context.window->register_window(
        context.window->user, &spec, &context.window_handle);
    if (status.code != ANOMALY_STATUS_V1_OK ||
        context.window_handle.id == 0) {
      context.window_handle = {};
    }
  } catch (...) {
    context.window_handle = {};
  }
}

void ReleaseSettingsWindow(Context &context) noexcept {
  if (context.window_handle.id != 0 && WindowReady(context.window)) {
    static_cast<void>(context.window->release_window(
        context.window->user, context.window_handle));
  }
  context.window_handle = {};
}

AnomalyStatusV1 ANOMALY_CALL Start(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->elapsed = 0.0;
  context->update_rate_window = 0.0;
  context->update_rate_frames = 0;
  context->update_hz = 0.0;
  context->view_output_hz = 0.0;
  context->interpolation_ratio = 0.0;
  context->view_output_frames.store(0, std::memory_order_relaxed);
  context->configured_preset =
      context->preset.load(std::memory_order_acquire);
  context->chase.SetConfig(context->configured_preset == 0
                               ? nfs2015_chase::GamePreset()
                               : nfs2015_chase::CinematicPreset());
  context->reset_requested.store(true, std::memory_order_release);
  // Signature learning is a Lifecycle-domain operation; cache the target here
  // so the Game tick only has to create the hook.
  std::uintptr_t view_point_target = 0;
  if (ResolveSignature(*context, kCameraViewPointPattern, view_point_target)) {
    context->hook_target.store(view_point_target, std::memory_order_release);
  }
  ResetMotion(*context);
  context->wheel_steer = WheelSteerState{};
  context->vehicle_present_last = false;
  context->last_vehicle_present_time = 0.0;
  context->vehicle_getter_handles = {};
  context->next_vehicle_getter_retry = {};
  context->next_vehicle_query_time = 0.0;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->observed_valid.store(false, std::memory_order_release);
  PublishInactive(*context);
  // T0: a new session starts from a clean counter baseline so the recorded
  // numbers describe this run only.
  for (std::size_t index = 0; index < kSnapReasonCount; ++index) {
    context->diag.snap_by_reason[index].store(0, std::memory_order_relaxed);
  }
  context->diag.yaw_source_switches.store(0, std::memory_order_relaxed);
  context->diag.observed_read_failures.store(0, std::memory_order_relaxed);
  context->diag.lease_failures.store(0, std::memory_order_relaxed);
  context->diag.view_read_torn.store(0, std::memory_order_relaxed);
  context->diag.detour_calls.store(0, std::memory_order_relaxed);
  context->diag.detour_calls_window.store(0, std::memory_order_relaxed);
  context->diag.detour_no_publish.store(0, std::memory_order_relaxed);
  context->diag.player_sequence_jumps.store(0, std::memory_order_relaxed);
  context->diag.camera_yaw_esp_misses.store(0, std::memory_order_relaxed);
  context->diag.slip_from_position_degrees.store(0.0,
                                                 std::memory_order_relaxed);
  context->diag.slip_sign_changes.store(0, std::memory_order_relaxed);
  context->diag.slip_last_sign.store(0, std::memory_order_relaxed);
  context->diag.slip_max_step_ddeg.store(0, std::memory_order_relaxed);
  context->last_slip_degrees = 0.0;
  context->have_last_slip = false;
  context->diag.render_speed_floor_mmps.store(UINT64_MAX,
                                              std::memory_order_relaxed);
  context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
  context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                             std::memory_order_relaxed);
  context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
  context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
  context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                           std::memory_order_relaxed);
  context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
  context->camera_cost.Reset();
  context->camera_ms_last = 0.0;
  // A fresh session re-runs the P0 probe from scratch. Stop already dropped the
  // old subscription, so nothing can be in flight while this resets.
  context->probe = Probe{};
  // Task group 1: check the profile hash, then resolve `ue5.GWorld` from the
  // Lifecycle domain where signature learning is allowed. The Game tick
  // retries a failed pattern scan, but never bypasses the hash gate.
  context->render_x.Reset();
  ResetViewModeRecon(*context);
  EvaluateAnchorInsurance(*context);
  context->detour_hz = 0.0;
  context->brake_last_target = 0.0;
  static_cast<void>(RegisterToggleHotkey(*context));
  RegisterSettingsWindow(*context);
  // Install the hook from the Game tick, not from this lifecycle callback: the
  // target function is called on the Game thread, so create/enable cannot
  // re-enter the detour before the original pointer is published.
  context->next_hook_retry = 0.0;
  context->started.store(true, std::memory_order_release);
  return anomaly::sdk::Ok();
}

AnomalyStatusV1 ANOMALY_CALL Stop(void *plugin_context,
                                  std::uint32_t /*deadline_milliseconds*/) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return Status(ANOMALY_STATUS_V1_INVALID_ARGUMENT);
  }
  context->started.store(false, std::memory_order_release);
  // Put the front wheels back to their stock relative rotation before the
  // vehicle getter caches are dropped. If the game is already tearing down the
  // car the writes fail harmlessly; a normal stop restores the visual pose.
  ReleaseWheelSteer(*context);
  // Both callbacks check `started` or `g_active` first, so dropping these two
  // before any release means the revocation below has nothing to wait for.
  // Draining a callback that is still running the model is what turns a stop
  // into a timeout, and a timeout into a quarantined plugin that the Host will
  // not load again until the process restarts.
  g_active.store(nullptr, std::memory_order_release);
  context->chase_initialized = false;
  ++context->sample_epoch;
  context->sample_mean_interval_ns.store(0.0, std::memory_order_relaxed);
  context->game_offset_valid = false;
  context->game_offset_last_ns = 0;
  context->car_ring.Clear();
  PublishInactive(*context);
  ReleaseToggleHotkey(*context);
  ReleaseProbeSubscription(*context);
  ReleaseSettingsWindow(*context);
  ReleaseHook(*context);
  ReleaseToggleViewModeHook(*context);
  if (context->settings_revision.load(std::memory_order_acquire) !=
      context->persisted_settings_revision.load(std::memory_order_acquire)) {
    static_cast<void>(PersistSettings(*context));
  }
  std::scoped_lock lock(context->display_mutex);
  context->display.started = false;
  context->display.hook_ready = false;
  context->display.active = false;
  context->display.driving = false;
  context->display.reversing = false;
  context->display.speed_source = kSpeedSourceUnavailable;
  context->display.speed_sign_known = false;
  context->vehicle_query_known = false;
  context->vehicle_query_present = false;
  context->vehicle_query_object = 0;
  context->vehicle_query_speed_known = false;
  context->vehicle_query_speed_fresh = false;
  context->vehicle_query_speed_kmh = 0.0;
  context->vehicle_objects_generation = 0;
  context->vehicle_speed_object = 0;
  context->vehicle_speed_function = {};
  context->vehicle_movement_handle = {};
  context->vehicle_wheeled_class = {};
  context->vehicle_speed_wheeled = false;
  context->next_vehicle_wheeled_class_retry = 0.0;
  context->next_vehicle_speed_function_retry = 0.0;
  context->vehicle_speed_diag_logged = false;
  context->vehicle_speed_active_logged = false;
  context->next_vehicle_query_time = 0.0;
  return anomaly::sdk::Ok();
}

void ANOMALY_CALL Unload(void *plugin_context) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  static_cast<void>(Stop(context, 0));
  ReleaseSettingsSchema(*context);
  delete context;
}

void ANOMALY_CALL Update(void *plugin_context, const double delta_seconds) {
  auto *context = static_cast<Context *>(plugin_context);
  if (context == nullptr) {
    return;
  }
  try {
    if (!context->started.load(std::memory_order_acquire)) {
      UpdateDisplay(*context);
      return;
    }
    const double dt =
        std::isfinite(delta_seconds)
            ? nfs2015_chase::Clamp(delta_seconds, 0.0, 0.25)
            : 0.0;
    context->elapsed += dt;
    if (dt > 0.0) {
      context->update_rate_window += dt;
      ++context->update_rate_frames;
      if (context->update_rate_window >= kUpdateRateWindowSeconds) {
        context->update_hz =
            static_cast<double>(context->update_rate_frames) /
            context->update_rate_window;
        const std::uint64_t outputs =
            context->view_output_frames.exchange(0, std::memory_order_relaxed);
        // Every detour call that produced a pose is now a render-clock model
        // step, so the "interpolated" share is simply the presented share.
        const std::uint64_t interpolated = outputs;
        context->view_output_hz =
            static_cast<double>(outputs) / context->update_rate_window;
        context->interpolation_ratio =
            outputs != 0
                ? static_cast<double>(interpolated) /
                      static_cast<double>(outputs)
                : 0.0;
        // Q1/Q4: how often the presenter is actually asked for a pose, next
        // to the Game tick rate. These two should track the render rate once
        // T2 moves the model off the Game tick.
        const std::uint64_t detour_calls =
            context->diag.detour_calls_window.exchange(
                0, std::memory_order_relaxed);
        context->detour_hz =
            static_cast<double>(detour_calls) / context->update_rate_window;
        context->diag.render_speed_floor_mmps.store(
            UINT64_MAX, std::memory_order_relaxed);
        context->diag.render_speed_peak_mmps.store(0, std::memory_order_relaxed);
        context->diag.render_turn_floor_cdps.store(UINT64_MAX,
                                                   std::memory_order_relaxed);
        context->diag.render_turn_peak_cdps.store(0, std::memory_order_relaxed);
        context->diag.car_step_dcm.store(0, std::memory_order_relaxed);
        context->probe.camera_yaw_step_max_ddeg = 0.0;
        context->probe.camera_yaw_step_sum_ddeg = 0.0;
        context->probe.camera_yaw_step_count = 0;
        context->probe.height_step_cm = 0.0;
        context->probe.track_anchor.count = 0;
        context->probe.eye_count = 0;
        context->probe.track_car.count = 0;
        context->diag.judder_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_count.store(0, std::memory_order_relaxed);
        context->diag.judder_car_peak_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_sum_dpx.store(0, std::memory_order_relaxed);
        context->diag.judder_car_count.store(0, std::memory_order_relaxed);
        context->diag.reset_window.store(0, std::memory_order_relaxed);
        ResetResidual(context->diag.flow_peak_ccm,
                      context->diag.flow_sum_ccm,
                      context->diag.flow_count);
        ResetResidual(context->diag.frame_peak_us,
                      context->diag.frame_sum_us,
                      context->diag.frame_count);
        ResetResidual(context->diag.pitch_peak_cdeg,
                      context->diag.pitch_sum_cdeg,
                      context->diag.pitch_count);
        ResetResidual(context->diag.roll_peak_cdeg,
                      context->diag.roll_sum_cdeg,
                      context->diag.roll_count);
        ResetResidual(context->diag.lens_peak_cdeg,
                      context->diag.lens_sum_cdeg,
                      context->diag.lens_count);
        context->diag.tick_interval_min_ns.store(UINT64_MAX,
                                                 std::memory_order_relaxed);
        context->diag.tick_interval_max_ns.store(0, std::memory_order_relaxed);
        context->render_x.ResetWindow();
        context->diag.slip_max_step_ddeg.store(0,
                                               std::memory_order_relaxed);
        context->update_rate_window = 0.0;
        context->update_rate_frames = 0;
      }
    }
    UpdateViewModeRecon(*context);
    RetryHookIfNeeded(*context);
    LogViewUniformHookDiagnostics(*context);
    LogRouteBDiagnostics(*context);
    LogRouteBTiming(*context);
    static_cast<void>(ResolveCameraPov(*context));
    // T0: time the Game-tick work that the camera itself owns. The host
    // callback as a whole also persists settings and resolves the POV, so only
    // UpdateCamera is bracketed here.
    const std::int64_t camera_begin_ns = NowNanoseconds();
    UpdateCamera(*context, dt);
    context->camera_ms_last =
        static_cast<double>(NowNanoseconds() - camera_begin_ns) * 1e-6;
    context->camera_cost.Add(context->camera_ms_last);
    // Probe: read-only reconnaissance, one Game tick at a time. It runs after
    // UpdateCamera so the yaw it compares is this tick's sample.
    RunProbe(*context, dt);
    UpdateDisplay(*context);
    const auto revision =
        context->settings_revision.load(std::memory_order_acquire);
    if (revision !=
        context->persisted_settings_revision.load(std::memory_order_acquire)) {
      const auto now = GetTickCount64();
      const auto changed_at =
          context->settings_changed_at.load(std::memory_order_acquire);
      if (now - changed_at >= kSettingsSaveDelayMilliseconds) {
        if (!PersistSettings(*context)) {
          // Back off instead of retrying a failing config write every tick.
          context->settings_changed_at.store(now, std::memory_order_release);
        }
      }
    }
  } catch (...) {
    // Host callback boundary: plugin exceptions must not escape.
  }
}

}  // namespace nfs2015_chase_cam::detail
