#include "controller_step.h"

#include <stddef.h>

static void call_if_present(ControllerStepCallback callback) {
  if (callback) callback();
}

void controller_step_run(
    int step,
    const ControllerLifecycleScheduleConfig *schedule,
    const ControllerStepCallbacks *callbacks) {
  if (!callbacks) return;
  const ControllerLifecycleTasks tasks =
      controller_lifecycle_tasks_for_step(step, schedule);

  if (tasks.reload_zones) {
    call_if_present(callbacks->reload_zones);
    call_if_present(callbacks->reload_surface_zones);
  }
  if (tasks.reload_route) call_if_present(callbacks->reload_route);
  if (tasks.reload_motion) call_if_present(callbacks->reload_motion);
  if (tasks.reload_runtime_command) call_if_present(callbacks->reload_runtime_command);
  call_if_present(callbacks->capture_lidar);
  call_if_present(callbacks->merge_trace);
  if (tasks.write_maps) call_if_present(callbacks->write_map);
  if (tasks.capture_camera) call_if_present(callbacks->capture_camera);
  if (tasks.write_camera_frame) call_if_present(callbacks->write_camera_frame);
  if (tasks.write_maps) call_if_present(callbacks->write_camera_map);
  call_if_present(callbacks->navigate);
  call_if_present(callbacks->update_avoidance_metrics);
  call_if_present(callbacks->write_snapshot);
}
