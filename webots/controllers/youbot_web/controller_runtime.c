#include "controller_runtime.h"

#include <string.h>

void controller_runtime_init(ControllerRuntime *runtime) {
  if (!runtime) return;
  memset(runtime, 0, sizeof(*runtime));
  runtime->navigation_mode = NAV_MODE_IDLE;
  runtime->navigation_waypoint_index = -1;
  runtime->avoidance.turn_sign = 1.0;
  controller_mapping_survey_state_init(&runtime->mapping_survey);
}

ControllerRuntimeMotionCommand controller_runtime_calculate_motion_command(
    const ControllerRuntime *runtime) {
  ControllerRuntimeMotionCommand command = {CONTROLLER_RUNTIME_MOTION_STOP};
  if (runtime && !runtime->route_finished &&
      runtime->current_waypoint_index < runtime->route.count) {
    command.command = CONTROLLER_RUNTIME_MOTION_CALCULATE;
  }
  return command;
}

ControllerRuntimeFrameDecision controller_runtime_process_sensor_frame(
    ControllerRuntime *runtime,
    const ControllerRuntimeSensorFrame *frame,
    ControllerRuntimeFrameResult *result) {
  if (!result) return CONTROLLER_RUNTIME_FRAME_WAIT_FOR_ROUTE;
  result->motion = (ControllerRuntimeMotionCommand){CONTROLLER_RUNTIME_MOTION_STOP};
  if (!runtime || !frame || runtime->route.count <= 0) {
    return CONTROLLER_RUNTIME_FRAME_WAIT_FOR_ROUTE;
  }
  if (runtime->route_finished) return CONTROLLER_RUNTIME_FRAME_STOP_FINISHED;

  result->motion = controller_runtime_calculate_motion_command(runtime);
  return result->motion.command == CONTROLLER_RUNTIME_MOTION_CALCULATE
             ? CONTROLLER_RUNTIME_FRAME_READY
             : CONTROLLER_RUNTIME_FRAME_STOP_FINISHED;
}
