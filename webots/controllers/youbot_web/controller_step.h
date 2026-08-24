#ifndef YOUBOT_WEB_CONTROLLER_STEP_H
#define YOUBOT_WEB_CONTROLLER_STEP_H

#include "controller_lifecycle.h"

typedef void (*ControllerStepCallback)(void);

typedef struct {
  ControllerStepCallback reload_zones;
  ControllerStepCallback reload_surface_zones;
  ControllerStepCallback reload_route;
  ControllerStepCallback reload_motion;
  ControllerStepCallback reload_runtime_command;
  ControllerStepCallback capture_lidar;
  ControllerStepCallback merge_trace;
  ControllerStepCallback write_map;
  ControllerStepCallback capture_camera;
  ControllerStepCallback write_camera_frame;
  ControllerStepCallback write_camera_map;
  ControllerStepCallback navigate;
  ControllerStepCallback update_avoidance_metrics;
  ControllerStepCallback write_snapshot;
} ControllerStepCallbacks;

void controller_step_run(
    int step,
    const ControllerLifecycleScheduleConfig *schedule,
    const ControllerStepCallbacks *callbacks);

#endif
