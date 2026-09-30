#ifndef CONTROLLER_MOTION_PROFILE_RELOAD_SERVICE_H
#define CONTROLLER_MOTION_PROFILE_RELOAD_SERVICE_H

#include "controller_webots_motion_state.h"

typedef enum {
  CONTROLLER_MOTION_PROFILE_RELOAD_NOT_DUE,
  CONTROLLER_MOTION_PROFILE_RELOAD_MISSING,
  CONTROLLER_MOTION_PROFILE_RELOAD_UNCHANGED,
  CONTROLLER_MOTION_PROFILE_RELOAD_CHANGED,
} ControllerMotionProfileReloadResult;

typedef struct {
  ControllerWebotsMotionState *motion_state;
  const char *path;
  long long last_modified;
} ControllerMotionProfileReloadService;

void controller_motion_profile_reload_service_init(
    ControllerMotionProfileReloadService *service,
    ControllerWebotsMotionState *motion_state,
    const char *path);
ControllerMotionProfileReloadResult controller_motion_profile_reload_service_run(
    ControllerMotionProfileReloadService *service,
    int step_counter,
    int reload_interval);

#endif
