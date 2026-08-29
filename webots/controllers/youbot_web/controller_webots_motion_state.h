#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_MOTION_STATE_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_MOTION_STATE_H

#include "controller_motion_profile.h"

typedef struct {
  ControllerMotionProfile profile;
  ControllerMotionLimits limits;
} ControllerWebotsMotionState;

void controller_webots_motion_state_apply(ControllerWebotsMotionState *state);
int controller_webots_motion_state_load(
    ControllerWebotsMotionState *state, const char *path);

#endif
