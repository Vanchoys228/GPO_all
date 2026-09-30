#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_STATE_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_STATE_H
#include "controller_runtime.h"
void controller_navigation_state_reset(ControllerRuntime *runtime, double start_x, double start_z);
void controller_navigation_state_begin(ControllerRuntime *runtime, int waypoint, double x, double z);
void controller_navigation_state_ensure(ControllerRuntime *runtime, double x, double z);
void controller_navigation_state_clear_local(ControllerRuntime *runtime, double start_x, double start_z);
#endif
