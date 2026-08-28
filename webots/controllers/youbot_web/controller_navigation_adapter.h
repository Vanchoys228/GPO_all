#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_ADAPTER_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_ADAPTER_H
#include "controller_runtime.h"
typedef struct { int stop; int reset_navigation; int clear_route_finished; int clear_avoidance_hold; int relocalize; double segment_start_x; double segment_start_z; double distance_to_target; const char *status; } ControllerNavigationAdapterEffect;
int controller_navigation_adapter_terminal_effect(const ControllerRuntimeNavigationResult *result, ControllerNavigationAdapterEffect *effect);
#endif
