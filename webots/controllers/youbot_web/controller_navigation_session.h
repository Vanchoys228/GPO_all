#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_SESSION_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_SESSION_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_NAVIGATION_SESSION_WAIT_FOR_ROUTE = 0,
  CONTROLLER_NAVIGATION_SESSION_STOP_FINISHED = 1,
  CONTROLLER_NAVIGATION_SESSION_RELOCALIZE = 2,
  CONTROLLER_NAVIGATION_SESSION_CONTINUE = 3,
} ControllerNavigationSessionDecision;

typedef struct {
  double distance_to_target;
  double segment_start_x;
  double segment_start_z;
} ControllerNavigationSessionOutput;

ControllerNavigationSessionDecision controller_navigation_session_evaluate(
    const RouteData *route,
    int route_finished,
    int manual_relocation_detected,
    int current_waypoint_index,
    double x,
    double z,
    ControllerNavigationSessionOutput *output);

#endif
