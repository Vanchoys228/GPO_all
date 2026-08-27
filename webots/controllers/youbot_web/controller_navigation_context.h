#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_CONTEXT_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_CONTEXT_H

#include "controller_types.h"

typedef struct {
  double last_x;
  double last_z;
  double last_heading;
  int valid;
} ControllerNavigationPoseHistory;

typedef struct {
  double dx;
  double dz;
  double distance;
  double heading_to_target;
  double heading_error;
  int is_final_waypoint;
} ControllerNavigationTargetContext;

int controller_navigation_context_update_pose(
    ControllerNavigationPoseHistory *history,
    double x,
    double z,
    double heading,
    double relocation_distance,
    double relocation_heading_rad);
void controller_navigation_context_target(
    const Waypoint *target,
    int waypoint_index,
    int waypoint_count,
    double x,
    double z,
    double heading,
    ControllerNavigationTargetContext *context);

#endif
