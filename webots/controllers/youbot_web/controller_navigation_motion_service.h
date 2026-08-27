#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_MOTION_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_MOTION_SERVICE_H

#include "controller_navigation_lidar.h"
#include "controller_navigation_tracking.h"

typedef struct {
  ControllerNavigationTrackingInput tracking;
  ControllerNavigationLidarInput lidar;
} ControllerNavigationMotionServiceInput;

typedef struct {
  ControllerNavigationTrackingOutput tracking;
  ControllerNavigationLidarOutput lidar;
} ControllerNavigationMotionServiceOutput;

int controller_navigation_motion_service_compute(
    const ControllerNavigationTrackingConfig *tracking_config,
    const ControllerNavigationLidarConfig *lidar_config,
    const ControllerNavigationMotionServiceInput *input,
    ControllerNavigationMotionServiceOutput *output);

#endif
