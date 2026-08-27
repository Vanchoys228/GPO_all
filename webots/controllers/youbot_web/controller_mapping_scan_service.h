#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SCAN_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SCAN_SERVICE_H

#include "controller_mapping_scan.h"
#include "controller_survey_state.h"

typedef struct {
  int max_scan_points;
  ControllerMappingScanConfig scan;
} ControllerMappingScanServiceConfig;

typedef struct {
  int mapping_survey;
  RouteData *route;
  ControllerMappingSurveyState *state;
  int current_waypoint_index;
  double robot_x;
  double robot_y;
  double heading;
  const Waypoint *target;
  const LidarObstacleContext *lidar_context;
  double turn_sign;
  double segment_start_x;
  double segment_start_y;
} ControllerMappingScanServiceInput;

typedef struct {
  int scan_point_count;
  double obstacle_x;
  double obstacle_y;
} ControllerMappingScanServiceOutput;

int controller_mapping_scan_service_start(
    const ControllerMappingScanServiceConfig *config,
    const ControllerMappingScanServiceInput *input,
    ControllerMappingScanPointAllowed point_allowed,
    void *point_allowed_context,
    ControllerMappingScanServiceOutput *output);

#endif
