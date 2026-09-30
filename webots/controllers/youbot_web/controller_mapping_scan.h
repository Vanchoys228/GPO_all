#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SCAN_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SCAN_H

#include "controller_types.h"

typedef int (*ControllerMappingScanPointAllowed)(void *context, double x, double y);

typedef struct {
  double sensor_local_x;
  double sensor_local_y;
  double min_range;
  double max_range;
  double min_repeat_distance;
  double radius;
  int circle_point_count;
  double min_point_spacing;
} ControllerMappingScanConfig;

typedef struct {
  double robot_x;
  double robot_y;
  double heading;
  double obstacle_range;
  double obstacle_beam_angle;
  double last_scan_x;
  double last_scan_y;
  double turn_sign;
  double segment_start_x;
  double segment_start_y;
  double target_x;
  double target_y;
} ControllerMappingScanInput;

int controller_mapping_scan_build(
    const ControllerMappingScanConfig *config,
    const ControllerMappingScanInput *input,
    ControllerMappingScanPointAllowed point_allowed,
    void *point_allowed_context,
    SurveyPoint *points,
    int max_points,
    int *point_count,
    double *obstacle_x,
    double *obstacle_y);
int controller_mapping_scan_insert_route(
    RouteData *route, int waypoint_index, const SurveyPoint *points, int point_count);

#endif
