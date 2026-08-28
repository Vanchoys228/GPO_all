#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ESCAPE_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ESCAPE_H

#include "controller_types.h"

typedef struct {
  const RouteData *route;
  int route_active;
  int start_index;
  double robot_x;
  double robot_y;
  double minimum_distance;
  int scan_ahead;
} ControllerMappingSurveyEscapeInput;

typedef int (*ControllerMappingSurveyEscapeAllowed)(void *context, const Waypoint *candidate);

int controller_mapping_survey_find_escape_waypoint(
    const ControllerMappingSurveyEscapeInput *input,
    ControllerMappingSurveyEscapeAllowed allowed,
    void *context);

#endif
