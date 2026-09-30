#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ESCAPE_SAFETY_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ESCAPE_SAFETY_H

#include "controller_mapping_survey_runtime_safety.h"

typedef struct {
  ControllerMappingSurveySafetyContext safety;
  int room_zone_index;
  double grid_cell;
  double obstacle_clearance;
  double segment_clearance;
  double ignore_radius;
} ControllerMappingSurveyEscapeSafetyContext;

int controller_mapping_survey_escape_candidate_allowed(
    const ControllerMappingSurveyEscapeSafetyContext *context,
    SurveyPoint robot,
    const Waypoint *candidate);

#endif
