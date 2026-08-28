#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_SAFETY_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_SAFETY_H
#include "controller_types.h"
typedef int (*ControllerMappingSurveyPointSafe)(void *context, double x, double y, double clearance);
int controller_mapping_survey_segment_safe(double ax, double ay, double bx, double by, double grid_cell, double clearance, ControllerMappingSurveyPointSafe point_safe, void *context);
int controller_mapping_survey_segment_stays_in_room(const LimitZone *room, double ax, double ay, double bx, double by, double grid_cell);
#endif
