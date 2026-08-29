#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_CONTOUR_ADAPTER_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_CONTOUR_ADAPTER_H

#include "controller_types.h"

typedef int (*ControllerSurveyContourAdapterSafe)(void *context, SurveyPoint point);
int controller_survey_contour_adapter_append(const LimitZone *room, SurveyPoint *route, int *count, int capacity, double spacing, double max_step, double offset, double robot_x, double robot_y, ControllerSurveyContourAdapterSafe safe, void *context);
#endif
