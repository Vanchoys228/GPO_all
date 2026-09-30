#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_CONTOUR_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_CONTOUR_H

#include "controller_types.h"

typedef int (*ControllerSurveyContourPointSafeFn)(void *context, SurveyPoint point);
typedef void (*ControllerSurveyContourAddPointFn)(void *context, SurveyPoint point);
typedef void (*ControllerSurveyContourAddSegmentFn)(
    void *context,
    SurveyPoint from,
    SurveyPoint to);

int controller_survey_append_contour(
    const SurveyPoint *contour,
    int contour_count,
    double robot_x,
    double robot_y,
    int *route_count,
    ControllerSurveyContourPointSafeFn point_is_safe,
    ControllerSurveyContourAddPointFn add_point,
    ControllerSurveyContourAddSegmentFn add_segment,
    void *context);

#endif
