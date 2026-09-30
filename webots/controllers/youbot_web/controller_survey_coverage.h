#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_COVERAGE_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_COVERAGE_H

#include "controller_types.h"

typedef int (*ControllerSurveyBuildIntervalsFn)(
    void *context,
    double coordinate,
    SurveyInterval *intervals,
    int capacity);
typedef void (*ControllerSurveyAppendSegmentFn)(
    void *context,
    SurveyPoint start,
    SurveyPoint end);

int controller_survey_choose_axis_start(
    double axis_min,
    double axis_max,
    double strip_spacing,
    int vertical,
    int sweep_from_high,
    SurveyPoint current,
    int interval_capacity,
    ControllerSurveyBuildIntervalsFn build_intervals,
    void *context,
    int *start_positive,
    double *best_distance);

void controller_survey_append_axis_coverage(
    double axis_min,
    double axis_max,
    double strip_spacing,
    int vertical,
    int start_positive,
    int sweep_from_high,
    int *route_count,
    int route_capacity,
    int interval_capacity,
    ControllerSurveyBuildIntervalsFn build_intervals,
    ControllerSurveyAppendSegmentFn append_segment,
    void *context);

void controller_survey_append_best_axis_coverage(
    double axis_min,
    double axis_max,
    double strip_spacing,
    int vertical,
    SurveyPoint current,
    int *route_count,
    int route_capacity,
    int interval_capacity,
    ControllerSurveyBuildIntervalsFn build_intervals,
    ControllerSurveyAppendSegmentFn append_segment,
    void *context);

#endif
