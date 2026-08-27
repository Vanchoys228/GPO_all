#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_ROUTE_BUILDER_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_ROUTE_BUILDER_H

#include "controller_types.h"

typedef struct {
  int (*start_is_safe)(void *context, SurveyPoint start);
  void (*add_start)(void *context, SurveyPoint start);
  int (*append_room_contour)(void *context);
  void (*append_grid_boundary)(void *context);
  void (*append_horizontal_coverage)(void *context);
  void (*append_vertical_coverage)(void *context);
} ControllerSurveyRouteCallbacks;

int controller_survey_build_route_phases(
    MappingSurveyMode mode,
    SurveyPoint robot,
    int *route_count,
    int *interior_start_index,
    const ControllerSurveyRouteCallbacks *callbacks,
    void *context);

#endif
