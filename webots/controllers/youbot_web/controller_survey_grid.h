#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_GRID_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_GRID_H

#include "controller_types.h"

typedef struct {
  double default_min_x;
  double default_max_x;
  double default_min_y;
  double default_max_y;
  double max_extent_x;
  double max_extent_y;
  double base_cell;
  int max_cells;
} ControllerSurveyGridConfig;

typedef struct {
  const ZoneData *zones;
  const MapCell *map;
  int map_count;
  int room_zone_index;
  double robot_x;
  double robot_y;
  double clearance;
  int has_field_bounds;
  double field_min_x;
  double field_max_x;
  double field_min_y;
  double field_max_y;
} ControllerSurveyGridInput;

typedef int (*ControllerSurveyPointSafeFn)(
    void *context,
    double x,
    double y,
    int room_zone_index,
    double clearance);

int controller_survey_grid_build(
    SurveyGrid *grid,
    const ControllerSurveyGridConfig *config,
    const ControllerSurveyGridInput *input,
    ControllerSurveyPointSafeFn point_is_safe,
    void *context);

#endif
