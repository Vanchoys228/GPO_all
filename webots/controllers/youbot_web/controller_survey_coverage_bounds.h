#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_COVERAGE_BOUNDS_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_COVERAGE_BOUNDS_H

#include "controller_types.h"

void controller_survey_get_coverage_bounds(
    const SurveyGrid *grid,
    const ZoneData *zones,
    int room_zone_index,
    double interior_offset,
    double *min_x,
    double *max_x,
    double *min_y,
    double *max_y);
void controller_survey_select_sweep_start(
    int has_low,
    int low_positive,
    double low_distance,
    int has_high,
    int high_positive,
    double high_distance,
    int *sweep_from_high,
    int *start_positive);

#endif
