#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_INTERVALS_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_INTERVALS_H

#include "controller_types.h"

void controller_survey_sort_values(double *values, int count);
void controller_survey_subtract_interval(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    double block_start,
    double block_end);

#endif
