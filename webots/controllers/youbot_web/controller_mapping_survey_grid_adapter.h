#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_GRID_ADAPTER_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_GRID_ADAPTER_H

#include "controller_mapping_survey_runtime_safety.h"
#include "controller_survey_grid.h"

typedef struct {
  ControllerMappingSurveySafetyContext safety;
  ControllerSurveyGridConfig grid_config;
  const RuntimeCommand *command;
} ControllerMappingSurveyGridAdapter;

int controller_mapping_survey_grid_adapter_build(
    ControllerMappingSurveyGridAdapter *adapter,
    SurveyGrid *grid,
    int room_zone_index,
    SurveyPoint robot,
    double clearance);

#endif
