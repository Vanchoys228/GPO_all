#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_COVERAGE_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_COVERAGE_SERVICE_H

#include "controller_mapping_survey_runtime_safety.h"

typedef struct {
  ControllerMappingSurveySafetyContext safety;
  double interior_offset;
  double min_strip_length;
  double strip_spacing;
  double route_spacing;
  double epsilon;
  int route_capacity;
} ControllerMappingSurveyCoverageService;

void controller_mapping_survey_coverage_service_append_horizontal(
    const ControllerMappingSurveyCoverageService *service,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index);
void controller_mapping_survey_coverage_service_append_vertical(
    const ControllerMappingSurveyCoverageService *service,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index);

#endif
