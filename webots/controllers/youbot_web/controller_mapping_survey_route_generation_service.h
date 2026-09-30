#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ROUTE_GENERATION_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_ROUTE_GENERATION_SERVICE_H

#include "controller_survey_generator.h"

typedef struct {
  const ControllerSurveyGeneratorCallbacks *callbacks;
  void *context;
  void (*set_error)(const char *message);
} ControllerMappingSurveyRouteGenerationService;

int controller_mapping_survey_route_generation_service_generate(
    const ControllerMappingSurveyRouteGenerationService *service,
    int clear_map_before_start,
    MappingSurveyMode survey_mode,
    int *room_zone_index,
    int *interior_start_index);

#endif
