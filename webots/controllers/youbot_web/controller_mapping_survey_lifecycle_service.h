#ifndef CONTROLLER_MAPPING_SURVEY_LIFECYCLE_SERVICE_H
#define CONTROLLER_MAPPING_SURVEY_LIFECYCLE_SERVICE_H

#include "controller_survey_default_route.h"

int controller_mapping_survey_lifecycle_service_ensure_default_route(
    const char *route_path,
    long long route_mtime,
    const ControllerSurveyDefaultRouteConfig *config,
    void (*clear_map)(void),
    void (*set_status)(const char *status));

#endif
