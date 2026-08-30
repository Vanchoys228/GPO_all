#ifndef CONTROLLER_MAPPING_SURVEY_CONTOUR_SERVICE_H
#define CONTROLLER_MAPPING_SURVEY_CONTOUR_SERVICE_H

#include "controller_types.h"

typedef int (*ControllerMappingSurveyContourPointSafe)(
    void *context,
    double x,
    double y,
    int room_zone_index,
    double clearance);

typedef struct {
  const ZoneData *zones;
  ControllerMappingSurveyContourPointSafe point_safe;
  void *point_safe_context;
  double contour_offset;
  double safety_clearance_factor;
  double route_spacing;
  double max_contour_step;
} ControllerMappingSurveyContourService;

int controller_mapping_survey_contour_service_append(
    const ControllerMappingSurveyContourService *service,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    double robot_x,
    double robot_y);

#endif
