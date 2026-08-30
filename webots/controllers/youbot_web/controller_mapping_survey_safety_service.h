#ifndef CONTROLLER_MAPPING_SURVEY_SAFETY_SERVICE_H
#define CONTROLLER_MAPPING_SURVEY_SAFETY_SERVICE_H

#include "controller_mapping_runtime.h"
#include "controller_mapping_survey_runtime_safety.h"
#include "controller_perception_runtime.h"
#include "controller_runtime.h"

typedef struct {
  double trace_ttl_seconds;
  double trace_clearance;
  double max_extent_x;
  double max_extent_y;
  double obstacle_clearance;
  double grid_cell;
} ControllerMappingSurveySafetyServiceConfig;

typedef struct {
  const ControllerRuntime *runtime;
  const ControllerMappingRuntime *mapping_runtime;
  const ControllerPerceptionRuntime *perception_runtime;
  ControllerMappingSurveySafetyServiceConfig config;
} ControllerMappingSurveySafetyService;

void controller_mapping_survey_safety_service_init(
    ControllerMappingSurveySafetyService *service,
    const ControllerRuntime *runtime,
    const ControllerMappingRuntime *mapping_runtime,
    const ControllerPerceptionRuntime *perception_runtime,
    ControllerMappingSurveySafetyServiceConfig config);
ControllerMappingSurveySafetyContext controller_mapping_survey_safety_service_context(
    const ControllerMappingSurveySafetyService *service,
    double now);
int controller_mapping_survey_safety_service_point_safe(
    const ControllerMappingSurveySafetyService *service,
    double x,
    double y,
    int room_zone_index,
    double clearance,
    double now);
int controller_mapping_survey_safety_service_scan_point_allowed(
    const ControllerMappingSurveySafetyService *service,
    double x,
    double y,
    int room_zone_index,
    double boundary_clearance,
    double obstacle_clearance,
    double now);
int controller_mapping_survey_safety_service_find_escape_waypoint(
    const ControllerMappingSurveySafetyService *service,
    double robot_x,
    double robot_y,
    int start_index,
    double minimum_distance,
    int scan_ahead,
    double obstacle_clearance,
    double segment_clearance,
    double ignore_radius,
    double now);

#endif
