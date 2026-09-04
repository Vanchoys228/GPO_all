#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_RUNTIME_H

#include "controller_mapping_survey_runtime_safety.h"
#include "controller_survey_integration.h"

ControllerMappingSurveySafetyContext mapping_survey_safety_context(void);
int generate_mapping_survey_route(
    const char *path,
    int clear_map_before_start,
    const RuntimeCommand *command);
ControllerSurveyIntegrationOps survey_integration_ops(void);
int escape_mapping_survey_orbit(double x, double y);
void wait_for_fresh_route(void);
int insert_mapping_survey_obstacle_scan_route(
    double x,
    double y,
    double heading,
    const Waypoint *target,
    const LidarObstacleContext *lidar_context,
    double turn_sign);

#endif
