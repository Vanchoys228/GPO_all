#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_INTEGRATION_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_INTEGRATION_H

#include "controller_survey_state.h"
#include "controller_types.h"

typedef struct {
  void (*clear_error)(void);
  void (*apply_speed)(double speed_mps);
  int (*generate)(
      const char *path, int clear_map, const RuntimeCommand *command);
  int (*load_route)(RouteData *route);
  void (*reset_avoidance)(void);
  void (*reset_navigation)(void);
  void (*set_status)(const char *status);
  double (*current_time)(void);
  void (*merge_map)(double now);
  int (*find_escape)(double x, double y, int start_index);
  void (*clear_navigation)(void);
  void (*begin_navigation)(int waypoint_index, double x, double y);
} ControllerSurveyIntegrationOps;

int controller_survey_integration_start(
    const char *route_path,
    const RuntimeCommand *command,
    RouteData *active_route,
    int *current_waypoint_index,
    int *route_finished,
    ControllerMappingSurveyState *state,
    const ControllerSurveyIntegrationOps *ops);

int controller_survey_integration_replan(
    const char *route_path,
    SurveyPoint robot,
    RouteData *active_route,
    int *current_waypoint_index,
    int *route_finished,
    ControllerMappingSurveyState *state,
    int cooldown_steps,
    const ControllerSurveyIntegrationOps *ops);

int controller_survey_integration_escape_orbit(
    const char *route_path,
    SurveyPoint robot,
    RouteData *active_route,
    int *current_waypoint_index,
    int *route_finished,
    double *distance_to_target,
    ControllerMappingSurveyState *state,
    int cooldown_steps,
    const ControllerSurveyIntegrationOps *ops);

#endif
