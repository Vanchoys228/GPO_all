#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_LIFECYCLE_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_LIFECYCLE_H

#include "controller_survey_state.h"
#include "controller_types.h"

typedef struct {
  void (*clear_error)(void *context);
  void (*apply_speed)(void *context, double speed_mps);
  int (*generate)(void *context, const RuntimeCommand *command);
  int (*load_route)(void *context, RouteData *route);
  void (*reset_avoidance)(void *context);
  void (*reset_navigation)(void *context);
  void (*mark_started)(void *context);
} ControllerSurveyStartCallbacks;

typedef struct {
  void (*merge_map)(void *context);
  int (*generate)(void *context);
  int (*load_route)(void *context, RouteData *route);
  int (*find_escape)(void *context, SurveyPoint robot, int start_index);
  void (*reset_navigation)(void *context);
  void (*clear_error)(void *context);
  void (*mark_replanned)(void *context);
} ControllerSurveyReplanCallbacks;

typedef struct {
  void (*merge_map)(void *context);
  int (*find_escape)(void *context, SurveyPoint robot, int start_index);
  void (*clear_navigation)(void *context);
  void (*begin_navigation)(
      void *context, int waypoint_index, SurveyPoint robot);
  void (*update_distance)(
      void *context, int waypoint_index, SurveyPoint robot);
  void (*clear_error)(void *context);
  void (*mark_skipped)(void *context);
  int (*replan)(void *context);
} ControllerSurveyOrbitCallbacks;

void controller_survey_lifecycle_accept_route(
    RouteData *active_route,
    const RouteData *loaded_route,
    int *current_waypoint_index,
    int *route_finished,
    ControllerMappingSurveyState *state,
    int mapping_survey_route,
    MappingSurveyMode mode);

int controller_survey_lifecycle_start(
    const RuntimeCommand *command,
    RouteData *active_route,
    int *current_waypoint_index,
    int *route_finished,
    ControllerMappingSurveyState *state,
    const ControllerSurveyStartCallbacks *callbacks,
    void *context);

int controller_survey_lifecycle_replan(
    SurveyPoint robot,
    RouteData *active_route,
    int *current_waypoint_index,
    int *route_finished,
    ControllerMappingSurveyState *state,
    int cooldown_steps,
    const ControllerSurveyReplanCallbacks *callbacks,
    void *context);

int controller_survey_lifecycle_escape_orbit(
    SurveyPoint robot,
    int route_count,
    int *current_waypoint_index,
    ControllerMappingSurveyState *state,
    int cooldown_steps,
    const ControllerSurveyOrbitCallbacks *callbacks,
    void *context);

#endif
