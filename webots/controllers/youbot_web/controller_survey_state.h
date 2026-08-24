#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_STATE_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_STATE_H

#include "controller_types.h"

typedef struct {
  int room_zone_index;
  MappingSurveyMode mode;
  int interior_start_index;
  int obstacle_scan_active;
  int obstacle_scan_end_index;
  int obstacle_scan_cooldown_steps;
  double last_scan_x;
  double last_scan_y;
  int replan_cooldown_steps;
} ControllerMappingSurveyState;

void controller_mapping_survey_state_init(ControllerMappingSurveyState *state);
void controller_mapping_survey_state_reset_route(ControllerMappingSurveyState *state);
void controller_mapping_survey_state_prepare(
    ControllerMappingSurveyState *state, MappingSurveyMode mode);
void controller_mapping_survey_state_tick(ControllerMappingSurveyState *state);
void controller_mapping_survey_state_begin_scan(
    ControllerMappingSurveyState *state, int end_index, double obstacle_x, double obstacle_y);
int controller_mapping_survey_state_complete_scan(
    ControllerMappingSurveyState *state, int current_waypoint_index, int cooldown_steps);

#endif
