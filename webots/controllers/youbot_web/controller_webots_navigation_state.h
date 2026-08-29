#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_NAVIGATION_STATE_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_NAVIGATION_STATE_H

typedef struct {
  char status[64];
  char error[160];
} ControllerWebotsNavigationState;

void controller_webots_navigation_state_init(ControllerWebotsNavigationState *state);
void controller_webots_navigation_state_set_status(
    ControllerWebotsNavigationState *state, const char *status);
void controller_webots_navigation_state_set_error(
    ControllerWebotsNavigationState *state, const char *error);
void controller_webots_navigation_state_clear_error(ControllerWebotsNavigationState *state);

#endif
