#ifndef CONTROLLER_APPLICATION_STATE_H
#define CONTROLLER_APPLICATION_STATE_H

typedef struct {
  char status[128];
  char error[256];
  int step_counter;
  double route_avoidance_time_sec;
  int route_avoidance_steps;
} ControllerApplicationState;

void controller_application_state_init(ControllerApplicationState *state);
void controller_application_state_set_status(ControllerApplicationState *state, const char *status);
void controller_application_state_set_error(ControllerApplicationState *state, const char *error);
void controller_application_state_clear_error(ControllerApplicationState *state);
void controller_application_state_reset_route_avoidance(ControllerApplicationState *state);
void controller_application_state_tick_route_avoidance(
    ControllerApplicationState *state,
    int avoidance_active,
    double elapsed_seconds);
int controller_application_state_route_off_route(const ControllerApplicationState *state);

#endif
