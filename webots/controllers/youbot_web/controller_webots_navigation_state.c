#include "controller_webots_navigation_state.h"

#include <string.h>

void controller_webots_navigation_state_init(ControllerWebotsNavigationState *state) {
  if (!state) return;
  strcpy(state->status, "booting");
  state->error[0] = '\0';
}

void controller_webots_navigation_state_set_status(
    ControllerWebotsNavigationState *state, const char *status) {
  if (!state || !status) return;
  strncpy(state->status, status, sizeof(state->status) - 1);
  state->status[sizeof(state->status) - 1] = '\0';
}

void controller_webots_navigation_state_set_error(
    ControllerWebotsNavigationState *state, const char *error) {
  if (!state || !error) return;
  strncpy(state->error, error, sizeof(state->error) - 1);
  state->error[sizeof(state->error) - 1] = '\0';
}

void controller_webots_navigation_state_clear_error(ControllerWebotsNavigationState *state) {
  if (state) state->error[0] = '\0';
}
