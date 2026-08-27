#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_PRESENTATION_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_PRESENTATION_H

#include "controller_avoidance.h"

typedef struct {
  const char *status;
  int priority_updated;
  double priority_turn_sign;
  int priority_hold_steps;
} ControllerAvoidancePresentationOutput;

void controller_avoidance_presentation_apply(
    ControllerAvoidanceState *state,
    const ControllerAvoidanceCommand *command,
    int priority_hold_steps,
    int avoidance_hold_steps,
    int has_best_gap,
    ControllerAvoidancePresentationOutput *output);

#endif
