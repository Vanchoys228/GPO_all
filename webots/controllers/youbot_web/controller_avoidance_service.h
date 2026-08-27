#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_SERVICE_H

#include "controller_avoidance_lifecycle.h"
#include "controller_avoidance_presentation.h"
#include "controller_avoidance_recovery.h"

typedef struct {
  const ControllerAvoidanceProgressConfig *progress_config;
  ControllerAvoidanceProgressInput progress_input;
  const ControllerAvoidanceLifecycleConfig *lifecycle_config;
  ControllerAvoidanceLifecycleInput lifecycle_input;
  int has_next_waypoint;
  const ControllerAvoidanceCommandConfig *command_config;
  ControllerAvoidanceCommandInput command_input;
  int priority_hold_steps;
  int avoidance_hold_steps;
  int has_best_gap;
} ControllerAvoidanceServiceInput;

typedef struct {
  ControllerAvoidanceRecoveryOutput recovery;
  int has_command;
  ControllerAvoidanceCommand command;
  ControllerAvoidancePresentationOutput presentation;
} ControllerAvoidanceServiceOutput;

int controller_avoidance_service_process_active(
    ControllerAvoidanceState *state,
    const ControllerAvoidanceServiceInput *input,
    ControllerAvoidanceServiceOutput *output);

#endif
