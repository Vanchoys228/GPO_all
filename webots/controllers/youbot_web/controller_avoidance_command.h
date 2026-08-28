#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_COMMAND_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_COMMAND_H

#include "controller_avoidance.h"

double controller_avoidance_choose_turn_sign(
    const ControllerAvoidanceTurnInput *input);
void controller_avoidance_compute_command(
    const ControllerAvoidanceCommandInput *input,
    const ControllerAvoidanceCommandConfig *config,
    ControllerAvoidanceCommand *command);

#endif
