#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_STATE_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_STATE_H

#include "controller_avoidance.h"

void controller_avoidance_state_reset(
    ControllerAvoidanceState *state,
    double start_x,
    double start_z);
void controller_avoidance_state_begin(
    ControllerAvoidanceState *state,
    double x,
    double z,
    double heading,
    double target_distance,
    double turn_sign,
    int hold_steps);
void controller_avoidance_set_detour(
    ControllerAvoidanceState *state,
    double x,
    double z,
    double heading,
    const LidarObstacleContext *context,
    double turn_sign,
    const ControllerAvoidanceDetourConfig *config);
void controller_avoidance_update_progress(
    ControllerAvoidanceState *state,
    const ControllerAvoidanceProgressInput *input,
    const ControllerAvoidanceProgressConfig *config);

#endif
