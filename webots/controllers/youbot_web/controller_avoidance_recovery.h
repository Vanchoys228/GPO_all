#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_RECOVERY_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_RECOVERY_H

#include "controller_avoidance_lifecycle.h"

typedef struct {
  int rejoin_route;
  int attempt_orbit_escape;
  int reacquire_free_space;
} ControllerAvoidanceRecoveryOutput;

typedef enum {
  CONTROLLER_AVOIDANCE_RECOVERY_CONTINUE = 0,
  CONTROLLER_AVOIDANCE_RECOVERY_STOP_AFTER_ESCAPE = 1,
  CONTROLLER_AVOIDANCE_RECOVERY_SKIP_LOOPED_WAYPOINT = 2,
} ControllerAvoidanceRecoveryAfterOrbit;

void controller_avoidance_recovery_resolve(
    const ControllerAvoidanceLifecycleDecision *decision,
    int has_next_waypoint,
    ControllerAvoidanceRecoveryOutput *output);
ControllerAvoidanceRecoveryAfterOrbit controller_avoidance_recovery_after_orbit(
    int orbit_escape_succeeded,
    int has_next_waypoint);

#endif
