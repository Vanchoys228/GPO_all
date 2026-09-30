#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_RUNTIME_H

#include "controller_drive.h"

void drive_webots_base(
    void *context,
    const ControllerDriveConfig *config,
    double vx,
    double vy,
    double omega);
void stop_robot(void);
void init_pose_tracking(void);
void reset_robot_pose(void);
void read_pose(double *x, double *y, double *heading);
void reset_navigation_mode(void);
void begin_navigation_for_waypoint(int waypoint_index, double x, double y);
void clear_local_navigation_state(void);
void reset_route_avoidance_metrics(void);
int route_off_route_active_now(void);
void update_route_avoidance_metrics(void);
void run_navigation_step(void);

#endif
