#ifndef YOUBOT_WEB_CONTROLLER_DRIVE_H
#define YOUBOT_WEB_CONTROLLER_DRIVE_H

typedef struct {
  double wheel_radius;
  double wheel_coupling;
  double max_wheel_speed_rad_s;
  double acceleration_limit_rad_s2;
  double deceleration_limit_rad_s2;
  double time_step_seconds;
} ControllerDriveConfig;

void controller_drive_step(
    const ControllerDriveConfig *config,
    double vx,
    double vy,
    double omega,
    const int enabled[4],
    double applied_wheel_speeds[4]);

#endif
