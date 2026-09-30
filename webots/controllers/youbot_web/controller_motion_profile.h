#ifndef YOUBOT_WEB_CONTROLLER_MOTION_PROFILE_H
#define YOUBOT_WEB_CONTROLLER_MOTION_PROFILE_H

typedef struct {
  double cruise_speed_mps;
  double payload_kg;
  double battery_range_units;
} ControllerMotionProfile;

typedef struct {
  double linear_speed_mps;
  double angular_speed_rad_s;
  double battery_speed_factor;
} ControllerMotionLimits;

int controller_motion_profile_load_file(
    const char *path,
    ControllerMotionProfile *profile);
void controller_motion_profile_apply(
    ControllerMotionProfile *profile,
    ControllerMotionLimits *limits);

#endif
