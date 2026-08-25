#ifndef YOUBOT_WEB_CONTROLLER_PATHS_H
#define YOUBOT_WEB_CONTROLLER_PATHS_H

#define CONTROLLER_PATH_CAPACITY 512

typedef struct {
  char route[CONTROLLER_PATH_CAPACITY];
  char limit_zones[CONTROLLER_PATH_CAPACITY];
  char surface_zones[CONTROLLER_PATH_CAPACITY];
  char robot_state[CONTROLLER_PATH_CAPACITY];
  char robot_state_temp[CONTROLLER_PATH_CAPACITY];
  char motion_profile[CONTROLLER_PATH_CAPACITY];
  char runtime_command[CONTROLLER_PATH_CAPACITY];
  char obstacle_map[CONTROLLER_PATH_CAPACITY];
  char obstacle_map_temp[CONTROLLER_PATH_CAPACITY];
  char obstacle_map_csv[CONTROLLER_PATH_CAPACITY];
  char obstacle_map_csv_temp[CONTROLLER_PATH_CAPACITY];
  char camera_map[CONTROLLER_PATH_CAPACITY];
  char camera_map_temp[CONTROLLER_PATH_CAPACITY];
  char camera_map_csv[CONTROLLER_PATH_CAPACITY];
  char camera_map_csv_temp[CONTROLLER_PATH_CAPACITY];
  char camera_frame_bmp[CONTROLLER_PATH_CAPACITY];
  char camera_frame_bmp_temp[CONTROLLER_PATH_CAPACITY];
  char camera_frame_jpeg[CONTROLLER_PATH_CAPACITY];
  char camera_frame_jpeg_temp[CONTROLLER_PATH_CAPACITY];
} ControllerPaths;

int controller_paths_init(ControllerPaths *paths, const char *state_dir);

#endif
