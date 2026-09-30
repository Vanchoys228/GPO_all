#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_VIRTUAL_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_VIRTUAL_H

typedef struct {
  double lidar_fov;
  double effective_fov;
  double min_range;
  double max_range;
  double caution_range;
  double stop_range;
  double cluster_jump;
} ControllerCameraVirtualConfig;

typedef struct {
  double angle;
  double range;
  int beams;
} ControllerCameraVirtualCluster;

typedef struct {
  int total_beams;
  int close_beams;
  double weighted_offset_sum;
  double weight_sum;
  int cluster_count;
} ControllerCameraVirtualSummary;

typedef struct {
  int screen_x;
  int bottom_y;
  int width;
  int height;
  double danger;
} ControllerCameraVirtualBox;

typedef struct {
  int visible;
  double score;
  double center_offset;
  int detection_count;
} ControllerCameraVirtualObservation;

void controller_camera_virtual_collect(
    const float *ranges,
    int resolution,
    const ControllerCameraVirtualConfig *config,
    ControllerCameraVirtualCluster *clusters,
    int capacity,
    ControllerCameraVirtualSummary *summary);
void controller_camera_virtual_sort_by_range_desc(
    ControllerCameraVirtualCluster *clusters,
    int count);
int controller_camera_virtual_box(
    const ControllerCameraVirtualCluster *cluster,
    const ControllerCameraVirtualConfig *config,
    int frame_width,
    int frame_height,
    ControllerCameraVirtualBox *box);
ControllerCameraVirtualObservation controller_camera_virtual_observation(
    const ControllerCameraVirtualSummary *summary,
    double min_score);

#endif
