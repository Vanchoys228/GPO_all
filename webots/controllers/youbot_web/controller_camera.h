#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_H

typedef struct {
  int red;
  int green;
  int blue;
} ControllerCameraPixel;

typedef ControllerCameraPixel (*ControllerCameraPixelReader)(void *context, int x, int y);

typedef struct {
  int width;
  int height;
  double crop_x_min;
  double crop_x_max;
  double crop_y_min;
  double crop_y_max;
  int sample_step;
  double min_obstacle_score;
} ControllerCameraAnalysisConfig;

typedef struct {
  int visible;
  int samples;
  int hits;
  double score;
  double center_offset;
  double fallback_range_m;
  int min_hit_x;
  int max_hit_x;
  int min_hit_y;
  int max_hit_y;
} ControllerCameraObservation;

typedef struct {
  double x;
  double y;
  double heading;
} ControllerCameraPose;

typedef struct {
  double sensor_local_x;
  double sensor_local_y;
  double min_trace_range;
  double max_trace_range;
  double free_ray_min_range;
  double free_ray_margin;
  double free_ray_step;
  double near_robot_ignore_radius;
} ControllerCameraMapGeometryConfig;

typedef struct {
  double x;
  double y;
} ControllerCameraMapPoint;

ControllerCameraAnalysisConfig controller_camera_default_config(int width, int height);
void controller_camera_observation_reset(ControllerCameraObservation *observation);
void controller_camera_analyze(
    const ControllerCameraAnalysisConfig *config,
    ControllerCameraPixelReader read_pixel,
    void *pixel_context,
    ControllerCameraObservation *observation);
int controller_camera_obstacle_point(
    const ControllerCameraMapGeometryConfig *config,
    const ControllerCameraPose *pose,
    double relative_angle,
    double range,
    ControllerCameraMapPoint *point);
int controller_camera_free_ray_points(
    const ControllerCameraMapGeometryConfig *config,
    const ControllerCameraPose *pose,
    double relative_angle,
    double range,
    ControllerCameraMapPoint *points,
    int capacity);

#endif
