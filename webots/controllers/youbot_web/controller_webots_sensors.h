#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SENSORS_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SENSORS_H

#include <webots/robot.h>

typedef struct {
  WbDeviceTag lidar;
  WbDeviceTag camera;
} ControllerWebotsSensors;

void controller_webots_sensors_init(ControllerWebotsSensors *sensors);
int controller_webots_sensors_init_lidar(
    ControllerWebotsSensors *sensors,
    int time_step,
    int *resolution,
    double *fov,
    double *max_range);
int controller_webots_sensors_init_camera(
    ControllerWebotsSensors *sensors,
    int sampling_period,
    int *width,
    int *height,
    double *fov);
int controller_webots_sensors_has_lidar(const ControllerWebotsSensors *sensors);
int controller_webots_sensors_has_camera(const ControllerWebotsSensors *sensors);
const float *controller_webots_sensors_lidar_ranges(const ControllerWebotsSensors *sensors);
const unsigned char *controller_webots_sensors_camera_image(
    const ControllerWebotsSensors *sensors);
void controller_webots_sensors_camera_pixel(
    const unsigned char *image,
    int width,
    int x,
    int y,
    int *red,
    int *green,
    int *blue);
int controller_webots_sensors_save_camera_image(
    const ControllerWebotsSensors *sensors, const char *path, int quality);

#endif
