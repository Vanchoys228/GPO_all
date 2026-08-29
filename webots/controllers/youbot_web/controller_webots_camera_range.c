#include "controller_webots_camera_range.h"

#include "controller_camera_fusion.h"

double controller_webots_camera_range_from_lidar(
    const ControllerWebotsSensors *sensors,
    int lidar_available,
    int lidar_resolution,
    double lidar_fov,
    double relative_angle,
    double search_window_rad,
    double min_range,
    double max_range,
    double fallback_range) {
  if (!sensors || !lidar_available || !controller_webots_sensors_has_lidar(sensors) ||
      lidar_resolution <= 1 || lidar_fov <= 1e-9) return fallback_range;
  const float *ranges = controller_webots_sensors_lidar_ranges(sensors);
  if (!ranges) return fallback_range;
  return controller_camera_fusion_estimate_range(
      ranges, lidar_resolution, lidar_fov, relative_angle, search_window_rad,
      min_range, max_range, fallback_range);
}
