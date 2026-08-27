#include "controller_navigation_perception.h"

static ControllerNavigationPerceptionConfig config(void) {
  ControllerNavigationPerceptionConfig value = {0};
  value.camera_range_fallback = 1.0;
  value.camera_min_fov = 0.8;
  value.camera_front_fov_factor = 0.46;
  value.camera_range_margin = 0.18;
  value.camera_min_score = 0.7;
  value.camera_min_detection_count = 3;
  value.camera_offset_deadband = 0.1;
  value.avoidance.max_trace_range = 5.0;
  value.avoidance.track_caution_range = 1.0;
  value.avoidance.expected_wall_soft_stop_range = 0.5;
  value.avoidance.expected_wall_slowdown_range = 0.8;
  value.avoidance.pass_center_clear_range = 0.8;
  value.avoidance.pass_gap_max_angle_rad = 0.5;
  value.avoidance.pass_side_danger_range = 0.4;
  value.avoidance.avoid_side_trigger_range = 0.5;
  value.avoidance.track_hard_priority_range = 0.7;
  value.avoidance.reflex_side_release_range = 0.8;
  value.avoidance.avoid_recover_range = 1.0;
  value.avoidance.avoid_trigger_range = 0.7;
  value.avoidance.avoid_stop_range = 0.4;
  value.avoidance.track_slow_range = 1.2;
  return value;
}

int main(void) {
  LidarObstacleContext lidar = {0};
  ControllerNavigationPerceptionConfig perception_config = config();
  ControllerNavigationPerceptionInput input = {
      .lidar_context = &lidar,
      .camera_visible = 1,
      .camera_angle = 0.0,
      .camera_fov = 1.0,
      .camera_range = 0.6,
      .camera_score = 0.8,
      .camera_detection_count = 1,
      .camera_center_offset = 0.3,
  };
  ControllerNavigationPerceptionOutput output = {0};
  controller_navigation_perception_prepare(
      &input, &perception_config, &output);
  if (!output.camera_visual_front_obstacle) return 1;
  if (output.camera_preferred_turn_sign != 1.0) return 2;
  if (!output.avoidance.should_start_avoidance) return 3;

  input.camera_angle = 0.6;
  controller_navigation_perception_prepare(
      &input, &perception_config, &output);
  if (output.camera_visual_front_obstacle) return 4;

  input.camera_angle = 0.0;
  input.camera_score = 0.1;
  input.camera_detection_count = 2;
  controller_navigation_perception_prepare(
      &input, &perception_config, &output);
  if (output.camera_visual_front_obstacle) return 5;

  input.camera_detection_count = 3;
  input.camera_range = 0.0;
  input.camera_center_offset = -0.3;
  controller_navigation_perception_prepare(
      &input, &perception_config, &output);
  if (!output.camera_visual_front_obstacle) return 6;
  if (output.camera_preferred_turn_sign != -1.0) return 7;

  input.camera_center_offset = 0.05;
  controller_navigation_perception_prepare(
      &input, &perception_config, &output);
  if (output.camera_preferred_turn_sign != 0.0) return 8;

  return 0;
}
