#include <webots/robot.h>
#include <webots/supervisor.h>

#include "controller_avoidance.h"
#include "controller_application_state.h"
#include "controller_avoidance_start.h"
#include "controller_avoidance_lifecycle.h"
#include "controller_avoidance_recovery.h"
#include "controller_avoidance_service.h"
#include "controller_avoidance_presentation.h"
#include "controller_camera.h"
#include "controller_camera_fusion.h"
#include "controller_camera_map.h"
#include "controller_camera_map_io.h"
#include "controller_camera_render.h"
#include "controller_camera_virtual.h"
#include "controller_control_config.h"
#include "controller_io.h"
#include "controller_drive.h"
#include "controller_lidar_math.h"
#include "controller_lidar_scan.h"
#include "controller_lifecycle.h"
#include "controller_math.h"
#include "controller_mapping_route_io.h"
#include "controller_mapping_obstacles.h"
#include "controller_mapping_scan.h"
#include "controller_mapping_scan_service.h"
#include "controller_mapping_scan_transition.h"
#include "controller_mapping_survey_contour_service.h"
#include "controller_mapping_survey_coverage_service.h"
#include "controller_mapping_survey_generator_callbacks.h"
#include "controller_mapping_survey_route_generation_service.h"
#include "controller_mapping_survey_grid_adapter.h"
#include "controller_mapping_survey_safety.h"
#include "controller_mapping_survey_safety_service.h"
#include "controller_mapping_survey_lifecycle_service.h"
#include "controller_mapping_survey_runtime_safety.h"
#include "controller_mapping_runtime.h"
#include "controller_motion_profile.h"
#include "controller_motion_profile_reload_service.h"
#include "controller_navigation_context.h"
#include "controller_navigation_adapter.h"
#include "controller_navigation_state.h"
#include "controller_navigation_metrics.h"
#include "controller_navigation_lidar.h"
#include "controller_navigation_perception.h"
#include "controller_navigation_presentation.h"
#include "controller_navigation_route.h"
#include "controller_navigation_session.h"
#include "controller_navigation_service.h"
#include "controller_navigation_tracking.h"
#include "controller_navigation_zone_guard.h"
#include "controller_obstacle_map.h"
#include "controller_paths.h"
#include "controller_perception_runtime.h"
#include "controller_route.h"
#include "controller_route_zone_service.h"
#include "controller_route_zone_reload_service.h"
#include "controller_runtime.h"
#include "controller_runtime_command.h"
#include "controller_runtime_command_reload_service.h"
#include "controller_survey_grid.h"
#include "controller_survey_geometry.h"
#include "controller_survey_integration.h"
#include "controller_survey_lifecycle.h"
#include "controller_survey_state.h"
#include "controller_step.h"
#include "controller_telemetry.h"
#include "controller_telemetry_publisher.h"
#include "controller_telemetry_service.h"
#include "controller_types.h"
#include "controller_webots_devices.h"
#include "controller_webots_adapter.h"
#include "controller_webots_camera_adapter.h"
#include "controller_webots_pose.h"
#include "controller_webots_simulation.h"
#include "controller_webots_motion_state.h"
#include "controller_webots_camera_range.h"
#include "controller_webots_camera_perception.h"
#include "controller_webots_camera_map_sync.h"
#include "controller_webots_zone_sync.h"
#include "controller_webots_sensors.h"
#include "controller_zone_geometry.h"
#include "controller_zones.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 16
#define PI 3.14159265358979323846
#define KINEMATIC_LINEAR_SPEED 0.80
#define KINEMATIC_ANGULAR_SPEED 1.6
#define DEFAULT_CRUISE_SPEED_MPS 0.22
#define MIN_CRUISE_SPEED_MPS 0.05
#define MAX_CRUISE_SPEED_MPS 0.8
#define DEFAULT_PAYLOAD_KG 0.0
#define DEFAULT_BATTERY_RANGE_UNITS 100.0
#define POSITION_TOLERANCE 0.05
#define HEADING_TOLERANCE_RAD 0.08
#define FINAL_ALIGN_DISTANCE 0.18
#define TRACK_SLOW_RADIUS 0.22
#define TURN_ENTER_ERROR_RAD 0.42
#define TURN_EXIT_ERROR_RAD 0.12
#define TRACK_REENTER_TURN_RAD 0.56
#define TURN_HEADING_GAIN 3.8
#define TRACK_HEADING_GAIN 3.0
#define FINAL_ALIGN_GAIN 3.4
#define TRACK_CROSS_TRACK_GAIN 0.22
#define TRACK_LOOKAHEAD_MIN 0.16
#define TRACK_LOOKAHEAD_MAX 0.45
#define TRACK_MIN_LINEAR_SPEED 0.045
#define TRACK_DIRECT_HEADING_CROSSTRACK 0.10
#define TRACK_REANCHOR_CROSSTRACK 0.12
#define TRACK_RELAXED_MODE_CROSSTRACK 0.035
#define MIN_ANGULAR_COMMAND 0.28
#define START_X 0.0
#define START_Z 0.0
#define START_HEIGHT 0.102838
#define ROUTE_RELOAD_INTERVAL 20
#define MOTION_RELOAD_INTERVAL 20
#define ZONE_RELOAD_INTERVAL 10
#define RUNTIME_COMMAND_RELOAD_INTERVAL 6
#define MAX_ZONE_NODES 512
#define MAX_SURFACE_ZONE_NODES 128
#define MAX_RUNTIME_OBSTACLE_NODES 96
#define MAX_OBSTACLE_TRACE_POINTS 520
#define WALL_THICKNESS 0.08
#define WALL_HEIGHT 0.45
#define ZONE_CLEARANCE 0.32
#define LIDAR_SAMPLE_STRIDE 2
#define LIDAR_TRACE_SPACING 0.1
#define LIDAR_MIN_TRACE_RANGE 0.12
#define LIDAR_MAX_TRACE_RANGE 3.0
#define LIDAR_RANGE_JUMP_TOLERANCE 0.55
#define LIDAR_NEAR_ROBOT_IGNORE_RADIUS 0.25
#define LIDAR_TRACE_TTL_SECONDS 6.0
#define LIDAR_TRACE_MIN_CONFIDENCE 0.0
#define LIDAR_SNAP_STEP 0.03
#define LIDAR_LOCAL_X 0.24
#define LIDAR_LOCAL_Y 0.0
#define LIDAR_FRONT_SECTOR_RAD 0.40
#define LIDAR_CENTER_SECTOR_RAD 0.13
#define LIDAR_FRONT_CORNER_MIN_RAD 0.14
#define LIDAR_FRONT_CORNER_MAX_RAD 0.42
#define LIDAR_AVOID_TRIGGER_RANGE 1.18
#define LIDAR_AVOID_STOP_RANGE 0.31
#define LIDAR_AVOID_RECOVER_RANGE 1.35
#define LIDAR_AVOID_REVERSE_RANGE 0.21
#define LIDAR_AVOID_HOLD_STEPS 34
#define LIDAR_AVOID_SIDE_TRIGGER_RANGE 0.36
#define LIDAR_AVOID_SIDE_DANGER_RANGE 0.15
#define LIDAR_AVOID_FOLLOW_RANGE 0.74
#define LIDAR_AVOID_FOLLOW_TARGET 0.29
#define LIDAR_AVOID_RELEASE_STEPS 24
#define LIDAR_AVOID_MIN_CONTOUR_STEPS 18
#define LIDAR_AVOID_CLEAR_STEPS 12
#define LIDAR_AVOID_LEAVE_PROGRESS 0.16
#define LIDAR_AVOID_LEAVE_HEADING_RAD 0.44
#define LIDAR_AVOID_TARGET_CLEAR_RANGE 1.16
#define LIDAR_AVOID_ESCAPE_STEPS 16
#define LIDAR_AVOID_STUCK_STEPS 24
#define LIDAR_AVOID_STUCK_POSE_EPS 0.004
#define LIDAR_AVOID_STUCK_PROGRESS_EPS 0.001
#define LIDAR_TRACK_CAUTION_RANGE 1.45
#define LIDAR_TRACK_HARD_PRIORITY_RANGE 1.24
#define LIDAR_TRACK_SLOW_RANGE 1.12
#define LIDAR_TRACK_SIDE_BIAS_RANGE 0.66
#define LIDAR_TRACK_MAX_HEADING_BIAS 0.40
#define LIDAR_PRIORITY_HOLD_STEPS 18
#define LIDAR_PRIORITY_SWITCH_MARGIN 0.18
#define LIDAR_PRIORITY_CENTER_MARGIN 0.08
#define POSE_RELOCATION_DISTANCE 0.45
#define POSE_RELOCATION_HEADING_RAD 1.20
#define FREE_SPACE_RECOVERY_STEPS 10
#define ZONE_WALL_EXPECTED_TOLERANCE 0.18
#define EXPECTED_WALL_SOFT_STOP_RANGE 0.24
#define EXPECTED_WALL_SLOWDOWN_RANGE 0.46
#define LIDAR_REFLEX_SIDE_RELEASE_RANGE 0.62
#define LIDAR_REFLEX_SWITCH_MARGIN 0.18
#define LIDAR_REFLEX_TARGET_GAIN 0.16
#define LIDAR_REFLEX_MAX_LINEAR_SPEED_FACTOR 0.58
#define LIDAR_GAP_MIN_RANGE 0.46
#define LIDAR_GAP_SWITCH_RANGE_BONUS 0.14
#define LIDAR_DETOUR_FORWARD_M 0.72
#define LIDAR_DETOUR_SIDE_M 0.54
#define LIDAR_DETOUR_REACHED_M 0.26
#define LIDAR_DETOUR_MAX_RANGE_M 1.35
#define LIDAR_PASS_CENTER_CLEAR_RANGE 0.78
#define LIDAR_PASS_GAP_MAX_ANGLE_RAD 0.54
#define LIDAR_PASS_SIDE_DANGER_RANGE 0.18
#define LIDAR_PASS_CRUISE_SPEED_FACTOR 0.88
#define LIDAR_PASS_MAX_SPEED_FACTOR 0.90
#define LIDAR_AVOID_DRIVE_MAX_SPEED_FACTOR 0.72
#define LIDAR_PASS_STEER_GAIN 2.2
#define WHEEL_RADIUS 0.05
#define WHEEL_BASE_LONGITUDINAL 0.228
#define WHEEL_BASE_LATERAL 0.158
#define MAX_WHEEL_SPEED_RAD_S 18.0
#define WHEEL_ACCEL_LIMIT_RAD_S2 150.0
#define WHEEL_DECEL_LIMIT_RAD_S2 220.0
#define MAX_MAP_POINTS 4096
#define MAX_CAMERA_MAP_POINTS 2048
#define MAX_CAMERA_FREE_MAP_POINTS 4096
#define MAP_CELL_SIZE 0.06
#define CAMERA_MAP_CELL_SIZE 0.10
#define MAP_MERGE_MIN_HIT_COUNT 2
#define MAP_MERGE_MAX_AGE_S 1.0
#define MAP_WRITE_INTERVAL 60
#define CAMERA_WRITE_INTERVAL 12
#define CAMERA_CAPTURE_INTERVAL 4
#define CAMERA_FRAME_WIDTH 320
#define CAMERA_FRAME_HEIGHT 180
#define CAMERA_MAX_VIRTUAL_CLUSTERS 24
#define CAMERA_OBSTACLE_MIN_SCORE 0.025
#define CAMERA_OBSTACLE_OFFSET_DEADBAND 0.08
#define CAMERA_RANGE_FALLBACK_M 1.55
#define CAMERA_RANGE_SEARCH_WINDOW_RAD 0.10
#define CAMERA_FREE_RAY_STEP_M 0.18
#define CAMERA_FREE_RAY_MIN_RANGE_M 0.36
#define CAMERA_FREE_RAY_MARGIN_M 0.22
#define SURVEY_X_MIN -22.0
#define SURVEY_X_MAX 22.0
#define SURVEY_Y_MIN -17.0
#define SURVEY_Y_MAX 17.0
#define SURVEY_STRIP 1.2
#define MAPPING_SURVEY_GRID_CELL 0.25
#define MAPPING_SURVEY_MAX_BOUNDARY_POINTS 4096
#define MAPPING_SURVEY_CONTOUR_OFFSET 0.45
#define MAPPING_SURVEY_INTERIOR_OFFSET 0.70
#define MAPPING_SURVEY_STRIP 1.10
#define MAPPING_SURVEY_MIN_CONTOUR_STEP 1.15
#define MAPPING_SURVEY_MAX_CONTOUR_STEP 1.45
#define MAPPING_SURVEY_MIN_STRIP_LENGTH 0.65
#define MAPPING_SURVEY_RDP_EPS 0.34
#define MAPPING_SURVEY_MAP_OBSTACLE_CLEARANCE 0.45
#define MAPPING_SURVEY_MAX_EXTENT_X 22.0
#define MAPPING_SURVEY_MAX_EXTENT_Y 17.0
#define MAPPING_SURVEY_OBSTACLE_SCAN_RADIUS 0.88
#define MAPPING_SURVEY_OBSTACLE_SCAN_POINTS 10
#define MAPPING_SURVEY_OBSTACLE_SCAN_COOLDOWN_STEPS 260
#define MAPPING_SURVEY_OBSTACLE_SCAN_MIN_REPEAT_DISTANCE 1.05
#define MAPPING_SURVEY_AVOID_REPLAN_STEPS 140
#define MAPPING_SURVEY_AVOID_NO_PROGRESS_STEPS 60
#define MAPPING_SURVEY_AVOID_MAX_STEPS 360
#define MAPPING_SURVEY_AVOID_LOOP_RADIUS 0.48
#define MAPPING_SURVEY_AVOID_PROGRESS_EPS 0.035
#define MAPPING_SURVEY_AVOID_ORBIT_HEADING_RAD (PI * 2.15)
#define MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS 160
#define MAPPING_SURVEY_ESCAPE_SCAN_AHEAD 72
#define MAPPING_SURVEY_ESCAPE_MIN_TARGET_DISTANCE 0.72
#define MAPPING_SURVEY_ESCAPE_OBSTACLE_CLEARANCE 0.58
#define MAPPING_SURVEY_ESCAPE_SEGMENT_CLEARANCE 0.46
#define EPS 1e-9

#define point_in_zone controller_zone_geometry_point_in
#define point_near_zone controller_zone_geometry_point_near
#define point_near_zone_boundary controller_zone_geometry_point_near_boundary
#define zone_signed_area controller_zone_geometry_signed_area
#define segment_blocked_by_zones(ax, ay, bx, by, clearance, skip_zone_index) \
  controller_zone_geometry_segment_blocked(                                      \
      &controller_runtime.limit_zones, (ax), (ay), (bx), (by), (clearance), (skip_zone_index))
#define find_room_zone_index(robot_x, robot_y) \
  controller_zone_geometry_find_room(&controller_runtime.limit_zones, (robot_x), (robot_y))

static ControllerWebotsDevices webots_devices = {0};
static ControllerWebotsAdapter webots_adapter = {0};
static ControllerWebotsPose webots_pose = {0};
static ControllerWebotsSensors webots_sensors = {0};

static ControllerRuntime controller_runtime;

static ControllerRouteZoneService route_zone_service;
static ControllerRouteZoneReloadService route_zone_reload_service;
static ControllerWebotsSimulationNodeRegistry zone_node_registry = {0};
static ControllerWebotsSimulationNodeRegistry surface_zone_registry = {0};
static ControllerWebotsSimulationNodeRegistry runtime_obstacle_registry = {0};
static ControllerWebotsZoneSyncContext webots_zone_sync = {0};
static ControllerMappingRuntime mapping_runtime;
static ControllerMappingSurveySafetyService mapping_survey_safety_service;
#define mapping_store mapping_runtime.store
#define persistent_map mapping_store.persistent_map
#define persistent_map_count mapping_store.persistent_count
#define map_dirty mapping_store.persistent_dirty
#define mapping_camera_map mapping_store.camera_map
#define camera_map_count mapping_store.camera_count
#define camera_free_map mapping_store.camera_free_map
#define camera_free_map_count mapping_store.camera_free_count
#define camera_map_dirty mapping_store.camera_dirty
static ControllerPerceptionRuntime perception_runtime;
static ControllerApplicationState application_state;
#define step_counter application_state.step_counter
#define navigation_status application_state.status
#define navigation_error application_state.error
static ControllerWebotsMotionState motion_state = {
    {DEFAULT_CRUISE_SPEED_MPS, DEFAULT_PAYLOAD_KG, DEFAULT_BATTERY_RANGE_UNITS},
    {DEFAULT_CRUISE_SPEED_MPS, KINEMATIC_ANGULAR_SPEED, 1.0},
};
#define configured_cruise_speed_mps motion_state.profile.cruise_speed_mps
#define configured_payload_kg motion_state.profile.payload_kg
#define configured_battery_range_units motion_state.profile.battery_range_units
#define active_linear_speed_limit motion_state.limits.linear_speed_mps
#define active_angular_speed_limit motion_state.limits.angular_speed_rad_s
#define active_battery_speed_factor motion_state.limits.battery_speed_factor
static ControllerMotionProfileReloadService motion_profile_reload_service;
static ControllerRuntimeCommandReloadService runtime_command_reload_service;
#define route_avoidance_time_sec application_state.route_avoidance_time_sec
#define route_avoidance_steps application_state.route_avoidance_steps

static ControllerPaths controller_paths;
static ControllerControlConfig control_config;
#define ROUTE_PATH controller_paths.route
#define ZONE_PATH controller_paths.limit_zones
#define SURFACE_ZONE_PATH controller_paths.surface_zones
#define STATE_PATH controller_paths.robot_state
#define STATE_TEMP_PATH controller_paths.robot_state_temp
#define MOTION_PROFILE_PATH controller_paths.motion_profile
#define RUNTIME_COMMAND_PATH controller_paths.runtime_command
#define MAP_PATH controller_paths.obstacle_map
#define MAP_TEMP_PATH controller_paths.obstacle_map_temp
#define MAP_CSV_PATH controller_paths.obstacle_map_csv
#define MAP_CSV_TEMP_PATH controller_paths.obstacle_map_csv_temp
#define CAMERA_MAP_PATH controller_paths.camera_map
#define CAMERA_MAP_TEMP_PATH controller_paths.camera_map_temp
#define CAMERA_MAP_CSV_PATH controller_paths.camera_map_csv
#define CAMERA_MAP_CSV_TEMP_PATH controller_paths.camera_map_csv_temp
#define CAMERA_FRAME_BMP_PATH controller_paths.camera_frame_bmp
#define CAMERA_FRAME_BMP_TEMP_PATH controller_paths.camera_frame_bmp_temp
#define CAMERA_FRAME_JPEG_PATH controller_paths.camera_frame_jpeg
#define CAMERA_FRAME_JPEG_TEMP_PATH controller_paths.camera_frame_jpeg_temp

static int load_route(RouteData *route);
static void reset_navigation_mode(void);
static void set_status(const char *status);
static void read_pose(double *x, double *z, double *heading);
static void merge_camera_observation_into_map(double relative_angle, double range, int confidence_boost);
static void merge_camera_free_ray_into_map(double relative_angle, double range, int confidence_boost);
static double scaled_linear_floor(double factor) {
  const double floor_cap = fmin(TRACK_MIN_LINEAR_SPEED, active_linear_speed_limit);
  return clamp_value(active_linear_speed_limit * factor, 0.01, floor_cap);
}

static double scaled_linear_cap(double factor) {
  const double floor = scaled_linear_floor(0.24);
  return clamp_value(active_linear_speed_limit * factor, floor, active_linear_speed_limit);
}


static void maybe_reload_motion_profile(void) {
  if (controller_motion_profile_reload_service_run(
          &motion_profile_reload_service, step_counter, MOTION_RELOAD_INTERVAL) ==
      CONTROLLER_MOTION_PROFILE_RELOAD_CHANGED) {
    set_status("motion_profile_reloaded");
  }
}


static void set_error(const char *message) {
  controller_application_state_set_error(&application_state, message);
}

static void clear_error(void) {
  controller_application_state_clear_error(&application_state);
}

static void set_status(const char *status) {
  controller_application_state_set_status(&application_state, status);
}

static void init_sensors(void) {
  const ControllerWebotsAdapterSensorState sensor_state =
      controller_webots_adapter_init_sensors(
          &webots_sensors,
          TIME_STEP,
          CAMERA_CAPTURE_INTERVAL,
          CAMERA_FRAME_WIDTH,
          CAMERA_FRAME_HEIGHT,
          1.05);
  const ControllerPerceptionSensorMetadata metadata = {
      sensor_state.lidar_available,
      sensor_state.lidar_resolution,
      sensor_state.lidar_fov,
      sensor_state.lidar_max_range,
      sensor_state.camera_available,
      sensor_state.camera_virtual_mode,
      sensor_state.camera_width,
      sensor_state.camera_height,
      sensor_state.camera_fov,
  };
  controller_perception_runtime_configure(&perception_runtime, &metadata);
}

static double estimate_camera_range_from_lidar(double relative_angle, double fallback_range) {
  return controller_webots_camera_range_from_lidar(
      &webots_sensors,
      perception_runtime.lidar.available,
      perception_runtime.lidar.resolution,
      perception_runtime.lidar.fov,
      relative_angle, CAMERA_RANGE_SEARCH_WINDOW_RAD, LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE, fallback_range);
}

static void merge_camera_visible_frustum_into_map(double effective_fov, double default_range) {
  if (!perception_runtime.camera.available) return;

  const double half_fov = fmax(effective_fov, 0.8) * 0.5;
  for (int i = 0; i <= 8; ++i) {
    const double t = ((double)i / 8.0) * 2.0 - 1.0;
    const double angle = t * half_fov * 0.92;
    const double lidar_range = estimate_camera_range_from_lidar(angle, default_range);
    const double free_range = clamp_value(lidar_range, CAMERA_FREE_RAY_MIN_RANGE_M, LIDAR_MAX_TRACE_RANGE);
    merge_camera_free_ray_into_map(angle, free_range, 1);
  }
}

static void update_camera_obstacle_hint(void) {
  controller_perception_runtime_reset_camera_observation(
      &perception_runtime, step_counter);

  if (!perception_runtime.camera.available ||
      !controller_webots_sensors_has_camera(&webots_sensors) ||
      perception_runtime.camera.width <= 0 || perception_runtime.camera.height <= 0) return;

  const unsigned char *image = controller_webots_sensors_camera_image(&webots_sensors);
  if (!image) return;

  const double effective_fov =
      perception_runtime.camera.fov > EPS ? perception_runtime.camera.fov : 1.05;
  merge_camera_visible_frustum_into_map(effective_fov, CAMERA_RANGE_FALLBACK_M);

  ControllerWebotsCameraPerception perception;
  controller_webots_camera_perception_analyze(
      image,
      perception_runtime.camera.width,
      perception_runtime.camera.height,
      effective_fov,
      CAMERA_OBSTACLE_MIN_SCORE, &perception);
  ControllerPerceptionCameraObservation observation = {
      .score = perception.score,
      .update_step = step_counter,
  };
  if (perception.visible) {
    observation.visible = 1;
    observation.center_offset = perception.center_offset;
    observation.angle = perception.angle;
    observation.range =
        estimate_camera_range_from_lidar(observation.angle, perception.fallback_range_m);
    observation.detection_count = perception.detection_count;
    merge_camera_free_ray_into_map(observation.angle, observation.range, 2);
    merge_camera_observation_into_map(
        observation.angle,
        observation.range,
        perception.confidence_boost);
  }
  controller_perception_runtime_update_camera(&perception_runtime, &observation);
}

static void draw_virtual_camera_overlay(unsigned char *pixels, double effective_fov) {
  controller_camera_render_reticle(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);

  if (controller_runtime.route.count > 0 && controller_runtime.current_waypoint_index < controller_runtime.route.count &&
      controller_webots_pose_is_ready(&webots_pose)) {
    double x = 0.0;
    double z = 0.0;
    double heading = 0.0;
    read_pose(&x, &z, &heading);
    const Waypoint *target = &controller_runtime.route.waypoints[controller_runtime.current_waypoint_index];
    const double target_angle = wrap_angle(atan2(target->z - z, target->x - x) - heading);
    if (fabs(target_angle) < effective_fov * 0.5) {
      const double offset = clamp_value(target_angle / (effective_fov * 0.5), -1.0, 1.0);
      const int target_x = (int)((offset * 0.5 + 0.5) * (CAMERA_FRAME_WIDTH - 1));
      controller_camera_render_waypoint_marker(
          pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, target_x);
    }
  }
}

static int write_virtual_camera_frame() {
  static unsigned char pixels[CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT * 3];
  ControllerCameraVirtualCluster clusters[CAMERA_MAX_VIRTUAL_CLUSTERS];
  ControllerCameraVirtualSummary cluster_summary = {0};
  controller_camera_render_background(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);

  controller_perception_runtime_reset_camera_observation(
      &perception_runtime, step_counter);

  const double effective_fov =
      perception_runtime.camera.fov > EPS ? perception_runtime.camera.fov : 1.05;
  const ControllerCameraVirtualConfig virtual_config = {
      perception_runtime.lidar.fov,
      effective_fov,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE,
      LIDAR_TRACK_CAUTION_RANGE,
      LIDAR_AVOID_STOP_RANGE,
      0.42,
  };
  if (perception_runtime.lidar.available &&
      controller_webots_sensors_has_lidar(&webots_sensors) &&
      perception_runtime.lidar.resolution > 1 && perception_runtime.lidar.fov > EPS) {
    const float *ranges = controller_webots_sensors_lidar_ranges(&webots_sensors);
    controller_camera_virtual_collect(
        ranges,
        perception_runtime.lidar.resolution,
        &virtual_config,
        clusters,
        CAMERA_MAX_VIRTUAL_CLUSTERS,
        &cluster_summary);

    for (int i = 0; ranges && i < perception_runtime.lidar.resolution; ++i) {
      const double alpha =
          (double)i / (double)(perception_runtime.lidar.resolution - 1);
      const double beam_angle =
          -0.5 * perception_runtime.lidar.fov + alpha * perception_runtime.lidar.fov;
      const double range = ranges[i];
      const int valid = fabs(beam_angle) <= effective_fov * 0.5 &&
                        controller_math_is_finite(range) && range > LIDAR_MIN_TRACE_RANGE;
      if (valid && (i % 10) == 0) {
        merge_camera_free_ray_into_map(
            beam_angle,
            clamp_value(range, CAMERA_FREE_RAY_MIN_RANGE_M, LIDAR_MAX_TRACE_RANGE),
            1);
      }
    }
  }

  controller_camera_virtual_sort_by_range_desc(clusters, cluster_summary.cluster_count);

  for (int i = 0; i < cluster_summary.cluster_count; ++i) {
    const double range = clamp_value(clusters[i].range, 0.12, LIDAR_MAX_TRACE_RANGE);
    ControllerCameraVirtualBox box;
    if (!controller_camera_virtual_box(
            &clusters[i], &virtual_config, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, &box)) {
      continue;
    }
    controller_camera_render_box(
        pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
        box.screen_x, box.bottom_y, box.width, box.height, box.danger);
    merge_camera_observation_into_map(
        clusters[i].angle,
        range,
        1 + (int)clamp_value((double)clusters[i].beams / 2.0, 1.0, 8.0));
  }

  const ControllerCameraVirtualObservation virtual_observation =
      controller_camera_virtual_observation(&cluster_summary, CAMERA_OBSTACLE_MIN_SCORE);
  ControllerPerceptionCameraObservation observation = {
      .score = virtual_observation.score,
      .update_step = step_counter,
  };
  if (virtual_observation.visible) {
      observation.visible = 1;
      observation.center_offset = virtual_observation.center_offset;
      observation.angle = observation.center_offset * effective_fov * 0.5;
      observation.range =
          estimate_camera_range_from_lidar(observation.angle, CAMERA_RANGE_FALLBACK_M);
      observation.detection_count = virtual_observation.detection_count;
  }
  controller_perception_runtime_update_camera(&perception_runtime, &observation);

  draw_virtual_camera_overlay(pixels, effective_fov);
  return write_bmp24(CAMERA_FRAME_BMP_TEMP_PATH, pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);
}

static void maybe_write_camera_frame(void) {
  const ControllerCameraPublicationRequest request =
      controller_perception_runtime_camera_publication_request(
          &perception_runtime, (step_counter % CAMERA_WRITE_INTERVAL) == 0);
  if (!request.requested) return;

  if (request.virtual_mode) {
    if (write_virtual_camera_frame() == 0) {
      controller_webots_camera_adapter_publish_frame(
          CAMERA_FRAME_BMP_TEMP_PATH, CAMERA_FRAME_BMP_PATH, "camera_frame.bmp", "image/bmp",
          wb_robot_get_time(), &perception_runtime.camera.frame);
    }
    return;
  }

  if (!controller_webots_sensors_has_camera(&webots_sensors)) return;
  if (perception_runtime.camera.obstacle_update_step != step_counter) {
    update_camera_obstacle_hint();
  }

  if (controller_webots_sensors_save_camera_image(
          &webots_sensors, CAMERA_FRAME_JPEG_TEMP_PATH, 70) == 0) {
    controller_webots_camera_adapter_publish_frame(
        CAMERA_FRAME_JPEG_TEMP_PATH, CAMERA_FRAME_JPEG_PATH, "camera_frame.jpg", "image/jpeg",
        wb_robot_get_time(), &perception_runtime.camera.frame);
  }
}

static void maybe_update_camera_perception(void) {
  if (!perception_runtime.camera.available || perception_runtime.camera.virtual_mode ||
      !controller_webots_sensors_has_camera(&webots_sensors)) return;
  if ((step_counter % CAMERA_CAPTURE_INTERVAL) != 0) return;
  update_camera_obstacle_hint();
}

static void drive_webots_base(
    void *context,
    const ControllerDriveConfig *config,
    double vx,
    double vy,
    double omega) {
  controller_webots_devices_drive((ControllerWebotsDevices *)context, config, vx, vy, omega);
}

static void set_base_velocity(double vx, double vy, double omega) {
  controller_webots_adapter_apply_velocity(&webots_adapter, vx, vy, omega);
}
static void stop_robot() {
  controller_webots_adapter_stop(&webots_adapter);
}

static void init_pose_tracking() {
  controller_webots_adapter_init_pose(&webots_pose);
}

static void reset_robot_pose() {
  if (!controller_webots_adapter_reset_pose(
          &webots_pose, START_X, START_Z, START_HEIGHT, 0.0)) return;
  controller_webots_devices_reset_wheels(&webots_devices);
  controller_runtime.avoidance.hold_steps = 0;
  controller_runtime.avoidance.turn_sign = 1.0;
  controller_runtime.navigation_pose_history = (ControllerNavigationPoseHistory){
      START_X,
      START_Z,
      0.0,
      1,
  };
  reset_navigation_mode();
  stop_robot();
}

static void read_pose(double *x, double *z, double *heading) {
  controller_webots_adapter_read_pose(&webots_pose, x, z, heading);
}

static void apply_kinematic_step(
    double x,
    double z,
    double heading,
    double linear_speed,
    double angular_speed) {
  (void)x;
  (void)z;
  (void)heading;

  const double limited_linear = clamp_value(
      linear_speed,
      -active_linear_speed_limit,
      active_linear_speed_limit);
  const double limited_angular = clamp_value(
      angular_speed,
      -active_angular_speed_limit,
      active_angular_speed_limit);
  set_base_velocity(limited_linear, 0.0, limited_angular);
}

static double sign_or_one(double value) {
  return value < 0.0 ? -1.0 : 1.0;
}

static void reset_navigation_mode(void) {
  controller_navigation_state_reset(&controller_runtime, START_X, START_Z);
}

static void begin_navigation_for_waypoint(int waypoint_index, double current_x, double current_z) {
  controller_navigation_state_begin(&controller_runtime, waypoint_index, current_x, current_z);
}

static void ensure_navigation_waypoint_initialized(double current_x, double current_z) {
  controller_navigation_state_ensure(&controller_runtime, current_x, current_z);
}

static void clear_local_navigation_state(void) {
  controller_navigation_state_clear_local(&controller_runtime, START_X, START_Z);
}

static void reset_route_avoidance_metrics(void) {
  controller_application_state_reset_route_avoidance(&application_state);
}

static int route_off_route_active_now() {
  return controller_navigation_metrics_off_route(controller_runtime.route_finished, controller_runtime.route.count, controller_runtime.avoidance.active, navigation_status);
}

static void update_route_avoidance_metrics(void) {
  if (route_off_route_active_now()) {
    controller_application_state_tick_route_avoidance(
        &application_state, 1, (double)TIME_STEP / 1000.0);
  }
}

static void compute_lidar_obstacle_context(LidarObstacleContext *context,
                                           double target_beam_angle,
                                           double preferred_turn_sign) {
  if (!context) return;

  const double effective_max_range =
      perception_runtime.lidar.max_range > EPS
          ? fmin(perception_runtime.lidar.max_range, LIDAR_MAX_TRACE_RANGE)
          : LIDAR_MAX_TRACE_RANGE;
  controller_lidar_context_init(context, effective_max_range);

  if (!perception_runtime.lidar.available ||
      !controller_webots_sensors_has_lidar(&webots_sensors) ||
      perception_runtime.lidar.resolution <= 1 || perception_runtime.lidar.fov <= EPS) return;

  const float *ranges = controller_webots_sensors_lidar_ranges(&webots_sensors);
  if (!ranges) return;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  const double sensor_origin_x =
      robot_x + cos(heading) * LIDAR_LOCAL_X - sin(heading) * LIDAR_LOCAL_Y;
  const double sensor_origin_y =
      robot_y + sin(heading) * LIDAR_LOCAL_X + cos(heading) * LIDAR_LOCAL_Y;
  const double sigma = fmax(perception_runtime.lidar.fov * 0.22, 0.22);
  const ControllerLidarContextConfig context_config = {
      effective_max_range,
      LIDAR_TRACK_CAUTION_RANGE,
      LIDAR_AVOID_STOP_RANGE,
      LIDAR_GAP_MIN_RANGE,
      LIDAR_FRONT_SECTOR_RAD,
      LIDAR_CENTER_SECTOR_RAD,
      LIDAR_FRONT_CORNER_MIN_RAD,
      LIDAR_FRONT_CORNER_MAX_RAD,
      sigma,
  };

  for (int i = 0; i < perception_runtime.lidar.resolution; i += LIDAR_SAMPLE_STRIDE) {
    const double raw_range = (double)ranges[i];
    const int range_is_finite = controller_math_is_finite(raw_range);
    const double sensed_range = range_is_finite
                                    ? clamp_value(raw_range, 0.0, effective_max_range)
                                    : effective_max_range;
    const int obstacle_hit =
        range_is_finite &&
        raw_range >= LIDAR_MIN_TRACE_RANGE &&
        raw_range < effective_max_range - 0.02 &&
        controller_lidar_hit_is_consistent(
            ranges,
            perception_runtime.lidar.resolution,
            i,
            raw_range,
            effective_max_range,
            LIDAR_SAMPLE_STRIDE,
            LIDAR_MIN_TRACE_RANGE,
            LIDAR_RANGE_JUMP_TOLERANCE);

    const double alpha = perception_runtime.lidar.resolution > 1
                             ? (double)i / (double)(perception_runtime.lidar.resolution - 1)
                             : 0.5;
    const double beam_angle =
        -0.5 * perception_runtime.lidar.fov + alpha * perception_runtime.lidar.fov;
    int expected_zone_wall = 0;

    if (obstacle_hit) {
      const double world_angle = heading - beam_angle;
      const double hit_x = sensor_origin_x + cos(world_angle) * raw_range;
      const double hit_y = sensor_origin_y + sin(world_angle) * raw_range;

      for (int zone_index = 0; zone_index < controller_runtime.limit_zones.count; ++zone_index) {
        if (point_near_zone_boundary(hit_x,
                                     hit_y,
                                     &controller_runtime.limit_zones.zones[zone_index],
                                     ZONE_WALL_EXPECTED_TOLERANCE)) {
          expected_zone_wall = 1;
          break;
        }
      }
    }

    controller_lidar_context_observe(
        context,
        &context_config,
        sensed_range,
        beam_angle,
        obstacle_hit,
        expected_zone_wall,
        target_beam_angle,
        preferred_turn_sign);
  }
}

static void capture_lidar_trace(void) {
  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);
  const ControllerLidarScanConfig scan_config = {
      LIDAR_SAMPLE_STRIDE,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE,
      LIDAR_RANGE_JUMP_TOLERANCE,
      LIDAR_LOCAL_X,
      LIDAR_LOCAL_Y,
      LIDAR_FRONT_SECTOR_RAD,
      LIDAR_CENTER_SECTOR_RAD,
      LIDAR_FRONT_CORNER_MIN_RAD,
      LIDAR_FRONT_CORNER_MAX_RAD,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
      LIDAR_SNAP_STEP,
      LIDAR_TRACE_SPACING,
  };
  controller_perception_runtime_capture_lidar(
      &perception_runtime,
      &scan_config,
      controller_webots_sensors_lidar_ranges(&webots_sensors),
      robot_x,
      robot_y,
      heading,
      wb_robot_get_time(),
      LIDAR_TRACE_TTL_SECONDS);
}

static void merge_trace_into_map(double now_time) {
  controller_mapping_runtime_merge_trace(
      &mapping_runtime,
      perception_runtime.trace,
      perception_runtime.trace_count,
      now_time,
      MAP_MERGE_MAX_AGE_S,
      MAP_MERGE_MIN_HIT_COUNT,
      EPS);
}

static ControllerWebotsCameraMapSyncContext camera_map_sync_context(void) {
  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  return (ControllerWebotsCameraMapSyncContext){
      mapping_camera_map,
      &camera_map_count,
      MAX_CAMERA_MAP_POINTS,
      camera_free_map,
      &camera_free_map_count,
      MAX_CAMERA_FREE_MAP_POINTS,
      CAMERA_MAP_CELL_SIZE,
      EPS,
      {
          LIDAR_LOCAL_X,
          LIDAR_LOCAL_Y,
          LIDAR_MIN_TRACE_RANGE,
          LIDAR_MAX_TRACE_RANGE,
          CAMERA_FREE_RAY_MIN_RANGE_M,
          CAMERA_FREE_RAY_MARGIN_M,
          CAMERA_FREE_RAY_STEP_M,
          LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
      },
      {robot_x, robot_y, heading},
  };
}

static void merge_camera_free_ray_into_map(double relative_angle, double range, int confidence_boost) {
  ControllerWebotsCameraMapSyncContext context = camera_map_sync_context();
  if (controller_webots_camera_map_sync_free_ray(
          &context, relative_angle, range, confidence_boost)) {
    camera_map_dirty = 1;
  }
}

static void merge_camera_observation_into_map(double relative_angle, double range, int confidence_boost) {
  ControllerWebotsCameraMapSyncContext context = camera_map_sync_context();
  if (controller_webots_camera_map_sync_observation(
          &context, relative_angle, range, confidence_boost)) {
    camera_map_dirty = 1;
  }
}

static void clear_persistent_map(void) {
  controller_mapping_runtime_clear(&mapping_runtime);
}

static void maybe_write_map(void) {
  controller_mapping_runtime_write(
      &mapping_runtime, (step_counter % MAP_WRITE_INTERVAL) == 0);
}


static ControllerMappingSurveySafetyContext mapping_survey_safety_context(void) {
  return controller_mapping_survey_safety_service_context(
      &mapping_survey_safety_service, wb_robot_get_time());
}

static int survey_map_obstacle_near(double x, double y, double clearance) {
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_map_obstacle_near(&context, x, y, clearance);
}

static int survey_point_safe(double x, double y, int room_zone_index, double clearance) {
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_point_safe(
      &context, x, y, room_zone_index, clearance);
}

static int survey_known_obstacle_near(double x, double y, double clearance) {
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_known_obstacle_near(&context, x, y, clearance);
}

static int mapping_survey_segment_clear_of_known_obstacles(
    double ax,
    double ay,
    double bx,
    double by,
    double clearance) {
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_segment_clear(
      &context,
      ax,
      ay,
      bx,
      by,
      clearance,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS);
}

static int mapping_survey_segment_stays_in_room(double ax, double ay, double bx, double by) {
  if (controller_runtime.mapping_survey.room_zone_index < 0) return 1;
  const LimitZone *room = &controller_runtime.limit_zones.zones[controller_runtime.mapping_survey.room_zone_index];
  return controller_mapping_survey_segment_stays_in_room(room, ax, ay, bx, by, MAPPING_SURVEY_GRID_CELL);
}

static int find_mapping_survey_escape_waypoint(double x, double y, int start_index) {
  return controller_mapping_survey_safety_service_find_escape_waypoint(
      &mapping_survey_safety_service,
      x,
      y,
      start_index,
      MAPPING_SURVEY_ESCAPE_MIN_TARGET_DISTANCE,
      MAPPING_SURVEY_ESCAPE_SCAN_AHEAD,
      MAPPING_SURVEY_ESCAPE_OBSTACLE_CLEARANCE,
      MAPPING_SURVEY_ESCAPE_SEGMENT_CLEARANCE,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
      wb_robot_get_time());
}

#define survey_route_add(route, count, x, y) \
  controller_survey_route_add((route), (count), MAX_WAYPOINTS, 0.18, (x), (y))
static int mapping_survey_contour_point_is_safe(
    void *context,
    double x,
    double y,
    int room_zone_index,
    double clearance) {
  (void)context;
  return survey_point_safe(
      x, y, room_zone_index, clearance);
}

static int append_room_contour_phase(
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    double robot_x,
    double robot_y) {
  const ControllerMappingSurveyContourService contour_service = {
      &controller_runtime.limit_zones,
      mapping_survey_contour_point_is_safe,
      NULL,
      MAPPING_SURVEY_CONTOUR_OFFSET,
      0.72,
      0.18,
      MAPPING_SURVEY_MAX_CONTOUR_STEP};
  return controller_mapping_survey_contour_service_append(
      &contour_service, route, route_count, room_zone_index, robot_x, robot_y);
}

static int survey_build_grid(
    SurveyGrid *grid,
    int room_zone_index,
    double robot_x,
    double robot_y,
    double clearance,
    const RuntimeCommand *command) {
  ControllerMappingSurveyGridAdapter adapter = {
      mapping_survey_safety_context(),
      {
      SURVEY_X_MIN,
      SURVEY_X_MAX,
      SURVEY_Y_MIN,
      SURVEY_Y_MAX,
      MAPPING_SURVEY_MAX_EXTENT_X,
      MAPPING_SURVEY_MAX_EXTENT_Y,
      MAPPING_SURVEY_GRID_CELL,
      MAPPING_SURVEY_MAX_GRID_CELLS,
      },
      command};
  return controller_mapping_survey_grid_adapter_build(
      &adapter, grid, room_zone_index, (SurveyPoint){robot_x, robot_y}, clearance);
}

#define append_grid_boundary_contour_phase(grid, route, route_count, robot_x, robot_y) \
  controller_survey_append_boundary_contour( \
      (grid), (route), (route_count), MAX_WAYPOINTS, MAPPING_SURVEY_MAX_BOUNDARY_POINTS, \
      (robot_x), (robot_y), 0.18, MAPPING_SURVEY_MAX_CONTOUR_STEP, 3.2, MAPPING_SURVEY_RDP_EPS)

static ControllerMappingSurveyCoverageService mapping_survey_coverage_service(void) {
  return (ControllerMappingSurveyCoverageService){
      mapping_survey_safety_context(),
      MAPPING_SURVEY_INTERIOR_OFFSET,
      MAPPING_SURVEY_MIN_STRIP_LENGTH,
      MAPPING_SURVEY_STRIP,
      0.18,
      EPS,
      MAX_WAYPOINTS};
}

static void append_scanline_coverage_phase(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  const ControllerMappingSurveyCoverageService service = mapping_survey_coverage_service();
  controller_mapping_survey_coverage_service_append_horizontal(
      &service, grid, route, route_count, room_zone_index);
}

static void append_vertical_coverage_phase(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  const ControllerMappingSurveyCoverageService service = mapping_survey_coverage_service();
  controller_mapping_survey_coverage_service_append_vertical(
      &service, grid, route, route_count, room_zone_index);
}

static void write_mapping_survey_route_file(const char *path, const SurveyPoint *route, int route_count) {
  if (!controller_mapping_route_write(path, route, route_count, controller_runtime.mapping_survey.mode)) {
    set_error("Cannot write mapping survey route to route.csv");
  }
}

static void mapping_survey_generator_clear_map(void *context) {
  (void)context;
  clear_persistent_map();
}

static void mapping_survey_generator_prepare(void *context, MappingSurveyMode mode) {
  (void)context;
  controller_mapping_survey_state_prepare(&controller_runtime.mapping_survey, mode);
}

static SurveyPoint mapping_survey_generator_read_robot(void *context) {
  (void)context;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  read_pose(&x, &y, &heading);
  return (SurveyPoint){x, y};
}

static int mapping_survey_generator_find_room(void *context, SurveyPoint robot) {
  (void)context;
  return find_room_zone_index(robot.x, robot.y);
}

static int mapping_survey_generator_build_grid(
    void *context,
    SurveyGrid *grid,
    SurveyPoint robot,
    int room_zone_index,
    const RuntimeCommand *command) {
  (void)context;
  return survey_build_grid(
      grid, room_zone_index, robot.x, robot.y, MAPPING_SURVEY_INTERIOR_OFFSET, command);
}

static int mapping_survey_generator_flood_grid(
    void *context,
    SurveyGrid *grid,
    SurveyPoint robot) {
  (void)context;
  return controller_survey_flood_component(grid, robot.x, robot.y);
}

static int mapping_survey_generator_start_is_safe(
    void *context,
    SurveyPoint start,
    int room_zone_index,
    double clearance) {
  (void)context;
  return survey_point_safe(start.x, start.y, room_zone_index, clearance);
}

static void mapping_survey_generator_add_route_point(
    void *context,
    SurveyPoint *route,
    int *route_count,
    SurveyPoint point) {
  (void)context;
  survey_route_add(route, route_count, point.x, point.y);
}

static int mapping_survey_generator_append_room_contour(
    void *context,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    SurveyPoint robot) {
  (void)context;
  (void)grid;
  return append_room_contour_phase(route, route_count, room_zone_index, robot.x, robot.y);
}

static void mapping_survey_generator_append_grid_boundary(
    void *context,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    SurveyPoint robot) {
  (void)context;
  append_grid_boundary_contour_phase(grid, route, route_count, robot.x, robot.y);
}

static void mapping_survey_generator_append_horizontal_coverage(
    void *context,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  (void)context;
  append_scanline_coverage_phase(grid, route, route_count, room_zone_index);
}

static void mapping_survey_generator_append_vertical_coverage(
    void *context,
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  (void)context;
  append_vertical_coverage_phase(grid, route, route_count, room_zone_index);
}

static void mapping_survey_generator_write_route(
    void *context,
    const char *path,
    MappingSurveyMode mode,
    const SurveyPoint *route,
    int route_count) {
  (void)context;
  (void)mode;
  write_mapping_survey_route_file(path, route, route_count);
}

static int generate_mapping_survey_route(
    const char *path,
    int clear_map_before_start,
    const RuntimeCommand *command) {
  const MappingSurveyMode survey_mode =
      command ? command->survey_mode : controller_runtime.mapping_survey.mode;
  const ControllerMappingSurveyGeneratorCallbackOperations operations = {
      mapping_survey_generator_clear_map,
      mapping_survey_generator_prepare,
      mapping_survey_generator_read_robot,
      mapping_survey_generator_find_room,
      mapping_survey_generator_build_grid,
      mapping_survey_generator_flood_grid,
      mapping_survey_generator_start_is_safe,
      mapping_survey_generator_add_route_point,
      mapping_survey_generator_append_room_contour,
      mapping_survey_generator_append_grid_boundary,
      mapping_survey_generator_append_horizontal_coverage,
      mapping_survey_generator_append_vertical_coverage,
      mapping_survey_generator_write_route,
  };
  ControllerMappingSurveyGeneratorCallbacksAdapter adapter;
  controller_mapping_survey_generator_callbacks_adapter_init(
      &adapter,
      &operations,
      NULL,
      path,
      command,
      MAPPING_SURVEY_INTERIOR_OFFSET * 0.45);
  const ControllerMappingSurveyRouteGenerationService service = {
      controller_mapping_survey_generator_callbacks_adapter_callbacks(&adapter), &adapter, set_error};
  return controller_mapping_survey_route_generation_service_generate(
      &service,
      clear_map_before_start,
      survey_mode,
      &controller_runtime.mapping_survey.room_zone_index,
      &controller_runtime.mapping_survey.interior_start_index);
}

static void survey_integration_apply_speed(double speed_mps) {
  configured_cruise_speed_mps =
      clamp_value(speed_mps, MIN_CRUISE_SPEED_MPS, MAX_CRUISE_SPEED_MPS);
  controller_webots_motion_state_apply(&motion_state);
}

static ControllerSurveyIntegrationOps survey_integration_ops(void) {
  const ControllerSurveyIntegrationOps ops = {
      .clear_error = clear_error,
      .apply_speed = survey_integration_apply_speed,
      .generate = generate_mapping_survey_route,
      .load_route = load_route,
      .reset_avoidance = reset_route_avoidance_metrics,
      .reset_navigation = reset_navigation_mode,
      .set_status = set_status,
      .current_time = (double (*)(void))wb_robot_get_time,
      .merge_map = merge_trace_into_map,
      .find_escape = find_mapping_survey_escape_waypoint,
      .clear_navigation = clear_local_navigation_state,
      .begin_navigation = begin_navigation_for_waypoint,
  };
  return ops;
}

static void spawn_runtime_obstacle(const RuntimeCommand *command) {
  controller_webots_zone_sync_spawn_obstacle(&webots_zone_sync, command);
}

static void maybe_reload_runtime_command(void) {
  controller_runtime_command_reload_service_run(
      &runtime_command_reload_service, step_counter, RUNTIME_COMMAND_RELOAD_INTERVAL);
}

static void maybe_reload_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;
  controller_route_zone_reload_service_reload_limit(&route_zone_reload_service);
}

static void maybe_reload_surface_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;
  controller_route_zone_reload_service_reload_surface(&route_zone_reload_service);
}

static int load_route(RouteData *route) {
  const ControllerRouteLoadResult result =
      controller_route_load_file(ROUTE_PATH, route);
  if (result == CONTROLLER_ROUTE_LOAD_CANNOT_OPEN) {
    set_error("Cannot open route.csv");
    return 0;
  }
  if (result == CONTROLLER_ROUTE_LOAD_EMPTY) {
    set_error("Route file is empty");
    return 0;
  }
  clear_error();
  return 1;
}

static void maybe_reload_route(void) {
  if ((step_counter % ROUTE_RELOAD_INTERVAL) != 0) return;
  controller_route_zone_reload_service_reload_route(&route_zone_reload_service);
}

static int escape_mapping_survey_orbit(double x, double y) {
  const ControllerSurveyIntegrationOps ops = survey_integration_ops();
  return controller_survey_integration_escape_orbit(
      ROUTE_PATH,
      (SurveyPoint){x, y},
      &controller_runtime.route,
      &controller_runtime.current_waypoint_index,
      &controller_runtime.route_finished,
      &controller_runtime.distance_to_target,
      &controller_runtime.mapping_survey,
      MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS,
      &ops);
}

static void wait_for_fresh_route() {
  controller_runtime.route.count = 0;
  controller_runtime.route.last_modified = get_file_mtime(ROUTE_PATH);
  controller_runtime.current_waypoint_index = 0;
  controller_runtime.route_finished = 0;
  controller_mapping_survey_state_reset_route(&controller_runtime.mapping_survey);
  reset_route_avoidance_metrics();
  reset_navigation_mode();
  controller_runtime.distance_to_target = 0.0;
  clear_error();
  set_status("waiting_for_route");
  stop_robot();
}

static int mapping_survey_scan_point_allowed(double x, double y) {
  return controller_mapping_survey_safety_service_scan_point_allowed(
      &mapping_survey_safety_service,
      x,
      y,
      controller_runtime.mapping_survey.room_zone_index,
      ZONE_CLEARANCE,
      ZONE_CLEARANCE * 0.68,
      wb_robot_get_time());
}

static int mapping_survey_scan_point_allowed_callback(void *context, double x, double y) {
  (void)context;
  return mapping_survey_scan_point_allowed(x, y);
}

static int insert_mapping_survey_obstacle_scan_route(
    double x,
    double z,
    double heading,
    const Waypoint *target,
    const LidarObstacleContext *lidar_context,
    double turn_sign) {
  const ControllerMappingScanServiceConfig scan_config = {
      .max_scan_points = MAPPING_SURVEY_OBSTACLE_SCAN_POINTS + 3,
      .scan = {
          .sensor_local_x = LIDAR_LOCAL_X,
          .sensor_local_y = LIDAR_LOCAL_Y,
          .min_range = LIDAR_MIN_TRACE_RANGE,
          .max_range = LIDAR_MAX_TRACE_RANGE,
          .min_repeat_distance = MAPPING_SURVEY_OBSTACLE_SCAN_MIN_REPEAT_DISTANCE,
          .radius = MAPPING_SURVEY_OBSTACLE_SCAN_RADIUS,
          .circle_point_count = MAPPING_SURVEY_OBSTACLE_SCAN_POINTS,
          .min_point_spacing = 0.18,
      },
  };
  const ControllerMappingScanServiceInput scan_input = {
      .mapping_survey = controller_runtime.mapping_survey.route_active,
      .route = &controller_runtime.route,
      .state = &controller_runtime.mapping_survey,
      .current_waypoint_index = controller_runtime.current_waypoint_index,
      .robot_x = x,
      .robot_y = z,
      .heading = heading,
      .target = target,
      .lidar_context = lidar_context,
      .turn_sign = turn_sign,
      .segment_start_x = controller_runtime.navigation_segment_start_x,
      .segment_start_y = controller_runtime.navigation_segment_start_z,
  };
  ControllerMappingScanServiceOutput scan_output;
  if (!controller_mapping_scan_service_start(
          &scan_config,
          &scan_input,
          mapping_survey_scan_point_allowed_callback,
          NULL,
          &scan_output)) {
    return 0;
  }
  clear_local_navigation_state();
  controller_mapping_scan_transition_apply(&controller_runtime, x, z);
  clear_error();
  set_status("mapping_survey_circle_scan_started");
  return 1;
}

static void run_navigation_step(void) {
  double x = 0.0;
  double z = 0.0;
  double heading = 0.0;

  read_pose(&x, &z, &heading);
  const ControllerRuntimeSensorFrame sensor_frame = {x, z, heading, 0};
  const ControllerRuntimeNavigationResult navigation_result =
      controller_runtime_process_navigation_frame(
          &controller_runtime, &sensor_frame, &control_config.navigation.runtime);

  ControllerNavigationAdapterEffect terminal_effect;
  if (controller_navigation_adapter_terminal_effect(&navigation_result, &terminal_effect) &&
      !terminal_effect.relocalize && navigation_result.action != CONTROLLER_RUNTIME_ACTION_ROUTE_COMPLETED) {
    if (terminal_effect.clear_route_finished) controller_runtime.route_finished = 0;
    if (terminal_effect.clear_avoidance_hold) controller_runtime.avoidance.hold_steps = 0;
    if (terminal_effect.reset_navigation) reset_navigation_mode();
    controller_runtime.distance_to_target = terminal_effect.distance_to_target;
    set_status(terminal_effect.status); stop_robot(); return;
  }

  controller_mapping_survey_state_tick(&controller_runtime.mapping_survey);

  if (terminal_effect.relocalize) {
    clear_local_navigation_state();
    controller_runtime.navigation_waypoint_index = controller_runtime.current_waypoint_index;
    controller_runtime.navigation_segment_start_x = terminal_effect.segment_start_x;
    controller_runtime.navigation_segment_start_z = terminal_effect.segment_start_z;
    controller_runtime.navigation_mode = NAV_MODE_TRACK;
    controller_runtime.distance_to_target = terminal_effect.distance_to_target;
    clear_error();
    set_status(terminal_effect.status);
    stop_robot();
    return;
  }

  ensure_navigation_waypoint_initialized(x, z);

  if (navigation_result.action == CONTROLLER_RUNTIME_ACTION_ROUTE_COMPLETED) {
    reset_navigation_mode(); controller_runtime.distance_to_target = terminal_effect.distance_to_target;
    set_status(terminal_effect.status); stop_robot();
    return;
  }
  if (navigation_result.navigation.route_decision == CONTROLLER_NAVIGATION_ROUTE_ADVANCED)
    begin_navigation_for_waypoint(controller_runtime.current_waypoint_index, x, z);
  Waypoint target = navigation_result.target;
  int is_final_waypoint = navigation_result.navigation.route.route.is_final_waypoint;

  if (controller_mapping_survey_state_complete_scan(
          &controller_runtime.mapping_survey,
          controller_runtime.current_waypoint_index,
          MAPPING_SURVEY_OBSTACLE_SCAN_COOLDOWN_STEPS)) {
    clear_local_navigation_state();
    begin_navigation_for_waypoint(controller_runtime.current_waypoint_index, x, z);
    set_status("mapping_survey_resumed_route");
  }

  if (navigation_result.action == CONTROLLER_RUNTIME_ACTION_SKIP_BLOCKED_TARGET ||
      navigation_result.action == CONTROLLER_RUNTIME_ACTION_SKIP_BLOCKED_SEGMENT) {
    ++controller_runtime.current_waypoint_index;
    begin_navigation_for_waypoint(controller_runtime.current_waypoint_index, x, z);
    clear_error();
    set_status(
        navigation_result.action == CONTROLLER_RUNTIME_ACTION_SKIP_BLOCKED_TARGET
            ? "mapping_survey_skipped_blocked_waypoint"
            : "mapping_survey_skipped_blocked_segment");
    return;
  }
  if (navigation_result.action == CONTROLLER_RUNTIME_ACTION_BLOCKED_TARGET) {
    set_status("blocked_by_dynamic_zone");
    set_error("Current waypoint is blocked by a dynamic zone");
    controller_runtime.distance_to_target = hypot2(target.x - x, target.z - z);
    stop_robot();
    return;
  }
  if (navigation_result.action == CONTROLLER_RUNTIME_ACTION_BLOCKED_SEGMENT) {
    set_status("blocked_by_dynamic_zone");
    set_error("Dynamic zone blocks the current segment");
    controller_runtime.distance_to_target = hypot2(target.x - x, target.z - z);
    stop_robot();
    return;
  }

  ControllerNavigationTargetContext target_context;
  controller_navigation_context_target(
      &target,
      controller_runtime.current_waypoint_index,
      controller_runtime.route.count,
      x,
      z,
      heading,
      &target_context);
  const double target_distance_now = target_context.distance;
  const double heading_error_to_target = target_context.heading_error;

  LidarObstacleContext lidar_context;
  compute_lidar_obstacle_context(
      &lidar_context, -heading_error_to_target, controller_runtime.avoidance.turn_sign);
  const int avoidance_was_active = controller_runtime.avoidance.active;
  const ControllerNavigationPerceptionInput perception_input = {
      .lidar_context = &lidar_context,
      .lidar_available = perception_runtime.lidar.available,
      .camera_visible = perception_runtime.camera.obstacle_visible,
      .camera_angle = perception_runtime.camera.obstacle_angle,
      .camera_fov = perception_runtime.camera.fov,
      .camera_range = perception_runtime.camera.obstacle_range,
      .camera_score = perception_runtime.camera.obstacle_score,
      .camera_detection_count = perception_runtime.camera.detection_count,
      .camera_center_offset = perception_runtime.camera.obstacle_center_offset,
  };
  ControllerNavigationPerceptionOutput perception_output;
  controller_navigation_perception_prepare(
      &perception_input, &control_config.perception.navigation, &perception_output);
  const int camera_visual_front_obstacle =
      perception_output.camera_visual_front_obstacle;
  const double camera_preferred_turn_sign =
      perception_output.camera_preferred_turn_sign;
  const ControllerAvoidanceDetection avoidance_detection =
      perception_output.avoidance;
  const double center_obstacle_range = avoidance_detection.center_obstacle_range;
  const double left_front_corner_range = avoidance_detection.left_front_corner_range;
  const double right_front_corner_range = avoidance_detection.right_front_corner_range;
  const double left_obstacle_range = avoidance_detection.left_obstacle_range;
  const double right_obstacle_range = avoidance_detection.right_obstacle_range;
  const double expected_front_range = avoidance_detection.expected_front_range;
  const double near_front_range = avoidance_detection.near_front_range;
  const double left_lidar_context = avoidance_detection.left_lidar_context;
  const double right_lidar_context = avoidance_detection.right_lidar_context;
  const int expected_zone_wall_ahead = avoidance_detection.expected_zone_wall_ahead;
  const int expected_zone_wall_close = avoidance_detection.expected_zone_wall_close;
  const int expected_zone_wall_slowdown = avoidance_detection.expected_zone_wall_slowdown;
  const int center_passage_available = avoidance_detection.center_passage_available;
  const int obstacle_context_present = avoidance_detection.obstacle_context_present;
  const int should_start_avoidance = avoidance_detection.should_start_avoidance;

  if (should_start_avoidance &&
      insert_mapping_survey_obstacle_scan_route(
          x,
          z,
          heading,
          &target,
          &lidar_context,
          fabs(controller_runtime.lidar_priority_turn_sign) > 0.0 ? controller_runtime.lidar_priority_turn_sign : sign_or_one(heading_error_to_target))) {
    stop_robot();
    return;
  }

  const ControllerAvoidanceStartInput avoidance_start_input = {
      .lidar_context = &lidar_context,
      .detection = &avoidance_detection,
      .x = x,
      .z = z,
      .heading = heading,
      .target_distance = target_distance_now,
      .priority_turn_sign = controller_runtime.lidar_priority_turn_sign,
      .camera_turn_sign = camera_preferred_turn_sign,
      .heading_error = heading_error_to_target,
  };
  ControllerAvoidanceStartOutput avoidance_start_output;
  if (controller_avoidance_start(
          &controller_runtime.avoidance,
          &avoidance_start_input,
          &control_config.avoidance.start,
          &avoidance_start_output)) {
    controller_runtime.lidar_priority_turn_sign = avoidance_start_output.priority_turn_sign;
    controller_runtime.lidar_priority_hold_steps = avoidance_start_output.priority_hold_steps;
  }

  if (controller_runtime.avoidance.active) {
    const double detour_dx = controller_runtime.avoidance.detour_x - x;
    const double detour_dz = controller_runtime.avoidance.detour_z - z;
    const double detour_distance =
        controller_runtime.avoidance.detour_active ? hypot2(detour_dx, detour_dz) : 0.0;
    const double detour_heading_error =
        controller_runtime.avoidance.detour_active
            ? wrap_angle(atan2(detour_dz, detour_dx) - heading)
            : 0.0;
    const ControllerAvoidanceProgressInput progress_input = {
        .x = x,
        .z = z,
        .heading = heading,
        .target_distance = target_distance_now,
        .obstacle_context_present = obstacle_context_present,
        .detour_distance = detour_distance,
    };
    const ControllerAvoidanceLifecycleInput lifecycle_input = {
        .mapping_survey = controller_runtime.mapping_survey.route_active,
        .replan_cooldown_steps = controller_runtime.mapping_survey.replan_cooldown_steps,
        .x = x,
        .z = z,
        .target_distance = target_distance_now,
        .heading_error = heading_error_to_target,
        .near_front_range = near_front_range,
        .center_obstacle_range = center_obstacle_range,
    };
    const ControllerAvoidanceCommandInput command_input = {
        .context = &lidar_context,
        .detection = &avoidance_detection,
        .detour_heading_error = detour_heading_error,
        .heading_error_to_target = heading_error_to_target,
        .runtime_linear_speed_limit = active_linear_speed_limit,
        .runtime_angular_speed_limit = active_angular_speed_limit,
        .pass_min_speed = scaled_linear_floor(0.35),
        .pass_max_speed = scaled_linear_cap(LIDAR_PASS_MAX_SPEED_FACTOR),
        .avoid_min_speed = scaled_linear_floor(0.30),
        .avoid_max_speed = scaled_linear_cap(LIDAR_AVOID_DRIVE_MAX_SPEED_FACTOR),
        .reverse_speed = scaled_linear_cap(0.16),
    };
    const ControllerAvoidanceServiceInput avoidance_service_input = {
        .progress_config = &control_config.avoidance.progress,
        .progress_input = progress_input,
        .lifecycle_config = &control_config.avoidance.lifecycle,
        .lifecycle_input = lifecycle_input,
        .has_next_waypoint = controller_runtime.current_waypoint_index + 1 < controller_runtime.route.count,
        .command_config = &control_config.avoidance.command,
        .command_input = command_input,
        .priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS,
        .avoidance_hold_steps = LIDAR_AVOID_HOLD_STEPS,
        .has_best_gap = lidar_context.has_best_gap,
    };
    ControllerAvoidanceServiceOutput avoidance_service_output;
    controller_avoidance_service_process_active(
        &controller_runtime.avoidance, &avoidance_service_input, &avoidance_service_output);
    const ControllerAvoidanceRecoveryOutput recovery_output =
        avoidance_service_output.recovery;

    if (recovery_output.rejoin_route) {
      clear_local_navigation_state();
      controller_runtime.navigation_waypoint_index = controller_runtime.current_waypoint_index;
      controller_runtime.navigation_segment_start_x = x;
      controller_runtime.navigation_segment_start_z = z;
      controller_runtime.navigation_mode = NAV_MODE_TRACK;
      controller_runtime.distance_to_target = target_distance_now;
      clear_error();
      set_status("mapping_survey_rejoined_route");
    }

    if (recovery_output.attempt_orbit_escape) {
        const ControllerAvoidanceRecoveryAfterOrbit after_orbit =
            controller_avoidance_recovery_after_orbit(
                escape_mapping_survey_orbit(x, z),
                controller_runtime.current_waypoint_index + 1 < controller_runtime.route.count);
        if (after_orbit == CONTROLLER_AVOIDANCE_RECOVERY_STOP_AFTER_ESCAPE) {
          controller_runtime.distance_to_target = 0.0;
          stop_robot();
          return;
        }

        if (after_orbit == CONTROLLER_AVOIDANCE_RECOVERY_SKIP_LOOPED_WAYPOINT) {
          ++controller_runtime.current_waypoint_index;
          clear_local_navigation_state();
          begin_navigation_for_waypoint(controller_runtime.current_waypoint_index, x, z);
          controller_runtime.mapping_survey.replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
          controller_runtime.distance_to_target = hypot2(controller_runtime.route.waypoints[controller_runtime.current_waypoint_index].x - x,
                                      controller_runtime.route.waypoints[controller_runtime.current_waypoint_index].z - z);
          clear_error();
          set_status("mapping_survey_skipped_looped_obstacle");
          stop_robot();
          return;
        }
    }

    if (recovery_output.reacquire_free_space) {
      clear_local_navigation_state();
      controller_runtime.navigation_waypoint_index = controller_runtime.current_waypoint_index;
      controller_runtime.navigation_segment_start_x = x;
      controller_runtime.navigation_segment_start_z = z;
      controller_runtime.navigation_mode = NAV_MODE_TRACK;
      controller_runtime.distance_to_target = target_distance_now;
      clear_error();
      set_status("reacquired_free_space");
    }

    if (controller_runtime.avoidance.active && avoidance_service_output.has_command) {
      if (avoidance_service_output.presentation.priority_updated) {
        controller_runtime.lidar_priority_turn_sign = avoidance_service_output.presentation.priority_turn_sign;
        controller_runtime.lidar_priority_hold_steps = avoidance_service_output.presentation.priority_hold_steps;
      }
      set_status(avoidance_service_output.presentation.status);

      controller_runtime.distance_to_target = target_distance_now;
      clear_error();
      set_base_velocity(
          avoidance_service_output.command.linear_speed,
          0.0,
          avoidance_service_output.command.angular_speed);
      return;
    }
  }

  controller_avoidance_lifecycle_finish(&controller_runtime.avoidance, START_X, START_Z);
  if (avoidance_was_active) {
    controller_runtime.navigation_segment_start_x = x;
    controller_runtime.navigation_segment_start_z = z;
    controller_runtime.navigation_mode = NAV_MODE_TRACK;
  }
  controller_runtime.distance_to_target = target_distance_now;
  controller_runtime.navigation_segment_start_x = x;
  controller_runtime.navigation_segment_start_z = z;
  const int route_relaxed_mode_base = 1;
  ControllerNavigationTrackingInput tracking_input = {
      .mode = controller_runtime.navigation_mode,
      .is_final_waypoint = is_final_waypoint,
      .target_has_heading = target.has_heading,
      .target_heading = target.heading_rad,
      .current_heading = heading,
      .distance_to_target = controller_runtime.distance_to_target,
      .heading_error_to_target = heading_error_to_target,
      .updated_heading_error = heading_error_to_target,
      .route_relaxed_mode = route_relaxed_mode_base,
      .avoidance_active = controller_runtime.avoidance.active,
      .lidar_speed_scale = 1.0,
      .expected_wall_speed_scale = 1.0,
      .expected_zone_wall_slowdown = expected_zone_wall_slowdown,
      .expected_zone_wall_ahead = expected_zone_wall_ahead,
      .runtime_linear_speed_limit = active_linear_speed_limit,
      .runtime_angular_speed_limit = active_angular_speed_limit,
  };
  const ControllerNavigationLidarInput navigation_lidar_input = {
      .lidar_available = perception_runtime.lidar.available,
      .center_passage_available = center_passage_available,
      .center_obstacle_range = center_obstacle_range,
      .near_front_range = near_front_range,
      .left_lidar_context = left_lidar_context,
      .right_lidar_context = right_lidar_context,
      .best_gap_beam_angle = lidar_context.best_gap_beam_angle,
      .heading_error = heading_error_to_target,
      .camera_visual_front_obstacle = camera_visual_front_obstacle,
      .camera_obstacle_center_offset = perception_runtime.camera.obstacle_center_offset,
      .expected_zone_wall_close = expected_zone_wall_close,
      .expected_zone_wall_slowdown = expected_zone_wall_slowdown,
      .expected_front_range = expected_front_range,
      .current_priority_turn_sign = controller_runtime.lidar_priority_turn_sign,
      .current_priority_hold_steps = controller_runtime.lidar_priority_hold_steps,
  };
  const ControllerNavigationMotionServiceInput motion_input = {
      .tracking = tracking_input,
      .lidar = navigation_lidar_input,
  };
  ControllerNavigationServiceMotionOutput motion_output;
  const int final_alignment = controller_navigation_service_calculate_motion(
      &control_config.navigation.tracking,
      &control_config.navigation.lidar,
      &motion_input,
      &motion_output);
  controller_runtime.navigation_mode = motion_output.motion.tracking.mode;
  if (final_alignment) {
    clear_error();
    set_status("aligning_final_heading");
    apply_kinematic_step(
        x,
        z,
        heading,
        motion_output.motion.tracking.linear_speed,
        motion_output.motion.tracking.angular_speed);
    return;
  }
  controller_runtime.lidar_priority_turn_sign = motion_output.motion.lidar.priority_turn_sign;
  controller_runtime.lidar_priority_hold_steps = motion_output.motion.lidar.priority_hold_steps;
  clear_error();
  set_status(controller_navigation_presentation_tracking_status(
      motion_output.motion.tracking.status));
  apply_kinematic_step(
      x,
      z,
      heading,
      motion_output.motion.tracking.linear_speed,
      motion_output.motion.tracking.angular_speed);
}

static void write_state_snapshot(void) {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  read_pose(&x, &y, &heading);
  const ControllerTelemetryPublisherInput input = {
      .simulation_time = wb_robot_get_time(), .pose_x = x, .pose_y = y,
      .pose_z = START_HEIGHT, .pose_yaw = heading, .runtime = &controller_runtime,
      .perception = &perception_runtime, .status = navigation_status,
      .error = navigation_error, .off_route_active = route_off_route_active_now(),
      .avoidance_time_sec = route_avoidance_time_sec, .avoidance_steps = route_avoidance_steps,
      .cruise_speed_mps = configured_cruise_speed_mps, .payload_kg = configured_payload_kg,
      .battery_range_units = configured_battery_range_units,
      .battery_speed_factor = active_battery_speed_factor,
      .linear_speed_limit = active_linear_speed_limit,
      .angular_speed_limit = active_angular_speed_limit,
      .dynamic_zone_wall_count = zone_node_registry.count,
      .obstacle_map_cell_count = persistent_map_count, .obstacle_map_cell_size = MAP_CELL_SIZE,
      .camera_map_obstacle_cell_count = camera_map_count,
      .camera_map_free_cell_count = camera_free_map_count,
      .camera_map_cell_size = CAMERA_MAP_CELL_SIZE,
      .trace_ttl_seconds = LIDAR_TRACE_TTL_SECONDS,
      .trace_min_confidence = LIDAR_TRACE_MIN_CONFIDENCE,
  };
  ControllerTelemetryPublisherOutput output = {0};
  controller_telemetry_publisher_build(&input, &output);
  controller_telemetry_write_snapshot(STATE_TEMP_PATH, STATE_PATH, &output.snapshot);
}

static void merge_trace_for_controller_step(void) {
  merge_trace_into_map(wb_robot_get_time());
}

static const ControllerStepCallbacks controller_step_callbacks = {
    maybe_reload_zones,
    maybe_reload_surface_zones,
    maybe_reload_route,
    maybe_reload_motion_profile,
    maybe_reload_runtime_command,
    capture_lidar_trace,
    merge_trace_for_controller_step,
    maybe_write_map,
    maybe_update_camera_perception,
    maybe_write_camera_frame,
    NULL,
    run_navigation_step,
    update_route_avoidance_metrics,
    write_state_snapshot,
};

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  control_config = controller_control_config_default();
  controller_perception_runtime_init(&perception_runtime);

  if (!controller_paths_init(&controller_paths, getenv("WEB_STATE_DIR"))) {
    fprintf(stderr, "Failed to initialize Webots state paths.\n");
    return 1;
  }

  wb_robot_init();
  controller_webots_devices_init(&webots_devices);
  const ControllerDriveConfig drive_config = controller_webots_adapter_drive_config(
      control_config.drive.wheel_radius,
      control_config.drive.wheel_base_longitudinal,
      control_config.drive.wheel_base_lateral,
      control_config.drive.max_wheel_speed_rad_s,
      control_config.drive.acceleration_limit_rad_s2,
      control_config.drive.deceleration_limit_rad_s2,
      control_config.time_step_ms / 1000.0);
  controller_webots_adapter_init(
      &webots_adapter, &drive_config, drive_webots_base, &webots_devices);
  controller_webots_sensors_init(&webots_sensors);
  controller_application_state_init(&application_state);
  init_sensors();
  init_pose_tracking();
  webots_zone_sync = (ControllerWebotsZoneSyncContext){
      webots_pose.root_children_field,
      &zone_node_registry,
      &surface_zone_registry,
      &runtime_obstacle_registry,
      MAX_ZONE_NODES,
      MAX_SURFACE_ZONE_NODES,
      MAX_RUNTIME_OBSTACLE_NODES,
      WALL_THICKNESS,
      WALL_HEIGHT,
      -21.5,
      21.5,
      -16.5,
      16.5,
  };
  reset_robot_pose();
  controller_runtime_init(&controller_runtime);
  const ControllerMappingRuntimeConfig mapping_config = {
      .paths = {MAP_PATH, MAP_TEMP_PATH, MAP_CSV_PATH, MAP_CSV_TEMP_PATH,
                CAMERA_MAP_PATH, CAMERA_MAP_TEMP_PATH, CAMERA_MAP_CSV_PATH,
                CAMERA_MAP_CSV_TEMP_PATH},
      .map_cell_size = MAP_CELL_SIZE,
      .camera_cell_size = CAMERA_MAP_CELL_SIZE,
  };
  controller_mapping_runtime_init(&mapping_runtime, &mapping_config);
  controller_mapping_survey_safety_service_init(
      &mapping_survey_safety_service,
      &controller_runtime,
      &mapping_runtime,
      &perception_runtime,
      (ControllerMappingSurveySafetyServiceConfig){
          LIDAR_TRACE_TTL_SECONDS,
          0.18,
          MAPPING_SURVEY_MAX_EXTENT_X,
          MAPPING_SURVEY_MAX_EXTENT_Y,
          MAPPING_SURVEY_MAP_OBSTACLE_CLEARANCE,
          MAPPING_SURVEY_GRID_CELL,
      });
  controller_route_zone_service_init(&route_zone_service);
  controller_route_zone_reload_service_init(
      &route_zone_reload_service,
      &route_zone_service,
      &controller_runtime,
      &webots_zone_sync,
      (ControllerRouteZoneServicePaths){ROUTE_PATH, ZONE_PATH, SURFACE_ZONE_PATH},
      set_status,
      set_error,
      reset_route_avoidance_metrics,
      reset_navigation_mode);
  clear_persistent_map();
  controller_webots_camera_adapter_remove_frames(
      CAMERA_FRAME_BMP_PATH,
      CAMERA_FRAME_BMP_TEMP_PATH,
      CAMERA_FRAME_JPEG_PATH,
      CAMERA_FRAME_JPEG_TEMP_PATH);
  controller_motion_profile_reload_service_init(
      &motion_profile_reload_service, &motion_state, MOTION_PROFILE_PATH);
  controller_webots_motion_state_apply(&motion_state);
  controller_motion_profile_reload_service_run(&motion_profile_reload_service, 0, 1);
  maybe_reload_zones();
  maybe_reload_surface_zones();
  const ControllerSurveyIntegrationOps runtime_command_survey_ops = survey_integration_ops();
  controller_runtime_command_reload_service_init(
      &runtime_command_reload_service,
      RUNTIME_COMMAND_PATH,
      ROUTE_PATH,
      (ControllerRuntimeCommandLimits){
          SURVEY_X_MIN,
          SURVEY_X_MAX,
          SURVEY_Y_MIN,
          SURVEY_Y_MAX,
          MAPPING_SURVEY_MAX_EXTENT_X,
          MAPPING_SURVEY_MAX_EXTENT_Y,
      },
      &controller_runtime,
      &runtime_command_survey_ops,
      spawn_runtime_obstacle);
  runtime_command_reload_service.last_modified = get_file_mtime(RUNTIME_COMMAND_PATH);

  if (!controller_webots_pose_is_ready(&webots_pose) || !webots_pose.root_children_field) {
    set_status("error");
    set_error("Supervisor fields are not available");
  } else {
    controller_mapping_survey_lifecycle_service_ensure_default_route(
        ROUTE_PATH,
        get_file_mtime(ROUTE_PATH),
        &(ControllerSurveyDefaultRouteConfig){
            SURVEY_X_MIN, SURVEY_X_MAX, SURVEY_Y_MIN, SURVEY_Y_MAX, SURVEY_STRIP},
        clear_persistent_map,
        set_status);
    wait_for_fresh_route();
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    ++step_counter;
    controller_step_run(
        step_counter, &control_config.schedule.lifecycle, &controller_step_callbacks);
  }

  controller_mapping_runtime_write(&mapping_runtime, 1);
  controller_webots_zone_sync_remove_all(&webots_zone_sync);
  wb_robot_cleanup();
  return 0;
}
