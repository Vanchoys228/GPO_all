#include <webots/robot.h>
#include <webots/supervisor.h>

#include "controller_avoidance.h"
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
#include "controller_io.h"
#include "controller_drive.h"
#include "controller_lidar_math.h"
#include "controller_lidar_scan.h"
#include "controller_lidar_trace.h"
#include "controller_lifecycle.h"
#include "controller_math.h"
#include "controller_mapping_route_io.h"
#include "controller_mapping_obstacles.h"
#include "controller_mapping_scan.h"
#include "controller_mapping_scan_service.h"
#include "controller_mapping_scan_transition.h"
#include "controller_mapping_survey_escape.h"
#include "controller_mapping_survey_escape_safety.h"
#include "controller_mapping_survey_safety.h"
#include "controller_mapping_survey_runtime_safety.h"
#include "controller_mapping_store.h"
#include "controller_motion_profile.h"
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
#include "controller_route.h"
#include "controller_route_zone_service.h"
#include "controller_runtime.h"
#include "controller_runtime_command.h"
#include "controller_survey_contour.h"
#include "controller_survey_coverage.h"
#include "controller_survey_default_route.h"
#include "controller_survey_generator.h"
#include "controller_survey_grid.h"
#include "controller_survey_geometry.h"
#include "controller_survey_integration.h"
#include "controller_survey_lifecycle.h"
#include "controller_survey_route_builder.h"
#include "controller_survey_state.h"
#include "controller_step.h"
#include "controller_telemetry.h"
#include "controller_telemetry_service.h"
#include "controller_types.h"
#include "controller_webots_devices.h"
#include "controller_webots_adapter.h"
#include "controller_webots_camera_adapter.h"
#include "controller_webots_pose.h"
#include "controller_webots_simulation.h"
#include "controller_webots_motion_state.h"
#include "controller_webots_navigation_state.h"
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
static ControllerWebotsSimulationNodeRegistry zone_node_registry = {0};
static ControllerWebotsSimulationNodeRegistry surface_zone_registry = {0};
static ControllerWebotsSimulationNodeRegistry runtime_obstacle_registry = {0};
static ControllerWebotsZoneSyncContext webots_zone_sync = {0};
static ObstacleTracePoint obstacle_trace[MAX_OBSTACLE_TRACE_POINTS];
static int obstacle_trace_count = 0;
static ControllerMappingStore mapping_store;
#define persistent_map mapping_store.persistent_map
#define persistent_map_count mapping_store.persistent_count
#define map_dirty mapping_store.persistent_dirty
static MapCell *camera_map = NULL;
#define camera_map_count mapping_store.camera_count
#define camera_free_map mapping_store.camera_free_map
#define camera_free_map_count mapping_store.camera_free_count
#define camera_map_dirty mapping_store.camera_dirty
static int lidar_available = 0;
static int lidar_resolution = 0;
static double lidar_fov = 0.0;
static double lidar_max_range = 0.0;
static int lidar_last_hit_count = 0;
static int lidar_front_hit_count = 0;
static double lidar_front_min_range = 0.0;
static double lidar_center_min_range = 0.0;
static double lidar_left_front_min_range = 0.0;
static double lidar_right_front_min_range = 0.0;
static double lidar_left_min_range = 0.0;
static double lidar_right_min_range = 0.0;
static int camera_available = 0;
static int camera_width = 0;
static int camera_height = 0;
static double camera_fov = 0.0;
static ControllerWebotsCameraFrameMetadata camera_frame_metadata = {
    "camera_frame.bmp", "image/bmp", 0, 0.0};
static int camera_obstacle_visible = 0;
static double camera_obstacle_score = 0.0;
static double camera_obstacle_center_offset = 0.0;
static double camera_obstacle_angle = 0.0;
static double camera_obstacle_range = 0.0;
static int camera_detection_count = 0;
static int camera_obstacle_update_step = -1;
static int camera_virtual_mode = 0;
static int step_counter = 0;
static ControllerWebotsNavigationState navigation_state;
#define navigation_status navigation_state.status
#define navigation_error navigation_state.error
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
static long long motion_profile_last_modified = -1;
static long long runtime_command_last_modified = -1;
static long long last_processed_runtime_command_id = -1;
static double route_avoidance_time_sec = 0.0;
static int route_avoidance_steps = 0;

static ControllerPaths controller_paths;
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


static void apply_motion_profile() {
  controller_webots_motion_state_apply(&motion_state);
}

static int load_motion_profile() {
  return controller_webots_motion_state_load(&motion_state, MOTION_PROFILE_PATH);
}

static void maybe_reload_motion_profile(void) {
  if ((step_counter % MOTION_RELOAD_INTERVAL) != 0) return;

  const double previous_cruise_speed = configured_cruise_speed_mps;
  const double previous_payload_kg = configured_payload_kg;
  const double previous_battery_range = configured_battery_range_units;
  const double previous_linear_limit = active_linear_speed_limit;
  const double previous_angular_limit = active_angular_speed_limit;
  const long long mtime = get_file_mtime(MOTION_PROFILE_PATH);
  if (mtime < 0) return;

  if (load_motion_profile()) {
    const int profile_changed =
        mtime != motion_profile_last_modified ||
        fabs(configured_cruise_speed_mps - previous_cruise_speed) > 1e-6 ||
        fabs(configured_payload_kg - previous_payload_kg) > 1e-6 ||
        fabs(configured_battery_range_units - previous_battery_range) > 1e-6 ||
        fabs(active_linear_speed_limit - previous_linear_limit) > 1e-6 ||
        fabs(active_angular_speed_limit - previous_angular_limit) > 1e-6;
    motion_profile_last_modified = mtime;
    if (profile_changed) {
      set_status("motion_profile_reloaded");
    }
  }
}


static void set_error(const char *message) {
  controller_webots_navigation_state_set_error(&navigation_state, message);
}

static void clear_error(void) {
  controller_webots_navigation_state_clear_error(&navigation_state);
}

static void set_status(const char *status) {
  controller_webots_navigation_state_set_status(&navigation_state, status);
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
  lidar_available = sensor_state.lidar_available;
  lidar_resolution = sensor_state.lidar_resolution;
  lidar_fov = sensor_state.lidar_fov;
  lidar_max_range = sensor_state.lidar_max_range;
  camera_available = sensor_state.camera_available;
  camera_virtual_mode = sensor_state.camera_virtual_mode;
  camera_width = sensor_state.camera_width;
  camera_height = sensor_state.camera_height;
  camera_fov = sensor_state.camera_fov;
}

static double estimate_camera_range_from_lidar(double relative_angle, double fallback_range) {
  return controller_webots_camera_range_from_lidar(
      &webots_sensors, lidar_available, lidar_resolution, lidar_fov,
      relative_angle, CAMERA_RANGE_SEARCH_WINDOW_RAD, LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE, fallback_range);
}

static void merge_camera_visible_frustum_into_map(double effective_fov, double default_range) {
  if (!camera_available) return;

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
  camera_obstacle_update_step = step_counter;
  camera_obstacle_visible = 0;
  camera_obstacle_score = 0.0;
  camera_obstacle_center_offset = 0.0;
  camera_obstacle_angle = 0.0;
  camera_obstacle_range = 0.0;
  camera_detection_count = 0;

  if (!camera_available || !controller_webots_sensors_has_camera(&webots_sensors) ||
      camera_width <= 0 || camera_height <= 0) return;

  const unsigned char *image = controller_webots_sensors_camera_image(&webots_sensors);
  if (!image) return;

  const double effective_fov = camera_fov > EPS ? camera_fov : 1.05;
  merge_camera_visible_frustum_into_map(effective_fov, CAMERA_RANGE_FALLBACK_M);

  ControllerWebotsCameraPerception perception;
  controller_webots_camera_perception_analyze(
      image, camera_width, camera_height, effective_fov,
      CAMERA_OBSTACLE_MIN_SCORE, &perception);
  camera_obstacle_score = perception.score;
  if (perception.visible) {
    camera_obstacle_visible = 1;
    camera_obstacle_center_offset = perception.center_offset;
    camera_obstacle_angle = perception.angle;
    camera_obstacle_range =
        estimate_camera_range_from_lidar(camera_obstacle_angle, perception.fallback_range_m);
    camera_detection_count = perception.detection_count;
    merge_camera_free_ray_into_map(camera_obstacle_angle, camera_obstacle_range, 2);
    merge_camera_observation_into_map(
        camera_obstacle_angle,
        camera_obstacle_range,
        perception.confidence_boost);
  }
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

  camera_obstacle_visible = 0;
  camera_obstacle_score = 0.0;
  camera_obstacle_center_offset = 0.0;

  const double effective_fov = camera_fov > EPS ? camera_fov : 1.05;
  const ControllerCameraVirtualConfig virtual_config = {
      lidar_fov,
      effective_fov,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE,
      LIDAR_TRACK_CAUTION_RANGE,
      LIDAR_AVOID_STOP_RANGE,
      0.42,
  };
  if (lidar_available && controller_webots_sensors_has_lidar(&webots_sensors) &&
      lidar_resolution > 1 && lidar_fov > EPS) {
    const float *ranges = controller_webots_sensors_lidar_ranges(&webots_sensors);
    controller_camera_virtual_collect(
        ranges, lidar_resolution, &virtual_config, clusters, CAMERA_MAX_VIRTUAL_CLUSTERS,
        &cluster_summary);

    for (int i = 0; ranges && i < lidar_resolution; ++i) {
      const double alpha = (double)i / (double)(lidar_resolution - 1);
      const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
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
  camera_obstacle_score = virtual_observation.score;
  if (virtual_observation.visible) {
      camera_obstacle_visible = 1;
      camera_obstacle_center_offset = virtual_observation.center_offset;
      camera_obstacle_angle = camera_obstacle_center_offset * effective_fov * 0.5;
      camera_obstacle_range =
          estimate_camera_range_from_lidar(camera_obstacle_angle, CAMERA_RANGE_FALLBACK_M);
      camera_detection_count = virtual_observation.detection_count;
  }

  draw_virtual_camera_overlay(pixels, effective_fov);
  return write_bmp24(CAMERA_FRAME_BMP_TEMP_PATH, pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);
}

static void maybe_write_camera_frame(void) {
  if (!camera_available) return;
  if ((step_counter % CAMERA_WRITE_INTERVAL) != 0) return;

  if (camera_virtual_mode) {
    if (write_virtual_camera_frame() == 0) {
      controller_webots_camera_adapter_publish_frame(
          CAMERA_FRAME_BMP_TEMP_PATH, CAMERA_FRAME_BMP_PATH, "camera_frame.bmp", "image/bmp",
          wb_robot_get_time(), &camera_frame_metadata);
    }
    return;
  }

  if (!controller_webots_sensors_has_camera(&webots_sensors)) return;
  if (camera_obstacle_update_step != step_counter) {
    update_camera_obstacle_hint();
  }

  if (controller_webots_sensors_save_camera_image(
          &webots_sensors, CAMERA_FRAME_JPEG_TEMP_PATH, 70) == 0) {
    controller_webots_camera_adapter_publish_frame(
        CAMERA_FRAME_JPEG_TEMP_PATH, CAMERA_FRAME_JPEG_PATH, "camera_frame.jpg", "image/jpeg",
        wb_robot_get_time(), &camera_frame_metadata);
  }
}

static void maybe_update_camera_perception(void) {
  if (!camera_available || camera_virtual_mode ||
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
  route_avoidance_time_sec = 0.0;
  route_avoidance_steps = 0;
}

static int route_off_route_active_now() {
  return controller_navigation_metrics_off_route(controller_runtime.route_finished, controller_runtime.route.count, controller_runtime.avoidance.active, navigation_status);
}

static void update_route_avoidance_metrics(void) {
  if (route_off_route_active_now()) {
    controller_navigation_metrics_tick(1, (double)TIME_STEP / 1000.0, &route_avoidance_time_sec, &route_avoidance_steps);
  }
}

static void compute_lidar_obstacle_context(LidarObstacleContext *context,
                                           double target_beam_angle,
                                           double preferred_turn_sign) {
  if (!context) return;

  const double effective_max_range =
      lidar_max_range > EPS ? fmin(lidar_max_range, LIDAR_MAX_TRACE_RANGE) : LIDAR_MAX_TRACE_RANGE;
  controller_lidar_context_init(context, effective_max_range);

  if (!lidar_available || !controller_webots_sensors_has_lidar(&webots_sensors) ||
      lidar_resolution <= 1 || lidar_fov <= EPS) return;

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
  const double sigma = fmax(lidar_fov * 0.22, 0.22);
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

  for (int i = 0; i < lidar_resolution; i += LIDAR_SAMPLE_STRIDE) {
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
            lidar_resolution,
            i,
            raw_range,
            effective_max_range,
            LIDAR_SAMPLE_STRIDE,
            LIDAR_MIN_TRACE_RANGE,
            LIDAR_RANGE_JUMP_TOLERANCE);

    const double alpha = lidar_resolution > 1 ? (double)i / (double)(lidar_resolution - 1) : 0.5;
    const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
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
  lidar_last_hit_count = 0;
  lidar_front_hit_count = 0;
  lidar_front_min_range = LIDAR_MAX_TRACE_RANGE;
  lidar_center_min_range = LIDAR_MAX_TRACE_RANGE;
  lidar_left_front_min_range = LIDAR_MAX_TRACE_RANGE;
  lidar_right_front_min_range = LIDAR_MAX_TRACE_RANGE;
  lidar_left_min_range = LIDAR_MAX_TRACE_RANGE;
  lidar_right_min_range = LIDAR_MAX_TRACE_RANGE;
  if (!lidar_available || !controller_webots_sensors_has_lidar(&webots_sensors) ||
      lidar_resolution <= 1 || lidar_fov <= EPS) return;

  const float *ranges = controller_webots_sensors_lidar_ranges(&webots_sensors);
  if (!ranges) return;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);
  const double now_time = wb_robot_get_time();
  controller_lidar_trace_prune(
      obstacle_trace, &obstacle_trace_count, now_time, LIDAR_TRACE_TTL_SECONDS);
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
  ControllerLidarScanStats stats;
  controller_lidar_scan_capture(
      &scan_config,
      ranges,
      lidar_resolution,
      lidar_fov,
      lidar_max_range,
      robot_x,
      robot_y,
      heading,
      now_time,
      obstacle_trace,
      &obstacle_trace_count,
      MAX_OBSTACLE_TRACE_POINTS,
      &stats);
  lidar_last_hit_count = stats.hit_count;
  lidar_front_hit_count = stats.front_hit_count;
  lidar_front_min_range = stats.front_min_range;
  lidar_center_min_range = stats.center_min_range;
  lidar_left_front_min_range = stats.left_front_min_range;
  lidar_right_front_min_range = stats.right_front_min_range;
  lidar_left_min_range = stats.left_min_range;
  lidar_right_min_range = stats.right_min_range;
}

static void merge_trace_into_map(double now_time) {
  controller_lidar_trace_merge_into_map(
      obstacle_trace,
      obstacle_trace_count,
      now_time,
      MAP_MERGE_MAX_AGE_S,
      MAP_MERGE_MIN_HIT_COUNT,
      persistent_map,
      &persistent_map_count,
      MAX_MAP_POINTS,
      MAP_CELL_SIZE,
      EPS,
      &map_dirty);
}

static ControllerWebotsCameraMapSyncContext camera_map_sync_context(void) {
  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  return (ControllerWebotsCameraMapSyncContext){
      camera_map,
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

static ControllerMappingStorePaths mapping_store_paths(void) {
  return (ControllerMappingStorePaths){
      MAP_PATH, MAP_TEMP_PATH, MAP_CSV_PATH, MAP_CSV_TEMP_PATH,
      CAMERA_MAP_PATH, CAMERA_MAP_TEMP_PATH, CAMERA_MAP_CSV_PATH, CAMERA_MAP_CSV_TEMP_PATH};
}

static void clear_persistent_map() {
  const ControllerMappingStorePaths paths = mapping_store_paths();
  controller_mapping_store_clear(&mapping_store, &paths);
  camera_map = mapping_store.camera_map;
}

static void generate_survey_route(const char *path) {
  const ControllerSurveyDefaultRouteConfig config = {
      SURVEY_X_MIN, SURVEY_X_MAX, SURVEY_Y_MIN, SURVEY_Y_MAX, SURVEY_STRIP};
  controller_survey_default_route_write(path, &config);
}

static void maybe_write_map(void) {
  if ((step_counter % MAP_WRITE_INTERVAL) != 0) return;
  const ControllerMappingStorePaths paths = mapping_store_paths();
  controller_mapping_store_write(&mapping_store, &paths, MAP_CELL_SIZE, CAMERA_MAP_CELL_SIZE);
}


#define survey_expand_bounds(x, y, min_x, max_x, min_y, max_y) \
  controller_survey_expand_bounds((x), (y), (min_x), (max_x), (min_y), (max_y))

static ControllerMappingSurveySafetyContext mapping_survey_safety_context(void) {
  return (ControllerMappingSurveySafetyContext){
      &controller_runtime.limit_zones, persistent_map, persistent_map_count,
      obstacle_trace, obstacle_trace_count, wb_robot_get_time(),
      LIDAR_TRACE_TTL_SECONDS, 0.18, MAPPING_SURVEY_MAX_EXTENT_X,
      MAPPING_SURVEY_MAX_EXTENT_Y, MAPPING_SURVEY_MAP_OBSTACLE_CLEARANCE,
      MAPPING_SURVEY_GRID_CELL};
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

static int survey_segment_safe(double ax, double ay, double bx, double by, int room_zone_index, double clearance) {
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_segment_safe(
      &context, ax, ay, bx, by, room_zone_index, clearance);
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

static int mapping_survey_escape_candidate_allowed(void *context, const Waypoint *candidate) {
  const SurveyPoint *robot = context;
  const ControllerMappingSurveyEscapeSafetyContext safety = {
      mapping_survey_safety_context(),
      controller_runtime.mapping_survey.room_zone_index,
      MAPPING_SURVEY_GRID_CELL,
      MAPPING_SURVEY_ESCAPE_OBSTACLE_CLEARANCE,
      MAPPING_SURVEY_ESCAPE_SEGMENT_CLEARANCE,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
  };
  return controller_mapping_survey_escape_candidate_allowed(&safety, *robot, candidate);
}

static int find_mapping_survey_escape_waypoint(double x, double y, int start_index) {
  const SurveyPoint robot = {x, y};
  const ControllerMappingSurveyEscapeInput input = {
      &controller_runtime.route, controller_runtime.mapping_survey.route_active, start_index,
      x, y, MAPPING_SURVEY_ESCAPE_MIN_TARGET_DISTANCE, MAPPING_SURVEY_ESCAPE_SCAN_AHEAD};
  return controller_mapping_survey_find_escape_waypoint(
      &input, mapping_survey_escape_candidate_allowed, (void *)&robot);
}

#define survey_route_add(route, count, x, y) \
  controller_survey_route_add((route), (count), MAX_WAYPOINTS, 0.18, (x), (y))
#define survey_route_add_segment(route, count, from, to) \
  controller_survey_route_add_segment( \
      (route), (count), MAX_WAYPOINTS, 0.18, MAPPING_SURVEY_MAX_CONTOUR_STEP, (from), (to))

typedef struct {
  SurveyPoint *route;
  int *route_count;
  int room_zone_index;
} SurveyContourContext;

static int survey_contour_point_is_safe(void *context, SurveyPoint point) {
  const SurveyContourContext *contour = context;
  return survey_point_safe(
      point.x,
      point.y,
      contour->room_zone_index,
      MAPPING_SURVEY_CONTOUR_OFFSET * 0.72);
}

static void survey_contour_add_point(void *context, SurveyPoint point) {
  SurveyContourContext *contour = context;
  survey_route_add(contour->route, contour->route_count, point.x, point.y);
}

static void survey_contour_add_segment(
    void *context,
    SurveyPoint from,
    SurveyPoint to) {
  SurveyContourContext *contour = context;
  survey_route_add_segment(contour->route, contour->route_count, from, to);
}

static int append_room_contour_phase(
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    double robot_x,
    double robot_y) {
  if (room_zone_index < 0) return 0;
  SurveyPoint contour[MAX_ZONE_POINTS];
  int contour_count = 0;
  if (!controller_survey_build_offset_contour(
          &controller_runtime.limit_zones.zones[room_zone_index],
          MAPPING_SURVEY_CONTOUR_OFFSET,
          contour,
          MAX_ZONE_POINTS,
          &contour_count)) {
    return 0;
  }

  SurveyContourContext context = {route, route_count, room_zone_index};
  return controller_survey_append_contour(
      contour,
      contour_count,
      robot_x,
      robot_y,
      route_count,
      survey_contour_point_is_safe,
      survey_contour_add_point,
      survey_contour_add_segment,
      &context);
}

static int survey_grid_point_is_safe(
    void *context,
    double x,
    double y,
    int room_zone_index,
    double clearance) {
  (void)context;
  return survey_point_safe(x, y, room_zone_index, clearance);
}

static int survey_build_grid(
    SurveyGrid *grid,
    int room_zone_index,
    double robot_x,
    double robot_y,
    double clearance,
    const RuntimeCommand *command) {
  const ControllerSurveyGridConfig config = {
      SURVEY_X_MIN,
      SURVEY_X_MAX,
      SURVEY_Y_MIN,
      SURVEY_Y_MAX,
      MAPPING_SURVEY_MAX_EXTENT_X,
      MAPPING_SURVEY_MAX_EXTENT_Y,
      MAPPING_SURVEY_GRID_CELL,
      MAPPING_SURVEY_MAX_GRID_CELLS,
  };
  const ControllerSurveyGridInput input = {
      &controller_runtime.limit_zones,
      persistent_map,
      persistent_map_count,
      room_zone_index,
      robot_x,
      robot_y,
      clearance,
      command && command->has_field_bounds,
      command ? command->field_min_x : 0.0,
      command ? command->field_max_x : 0.0,
      command ? command->field_min_y : 0.0,
      command ? command->field_max_y : 0.0,
  };
  return controller_survey_grid_build(
      grid,
      &config,
      &input,
      survey_grid_point_is_safe,
      NULL);
}

#define survey_grid_index_for_point(grid, x, y) \
  controller_survey_grid_index_for_point((grid), (x), (y))
#define survey_grid_point(grid, index) controller_survey_grid_point((grid), (index))
#define survey_flood_component(grid, robot_x, robot_y) \
  controller_survey_flood_component((grid), (robot_x), (robot_y))
#define survey_cell_is_boundary(grid, index) \
  controller_survey_cell_is_boundary((grid), (index))
#define survey_rdp_keep(points, first, last, keep) \
  controller_survey_rdp_keep((points), (first), (last), MAPPING_SURVEY_RDP_EPS, (keep))

#define append_grid_boundary_contour_phase(grid, route, route_count, robot_x, robot_y) \
  controller_survey_append_boundary_contour( \
      (grid), (route), (route_count), MAX_WAYPOINTS, MAPPING_SURVEY_MAX_BOUNDARY_POINTS, \
      (robot_x), (robot_y), 0.18, MAPPING_SURVEY_MAX_CONTOUR_STEP, 3.2, MAPPING_SURVEY_RDP_EPS)

static int build_scanline_intervals(
    double y,
    int room_zone_index,
    const SurveyGrid *grid,
    SurveyInterval *intervals,
    int max_intervals) {
  return controller_survey_build_horizontal_intervals(
      y,
      room_zone_index,
      grid,
      &controller_runtime.limit_zones,
      persistent_map,
      persistent_map_count,
      MAPPING_SURVEY_INTERIOR_OFFSET,
      MAPPING_SURVEY_MIN_STRIP_LENGTH,
      EPS,
      intervals,
      max_intervals);
}

#define survey_find_grid_path(grid, from, to, path, path_count, max_path_count) \
  controller_survey_find_grid_path((grid), (from), (to), (path), (path_count), (max_path_count))

static void append_safe_transition(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    SurveyPoint target,
    int room_zone_index) {
  if (*route_count <= 0) {
    survey_route_add(route, route_count, target.x, target.y);
    return;
  }

  const SurveyPoint from = route[*route_count - 1];
  if (survey_segment_safe(from.x, from.y, target.x, target.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET)) {
    survey_route_add(route, route_count, target.x, target.y);
    return;
  }

  SurveyPoint path[256];
  int path_count = 0;
  if (grid && survey_find_grid_path(grid, from, target, path, &path_count, 256)) {
    for (int i = 1; i < path_count; ++i) {
      survey_route_add(route, route_count, path[i].x, path[i].y);
    }
    return;
  }
}

static void append_survey_coverage_segment(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    SurveyPoint a,
    SurveyPoint b) {
  if (!survey_point_safe(a.x, a.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET) ||
      !survey_point_safe(b.x, b.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET)) {
    return;
  }
  append_safe_transition(grid, route, route_count, a, room_zone_index);
  if (survey_segment_safe(a.x, a.y, b.x, b.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET)) {
    survey_route_add(route, route_count, b.x, b.y);
  } else {
    append_safe_transition(grid, route, route_count, b, room_zone_index);
  }
}

static int build_clipped_horizontal_intervals(
    SurveyGrid *grid,
    int room_zone_index,
    double y,
    double min_x,
    double max_x,
    SurveyInterval *intervals,
    int max_intervals) {
  SurveyInterval raw[64];
  const int raw_count = build_scanline_intervals(y, room_zone_index, grid, raw, 64);
  return controller_survey_clip_intervals(
      raw, raw_count, min_x, max_x, MAPPING_SURVEY_MIN_STRIP_LENGTH,
      intervals, max_intervals);
}

static int build_clipped_vertical_intervals(
    SurveyGrid *grid,
    int room_zone_index,
    double x,
    double min_y,
    double max_y,
    SurveyInterval *intervals,
    int max_intervals);

typedef struct {
  SurveyGrid *grid;
  SurveyPoint *route;
  int *route_count;
  int room_zone_index;
  double interval_min;
  double interval_max;
  int vertical;
} SurveyCoverageContext;

static int build_coverage_intervals(
    void *context,
    double coordinate,
    SurveyInterval *intervals,
    int capacity) {
  SurveyCoverageContext *coverage = context;
  if (coverage->vertical) {
    return build_clipped_vertical_intervals(
        coverage->grid,
        coverage->room_zone_index,
        coordinate,
        coverage->interval_min,
        coverage->interval_max,
        intervals,
        capacity);
  }
  return build_clipped_horizontal_intervals(
      coverage->grid,
      coverage->room_zone_index,
      coordinate,
      coverage->interval_min,
      coverage->interval_max,
      intervals,
      capacity);
}

static void append_coverage_segment(
    void *context,
    SurveyPoint start,
    SurveyPoint end) {
  SurveyCoverageContext *coverage = context;
  append_survey_coverage_segment(
      coverage->grid,
      coverage->route,
      coverage->route_count,
      coverage->room_zone_index,
      start,
      end);
}

static void append_scanline_coverage_phase(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  controller_survey_get_coverage_bounds(
      grid, &controller_runtime.limit_zones, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET,
      &min_x, &max_x, &min_y, &max_y);

  const SurveyPoint current =
      *route_count > 0 ? route[*route_count - 1] : (SurveyPoint){min_x, min_y};
  SurveyCoverageContext context = {
      grid, route, route_count, room_zone_index, min_x, max_x, 0};
  controller_survey_append_best_axis_coverage(
      min_y, max_y, MAPPING_SURVEY_STRIP, 0, current,
      route_count, MAX_WAYPOINTS, 64,
      build_coverage_intervals, append_coverage_segment, &context);
}

static int build_vertical_intervals(
    double x,
    int room_zone_index,
    const SurveyGrid *grid,
    SurveyInterval *intervals,
    int max_intervals) {
  return controller_survey_build_vertical_intervals(
      x,
      room_zone_index,
      grid,
      &controller_runtime.limit_zones,
      persistent_map,
      persistent_map_count,
      MAPPING_SURVEY_INTERIOR_OFFSET,
      MAPPING_SURVEY_MIN_STRIP_LENGTH,
      EPS,
      intervals,
      max_intervals);
}

static int build_clipped_vertical_intervals(
    SurveyGrid *grid,
    int room_zone_index,
    double x,
    double min_y,
    double max_y,
    SurveyInterval *intervals,
    int max_intervals) {
  SurveyInterval raw[64];
  const int raw_count = build_vertical_intervals(x, room_zone_index, grid, raw, 64);
  return controller_survey_clip_intervals(
      raw, raw_count, min_y, max_y, MAPPING_SURVEY_MIN_STRIP_LENGTH,
      intervals, max_intervals);
}

static void append_vertical_coverage_phase(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  controller_survey_get_coverage_bounds(
      grid, &controller_runtime.limit_zones, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET,
      &min_x, &max_x, &min_y, &max_y);

  const SurveyPoint current =
      *route_count > 0 ? route[*route_count - 1] : (SurveyPoint){min_x, min_y};
  SurveyCoverageContext context = {
      grid, route, route_count, room_zone_index, min_y, max_y, 1};
  controller_survey_append_best_axis_coverage(
      min_x, max_x, MAPPING_SURVEY_STRIP, 1, current,
      route_count, MAX_WAYPOINTS, 64,
      build_coverage_intervals, append_coverage_segment, &context);
}

static void write_mapping_survey_route_file(const char *path, const SurveyPoint *route, int route_count) {
  if (!controller_mapping_route_write(path, route, route_count, controller_runtime.mapping_survey.mode)) {
    set_error("Cannot write mapping survey route to route.csv");
  }
}

typedef struct {
  SurveyGrid *grid;
  SurveyPoint *route;
  int *route_count;
  int room_zone_index;
  double robot_x;
  double robot_y;
} SurveyRouteBuilderContext;

static int survey_route_start_is_safe(void *context, SurveyPoint start) {
  const SurveyRouteBuilderContext *builder = context;
  return survey_point_safe(
      start.x,
      start.y,
      builder->room_zone_index,
      MAPPING_SURVEY_INTERIOR_OFFSET * 0.45);
}

static void survey_route_add_start(void *context, SurveyPoint start) {
  SurveyRouteBuilderContext *builder = context;
  survey_route_add(builder->route, builder->route_count, start.x, start.y);
}

static int survey_route_append_room_contour(void *context) {
  SurveyRouteBuilderContext *builder = context;
  return append_room_contour_phase(
      builder->route,
      builder->route_count,
      builder->room_zone_index,
      builder->robot_x,
      builder->robot_y);
}

static void survey_route_append_grid_boundary(void *context) {
  SurveyRouteBuilderContext *builder = context;
  append_grid_boundary_contour_phase(
      builder->grid,
      builder->route,
      builder->route_count,
      builder->robot_x,
      builder->robot_y);
}

static void survey_route_append_horizontal(void *context) {
  SurveyRouteBuilderContext *builder = context;
  append_scanline_coverage_phase(
      builder->grid, builder->route, builder->route_count, builder->room_zone_index);
}

static void survey_route_append_vertical(void *context) {
  SurveyRouteBuilderContext *builder = context;
  append_vertical_coverage_phase(
      builder->grid, builder->route, builder->route_count, builder->room_zone_index);
}

typedef struct {
  const char *path;
  const RuntimeCommand *command;
  SurveyGrid grid;
} SurveyGeneratorContext;

static void survey_generator_clear_map(void *context) {
  (void)context;
  clear_persistent_map();
}

static void survey_generator_prepare(void *context, MappingSurveyMode mode) {
  (void)context;
  controller_mapping_survey_state_prepare(&controller_runtime.mapping_survey, mode);
}

static SurveyPoint survey_generator_read_robot(void *context) {
  (void)context;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  read_pose(&x, &y, &heading);
  return (SurveyPoint){x, y};
}

static int survey_generator_find_room(void *context, SurveyPoint robot) {
  (void)context;
  return find_room_zone_index(robot.x, robot.y);
}

static int survey_generator_build_grid(
    void *context,
    SurveyPoint robot,
    int room_zone_index) {
  SurveyGeneratorContext *generator = context;
  return survey_build_grid(
      &generator->grid,
      room_zone_index,
      robot.x,
      robot.y,
      MAPPING_SURVEY_INTERIOR_OFFSET,
      generator->command);
}

static int survey_generator_flood_grid(void *context, SurveyPoint robot) {
  SurveyGeneratorContext *generator = context;
  return survey_flood_component(&generator->grid, robot.x, robot.y);
}

static int survey_generator_build_route(
    void *context,
    MappingSurveyMode mode,
    SurveyPoint robot,
    int room_zone_index,
    SurveyPoint *route,
    int *route_count,
    int *interior_start_index) {
  (void)mode;
  SurveyGeneratorContext *generator = context;
  SurveyRouteBuilderContext builder = {
      &generator->grid, route, route_count, room_zone_index, robot.x, robot.y};
  const ControllerSurveyRouteCallbacks callbacks = {
      survey_route_start_is_safe,
      survey_route_add_start,
      survey_route_append_room_contour,
      survey_route_append_grid_boundary,
      survey_route_append_horizontal,
      survey_route_append_vertical,
  };
  return controller_survey_build_route_phases(
      controller_runtime.mapping_survey.mode,
      robot,
      route_count,
      interior_start_index,
      &callbacks,
      &builder);
}

static void survey_generator_write_route(
    void *context,
    MappingSurveyMode mode,
    const SurveyPoint *route,
    int route_count) {
  (void)mode;
  SurveyGeneratorContext *generator = context;
  write_mapping_survey_route_file(generator->path, route, route_count);
}

static int generate_mapping_survey_route(
    const char *path,
    int clear_map_before_start,
    const RuntimeCommand *command) {
  const MappingSurveyMode survey_mode =
      command ? command->survey_mode : controller_runtime.mapping_survey.mode;
  static SurveyGeneratorContext generator;
  generator.path = path;
  generator.command = command;
  const ControllerSurveyGeneratorCallbacks callbacks = {
      survey_generator_clear_map,
      survey_generator_prepare,
      survey_generator_read_robot,
      survey_generator_find_room,
      survey_generator_build_grid,
      survey_generator_flood_grid,
      survey_generator_build_route,
      survey_generator_write_route,
  };
  const ControllerSurveyGenerateResult result = controller_survey_generate(
      clear_map_before_start,
      survey_mode,
      &controller_runtime.mapping_survey.room_zone_index,
      &controller_runtime.mapping_survey.interior_start_index,
      &callbacks,
      &generator);
  if (result == CONTROLLER_SURVEY_GENERATE_GRID_FAILED) {
    set_error("Cannot build mapping survey occupancy grid");
  } else if (result == CONTROLLER_SURVEY_GENERATE_NO_COMPONENT) {
    set_error("Cannot find connected free room for mapping survey");
  } else if (result == CONTROLLER_SURVEY_GENERATE_EMPTY_ROUTE) {
    set_error("Mapping survey route is empty");
  }
  return result == CONTROLLER_SURVEY_GENERATE_OK;
}

static int load_runtime_command(RuntimeCommand *command) {
  const ControllerRuntimeCommandLimits limits = {
      SURVEY_X_MIN,
      SURVEY_X_MAX,
      SURVEY_Y_MIN,
      SURVEY_Y_MAX,
      MAPPING_SURVEY_MAX_EXTENT_X,
      MAPPING_SURVEY_MAX_EXTENT_Y,
  };
  return controller_runtime_command_load_file(
      RUNTIME_COMMAND_PATH,
      &limits,
      command);
}

static void survey_integration_apply_speed(double speed_mps) {
  configured_cruise_speed_mps =
      clamp_value(speed_mps, MIN_CRUISE_SPEED_MPS, MAX_CRUISE_SPEED_MPS);
  apply_motion_profile();
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

static void maybe_reload_runtime_command(void) {
  if ((step_counter % RUNTIME_COMMAND_RELOAD_INTERVAL) != 0) return;

  const long long mtime = get_file_mtime(RUNTIME_COMMAND_PATH);
  if (mtime < 0) return;
  if (mtime == runtime_command_last_modified) return;

  RuntimeCommand command = {0};
  if (!load_runtime_command(&command)) {
    runtime_command_last_modified = mtime;
    return;
  }

  runtime_command_last_modified = mtime;
  if (command.id <= last_processed_runtime_command_id) return;
  last_processed_runtime_command_id = command.id;

  if (command.has_start_mapping_survey) {
    const ControllerSurveyIntegrationOps ops = survey_integration_ops();
    controller_survey_integration_start(
        ROUTE_PATH,
        &command,
        &controller_runtime.route,
        &controller_runtime.current_waypoint_index,
        &controller_runtime.route_finished,
        &controller_runtime.mapping_survey,
        &ops);
    return;
  }

  controller_webots_zone_sync_spawn_obstacle(&webots_zone_sync, &command);
  clear_error();
  set_status("runtime_obstacle_spawned");
}

static void maybe_reload_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;

  const ControllerRouteZoneServicePaths paths = {
      ROUTE_PATH, ZONE_PATH, SURFACE_ZONE_PATH};
  const ControllerRouteZoneServiceReloadRequest request = {0, 1, 0};
  const ControllerRouteZoneServiceResult result =
      controller_route_zone_service_reload(
          &route_zone_service, &paths, &request,
          &controller_runtime.route, &controller_runtime.limit_zones, &controller_runtime.surface_zones);
  if (result.limit_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_OK &&
      result.limit_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_UNCHANGED) {
    set_error("Cannot parse limit_zones.txt");
    return;
  }
  if (result.limit_zones_changed) {
    controller_runtime.limit_zones = result.limit_zones;
    controller_webots_zone_sync_limit_zones(
        &webots_zone_sync, &controller_runtime.limit_zones);
    set_status(controller_runtime.limit_zones.count > 0 ? "zones_synced" : "zones_cleared");
  }
}

static void maybe_reload_surface_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;

  const ControllerRouteZoneServicePaths paths = {
      ROUTE_PATH, ZONE_PATH, SURFACE_ZONE_PATH};
  const ControllerRouteZoneServiceReloadRequest request = {0, 0, 1};
  const ControllerRouteZoneServiceResult result =
      controller_route_zone_service_reload(
          &route_zone_service, &paths, &request,
          &controller_runtime.route, &controller_runtime.limit_zones, &controller_runtime.surface_zones);
  if (result.surface_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_OK &&
      result.surface_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_UNCHANGED) {
    set_error("Cannot parse surface_zones.txt");
    return;
  }
  if (result.surface_zones_changed) {
    controller_runtime.surface_zones = result.surface_zones;
    controller_webots_zone_sync_surface_zones(
        &webots_zone_sync, &controller_runtime.surface_zones);
    set_status(controller_runtime.surface_zones.count > 0 ? "surfaces_synced" : "surfaces_cleared");
  }
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

  const ControllerRouteZoneServicePaths paths = {
      ROUTE_PATH, ZONE_PATH, SURFACE_ZONE_PATH};
  const ControllerRouteZoneServiceReloadRequest request = {1, 0, 0};
  const ControllerRouteZoneServiceResult result =
      controller_route_zone_service_reload(
          &route_zone_service, &paths, &request,
          &controller_runtime.route, &controller_runtime.limit_zones, &controller_runtime.surface_zones);
  if (result.route_status == CONTROLLER_ROUTE_ZONE_STATUS_ROUTE_CANNOT_OPEN) {
    set_error("Cannot open route.csv");
    return;
  }
  if (result.route_status == CONTROLLER_ROUTE_ZONE_STATUS_ROUTE_EMPTY) {
    set_error("Route file is empty");
    return;
  }
  if (result.route_changed) {
    controller_survey_lifecycle_accept_route(
        &controller_runtime.route,
        &result.route,
        &controller_runtime.current_waypoint_index,
        &controller_runtime.route_finished,
        &controller_runtime.mapping_survey,
        0,
        controller_runtime.mapping_survey.mode);
    reset_route_avoidance_metrics();
    reset_navigation_mode();
    set_status("route_reloaded");
  }
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
  const ControllerMappingSurveySafetyContext context = mapping_survey_safety_context();
  return controller_mapping_survey_runtime_scan_point_allowed(
      &context,
      x,
      y,
      controller_runtime.mapping_survey.room_zone_index,
      ZONE_CLEARANCE,
      ZONE_CLEARANCE * 0.68);
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
  const ControllerRuntimeNavigationConfig navigation_config = {
      POSITION_TOLERANCE,
      HEADING_TOLERANCE_RAD,
      ZONE_CLEARANCE,
      POSE_RELOCATION_DISTANCE,
      POSE_RELOCATION_HEADING_RAD};
  const ControllerRuntimeNavigationResult navigation_result =
      controller_runtime_process_navigation_frame(
          &controller_runtime, &sensor_frame, &navigation_config);

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
  const ControllerNavigationPerceptionConfig perception_config = {
      .camera_range_epsilon = EPS,
      .camera_range_fallback = CAMERA_RANGE_FALLBACK_M,
      .camera_min_fov = 0.8,
      .camera_front_fov_factor = 0.46,
      .camera_range_margin = 0.18,
      .camera_min_score = CAMERA_OBSTACLE_MIN_SCORE,
      .camera_min_detection_count = 3,
      .camera_offset_deadband = CAMERA_OBSTACLE_OFFSET_DEADBAND,
      .avoidance = {
          .max_trace_range = LIDAR_MAX_TRACE_RANGE,
          .track_caution_range = LIDAR_TRACK_CAUTION_RANGE,
          .expected_wall_soft_stop_range = EXPECTED_WALL_SOFT_STOP_RANGE,
          .expected_wall_slowdown_range = EXPECTED_WALL_SLOWDOWN_RANGE,
          .pass_center_clear_range = LIDAR_PASS_CENTER_CLEAR_RANGE,
          .pass_gap_max_angle_rad = LIDAR_PASS_GAP_MAX_ANGLE_RAD,
          .pass_side_danger_range = LIDAR_PASS_SIDE_DANGER_RANGE,
          .avoid_side_trigger_range = LIDAR_AVOID_SIDE_TRIGGER_RANGE,
          .track_hard_priority_range = LIDAR_TRACK_HARD_PRIORITY_RANGE,
          .reflex_side_release_range = LIDAR_REFLEX_SIDE_RELEASE_RANGE,
          .avoid_recover_range = LIDAR_AVOID_RECOVER_RANGE,
          .avoid_trigger_range = LIDAR_AVOID_TRIGGER_RANGE,
          .avoid_stop_range = LIDAR_AVOID_STOP_RANGE,
          .track_slow_range = LIDAR_TRACK_SLOW_RANGE,
      },
  };
  const ControllerNavigationPerceptionInput perception_input = {
      .lidar_context = &lidar_context,
      .lidar_available = lidar_available,
      .camera_visible = camera_obstacle_visible,
      .camera_angle = camera_obstacle_angle,
      .camera_fov = camera_fov,
      .camera_range = camera_obstacle_range,
      .camera_score = camera_obstacle_score,
      .camera_detection_count = camera_detection_count,
      .camera_center_offset = camera_obstacle_center_offset,
  };
  ControllerNavigationPerceptionOutput perception_output;
  controller_navigation_perception_prepare(
      &perception_input, &perception_config, &perception_output);
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

  const ControllerAvoidanceStartConfig avoidance_start_config = {
      .switch_margin = LIDAR_REFLEX_SWITCH_MARGIN,
      .initial_hold_steps = LIDAR_AVOID_MIN_CONTOUR_STEPS,
      .priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS,
      .detour = {
          .forward_distance = LIDAR_DETOUR_FORWARD_M,
          .max_distance = LIDAR_DETOUR_MAX_RANGE_M,
          .side_distance = LIDAR_DETOUR_SIDE_M,
      },
  };
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
          &avoidance_start_config,
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
    const ControllerAvoidanceProgressConfig progress_config = {
        .stuck_pose_epsilon = LIDAR_AVOID_STUCK_POSE_EPS,
        .stuck_progress_epsilon = LIDAR_AVOID_STUCK_PROGRESS_EPS,
        .best_progress_epsilon = MAPPING_SURVEY_AVOID_PROGRESS_EPS,
        .min_contour_steps = LIDAR_AVOID_MIN_CONTOUR_STEPS,
        .detour_reached_distance = LIDAR_DETOUR_REACHED_M,
    };
    const ControllerAvoidanceProgressInput progress_input = {
        .x = x,
        .z = z,
        .heading = heading,
        .target_distance = target_distance_now,
        .obstacle_context_present = obstacle_context_present,
        .detour_distance = detour_distance,
    };
    const ControllerAvoidanceLifecycleConfig lifecycle_config = {
        .min_contour_steps = LIDAR_AVOID_MIN_CONTOUR_STEPS,
        .leave_progress = LIDAR_AVOID_LEAVE_PROGRESS,
        .leave_heading_rad = LIDAR_AVOID_LEAVE_HEADING_RAD,
        .avoid_stop_range = LIDAR_AVOID_STOP_RANGE,
        .orbit_heading_rad = MAPPING_SURVEY_AVOID_ORBIT_HEADING_RAD,
        .no_progress_steps = MAPPING_SURVEY_AVOID_NO_PROGRESS_STEPS,
        .replan_steps = MAPPING_SURVEY_AVOID_REPLAN_STEPS,
        .max_steps = MAPPING_SURVEY_AVOID_MAX_STEPS,
        .loop_radius = MAPPING_SURVEY_AVOID_LOOP_RADIUS,
        .free_space_recovery_steps = FREE_SPACE_RECOVERY_STEPS,
        .clear_steps = LIDAR_AVOID_CLEAR_STEPS,
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
    const ControllerAvoidanceCommandConfig command_config = {
        .avoid_recover_range = LIDAR_AVOID_RECOVER_RANGE,
        .avoid_stop_range = LIDAR_AVOID_STOP_RANGE,
        .avoid_reverse_range = LIDAR_AVOID_REVERSE_RANGE,
        .gap_min_range = LIDAR_GAP_MIN_RANGE,
        .track_caution_range = LIDAR_TRACK_CAUTION_RANGE,
        .track_side_bias_range = LIDAR_TRACK_SIDE_BIAS_RANGE,
        .pass_side_danger_range = LIDAR_PASS_SIDE_DANGER_RANGE,
        .pass_center_clear_range = LIDAR_PASS_CENTER_CLEAR_RANGE,
        .pass_cruise_speed_factor = LIDAR_PASS_CRUISE_SPEED_FACTOR,
        .pass_steer_gain = LIDAR_PASS_STEER_GAIN,
        .min_angular_command = MIN_ANGULAR_COMMAND,
        .gap_switch_range_bonus = LIDAR_GAP_SWITCH_RANGE_BONUS,
        .stuck_steps_limit = LIDAR_AVOID_STUCK_STEPS,
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
        .progress_config = &progress_config,
        .progress_input = progress_input,
        .lifecycle_config = &lifecycle_config,
        .lifecycle_input = lifecycle_input,
        .has_next_waypoint = controller_runtime.current_waypoint_index + 1 < controller_runtime.route.count,
        .command_config = &command_config,
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
  const ControllerNavigationTrackingConfig tracking_config = {
      .final_align_distance = FINAL_ALIGN_DISTANCE,
      .heading_tolerance_rad = HEADING_TOLERANCE_RAD,
      .track_slow_radius = TRACK_SLOW_RADIUS,
      .turn_exit_error_rad = TURN_EXIT_ERROR_RAD,
      .track_reenter_turn_rad = TRACK_REENTER_TURN_RAD,
      .turn_enter_error_rad = TURN_ENTER_ERROR_RAD,
      .turn_heading_gain = TURN_HEADING_GAIN,
      .track_heading_gain = TRACK_HEADING_GAIN,
      .final_align_gain = FINAL_ALIGN_GAIN,
      .track_min_linear_speed = TRACK_MIN_LINEAR_SPEED,
      .min_angular_command = MIN_ANGULAR_COMMAND,
      .position_tolerance = POSITION_TOLERANCE,
  };
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
  const ControllerNavigationLidarConfig navigation_lidar_config = {
      .track_caution_range = LIDAR_TRACK_CAUTION_RANGE,
      .avoid_stop_range = LIDAR_AVOID_STOP_RANGE,
      .track_side_bias_range = LIDAR_TRACK_SIDE_BIAS_RANGE,
      .avoid_side_danger_range = LIDAR_AVOID_SIDE_DANGER_RANGE,
      .track_hard_priority_range = LIDAR_TRACK_HARD_PRIORITY_RANGE,
      .avoid_side_trigger_range = LIDAR_AVOID_SIDE_TRIGGER_RANGE,
      .max_heading_bias = LIDAR_TRACK_MAX_HEADING_BIAS,
      .priority_switch_margin = LIDAR_PRIORITY_SWITCH_MARGIN,
      .priority_center_margin = LIDAR_PRIORITY_CENTER_MARGIN,
      .priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS,
      .expected_wall_soft_stop_range = EXPECTED_WALL_SOFT_STOP_RANGE,
      .expected_wall_slowdown_range = EXPECTED_WALL_SLOWDOWN_RANGE,
  };
  const ControllerNavigationLidarInput navigation_lidar_input = {
      .lidar_available = lidar_available,
      .center_passage_available = center_passage_available,
      .center_obstacle_range = center_obstacle_range,
      .near_front_range = near_front_range,
      .left_lidar_context = left_lidar_context,
      .right_lidar_context = right_lidar_context,
      .best_gap_beam_angle = lidar_context.best_gap_beam_angle,
      .heading_error = heading_error_to_target,
      .camera_visual_front_obstacle = camera_visual_front_obstacle,
      .camera_obstacle_center_offset = camera_obstacle_center_offset,
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
      &tracking_config,
      &navigation_lidar_config,
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
  const double simulation_now = wb_robot_get_time();
  read_pose(&x, &y, &heading);

  ControllerTelemetryPoint trace_points[MAX_OBSTACLE_TRACE_POINTS];
  const int trace_point_count = controller_telemetry_service_collect_trace(
      obstacle_trace,
      obstacle_trace_count,
      simulation_now,
      LIDAR_TRACE_TTL_SECONDS,
      LIDAR_TRACE_MIN_CONFIDENCE,
      trace_points,
      MAX_OBSTACLE_TRACE_POINTS);

  ControllerTelemetryNavigation navigation = {0};
  controller_telemetry_service_build_navigation(
      navigation_status,
      navigation_error,
      controller_runtime.current_waypoint_index,
      controller_runtime.route_finished,
      controller_runtime.distance_to_target,
      controller_runtime.avoidance.active,
      route_off_route_active_now(),
      route_avoidance_time_sec,
      route_avoidance_steps,
      &controller_runtime.route,
      &navigation);
  const ControllerTelemetryMotionProfile motion_profile = {
      configured_cruise_speed_mps,
      configured_payload_kg,
      configured_battery_range_units,
      active_battery_speed_factor,
      active_linear_speed_limit,
      active_angular_speed_limit,
  };
  const ControllerTelemetryLidar lidar = {
      lidar_available,
      lidar_resolution,
      lidar_max_range,
      lidar_last_hit_count,
      lidar_front_hit_count,
      lidar_front_min_range,
      lidar_center_min_range,
      lidar_left_front_min_range,
      lidar_right_front_min_range,
      lidar_left_min_range,
      lidar_right_min_range,
  };
  const ControllerTelemetryCamera camera = {
      camera_available,
      camera_width,
      camera_height,
      camera_fov,
      camera_virtual_mode ? "virtual_lidar" : "webots_camera",
      camera_frame_metadata.file_name,
      camera_frame_metadata.mime_type,
      camera_frame_metadata.sequence,
      camera_frame_metadata.time,
      camera_obstacle_visible,
      camera_obstacle_score,
      camera_obstacle_center_offset,
      camera_obstacle_angle,
      camera_obstacle_range,
      camera_detection_count,
  };
  const ControllerTelemetryServiceSnapshotInput snapshot_input = {
      simulation_now,
      x,
      y,
      START_HEIGHT,
      heading,
      navigation,
      motion_profile,
      controller_runtime.limit_zones.count,
      zone_node_registry.count,
      lidar,
      camera,
      trace_points,
      trace_point_count,
      persistent_map_count,
      MAP_CELL_SIZE,
      camera_map_count + camera_free_map_count,
      camera_map_count,
      camera_free_map_count,
      CAMERA_MAP_CELL_SIZE,
      controller_runtime.route.waypoints,
      controller_runtime.route.count,
  };
  ControllerTelemetrySnapshot snapshot = {0};
  controller_telemetry_service_build_snapshot(&snapshot_input, &snapshot);

  controller_telemetry_write_snapshot(STATE_TEMP_PATH, STATE_PATH, &snapshot);
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
    maybe_write_map,
    run_navigation_step,
    update_route_avoidance_metrics,
    write_state_snapshot,
};

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  if (!controller_paths_init(&controller_paths, getenv("WEB_STATE_DIR"))) {
    fprintf(stderr, "Failed to initialize Webots state paths.\n");
    return 1;
  }

  wb_robot_init();
  controller_webots_devices_init(&webots_devices);
  const ControllerDriveConfig drive_config = controller_webots_adapter_drive_config(
      WHEEL_RADIUS,
      WHEEL_BASE_LONGITUDINAL,
      WHEEL_BASE_LATERAL,
      MAX_WHEEL_SPEED_RAD_S,
      WHEEL_ACCEL_LIMIT_RAD_S2,
      WHEEL_DECEL_LIMIT_RAD_S2,
      TIME_STEP / 1000.0);
  controller_webots_adapter_init(
      &webots_adapter, &drive_config, drive_webots_base, &webots_devices);
  controller_webots_sensors_init(&webots_sensors);
  controller_webots_navigation_state_init(&navigation_state);
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
  controller_mapping_store_init(&mapping_store);
  controller_route_zone_service_init(&route_zone_service);
  camera_map = mapping_store.camera_map;
  clear_persistent_map();
  controller_webots_camera_adapter_remove_frames(
      CAMERA_FRAME_BMP_PATH,
      CAMERA_FRAME_BMP_TEMP_PATH,
      CAMERA_FRAME_JPEG_PATH,
      CAMERA_FRAME_JPEG_TEMP_PATH);
  apply_motion_profile();
  motion_profile_last_modified = get_file_mtime(MOTION_PROFILE_PATH);
  if (motion_profile_last_modified >= 0) {
    load_motion_profile();
  }
  maybe_reload_zones();
  maybe_reload_surface_zones();
  runtime_command_last_modified = get_file_mtime(RUNTIME_COMMAND_PATH);

  if (!controller_webots_pose_is_ready(&webots_pose) || !webots_pose.root_children_field) {
    set_status("error");
    set_error("Supervisor fields are not available");
  } else {
    if (get_file_mtime(ROUTE_PATH) < 0) {
      clear_persistent_map();
      generate_survey_route(ROUTE_PATH);
      set_status("survey_route_generated");
    }
    wait_for_fresh_route();
  }

  const ControllerLifecycleScheduleConfig lifecycle_schedule = {
      ZONE_RELOAD_INTERVAL,
      ROUTE_RELOAD_INTERVAL,
      MOTION_RELOAD_INTERVAL,
      RUNTIME_COMMAND_RELOAD_INTERVAL,
      MAP_WRITE_INTERVAL,
      CAMERA_CAPTURE_INTERVAL,
      CAMERA_WRITE_INTERVAL,
  };

  while (wb_robot_step(TIME_STEP) != -1) {
    ++step_counter;
    controller_step_run(step_counter, &lifecycle_schedule, &controller_step_callbacks);
  }

  maybe_write_map();
  maybe_write_map();
  controller_webots_zone_sync_remove_all(&webots_zone_sync);
  wb_robot_cleanup();
  return 0;
}
