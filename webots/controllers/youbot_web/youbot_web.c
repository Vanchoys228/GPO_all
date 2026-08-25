#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "controller_avoidance.h"
#include "controller_camera.h"
#include "controller_camera_map.h"
#include "controller_camera_render.h"
#include "controller_io.h"
#include "controller_drive.h"
#include "controller_lidar_math.h"
#include "controller_lifecycle.h"
#include "controller_math.h"
#include "controller_motion_profile.h"
#include "controller_paths.h"
#include "controller_route.h"
#include "controller_runtime_command.h"
#include "controller_survey_geometry.h"
#include "controller_survey_state.h"
#include "controller_step.h"
#include "controller_telemetry.h"
#include "controller_types.h"
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

static WbDeviceTag wheels[4];
static WbDeviceTag arm_joints[5];
static WbDeviceTag gripper_fingers[2];
static WbDeviceTag front_lidar = 0;
static WbDeviceTag front_camera = 0;
static WbNodeRef self_node;
static WbFieldRef translation_field;
static WbFieldRef rotation_field;
static WbFieldRef root_children_field;
static double applied_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

static RouteData route_data = {0};
static ZoneData zone_data = {0};
static SurfaceZoneData surface_zone_data = {0};
static WbNodeRef zone_nodes[MAX_ZONE_NODES];
static char zone_node_defs[MAX_ZONE_NODES][64];
static int zone_node_count = 0;
static WbNodeRef surface_zone_nodes[MAX_SURFACE_ZONE_NODES];
static char surface_zone_defs[MAX_SURFACE_ZONE_NODES][64];
static int surface_zone_count = 0;
static WbNodeRef runtime_obstacle_nodes[MAX_RUNTIME_OBSTACLE_NODES];
static char runtime_obstacle_defs[MAX_RUNTIME_OBSTACLE_NODES][64];
static int runtime_obstacle_count = 0;
static ObstacleTracePoint obstacle_trace[MAX_OBSTACLE_TRACE_POINTS];
static int obstacle_trace_count = 0;
static MapCell persistent_map[MAX_MAP_POINTS];
static int persistent_map_count = 0;
static int map_dirty = 0;
static MapCell camera_map[MAX_CAMERA_MAP_POINTS];
static int camera_map_count = 0;
static MapCell camera_free_map[MAX_CAMERA_FREE_MAP_POINTS];
static int camera_free_map_count = 0;
static int camera_map_dirty = 0;
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
static int camera_frame_sequence = 0;
static double camera_frame_time = 0.0;
static int camera_obstacle_visible = 0;
static double camera_obstacle_score = 0.0;
static double camera_obstacle_center_offset = 0.0;
static double camera_obstacle_angle = 0.0;
static double camera_obstacle_range = 0.0;
static int camera_detection_count = 0;
static int camera_obstacle_update_step = -1;
static int camera_virtual_mode = 0;
static int current_waypoint_index = 0;
static int step_counter = 0;
static int route_finished = 0;
static NavigationMode navigation_mode = NAV_MODE_IDLE;
static int navigation_waypoint_index = -1;
static double navigation_segment_start_x = START_X;
static double navigation_segment_start_z = START_Z;
static char navigation_status[64] = "booting";
static char navigation_error[160] = "";
static double distance_to_target = 0.0;
static double configured_cruise_speed_mps = DEFAULT_CRUISE_SPEED_MPS;
static double configured_payload_kg = DEFAULT_PAYLOAD_KG;
static double configured_battery_range_units = DEFAULT_BATTERY_RANGE_UNITS;
static double runtime_linear_speed_limit = DEFAULT_CRUISE_SPEED_MPS;
static double runtime_angular_speed_limit = KINEMATIC_ANGULAR_SPEED;
static double runtime_battery_speed_factor = 1.0;
static double lidar_priority_turn_sign = 0.0;
static int lidar_priority_hold_steps = 0;
static ControllerAvoidanceState avoidance_state = {
    .mode = AVOID_MODE_NONE,
    .turn_sign = 1.0,
    .prev_x = START_X,
    .prev_z = START_Z,
    .start_x = START_X,
    .start_z = START_Z,
    .detour_x = START_X,
    .detour_z = START_Z,
};
static double last_pose_x = START_X;
static double last_pose_z = START_Z;
static double last_pose_heading = 0.0;
static int last_pose_valid = 0;
static long long motion_profile_last_modified = -1;
static long long runtime_command_last_modified = -1;
static long long last_processed_runtime_command_id = -1;
static int route_source_mapping_survey = 0;
static ControllerMappingSurveyState mapping_survey_state = {
    .room_zone_index = -1,
    .mode = MAPPING_SURVEY_MODE_SNAKE,
    .obstacle_scan_end_index = -1,
    .last_scan_x = 1e30,
    .last_scan_y = 1e30,
};
#define mapping_survey_room_zone_index mapping_survey_state.room_zone_index
#define mapping_survey_mode mapping_survey_state.mode
#define mapping_survey_interior_start_index mapping_survey_state.interior_start_index
#define mapping_survey_obstacle_scan_active mapping_survey_state.obstacle_scan_active
#define mapping_survey_obstacle_scan_end_index mapping_survey_state.obstacle_scan_end_index
#define mapping_survey_obstacle_scan_cooldown_steps mapping_survey_state.obstacle_scan_cooldown_steps
#define mapping_survey_last_scan_x mapping_survey_state.last_scan_x
#define mapping_survey_last_scan_y mapping_survey_state.last_scan_y
#define mapping_survey_replan_cooldown_steps mapping_survey_state.replan_cooldown_steps
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
static const char *camera_frame_file = "camera_frame.bmp";
static const char *camera_frame_mime = "image/bmp";

static int point_near_zone(double x, double y, const LimitZone *zone, double clearance);
static int point_near_zone_boundary(double x, double y, const LimitZone *zone, double tolerance);
static int load_route(RouteData *route);
static void reset_navigation_mode();
static void set_status(const char *status);
static int is_finite_double(double value);
static void read_pose(double *x, double *z, double *heading);
static void merge_camera_observation_into_map(double relative_angle, double range, int confidence_boost);
static void merge_camera_free_ray_into_map(double relative_angle, double range, int confidence_boost);
static double scaled_linear_floor(double factor) {
  const double floor_cap = fmin(TRACK_MIN_LINEAR_SPEED, runtime_linear_speed_limit);
  return clamp_value(runtime_linear_speed_limit * factor, 0.01, floor_cap);
}

static double scaled_linear_cap(double factor) {
  const double floor = scaled_linear_floor(0.24);
  return clamp_value(runtime_linear_speed_limit * factor, floor, runtime_linear_speed_limit);
}


static void apply_motion_profile() {
  ControllerMotionProfile profile = {
      configured_cruise_speed_mps,
      configured_payload_kg,
      configured_battery_range_units,
  };
  ControllerMotionLimits limits = {0};
  controller_motion_profile_apply(&profile, &limits);

  configured_cruise_speed_mps = profile.cruise_speed_mps;
  configured_payload_kg = profile.payload_kg;
  configured_battery_range_units = profile.battery_range_units;
  runtime_linear_speed_limit = limits.linear_speed_mps;
  runtime_angular_speed_limit = limits.angular_speed_rad_s;
  runtime_battery_speed_factor = limits.battery_speed_factor;
}

static int load_motion_profile() {
  ControllerMotionProfile profile = {
      configured_cruise_speed_mps,
      configured_payload_kg,
      configured_battery_range_units,
  };
  controller_motion_profile_load_file(MOTION_PROFILE_PATH, &profile);

  configured_cruise_speed_mps = profile.cruise_speed_mps;
  configured_payload_kg = profile.payload_kg;
  configured_battery_range_units = profile.battery_range_units;
  apply_motion_profile();
  return 1;
}

static void maybe_reload_motion_profile(void) {
  if ((step_counter % MOTION_RELOAD_INTERVAL) != 0) return;

  const double previous_cruise_speed = configured_cruise_speed_mps;
  const double previous_payload_kg = configured_payload_kg;
  const double previous_battery_range = configured_battery_range_units;
  const double previous_linear_limit = runtime_linear_speed_limit;
  const double previous_angular_limit = runtime_angular_speed_limit;
  const long long mtime = get_file_mtime(MOTION_PROFILE_PATH);
  if (mtime < 0) return;

  if (load_motion_profile()) {
    const int profile_changed =
        mtime != motion_profile_last_modified ||
        fabs(configured_cruise_speed_mps - previous_cruise_speed) > 1e-6 ||
        fabs(configured_payload_kg - previous_payload_kg) > 1e-6 ||
        fabs(configured_battery_range_units - previous_battery_range) > 1e-6 ||
        fabs(runtime_linear_speed_limit - previous_linear_limit) > 1e-6 ||
        fabs(runtime_angular_speed_limit - previous_angular_limit) > 1e-6;
    motion_profile_last_modified = mtime;
    if (profile_changed) {
      set_status("motion_profile_reloaded");
    }
  }
}


static void set_error(const char *message) {
  strncpy(navigation_error, message, sizeof(navigation_error) - 1);
  navigation_error[sizeof(navigation_error) - 1] = '\0';
}

static void clear_error() {
  navigation_error[0] = '\0';
}

static void set_status(const char *status) {
  strncpy(navigation_status, status, sizeof(navigation_status) - 1);
  navigation_status[sizeof(navigation_status) - 1] = '\0';
}

static void init_wheels() {
  char name[16];
  for (int i = 0; i < 4; ++i) {
    sprintf(name, "wheel%d", i + 1);
    wheels[i] = wb_robot_get_device(name);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
    applied_wheel_speeds[i] = 0.0;
  }
}

static void init_manipulator_pose() {
  static const char *arm_names[5] = {
      "arm1",
      "arm2",
      "arm3",
      "arm4",
      "arm5",
  };
  static const double arm_positions[5] = {
      0.0,
      1.57,
      -2.635,
      1.78,
      0.0,
  };
  static const char *finger_names[2] = {
      "finger::left",
      "finger::right",
  };
  const double finger_opening = 0.011;

  for (int i = 0; i < 5; ++i) {
    arm_joints[i] = wb_robot_get_device(arm_names[i]);
    if (arm_joints[i]) {
      wb_motor_set_velocity(arm_joints[i], 1.0);
      wb_motor_set_position(arm_joints[i], arm_positions[i]);
    }
  }

  for (int i = 0; i < 2; ++i) {
    gripper_fingers[i] = wb_robot_get_device(finger_names[i]);
    if (gripper_fingers[i]) {
      wb_motor_set_velocity(gripper_fingers[i], 0.03);
      wb_motor_set_position(gripper_fingers[i], finger_opening);
    }
  }
}

static void init_lidar() {
  front_lidar = wb_robot_get_device("front_lidar");
  if (!front_lidar) {
    lidar_available = 0;
    return;
  }

  wb_lidar_enable(front_lidar, TIME_STEP);
  lidar_available = 1;
  lidar_resolution = wb_lidar_get_horizontal_resolution(front_lidar);
  lidar_fov = wb_lidar_get_fov(front_lidar);
  lidar_max_range = wb_lidar_get_max_range(front_lidar);
}

static void init_camera() {
  front_camera = wb_robot_get_device("front_camera");
  if (!front_camera) {
    camera_available = 1;
    camera_virtual_mode = 1;
    camera_width = CAMERA_FRAME_WIDTH;
    camera_height = CAMERA_FRAME_HEIGHT;
    camera_fov = lidar_fov > EPS ? fmin(lidar_fov, 1.20) : 1.05;
    return;
  }

  camera_virtual_mode = 0;
  wb_camera_enable(front_camera, TIME_STEP * CAMERA_CAPTURE_INTERVAL);
  camera_available = 1;
  camera_width = wb_camera_get_width(front_camera);
  camera_height = wb_camera_get_height(front_camera);
  camera_fov = wb_camera_get_fov(front_camera);
}

static double estimate_camera_range_from_lidar(double relative_angle, double fallback_range) {
  if (!lidar_available || !front_lidar || lidar_resolution <= 1 || lidar_fov <= EPS) {
    return fallback_range;
  }

  const float *ranges = wb_lidar_get_range_image(front_lidar);
  if (!ranges) return fallback_range;

  double best_range = LIDAR_MAX_TRACE_RANGE;
  double best_angle_error = CAMERA_RANGE_SEARCH_WINDOW_RAD;
  for (int i = 0; i < lidar_resolution; ++i) {
    const double alpha = (double)i / (double)(lidar_resolution - 1);
    const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
    const double angle_error = fabs(beam_angle - relative_angle);
    const double range = ranges[i];
    if (angle_error > CAMERA_RANGE_SEARCH_WINDOW_RAD ||
        !is_finite_double(range) ||
        range <= LIDAR_MIN_TRACE_RANGE ||
        range >= LIDAR_MAX_TRACE_RANGE - 0.03) {
      continue;
    }

    if (range < best_range || angle_error < best_angle_error * 0.55) {
      best_range = range;
      best_angle_error = angle_error;
    }
  }

  if (best_range < LIDAR_MAX_TRACE_RANGE) return best_range;
  return fallback_range;
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

typedef struct {
  const unsigned char *image;
  int width;
} WebotsCameraPixelContext;

static ControllerCameraPixel read_webots_camera_pixel(void *context, int x, int y) {
  const WebotsCameraPixelContext *pixel_context = (const WebotsCameraPixelContext *)context;
  return (ControllerCameraPixel){
      wb_camera_image_get_red(pixel_context->image, pixel_context->width, x, y),
      wb_camera_image_get_green(pixel_context->image, pixel_context->width, x, y),
      wb_camera_image_get_blue(pixel_context->image, pixel_context->width, x, y),
  };
}

static void update_camera_obstacle_hint(void) {
  camera_obstacle_update_step = step_counter;
  camera_obstacle_visible = 0;
  camera_obstacle_score = 0.0;
  camera_obstacle_center_offset = 0.0;
  camera_obstacle_angle = 0.0;
  camera_obstacle_range = 0.0;
  camera_detection_count = 0;

  if (!camera_available || !front_camera || camera_width <= 0 || camera_height <= 0) return;

  const unsigned char *image = wb_camera_get_image(front_camera);
  if (!image) return;

  const double effective_fov = camera_fov > EPS ? camera_fov : 1.05;
  merge_camera_visible_frustum_into_map(effective_fov, CAMERA_RANGE_FALLBACK_M);

  ControllerCameraAnalysisConfig analysis_config =
      controller_camera_default_config(camera_width, camera_height);
  analysis_config.min_obstacle_score = CAMERA_OBSTACLE_MIN_SCORE;
  const WebotsCameraPixelContext pixel_context = {image, camera_width};
  ControllerCameraObservation observation;
  controller_camera_analyze(
      &analysis_config, read_webots_camera_pixel, (void *)&pixel_context, &observation);

  camera_obstacle_score = observation.score;
  if (observation.visible) {
    camera_obstacle_visible = 1;
    camera_obstacle_center_offset = observation.center_offset;
    camera_obstacle_angle = camera_obstacle_center_offset * fmax(effective_fov, 0.8) * 0.5;
    camera_obstacle_range =
        estimate_camera_range_from_lidar(camera_obstacle_angle, observation.fallback_range_m);
    camera_detection_count = observation.hits;
    merge_camera_free_ray_into_map(camera_obstacle_angle, camera_obstacle_range, 2);
    merge_camera_observation_into_map(
        camera_obstacle_angle,
        camera_obstacle_range,
        2 + (int)(camera_obstacle_score * 14.0) +
            (observation.max_hit_x - observation.min_hit_x > 18 ? 2 : 0));
  }
}

static void draw_virtual_camera_overlay(unsigned char *pixels, double effective_fov) {
  const int horizon = (int)(CAMERA_FRAME_HEIGHT * 0.42);
  const int center_x = CAMERA_FRAME_WIDTH / 2;
  controller_camera_render_line(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                center_x - 14, horizon, center_x + 14, horizon, 80, 220, 230);
  controller_camera_render_line(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                center_x, horizon - 10, center_x, horizon + 10, 80, 220, 230);

  if (route_data.count > 0 && current_waypoint_index < route_data.count && translation_field && rotation_field) {
    double x = 0.0;
    double z = 0.0;
    double heading = 0.0;
    read_pose(&x, &z, &heading);
    const Waypoint *target = &route_data.waypoints[current_waypoint_index];
    const double target_angle = wrap_angle(atan2(target->z - z, target->x - x) - heading);
    if (fabs(target_angle) < effective_fov * 0.5) {
      const double offset = clamp_value(target_angle / (effective_fov * 0.5), -1.0, 1.0);
      const int target_x = (int)((offset * 0.5 + 0.5) * (CAMERA_FRAME_WIDTH - 1));
      controller_camera_render_line(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                    target_x, horizon - 18, target_x, CAMERA_FRAME_HEIGHT - 8,
                                    55, 222, 170);
      controller_camera_render_rect(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                    target_x - 3, horizon - 21, target_x + 3, horizon - 15,
                                    55, 222, 170);
    }
  }
}

static int write_virtual_camera_frame() {
  static unsigned char pixels[CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT * 3];
  typedef struct {
    double angle;
    double range;
    int beams;
  } VirtualCameraCluster;
  VirtualCameraCluster clusters[CAMERA_MAX_VIRTUAL_CLUSTERS];
  int cluster_count = 0;
  controller_camera_render_background(pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);

  camera_obstacle_visible = 0;
  camera_obstacle_score = 0.0;
  camera_obstacle_center_offset = 0.0;

  const int horizon = (int)(CAMERA_FRAME_HEIGHT * 0.42);
  const double effective_fov = camera_fov > EPS ? camera_fov : 1.05;
  int total_beams = 0;
  int close_beams = 0;
  double weighted_offset_sum = 0.0;
  double weight_sum = 0.0;

  if (lidar_available && front_lidar && lidar_resolution > 1 && lidar_fov > EPS) {
    const float *ranges = wb_lidar_get_range_image(front_lidar);
    int in_cluster = 0;
    int cluster_beams = 0;
    double cluster_min_range = LIDAR_MAX_TRACE_RANGE;
    double cluster_angle_sum = 0.0;
    double cluster_weight_sum = 0.0;
    double previous_range = 0.0;

    for (int i = 0; ranges && i < lidar_resolution; ++i) {
      const double alpha = (double)i / (double)(lidar_resolution - 1);
      const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
      const int in_view = fabs(beam_angle) <= effective_fov * 0.5;

      const double range = ranges[i];
      const int valid = in_view && is_finite_double(range) && range > LIDAR_MIN_TRACE_RANGE;
      const int obstacle_hit = valid && range < (LIDAR_MAX_TRACE_RANGE - 0.04);

      if (valid) total_beams += 1;
      if (valid && (i % 10) == 0) {
        merge_camera_free_ray_into_map(
            beam_angle,
            clamp_value(range, CAMERA_FREE_RAY_MIN_RANGE_M, LIDAR_MAX_TRACE_RANGE),
            1);
      }
      if (valid && range < LIDAR_TRACK_CAUTION_RANGE) {
        const double screen_offset = clamp_value(beam_angle / (effective_fov * 0.5), -1.0, 1.0);
        close_beams += 1;
        const double closeness = clamp_value((LIDAR_TRACK_CAUTION_RANGE - range) /
                                                fmax(LIDAR_TRACK_CAUTION_RANGE - LIDAR_AVOID_STOP_RANGE, 0.05),
                                            0.0,
                                            1.0);
        weighted_offset_sum += screen_offset * (0.25 + closeness);
        weight_sum += 0.25 + closeness;
      }

      const int split_cluster =
          !obstacle_hit ||
          (in_cluster && fabs(range - previous_range) > 0.42);
      if (split_cluster && in_cluster) {
        if (cluster_count < CAMERA_MAX_VIRTUAL_CLUSTERS && cluster_beams >= 2) {
          clusters[cluster_count].angle = cluster_angle_sum / fmax(cluster_weight_sum, EPS);
          clusters[cluster_count].range = cluster_min_range;
          clusters[cluster_count].beams = cluster_beams;
          cluster_count += 1;
        }
        in_cluster = 0;
        cluster_beams = 0;
        cluster_min_range = LIDAR_MAX_TRACE_RANGE;
        cluster_angle_sum = 0.0;
        cluster_weight_sum = 0.0;
      }

      if (obstacle_hit) {
        const double weight = 1.0 / fmax(range, 0.16);
        in_cluster = 1;
        cluster_beams += 1;
        if (range < cluster_min_range) cluster_min_range = range;
        cluster_angle_sum += beam_angle * weight;
        cluster_weight_sum += weight;
        previous_range = range;
      }
    }

    if (in_cluster && cluster_count < CAMERA_MAX_VIRTUAL_CLUSTERS && cluster_beams >= 2) {
      clusters[cluster_count].angle = cluster_angle_sum / fmax(cluster_weight_sum, EPS);
      clusters[cluster_count].range = cluster_min_range;
      clusters[cluster_count].beams = cluster_beams;
      cluster_count += 1;
    }
  }

  for (int i = 0; i < cluster_count; ++i) {
    for (int j = i + 1; j < cluster_count; ++j) {
      if (clusters[i].range < clusters[j].range) {
        const VirtualCameraCluster tmp = clusters[i];
        clusters[i] = clusters[j];
        clusters[j] = tmp;
      }
    }
  }

  for (int i = 0; i < cluster_count; ++i) {
    const double range = clamp_value(clusters[i].range, 0.12, LIDAR_MAX_TRACE_RANGE);
    const double screen_offset = clamp_value(clusters[i].angle / (effective_fov * 0.5), -1.0, 1.0);
    const int screen_x = (int)((screen_offset * 0.5 + 0.5) * (CAMERA_FRAME_WIDTH - 1));
    const double depth01 = clamp_value((range - 0.15) / fmax(LIDAR_MAX_TRACE_RANGE - 0.15, 0.1), 0.0, 1.0);
    const int bottom = horizon + (int)((CAMERA_FRAME_HEIGHT - horizon - 4) * (1.0 - depth01 * 0.82));
    const int height = (int)clamp_value(82.0 / (range + 0.34), 16, 112);
    const int width = (int)clamp_value(clusters[i].beams * 2.8 + 36.0 / (range + 0.32), 14, 96);
    const double danger = clamp_value((LIDAR_TRACK_CAUTION_RANGE - range) /
                                          fmax(LIDAR_TRACK_CAUTION_RANGE - LIDAR_AVOID_STOP_RANGE, 0.05),
                                      0.0,
                                      1.0);
    controller_camera_render_box(
        pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
        screen_x, bottom, width, height, danger);
    merge_camera_observation_into_map(
        clusters[i].angle,
        range,
        1 + (int)clamp_value((double)clusters[i].beams / 2.0, 1.0, 8.0));
  }

  if (total_beams > 0 && close_beams > 0 && weight_sum > EPS) {
    camera_obstacle_score = (double)close_beams / (double)total_beams;
    if (camera_obstacle_score >= CAMERA_OBSTACLE_MIN_SCORE) {
      camera_obstacle_visible = 1;
      camera_obstacle_center_offset = clamp_value(weighted_offset_sum / weight_sum, -1.0, 1.0);
      camera_obstacle_angle = camera_obstacle_center_offset * effective_fov * 0.5;
      camera_obstacle_range =
          estimate_camera_range_from_lidar(camera_obstacle_angle, CAMERA_RANGE_FALLBACK_M);
      camera_detection_count = close_beams;
    }
  }

  draw_virtual_camera_overlay(pixels, effective_fov);
  return write_bmp24(CAMERA_FRAME_BMP_TEMP_PATH, pixels, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);
}

static void maybe_write_camera_frame(void) {
  if (!camera_available) return;
  if ((step_counter % CAMERA_WRITE_INTERVAL) != 0) return;

  if (camera_virtual_mode) {
    if (write_virtual_camera_frame() == 0) {
      if (replace_file(CAMERA_FRAME_BMP_TEMP_PATH, CAMERA_FRAME_BMP_PATH) == 0) {
        camera_frame_file = "camera_frame.bmp";
        camera_frame_mime = "image/bmp";
        camera_frame_sequence += 1;
        camera_frame_time = wb_robot_get_time();
      }
    }
    return;
  }

  if (!front_camera) return;
  if (camera_obstacle_update_step != step_counter) {
    update_camera_obstacle_hint();
  }

  if (wb_camera_save_image(front_camera, CAMERA_FRAME_JPEG_TEMP_PATH, 70) == 0) {
    if (replace_file(CAMERA_FRAME_JPEG_TEMP_PATH, CAMERA_FRAME_JPEG_PATH) == 0) {
      camera_frame_file = "camera_frame.jpg";
      camera_frame_mime = "image/jpeg";
      camera_frame_sequence += 1;
      camera_frame_time = wb_robot_get_time();
    }
  }
}

static void maybe_update_camera_perception(void) {
  if (!camera_available || camera_virtual_mode || !front_camera) return;
  if ((step_counter % CAMERA_CAPTURE_INTERVAL) != 0) return;
  update_camera_obstacle_hint();
}

static void set_base_velocity(double vx, double vy, double omega) {
  const ControllerDriveConfig config = {
      WHEEL_RADIUS,
      WHEEL_BASE_LONGITUDINAL + WHEEL_BASE_LATERAL,
      MAX_WHEEL_SPEED_RAD_S,
      WHEEL_ACCEL_LIMIT_RAD_S2,
      WHEEL_DECEL_LIMIT_RAD_S2,
      TIME_STEP / 1000.0,
  };
  int enabled[4];
  for (int i = 0; i < 4; ++i) {
    enabled[i] = wheels[i] != 0;
  }
  controller_drive_step(&config, vx, vy, omega, enabled, applied_wheel_speeds);
  for (int i = 0; i < 4; ++i) {
    if (wheels[i]) wb_motor_set_velocity(wheels[i], applied_wheel_speeds[i]);
  }
}

static void stop_robot() {
  set_base_velocity(0.0, 0.0, 0.0);
}

static int is_finite_double(double value) {
#ifdef _WIN32
  return _finite(value) != 0;
#else
  return isfinite(value);
#endif
}

static void init_pose_tracking() {
  self_node = wb_supervisor_node_get_self();
  translation_field = wb_supervisor_node_get_field(self_node, "translation");
  rotation_field = wb_supervisor_node_get_field(self_node, "rotation");

  WbNodeRef root_node = wb_supervisor_node_get_root();
  if (root_node) {
    root_children_field = wb_supervisor_node_get_field(root_node, "children");
  }
}

static void reset_robot_pose() {
  if (!self_node || !translation_field || !rotation_field) return;

  const double translation[3] = {START_X, START_Z, START_HEIGHT};
  const double rotation[4] = {0.0, 0.0, 1.0, 0.0};

  wb_supervisor_field_set_sf_vec3f(translation_field, translation);
  wb_supervisor_field_set_sf_rotation(rotation_field, rotation);
  wb_supervisor_node_reset_physics(self_node);
  for (int i = 0; i < 4; ++i) {
    applied_wheel_speeds[i] = 0.0;
    if (wheels[i]) wb_motor_set_velocity(wheels[i], 0.0);
  }
  avoidance_state.hold_steps = 0;
  avoidance_state.turn_sign = 1.0;
  last_pose_x = START_X;
  last_pose_z = START_Z;
  last_pose_heading = 0.0;
  last_pose_valid = 1;
  reset_navigation_mode();
  stop_robot();
}

static void read_pose(double *x, double *z, double *heading) {
  const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double *rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
  *x = translation[0];
  *z = translation[1];
  *heading = rotation[3] * (rotation[2] >= 0.0 ? 1.0 : -1.0);
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
      -runtime_linear_speed_limit,
      runtime_linear_speed_limit);
  const double limited_angular = clamp_value(
      angular_speed,
      -runtime_angular_speed_limit,
      runtime_angular_speed_limit);
  set_base_velocity(limited_linear, 0.0, limited_angular);
}

static double sign_or_one(double value) {
  return value < 0.0 ? -1.0 : 1.0;
}

static void reset_navigation_mode() {
  navigation_mode = NAV_MODE_IDLE;
  navigation_waypoint_index = -1;
  navigation_segment_start_x = START_X;
  navigation_segment_start_z = START_Z;
  lidar_priority_turn_sign = 0.0;
  lidar_priority_hold_steps = 0;
  controller_avoidance_state_reset(&avoidance_state, START_X, START_Z);
  mapping_survey_replan_cooldown_steps = 0;
}

static void begin_navigation_for_waypoint(int waypoint_index, double current_x, double current_z) {
  navigation_waypoint_index = waypoint_index;
  navigation_segment_start_x = current_x;
  navigation_segment_start_z = current_z;
  navigation_mode = NAV_MODE_TURN;
}

static void ensure_navigation_waypoint_initialized(double current_x, double current_z) {
  if (navigation_waypoint_index != current_waypoint_index) {
    begin_navigation_for_waypoint(current_waypoint_index, current_x, current_z);
  }
}

static void clear_local_navigation_state() {
  lidar_priority_turn_sign = 0.0;
  lidar_priority_hold_steps = 0;
  const double previous_x = avoidance_state.prev_x;
  const double previous_z = avoidance_state.prev_z;
  controller_avoidance_state_reset(&avoidance_state, START_X, START_Z);
  avoidance_state.prev_x = previous_x;
  avoidance_state.prev_z = previous_z;
}

static void reset_route_avoidance_metrics() {
  route_avoidance_time_sec = 0.0;
  route_avoidance_steps = 0;
}

static int route_off_route_active_now() {
  if (route_finished || route_data.count <= 0) return 0;
  if (avoidance_state.active) return 1;
  if (strncmp(navigation_status, "avoiding_", 9) == 0) return 1;
  if (strcmp(navigation_status, "passing_lidar_gap") == 0) return 1;
  if (strcmp(navigation_status, "tracking_lidar_priority") == 0) return 1;
  if (strcmp(navigation_status, "turning_lidar_priority") == 0) return 1;
  if (strcmp(navigation_status, "reacquired_free_space") == 0) return 1;
  return 0;
}

static void update_route_avoidance_metrics(void) {
  if (route_off_route_active_now()) {
    route_avoidance_steps += 1;
    route_avoidance_time_sec += (double)TIME_STEP / 1000.0;
  }
}

static void compute_lidar_obstacle_context(LidarObstacleContext *context,
                                           double target_beam_angle,
                                           double preferred_turn_sign) {
  if (!context) return;

  const double effective_max_range =
      lidar_max_range > EPS ? fmin(lidar_max_range, LIDAR_MAX_TRACE_RANGE) : LIDAR_MAX_TRACE_RANGE;
  controller_lidar_context_init(context, effective_max_range);

  if (!lidar_available || !front_lidar || lidar_resolution <= 1 || lidar_fov <= EPS) return;

  const float *ranges = wb_lidar_get_range_image(front_lidar);
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
    const int range_is_finite = is_finite_double(raw_range);
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

      for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
        if (point_near_zone_boundary(hit_x,
                                     hit_y,
                                     &zone_data.zones[zone_index],
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

static void prune_obstacle_trace(double now_time) {
  if (obstacle_trace_count <= 0) return;

  int write_index = 0;
  for (int read_index = 0; read_index < obstacle_trace_count; ++read_index) {
    const ObstacleTracePoint point = obstacle_trace[read_index];
    if (now_time - point.last_seen_time > LIDAR_TRACE_TTL_SECONDS) continue;
    obstacle_trace[write_index] = point;
    write_index += 1;
  }

  obstacle_trace_count = write_index;
}

static void append_obstacle_trace_point(double x, double y, double now_time) {
  if (!is_finite_double(x) || !is_finite_double(y)) return;

  int nearest_index = -1;
  double nearest_distance = 1e9;

  for (int i = 0; i < obstacle_trace_count; ++i) {
    const double dx = obstacle_trace[i].x - x;
    const double dy = obstacle_trace[i].y - y;
    const double distance = hypot2(dx, dy);
    if (distance <= LIDAR_TRACE_SPACING && distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  if (nearest_index >= 0) {
    ObstacleTracePoint *point = &obstacle_trace[nearest_index];
    point->x = point->x * 0.68 + x * 0.32;
    point->y = point->y * 0.68 + y * 0.32;
    point->last_seen_time = now_time;
    if (point->hit_count < 255) point->hit_count += 1;
    return;
  }

  if (obstacle_trace_count >= MAX_OBSTACLE_TRACE_POINTS) {
    int oldest_index = 0;
    double oldest_time = obstacle_trace[0].last_seen_time;
    for (int i = 1; i < obstacle_trace_count; ++i) {
      if (obstacle_trace[i].last_seen_time < oldest_time) {
        oldest_time = obstacle_trace[i].last_seen_time;
        oldest_index = i;
      }
    }
    obstacle_trace[oldest_index].x = x;
    obstacle_trace[oldest_index].y = y;
    obstacle_trace[oldest_index].last_seen_time = now_time;
    obstacle_trace[oldest_index].hit_count = 1;
    return;
  }

  obstacle_trace[obstacle_trace_count].x = x;
  obstacle_trace[obstacle_trace_count].y = y;
  obstacle_trace[obstacle_trace_count].last_seen_time = now_time;
  obstacle_trace[obstacle_trace_count].hit_count = 1;
  obstacle_trace_count += 1;
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
  if (!lidar_available || !front_lidar || lidar_resolution <= 1 || lidar_fov <= EPS) return;

  const float *ranges = wb_lidar_get_range_image(front_lidar);
  if (!ranges) return;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);
  const double now_time = wb_robot_get_time();
  prune_obstacle_trace(now_time);

  const double sensor_origin_x =
      robot_x + cos(heading) * LIDAR_LOCAL_X - sin(heading) * LIDAR_LOCAL_Y;
  const double sensor_origin_y =
      robot_y + sin(heading) * LIDAR_LOCAL_X + cos(heading) * LIDAR_LOCAL_Y;
  const double effective_max_range =
      lidar_max_range > EPS ? fmin(lidar_max_range, LIDAR_MAX_TRACE_RANGE) : LIDAR_MAX_TRACE_RANGE;
  lidar_front_min_range = effective_max_range;
  lidar_center_min_range = effective_max_range;
  lidar_left_front_min_range = effective_max_range;
  lidar_right_front_min_range = effective_max_range;
  lidar_left_min_range = effective_max_range;
  lidar_right_min_range = effective_max_range;

  for (int i = 0; i < lidar_resolution; i += LIDAR_SAMPLE_STRIDE) {
    const double range = (double)ranges[i];
    if (!is_finite_double(range)) continue;
    if (range < LIDAR_MIN_TRACE_RANGE || range > effective_max_range) continue;
    if (range >= effective_max_range - 0.02) continue;
    if (!controller_lidar_hit_is_consistent(
            ranges,
            lidar_resolution,
            i,
            range,
            effective_max_range,
            LIDAR_SAMPLE_STRIDE,
            LIDAR_MIN_TRACE_RANGE,
            LIDAR_RANGE_JUMP_TOLERANCE)) {
      continue;
    }

    const double alpha =
        lidar_resolution > 1 ? (double)i / (double)(lidar_resolution - 1) : 0.5;
    const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
    if (fabs(beam_angle) <= LIDAR_FRONT_SECTOR_RAD) {
      lidar_front_hit_count += 1;
      if (range < lidar_front_min_range) lidar_front_min_range = range;
      if (fabs(beam_angle) <= LIDAR_CENTER_SECTOR_RAD && range < lidar_center_min_range) {
        lidar_center_min_range = range;
      }
      if (beam_angle <= -LIDAR_FRONT_CORNER_MIN_RAD &&
          beam_angle >= -LIDAR_FRONT_CORNER_MAX_RAD &&
          range < lidar_left_front_min_range) {
        lidar_left_front_min_range = range;
      }
      if (beam_angle >= LIDAR_FRONT_CORNER_MIN_RAD &&
          beam_angle <= LIDAR_FRONT_CORNER_MAX_RAD &&
          range < lidar_right_front_min_range) {
        lidar_right_front_min_range = range;
      }
    } else if (beam_angle < 0.0) {
      if (range < lidar_left_min_range) lidar_left_min_range = range;
    } else {
      if (range < lidar_right_min_range) lidar_right_min_range = range;
    }
    const double world_angle = heading - beam_angle;
    const double hit_x = sensor_origin_x + cos(world_angle) * range;
    const double hit_y = sensor_origin_y + sin(world_angle) * range;
    if (hypot2(hit_x - robot_x, hit_y - robot_y) < LIDAR_NEAR_ROBOT_IGNORE_RADIUS) continue;
    const double snapped_hit_x = round(hit_x / LIDAR_SNAP_STEP) * LIDAR_SNAP_STEP;
    const double snapped_hit_y = round(hit_y / LIDAR_SNAP_STEP) * LIDAR_SNAP_STEP;
    append_obstacle_trace_point(snapped_hit_x, snapped_hit_y, now_time);
    lidar_last_hit_count += 1;
  }
}

static void merge_trace_into_map(double now_time) {
  for (int i = 0; i < obstacle_trace_count; ++i) {
    const ObstacleTracePoint *point = &obstacle_trace[i];
    if (now_time - point->last_seen_time > MAP_MERGE_MAX_AGE_S) continue;
    if (point->hit_count < MAP_MERGE_MIN_HIT_COUNT) continue;

    const double cell_x = round(point->x / MAP_CELL_SIZE) * MAP_CELL_SIZE;
    const double cell_y = round(point->y / MAP_CELL_SIZE) * MAP_CELL_SIZE;
    const double half_cell = MAP_CELL_SIZE * 0.5 + EPS;
    int found_index = -1;

    for (int j = 0; j < persistent_map_count; ++j) {
      if (fabs(persistent_map[j].x - cell_x) < half_cell &&
          fabs(persistent_map[j].y - cell_y) < half_cell) {
        found_index = j;
        break;
      }
    }

    if (found_index >= 0) {
      if (persistent_map[found_index].confidence < 255) {
        persistent_map[found_index].confidence += 1;
        map_dirty = 1;
      }
      continue;
    }

    if (persistent_map_count >= MAX_MAP_POINTS) continue;
    persistent_map[persistent_map_count++] = (MapCell){cell_x, cell_y, 1};
    map_dirty = 1;
  }
}

static void append_camera_map_cell(double x, double y, int confidence_boost) {
  if (controller_camera_map_append_obstacle(
          camera_map,
          &camera_map_count,
          MAX_CAMERA_MAP_POINTS,
          camera_free_map,
          &camera_free_map_count,
          CAMERA_MAP_CELL_SIZE,
          EPS,
          x,
          y,
          confidence_boost)) {
    camera_map_dirty = 1;
  }
}

static void append_camera_free_map_cell(double x, double y, int confidence_boost) {
  if (controller_camera_map_append_free(
          camera_map,
          camera_map_count,
          camera_free_map,
          &camera_free_map_count,
          MAX_CAMERA_FREE_MAP_POINTS,
          CAMERA_MAP_CELL_SIZE,
          EPS,
          x,
          y,
          confidence_boost)) {
    camera_map_dirty = 1;
  }
}

static void merge_camera_free_ray_into_map(double relative_angle, double range, int confidence_boost) {
  if (!is_finite_double(relative_angle) || !is_finite_double(range)) return;
  if (range < CAMERA_FREE_RAY_MIN_RANGE_M) return;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  const ControllerCameraPose pose = {robot_x, robot_y, heading};
  const ControllerCameraMapGeometryConfig geometry_config = {
      LIDAR_LOCAL_X,
      LIDAR_LOCAL_Y,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE,
      CAMERA_FREE_RAY_MIN_RANGE_M,
      CAMERA_FREE_RAY_MARGIN_M,
      CAMERA_FREE_RAY_STEP_M,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
  };
  ControllerCameraMapPoint points[32];
  const int point_count = controller_camera_free_ray_points(
      &geometry_config, &pose, relative_angle, range, points, 32);
  for (int i = 0; i < point_count; ++i) {
    append_camera_free_map_cell(points[i].x, points[i].y, confidence_boost);
  }
}

static void merge_camera_observation_into_map(double relative_angle, double range, int confidence_boost) {
  if (!is_finite_double(relative_angle) || !is_finite_double(range)) return;
  if (range < LIDAR_MIN_TRACE_RANGE || range > LIDAR_MAX_TRACE_RANGE) return;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  const ControllerCameraPose pose = {robot_x, robot_y, heading};
  const ControllerCameraMapGeometryConfig geometry_config = {
      LIDAR_LOCAL_X,
      LIDAR_LOCAL_Y,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE,
      CAMERA_FREE_RAY_MIN_RANGE_M,
      CAMERA_FREE_RAY_MARGIN_M,
      CAMERA_FREE_RAY_STEP_M,
      LIDAR_NEAR_ROBOT_IGNORE_RADIUS,
  };
  ControllerCameraMapPoint point;
  if (!controller_camera_obstacle_point(
          &geometry_config, &pose, relative_angle, range, &point)) {
    return;
  }
  append_camera_map_cell(point.x, point.y, confidence_boost);
}

static void clear_camera_map() {
  camera_map_count = 0;
  camera_free_map_count = 0;
  camera_map_dirty = 0;
  remove(CAMERA_MAP_PATH);
  remove(CAMERA_MAP_TEMP_PATH);
  remove(CAMERA_MAP_CSV_PATH);
  remove(CAMERA_MAP_CSV_TEMP_PATH);
}

static void clear_persistent_map() {
  persistent_map_count = 0;
  map_dirty = 0;
  remove(MAP_PATH);
  remove(MAP_TEMP_PATH);
  remove(MAP_CSV_PATH);
  remove(MAP_CSV_TEMP_PATH);
  clear_camera_map();
}

static void generate_survey_route(const char *path) {
  FILE *file = fopen(path, "w");
  if (!file) return;

  fprintf(file, "%.3f,%.3f\n", SURVEY_X_MIN, SURVEY_Y_MIN);
  int forward = 1;
  for (double y = SURVEY_Y_MIN; y <= SURVEY_Y_MAX + 0.01; y += SURVEY_STRIP) {
    if (forward) {
      fprintf(file, "%.3f,%.3f\n", SURVEY_X_MIN, y);
      fprintf(file, "%.3f,%.3f\n", SURVEY_X_MAX, y);
    } else {
      fprintf(file, "%.3f,%.3f\n", SURVEY_X_MAX, y);
      fprintf(file, "%.3f,%.3f\n", SURVEY_X_MIN, y);
    }
    forward = !forward;
  }

  fclose(file);
}

static void maybe_write_map(void) {
  if (!map_dirty) return;
  if ((step_counter % MAP_WRITE_INTERVAL) != 0) return;

  int json_written = 0;
  int csv_written = 0;

  FILE *json_file = fopen(MAP_TEMP_PATH, "w");
  if (json_file) {
    fprintf(json_file, "{\n");
    fprintf(json_file, "  \"cellSize\": %.4f,\n", MAP_CELL_SIZE);
    fprintf(json_file, "  \"totalCells\": %d,\n", persistent_map_count);
    fprintf(json_file, "  \"cells\": [\n");
    for (int i = 0; i < persistent_map_count; ++i) {
      fprintf(
          json_file,
          "    {\"x\": %.4f, \"y\": %.4f, \"confidence\": %d}%s\n",
          persistent_map[i].x,
          persistent_map[i].y,
          persistent_map[i].confidence,
          i + 1 < persistent_map_count ? "," : "");
    }
    fprintf(json_file, "  ]\n");
    fprintf(json_file, "}\n");
    fclose(json_file);
    json_written = replace_file(MAP_TEMP_PATH, MAP_PATH) == 0;
  }

  FILE *csv_file = fopen(MAP_CSV_TEMP_PATH, "w");
  if (csv_file) {
    fprintf(csv_file, "x,y,confidence\n");
    for (int i = 0; i < persistent_map_count; ++i) {
      fprintf(
          csv_file,
          "%.4f,%.4f,%d\n",
          persistent_map[i].x,
          persistent_map[i].y,
          persistent_map[i].confidence);
    }
    fclose(csv_file);
    csv_written = replace_file(MAP_CSV_TEMP_PATH, MAP_CSV_PATH) == 0;
  }

  if (json_written && csv_written) {
    map_dirty = 0;
  }
}

static void maybe_write_camera_map(void) {
  if (!camera_map_dirty) return;
  if ((step_counter % MAP_WRITE_INTERVAL) != 0) return;

  int json_written = 0;
  int csv_written = 0;

  FILE *json_file = fopen(CAMERA_MAP_TEMP_PATH, "w");
  if (json_file) {
    fprintf(json_file, "{\n");
    fprintf(json_file, "  \"cellSize\": %.4f,\n", CAMERA_MAP_CELL_SIZE);
    fprintf(json_file, "  \"cellCount\": %d,\n", camera_map_count + camera_free_map_count);
    fprintf(json_file, "  \"totalCells\": %d,\n", camera_map_count + camera_free_map_count);
    fprintf(json_file, "  \"obstacleCellCount\": %d,\n", camera_map_count);
    fprintf(json_file, "  \"freeCellCount\": %d,\n", camera_free_map_count);
    fprintf(json_file, "  \"source\": \"camera\",\n");
    fprintf(json_file, "  \"cells\": [\n");
    for (int i = 0; i < camera_map_count; ++i) {
      fprintf(
          json_file,
          "    {\"x\": %.4f, \"y\": %.4f, \"confidence\": %d}%s\n",
          camera_map[i].x,
          camera_map[i].y,
          camera_map[i].confidence,
          i + 1 < camera_map_count ? "," : "");
    }
    fprintf(json_file, "  ],\n");
    fprintf(json_file, "  \"freeCells\": [\n");
    for (int i = 0; i < camera_free_map_count; ++i) {
      fprintf(
          json_file,
          "    {\"x\": %.4f, \"y\": %.4f, \"confidence\": %d}%s\n",
          camera_free_map[i].x,
          camera_free_map[i].y,
          camera_free_map[i].confidence,
          i + 1 < camera_free_map_count ? "," : "");
    }
    fprintf(json_file, "  ]\n");
    fprintf(json_file, "}\n");
    fclose(json_file);
    json_written = replace_file(CAMERA_MAP_TEMP_PATH, CAMERA_MAP_PATH) == 0;
  }

  FILE *csv_file = fopen(CAMERA_MAP_CSV_TEMP_PATH, "w");
  if (csv_file) {
    fprintf(csv_file, "type,x,y,confidence\n");
    for (int i = 0; i < camera_map_count; ++i) {
      fprintf(
          csv_file,
          "obstacle,%.4f,%.4f,%d\n",
          camera_map[i].x,
          camera_map[i].y,
          camera_map[i].confidence);
    }
    for (int i = 0; i < camera_free_map_count; ++i) {
      fprintf(
          csv_file,
          "free,%.4f,%.4f,%d\n",
          camera_free_map[i].x,
          camera_free_map[i].y,
          camera_free_map[i].confidence);
    }
    fclose(csv_file);
    csv_written = replace_file(CAMERA_MAP_CSV_TEMP_PATH, CAMERA_MAP_CSV_PATH) == 0;
  }

  if (json_written && csv_written) {
    camera_map_dirty = 0;
  }
}

static double cross_product(double ax, double ay, double bx, double by, double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static int point_on_segment(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by) {
  const double cross = cross_product(ax, ay, bx, by, px, py);
  if (fabs(cross) > 1e-7) return 0;

  const double min_x = fmin(ax, bx) - 1e-7;
  const double max_x = fmax(ax, bx) + 1e-7;
  const double min_y = fmin(ay, by) - 1e-7;
  const double max_y = fmax(ay, by) + 1e-7;
  return px >= min_x && px <= max_x && py >= min_y && py <= max_y;
}

static int segments_intersect(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double dx,
    double dy) {
  const double o1 = cross_product(ax, ay, bx, by, cx, cy);
  const double o2 = cross_product(ax, ay, bx, by, dx, dy);
  const double o3 = cross_product(cx, cy, dx, dy, ax, ay);
  const double o4 = cross_product(cx, cy, dx, dy, bx, by);

  if (((o1 > EPS && o2 < -EPS) || (o1 < -EPS && o2 > EPS)) &&
      ((o3 > EPS && o4 < -EPS) || (o3 < -EPS && o4 > EPS))) {
    return 1;
  }

  if (fabs(o1) <= EPS && point_on_segment(cx, cy, ax, ay, bx, by)) return 1;
  if (fabs(o2) <= EPS && point_on_segment(dx, dy, ax, ay, bx, by)) return 1;
  if (fabs(o3) <= EPS && point_on_segment(ax, ay, cx, cy, dx, dy)) return 1;
  if (fabs(o4) <= EPS && point_on_segment(bx, by, cx, cy, dx, dy)) return 1;
  return 0;
}

static double distance_point_to_segment(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by) {
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab_length_squared = abx * abx + aby * aby;
  if (ab_length_squared <= EPS) {
    return hypot2(px - ax, py - ay);
  }

  double t = ((px - ax) * abx + (py - ay) * aby) / ab_length_squared;
  t = clamp_value(t, 0.0, 1.0);
  const double closest_x = ax + abx * t;
  const double closest_y = ay + aby * t;
  return hypot2(px - closest_x, py - closest_y);
}

static double distance_between_segments(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double dx,
    double dy) {
  if (segments_intersect(ax, ay, bx, by, cx, cy, dx, dy)) {
    return 0.0;
  }

  double min_distance = distance_point_to_segment(ax, ay, cx, cy, dx, dy);
  const double d2 = distance_point_to_segment(bx, by, cx, cy, dx, dy);
  const double d3 = distance_point_to_segment(cx, cy, ax, ay, bx, by);
  const double d4 = distance_point_to_segment(dx, dy, ax, ay, bx, by);

  if (d2 < min_distance) min_distance = d2;
  if (d3 < min_distance) min_distance = d3;
  if (d4 < min_distance) min_distance = d4;

  return min_distance;
}

static int point_in_zone(double x, double y, const LimitZone *zone) {
  if (!zone || zone->point_count < 3) return 0;
  int inside = 0;

  for (int i = 0, j = zone->point_count - 1; i < zone->point_count; j = i, i += 1) {
    const double ax = zone->points[i].x;
    const double ay = zone->points[i].y;
    const double bx = zone->points[j].x;
    const double by = zone->points[j].y;

    if (point_on_segment(x, y, ax, ay, bx, by)) return 1;

    const int intersects =
        (ay > y) != (by > y) &&
        x < ((bx - ax) * (y - ay)) / ((by - ay) + EPS) + ax;
    if (intersects) inside = !inside;
  }

  return inside;
}

static int point_near_zone(double x, double y, const LimitZone *zone, double clearance) {
  if (point_in_zone(x, y, zone)) return 1;

  for (int i = 0; i < zone->point_count; ++i) {
    const int next = (i + 1) % zone->point_count;
    const double distance = distance_point_to_segment(
        x,
        y,
        zone->points[i].x,
        zone->points[i].y,
        zone->points[next].x,
        zone->points[next].y);
    if (distance <= clearance + EPS) return 1;
  }

  return 0;
}

static int point_near_zone_boundary(double x, double y, const LimitZone *zone, double tolerance) {
  if (!zone || zone->point_count < 2) return 0;

  for (int i = 0; i < zone->point_count; ++i) {
    const int next = (i + 1) % zone->point_count;
    const double distance = distance_point_to_segment(
        x,
        y,
        zone->points[i].x,
        zone->points[i].y,
        zone->points[next].x,
        zone->points[next].y);
    if (distance <= tolerance + EPS) return 1;
  }

  return 0;
}

static int segment_blocked_by_zones(
    double ax,
    double ay,
    double bx,
    double by,
    double clearance,
    int skip_zone_index) {
  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (zone_index == skip_zone_index) continue;
    const LimitZone *zone = &zone_data.zones[zone_index];

    if (point_near_zone(ax, ay, zone, clearance) || point_near_zone(bx, by, zone, clearance)) {
      return 1;
    }

    for (int point_index = 0; point_index < zone->point_count; ++point_index) {
      const int next = (point_index + 1) % zone->point_count;
      const double edge_ax = zone->points[point_index].x;
      const double edge_ay = zone->points[point_index].y;
      const double edge_bx = zone->points[next].x;
      const double edge_by = zone->points[next].y;

      if (segments_intersect(ax, ay, bx, by, edge_ax, edge_ay, edge_bx, edge_by)) {
        return 1;
      }

      const double distance_to_edge =
          distance_between_segments(ax, ay, bx, by, edge_ax, edge_ay, edge_bx, edge_by);
      if (distance_to_edge <= clearance + EPS) {
        return 1;
      }
    }
  }

  return 0;
}

static double zone_signed_area(const LimitZone *zone) {
  if (!zone || zone->point_count < 3) return 0.0;
  double area = 0.0;
  for (int i = 0; i < zone->point_count; ++i) {
    const int next = (i + 1) % zone->point_count;
    area += zone->points[i].x * zone->points[next].y -
            zone->points[next].x * zone->points[i].y;
  }
  return area * 0.5;
}

static int find_room_zone_index(double robot_x, double robot_y) {
  int best_index = -1;
  double best_area = 0.0;
  for (int i = 0; i < zone_data.count; ++i) {
    const LimitZone *zone = &zone_data.zones[i];
    if (!point_in_zone(robot_x, robot_y, zone)) continue;
    const double area = fabs(zone_signed_area(zone));
    if (area > best_area) {
      best_area = area;
      best_index = i;
    }
  }
  return best_index;
}

#define survey_expand_bounds(x, y, min_x, max_x, min_y, max_y) \
  controller_survey_expand_bounds((x), (y), (min_x), (max_x), (min_y), (max_y))

static int survey_map_obstacle_near(double x, double y, double clearance) {
  const double clearance_sq = clearance * clearance;
  for (int i = 0; i < persistent_map_count; ++i) {
    const double dx = persistent_map[i].x - x;
    const double dy = persistent_map[i].y - y;
    if (dx * dx + dy * dy <= clearance_sq) return 1;
  }
  return 0;
}

static int survey_point_safe(double x, double y, int room_zone_index, double clearance) {
  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    if (!point_in_zone(x, y, room)) return 0;
    if (point_near_zone_boundary(x, y, room, clearance * 0.72)) return 0;
  } else if (x < -MAPPING_SURVEY_MAX_EXTENT_X || x > MAPPING_SURVEY_MAX_EXTENT_X ||
             y < -MAPPING_SURVEY_MAX_EXTENT_Y || y > MAPPING_SURVEY_MAX_EXTENT_Y) {
    return 0;
  }

  for (int i = 0; i < zone_data.count; ++i) {
    if (i == room_zone_index) continue;
    if (point_near_zone(x, y, &zone_data.zones[i], clearance)) return 0;
  }

  return !survey_map_obstacle_near(x, y, fmax(clearance, MAPPING_SURVEY_MAP_OBSTACLE_CLEARANCE));
}

static int survey_segment_safe(double ax, double ay, double bx, double by, int room_zone_index, double clearance) {
  const double length = hypot2(bx - ax, by - ay);
  const int steps = (int)ceil(length / fmax(MAPPING_SURVEY_GRID_CELL * 0.72, 0.05));
  for (int i = 0; i <= steps; ++i) {
    const double t = steps > 0 ? (double)i / (double)steps : 0.0;
    const double x = ax + (bx - ax) * t;
    const double y = ay + (by - ay) * t;
    if (!survey_point_safe(x, y, room_zone_index, clearance)) return 0;
  }
  return 1;
}

static int survey_recent_trace_obstacle_near(double x, double y, double clearance) {
  const double now_time = wb_robot_get_time();
  const double clearance_sq = clearance * clearance;
  for (int i = 0; i < obstacle_trace_count; ++i) {
    if (controller_lidar_trace_confidence(
            &obstacle_trace[i], now_time, LIDAR_TRACE_TTL_SECONDS) < 0.18) {
      continue;
    }
    const double dx = obstacle_trace[i].x - x;
    const double dy = obstacle_trace[i].y - y;
    if (dx * dx + dy * dy <= clearance_sq) return 1;
  }
  return 0;
}

static int survey_known_obstacle_near(double x, double y, double clearance) {
  return survey_map_obstacle_near(x, y, clearance) ||
         survey_recent_trace_obstacle_near(x, y, clearance);
}

static int mapping_survey_segment_clear_of_known_obstacles(
    double ax,
    double ay,
    double bx,
    double by,
    double clearance) {
  const double length = hypot2(bx - ax, by - ay);
  const int steps = (int)ceil(length / fmax(MAPPING_SURVEY_GRID_CELL * 0.9, 0.06));
  for (int i = 0; i <= steps; ++i) {
    const double t = steps > 0 ? (double)i / (double)steps : 0.0;
    if (length * t < LIDAR_NEAR_ROBOT_IGNORE_RADIUS + 0.18) continue;
    const double x = ax + (bx - ax) * t;
    const double y = ay + (by - ay) * t;
    if (survey_known_obstacle_near(x, y, clearance)) return 0;
  }
  return 1;
}

static int mapping_survey_segment_stays_in_room(double ax, double ay, double bx, double by) {
  if (mapping_survey_room_zone_index < 0) return 1;

  const LimitZone *room = &zone_data.zones[mapping_survey_room_zone_index];
  const double length = hypot2(bx - ax, by - ay);
  const int steps = (int)ceil(length / fmax(MAPPING_SURVEY_GRID_CELL, 0.08));
  for (int i = 1; i <= steps; ++i) {
    const double t = steps > 0 ? (double)i / (double)steps : 1.0;
    const double x = ax + (bx - ax) * t;
    const double y = ay + (by - ay) * t;
    if (!point_in_zone(x, y, room)) return 0;
  }
  return 1;
}

static int find_mapping_survey_escape_waypoint(double x, double y, int start_index) {
  if (!route_source_mapping_survey || route_data.count <= 0) return -1;

  const int first = (int)clamp_value((double)start_index + 1.0, 0.0, (double)route_data.count);
  const int last = (int)fmin(
      (double)route_data.count,
      (double)first + (double)MAPPING_SURVEY_ESCAPE_SCAN_AHEAD);

  for (int i = first; i < last; ++i) {
    const Waypoint *candidate = &route_data.waypoints[i];
    const double candidate_distance = hypot2(candidate->x - x, candidate->z - y);
    if (candidate_distance < MAPPING_SURVEY_ESCAPE_MIN_TARGET_DISTANCE) continue;
    if (survey_known_obstacle_near(
            candidate->x,
            candidate->z,
            MAPPING_SURVEY_ESCAPE_OBSTACLE_CLEARANCE)) {
      continue;
    }
    if (!mapping_survey_segment_stays_in_room(x, y, candidate->x, candidate->z)) {
      continue;
    }
    if (segment_blocked_by_zones(
            x,
            y,
            candidate->x,
            candidate->z,
            MAPPING_SURVEY_ESCAPE_SEGMENT_CLEARANCE,
            mapping_survey_room_zone_index)) {
      continue;
    }
    if (!mapping_survey_segment_clear_of_known_obstacles(
            x,
            y,
            candidate->x,
            candidate->z,
            MAPPING_SURVEY_ESCAPE_SEGMENT_CLEARANCE)) {
      continue;
    }
    return i;
  }

  return -1;
}

#define survey_route_add(route, count, x, y) \
  controller_survey_route_add((route), (count), MAX_WAYPOINTS, 0.18, (x), (y))
#define survey_route_add_segment(route, count, from, to) \
  controller_survey_route_add_segment( \
      (route), (count), MAX_WAYPOINTS, 0.18, MAPPING_SURVEY_MAX_CONTOUR_STEP, (from), (to))

static int line_intersection(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy,
    double *out_x, double *out_y) {
  const double r_x = bx - ax;
  const double r_y = by - ay;
  const double s_x = dx - cx;
  const double s_y = dy - cy;
  const double denom = r_x * s_y - r_y * s_x;
  if (fabs(denom) < 1e-8) return 0;
  const double t = ((cx - ax) * s_y - (cy - ay) * s_x) / denom;
  *out_x = ax + r_x * t;
  *out_y = ay + r_y * t;
  return 1;
}

static int build_offset_zone_contour(
    const LimitZone *room,
    double offset,
    SurveyPoint *out,
    int *out_count) {
  if (!room || room->point_count < 3) return 0;
  const double area = zone_signed_area(room);
  const double orient = area >= 0.0 ? 1.0 : -1.0;
  int count = 0;

  for (int i = 0; i < room->point_count; ++i) {
    const int prev = (i + room->point_count - 1) % room->point_count;
    const int next = (i + 1) % room->point_count;
    const double px = room->points[prev].x;
    const double py = room->points[prev].y;
    const double cx = room->points[i].x;
    const double cy = room->points[i].y;
    const double nx = room->points[next].x;
    const double ny = room->points[next].y;
    const double e1x = cx - px;
    const double e1y = cy - py;
    const double e2x = nx - cx;
    const double e2y = ny - cy;
    const double e1l = hypot2(e1x, e1y);
    const double e2l = hypot2(e2x, e2y);
    if (e1l <= EPS || e2l <= EPS) continue;

    const double n1x = orient * (-e1y / e1l);
    const double n1y = orient * (e1x / e1l);
    const double n2x = orient * (-e2y / e2l);
    const double n2y = orient * (e2x / e2l);
    double ox = cx + (n1x + n2x) * 0.5 * offset;
    double oy = cy + (n1y + n2y) * 0.5 * offset;

    if (!line_intersection(
            px + n1x * offset, py + n1y * offset,
            cx + n1x * offset, cy + n1y * offset,
            cx + n2x * offset, cy + n2y * offset,
            nx + n2x * offset, ny + n2y * offset,
            &ox, &oy)) {
      const double bx = n1x + n2x;
      const double by = n1y + n2y;
      const double bl = hypot2(bx, by);
      if (bl > EPS) {
        ox = cx + bx / bl * offset;
        oy = cy + by / bl * offset;
      }
    }

    if (!point_in_zone(ox, oy, room)) {
      ox = cx + (n1x + n2x) * 0.25 * offset;
      oy = cy + (n1y + n2y) * 0.25 * offset;
    }

    if (count < MAX_ZONE_POINTS) {
      out[count++] = (SurveyPoint){ox, oy};
    }
  }

  *out_count = count;
  return count >= 3;
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
  if (!build_offset_zone_contour(
          &zone_data.zones[room_zone_index],
          MAPPING_SURVEY_CONTOUR_OFFSET,
          contour,
          &contour_count)) {
    return 0;
  }

  int start_index = 0;
  double best_dist = 1e30;
  for (int i = 0; i < contour_count; ++i) {
    const double dist = hypot2(contour[i].x - robot_x, contour[i].y - robot_y);
    if (dist < best_dist) {
      best_dist = dist;
      start_index = i;
    }
  }

  SurveyPoint start = contour[start_index];
  if (survey_point_safe(start.x, start.y, room_zone_index, MAPPING_SURVEY_CONTOUR_OFFSET * 0.72)) {
    survey_route_add(route, route_count, start.x, start.y);
  }

  SurveyPoint previous = start;
  for (int step = 1; step <= contour_count; ++step) {
    const int index = (start_index + step) % contour_count;
    SurveyPoint next = contour[index];
    if (!survey_point_safe(next.x, next.y, room_zone_index, MAPPING_SURVEY_CONTOUR_OFFSET * 0.72)) {
      continue;
    }
    survey_route_add_segment(route, route_count, previous, next);
    previous = next;
  }
  survey_route_add_segment(route, route_count, previous, start);
  return *route_count > 2;
}

static int survey_build_grid(
    SurveyGrid *grid,
    int room_zone_index,
    double robot_x,
    double robot_y,
    double clearance,
    const RuntimeCommand *command) {
  double min_x =
      command && command->has_field_bounds ? command->field_min_x : SURVEY_X_MIN;
  double max_x =
      command && command->has_field_bounds ? command->field_max_x : SURVEY_X_MAX;
  double min_y =
      command && command->has_field_bounds ? command->field_min_y : SURVEY_Y_MIN;
  double max_y =
      command && command->has_field_bounds ? command->field_max_y : SURVEY_Y_MAX;

  survey_expand_bounds(robot_x, robot_y, &min_x, &max_x, &min_y, &max_y);
  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    min_x = max_x = room->points[0].x;
    min_y = max_y = room->points[0].y;
    for (int i = 1; i < room->point_count; ++i) {
      survey_expand_bounds(room->points[i].x, room->points[i].y, &min_x, &max_x, &min_y, &max_y);
    }
  } else {
    for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
      for (int point_index = 0; point_index < zone_data.zones[zone_index].point_count; ++point_index) {
        survey_expand_bounds(
            zone_data.zones[zone_index].points[point_index].x,
            zone_data.zones[zone_index].points[point_index].y,
            &min_x,
            &max_x,
            &min_y,
            &max_y);
      }
    }
    for (int i = 0; i < persistent_map_count; ++i) {
      survey_expand_bounds(persistent_map[i].x, persistent_map[i].y, &min_x, &max_x, &min_y, &max_y);
    }
  }

  const double margin = clearance + 1.0;
  min_x = clamp_value(min_x - margin, -MAPPING_SURVEY_MAX_EXTENT_X, MAPPING_SURVEY_MAX_EXTENT_X);
  max_x = clamp_value(max_x + margin, -MAPPING_SURVEY_MAX_EXTENT_X, MAPPING_SURVEY_MAX_EXTENT_X);
  min_y = clamp_value(min_y - margin, -MAPPING_SURVEY_MAX_EXTENT_Y, MAPPING_SURVEY_MAX_EXTENT_Y);
  max_y = clamp_value(max_y + margin, -MAPPING_SURVEY_MAX_EXTENT_Y, MAPPING_SURVEY_MAX_EXTENT_Y);

  double cell = MAPPING_SURVEY_GRID_CELL;
  int width = (int)ceil((max_x - min_x) / cell) + 1;
  int height = (int)ceil((max_y - min_y) / cell) + 1;
  while (width * height > MAPPING_SURVEY_MAX_GRID_CELLS) {
    cell *= 1.18;
    width = (int)ceil((max_x - min_x) / cell) + 1;
    height = (int)ceil((max_y - min_y) / cell) + 1;
  }

  grid->min_x = min_x;
  grid->min_y = min_y;
  grid->cell = cell;
  grid->width = width;
  grid->height = height;
  grid->count = width * height;
  memset(grid->free_cell, 0, (size_t)grid->count);
  memset(grid->component_cell, 0, (size_t)grid->count);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      const double wx = min_x + (double)x * cell;
      const double wy = min_y + (double)y * cell;
      grid->free_cell[index] = survey_point_safe(wx, wy, room_zone_index, clearance) ? 1 : 0;
    }
  }

  return grid->count > 0;
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

#define survey_sort_values(values, count) controller_survey_sort_values((values), (count))
#define survey_subtract_interval(intervals, count, block_start, block_end) \
  controller_survey_subtract_interval((intervals), (count), 64, (block_start), (block_end))

static void survey_subtract_zone_band(
    SurveyInterval *intervals,
    int *count,
    const LimitZone *zone,
    double y,
    double clearance) {
  if (!zone || zone->point_count < 3 || *count <= 0) return;

  double min_y = zone->points[0].y;
  double max_y = zone->points[0].y;
  double near_min_x = 1e30;
  double near_max_x = -1e30;
  int near_band_hit = 0;

  for (int i = 0; i < zone->point_count; ++i) {
    if (zone->points[i].y < min_y) min_y = zone->points[i].y;
    if (zone->points[i].y > max_y) max_y = zone->points[i].y;
    if (fabs(zone->points[i].y - y) <= clearance) {
      if (zone->points[i].x < near_min_x) near_min_x = zone->points[i].x;
      if (zone->points[i].x > near_max_x) near_max_x = zone->points[i].x;
      near_band_hit = 1;
    }
  }

  if (y < min_y - clearance || y > max_y + clearance) return;

  static const double sample_offsets[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    const double sample_y = y + sample_offsets[sample_index] * clearance;
    double xs[MAX_ZONE_POINTS];
    int xs_count = 0;

    for (int i = 0; i < zone->point_count; ++i) {
      const int next = (i + 1) % zone->point_count;
      const double ax = zone->points[i].x;
      const double ay = zone->points[i].y;
      const double bx = zone->points[next].x;
      const double by = zone->points[next].y;

      if (fabs(ay - by) < EPS) {
        if (fabs(ay - y) <= clearance) {
          if (fmin(ax, bx) < near_min_x) near_min_x = fmin(ax, bx);
          if (fmax(ax, bx) > near_max_x) near_max_x = fmax(ax, bx);
          near_band_hit = 1;
        }
        continue;
      }

      if ((ay <= sample_y && by > sample_y) || (by <= sample_y && ay > sample_y)) {
        xs[xs_count++] = ax + (sample_y - ay) * (bx - ax) / (by - ay);
      }
    }

    survey_sort_values(xs, xs_count);
    for (int i = 0; i + 1 < xs_count; i += 2) {
      survey_subtract_interval(
          intervals,
          count,
          xs[i] - clearance,
          xs[i + 1] + clearance);
    }
  }

  if (near_band_hit && near_max_x >= near_min_x) {
    survey_subtract_interval(
        intervals,
        count,
        near_min_x - clearance,
        near_max_x + clearance);
  }
}

static int build_scanline_intervals(
    double y,
    int room_zone_index,
    const SurveyGrid *grid,
    SurveyInterval *intervals,
    int max_intervals) {
  int count = 0;
  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    double xs[MAX_ZONE_POINTS];
    int xs_count = 0;
    for (int i = 0; i < room->point_count; ++i) {
      const int next = (i + 1) % room->point_count;
      const double ax = room->points[i].x;
      const double ay = room->points[i].y;
      const double bx = room->points[next].x;
      const double by = room->points[next].y;
      if (fabs(ay - by) < EPS) continue;
      if ((ay <= y && by > y) || (by <= y && ay > y)) {
        xs[xs_count++] = ax + (y - ay) * (bx - ax) / (by - ay);
      }
    }
    survey_sort_values(xs, xs_count);
    for (int i = 0; i + 1 < xs_count && count < max_intervals; i += 2) {
      double start = xs[i] + MAPPING_SURVEY_INTERIOR_OFFSET;
      double end = xs[i + 1] - MAPPING_SURVEY_INTERIOR_OFFSET;
      if (end - start >= MAPPING_SURVEY_MIN_STRIP_LENGTH) {
        intervals[count++] = (SurveyInterval){start, end};
      }
    }
  } else if (grid) {
    const int row = (int)round((y - grid->min_y) / grid->cell);
    if (row >= 0 && row < grid->height) {
      int run_start = -1;
      for (int x = 0; x <= grid->width; ++x) {
        const int in_component = x < grid->width && grid->component_cell[row * grid->width + x];
        if (in_component && run_start < 0) run_start = x;
        if ((!in_component || x == grid->width) && run_start >= 0) {
          const int run_end = x - 1;
          const double start = grid->min_x + (double)run_start * grid->cell + MAPPING_SURVEY_INTERIOR_OFFSET;
          const double end = grid->min_x + (double)run_end * grid->cell - MAPPING_SURVEY_INTERIOR_OFFSET;
          if (end - start >= MAPPING_SURVEY_MIN_STRIP_LENGTH && count < max_intervals) {
            intervals[count++] = (SurveyInterval){start, end};
          }
          run_start = -1;
        }
      }
    }
  }

  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (zone_index == room_zone_index) continue;
    survey_subtract_zone_band(
        intervals,
        &count,
        &zone_data.zones[zone_index],
        y,
        MAPPING_SURVEY_INTERIOR_OFFSET);
  }

  for (int i = 0; i < persistent_map_count; ++i) {
    if (fabs(persistent_map[i].y - y) > MAPPING_SURVEY_INTERIOR_OFFSET) continue;
    survey_subtract_interval(
        intervals,
        &count,
        persistent_map[i].x - MAPPING_SURVEY_INTERIOR_OFFSET,
        persistent_map[i].x + MAPPING_SURVEY_INTERIOR_OFFSET);
  }

  int write = 0;
  for (int i = 0; i < count; ++i) {
    if (intervals[i].end - intervals[i].start < MAPPING_SURVEY_MIN_STRIP_LENGTH) continue;
    intervals[write++] = intervals[i];
  }
  return write;
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

static void survey_get_coverage_bounds(
    const SurveyGrid *grid,
    int room_zone_index,
    double *min_x,
    double *max_x,
    double *min_y,
    double *max_y) {
  *min_x = grid->min_x + MAPPING_SURVEY_INTERIOR_OFFSET;
  *max_x = grid->min_x + (double)(grid->width - 1) * grid->cell - MAPPING_SURVEY_INTERIOR_OFFSET;
  *min_y = grid->min_y + MAPPING_SURVEY_INTERIOR_OFFSET;
  *max_y = grid->min_y + (double)(grid->height - 1) * grid->cell - MAPPING_SURVEY_INTERIOR_OFFSET;

  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    *min_x = *max_x = room->points[0].x;
    *min_y = *max_y = room->points[0].y;
    for (int i = 1; i < room->point_count; ++i) {
      if (room->points[i].x < *min_x) *min_x = room->points[i].x;
      if (room->points[i].x > *max_x) *max_x = room->points[i].x;
      if (room->points[i].y < *min_y) *min_y = room->points[i].y;
      if (room->points[i].y > *max_y) *max_y = room->points[i].y;
    }
    *min_x += MAPPING_SURVEY_INTERIOR_OFFSET;
    *max_x -= MAPPING_SURVEY_INTERIOR_OFFSET;
    *min_y += MAPPING_SURVEY_INTERIOR_OFFSET;
    *max_y -= MAPPING_SURVEY_INTERIOR_OFFSET;
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

static int choose_horizontal_snake_start(
    SurveyGrid *grid,
    int room_zone_index,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    SurveyPoint current,
    int sweep_from_top,
    int *start_forward,
    double *best_distance) {
  for (int scan_index = 0; scan_index < 4096; ++scan_index) {
    const double y = sweep_from_top
                         ? max_y - (double)scan_index * MAPPING_SURVEY_STRIP
                         : min_y + (double)scan_index * MAPPING_SURVEY_STRIP;
    if (sweep_from_top ? y < min_y - 0.01 : y > max_y + 0.01) break;

    SurveyInterval intervals[64];
    const int interval_count = build_clipped_horizontal_intervals(
        grid,
        room_zone_index,
        y,
        min_x,
        max_x,
        intervals,
        64);
    if (interval_count <= 0) continue;

    const SurveyPoint left = {intervals[0].start, y};
    const SurveyPoint right = {intervals[interval_count - 1].end, y};
    controller_survey_choose_snake_start(
        current, left, right, start_forward, best_distance);
    return 1;
  }

  return 0;
}

static void append_scanline_coverage_phase_for_bounds(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    int start_forward,
    int sweep_from_top) {
  int forward = start_forward ? 1 : 0;

  for (int scan_index = 0; *route_count < MAX_WAYPOINTS - 4; ++scan_index) {
    const double y = sweep_from_top
                         ? max_y - (double)scan_index * MAPPING_SURVEY_STRIP
                         : min_y + (double)scan_index * MAPPING_SURVEY_STRIP;
    if (sweep_from_top ? y < min_y - 0.01 : y > max_y + 0.01) break;

    SurveyInterval intervals[64];
    int interval_count = build_clipped_horizontal_intervals(
        grid,
        room_zone_index,
        y,
        min_x,
        max_x,
        intervals,
        64);

    if (!forward) {
      controller_survey_reverse_intervals(intervals, interval_count);
    }

    for (int i = 0; i < interval_count && *route_count < MAX_WAYPOINTS - 2; ++i) {
      SurveyPoint a = forward
                          ? (SurveyPoint){intervals[i].start, y}
                          : (SurveyPoint){intervals[i].end, y};
      SurveyPoint b = forward
                          ? (SurveyPoint){intervals[i].end, y}
                          : (SurveyPoint){intervals[i].start, y};
      append_survey_coverage_segment(grid, route, route_count, room_zone_index, a, b);
    }
    forward = !forward;
  }
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
  survey_get_coverage_bounds(grid, room_zone_index, &min_x, &max_x, &min_y, &max_y);

  const SurveyPoint current =
      *route_count > 0 ? route[*route_count - 1] : (SurveyPoint){min_x, min_y};
  int bottom_forward = 1;
  int top_forward = 1;
  double bottom_distance = 1e30;
  double top_distance = 1e30;
  const int has_bottom = choose_horizontal_snake_start(
      grid,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      current,
      0,
      &bottom_forward,
      &bottom_distance);
  const int has_top = choose_horizontal_snake_start(
      grid,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      current,
      1,
      &top_forward,
      &top_distance);
  const int sweep_from_top = has_top && (!has_bottom || top_distance < bottom_distance);
  const int start_forward = sweep_from_top ? top_forward : bottom_forward;

  append_scanline_coverage_phase_for_bounds(
      grid,
      route,
      route_count,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      start_forward,
      sweep_from_top);
}

static void survey_subtract_zone_vertical_band(
    SurveyInterval *intervals,
    int *count,
    const LimitZone *zone,
    double x,
    double clearance) {
  if (!zone || zone->point_count < 3 || *count <= 0) return;

  double min_x = zone->points[0].x;
  double max_x = zone->points[0].x;
  double near_min_y = 1e30;
  double near_max_y = -1e30;
  int near_band_hit = 0;

  for (int i = 0; i < zone->point_count; ++i) {
    if (zone->points[i].x < min_x) min_x = zone->points[i].x;
    if (zone->points[i].x > max_x) max_x = zone->points[i].x;
    if (fabs(zone->points[i].x - x) <= clearance) {
      if (zone->points[i].y < near_min_y) near_min_y = zone->points[i].y;
      if (zone->points[i].y > near_max_y) near_max_y = zone->points[i].y;
      near_band_hit = 1;
    }
  }

  if (x < min_x - clearance || x > max_x + clearance) return;

  static const double sample_offsets[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    const double sample_x = x + sample_offsets[sample_index] * clearance;
    double ys[MAX_ZONE_POINTS];
    int ys_count = 0;

    for (int i = 0; i < zone->point_count; ++i) {
      const int next = (i + 1) % zone->point_count;
      const double ax = zone->points[i].x;
      const double ay = zone->points[i].y;
      const double bx = zone->points[next].x;
      const double by = zone->points[next].y;

      if (fabs(ax - bx) < EPS) {
        if (fabs(ax - x) <= clearance) {
          if (fmin(ay, by) < near_min_y) near_min_y = fmin(ay, by);
          if (fmax(ay, by) > near_max_y) near_max_y = fmax(ay, by);
          near_band_hit = 1;
        }
        continue;
      }

      if ((ax <= sample_x && bx > sample_x) || (bx <= sample_x && ax > sample_x)) {
        ys[ys_count++] = ay + (sample_x - ax) * (by - ay) / (bx - ax);
      }
    }

    survey_sort_values(ys, ys_count);
    for (int i = 0; i + 1 < ys_count; i += 2) {
      survey_subtract_interval(
          intervals,
          count,
          ys[i] - clearance,
          ys[i + 1] + clearance);
    }
  }

  if (near_band_hit && near_max_y >= near_min_y) {
    survey_subtract_interval(
        intervals,
        count,
        near_min_y - clearance,
        near_max_y + clearance);
  }
}

static int build_vertical_intervals(
    double x,
    int room_zone_index,
    const SurveyGrid *grid,
    SurveyInterval *intervals,
    int max_intervals) {
  int count = 0;
  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    double ys[MAX_ZONE_POINTS];
    int ys_count = 0;
    for (int i = 0; i < room->point_count; ++i) {
      const int next = (i + 1) % room->point_count;
      const double ax = room->points[i].x;
      const double ay = room->points[i].y;
      const double bx = room->points[next].x;
      const double by = room->points[next].y;
      if (fabs(ax - bx) < EPS) continue;
      if ((ax <= x && bx > x) || (bx <= x && ax > x)) {
        ys[ys_count++] = ay + (x - ax) * (by - ay) / (bx - ax);
      }
    }
    survey_sort_values(ys, ys_count);
    for (int i = 0; i + 1 < ys_count && count < max_intervals; i += 2) {
      double start = ys[i] + MAPPING_SURVEY_INTERIOR_OFFSET;
      double end = ys[i + 1] - MAPPING_SURVEY_INTERIOR_OFFSET;
      if (end - start >= MAPPING_SURVEY_MIN_STRIP_LENGTH) {
        intervals[count++] = (SurveyInterval){start, end};
      }
    }
  } else if (grid) {
    const int col = (int)round((x - grid->min_x) / grid->cell);
    if (col >= 0 && col < grid->width) {
      int run_start = -1;
      for (int y = 0; y <= grid->height; ++y) {
        const int in_component = y < grid->height && grid->component_cell[y * grid->width + col];
        if (in_component && run_start < 0) run_start = y;
        if ((!in_component || y == grid->height) && run_start >= 0) {
          const int run_end = y - 1;
          const double start = grid->min_y + (double)run_start * grid->cell + MAPPING_SURVEY_INTERIOR_OFFSET;
          const double end = grid->min_y + (double)run_end * grid->cell - MAPPING_SURVEY_INTERIOR_OFFSET;
          if (end - start >= MAPPING_SURVEY_MIN_STRIP_LENGTH && count < max_intervals) {
            intervals[count++] = (SurveyInterval){start, end};
          }
          run_start = -1;
        }
      }
    }
  }

  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (zone_index == room_zone_index) continue;
    survey_subtract_zone_vertical_band(
        intervals,
        &count,
        &zone_data.zones[zone_index],
        x,
        MAPPING_SURVEY_INTERIOR_OFFSET);
  }

  for (int i = 0; i < persistent_map_count; ++i) {
    if (fabs(persistent_map[i].x - x) > MAPPING_SURVEY_INTERIOR_OFFSET) continue;
    survey_subtract_interval(
        intervals,
        &count,
        persistent_map[i].y - MAPPING_SURVEY_INTERIOR_OFFSET,
        persistent_map[i].y + MAPPING_SURVEY_INTERIOR_OFFSET);
  }

  int write = 0;
  for (int i = 0; i < count; ++i) {
    if (intervals[i].end - intervals[i].start < MAPPING_SURVEY_MIN_STRIP_LENGTH) continue;
    intervals[write++] = intervals[i];
  }
  return write;
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

static int choose_vertical_snake_start(
    SurveyGrid *grid,
    int room_zone_index,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    SurveyPoint current,
    int sweep_from_right,
    int *start_upward,
    double *best_distance) {
  for (int scan_index = 0; scan_index < 4096; ++scan_index) {
    const double x = sweep_from_right
                         ? max_x - (double)scan_index * MAPPING_SURVEY_STRIP
                         : min_x + (double)scan_index * MAPPING_SURVEY_STRIP;
    if (sweep_from_right ? x < min_x - 0.01 : x > max_x + 0.01) break;

    SurveyInterval intervals[64];
    const int interval_count = build_clipped_vertical_intervals(
        grid,
        room_zone_index,
        x,
        min_y,
        max_y,
        intervals,
        64);
    if (interval_count <= 0) continue;

    const SurveyPoint bottom = {x, intervals[0].start};
    const SurveyPoint top = {x, intervals[interval_count - 1].end};
    controller_survey_choose_snake_start(
        current, bottom, top, start_upward, best_distance);
    return 1;
  }

  return 0;
}

static void append_vertical_coverage_phase_for_bounds(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    int start_upward,
    int sweep_from_right) {
  int upward = start_upward ? 1 : 0;

  for (int scan_index = 0; *route_count < MAX_WAYPOINTS - 4; ++scan_index) {
    const double x = sweep_from_right
                         ? max_x - (double)scan_index * MAPPING_SURVEY_STRIP
                         : min_x + (double)scan_index * MAPPING_SURVEY_STRIP;
    if (sweep_from_right ? x < min_x - 0.01 : x > max_x + 0.01) break;

    SurveyInterval intervals[64];
    int interval_count = build_clipped_vertical_intervals(
        grid,
        room_zone_index,
        x,
        min_y,
        max_y,
        intervals,
        64);

    if (!upward) {
      controller_survey_reverse_intervals(intervals, interval_count);
    }

    for (int i = 0; i < interval_count && *route_count < MAX_WAYPOINTS - 2; ++i) {
      SurveyPoint a = upward
                          ? (SurveyPoint){x, intervals[i].start}
                          : (SurveyPoint){x, intervals[i].end};
      SurveyPoint b = upward
                          ? (SurveyPoint){x, intervals[i].end}
                          : (SurveyPoint){x, intervals[i].start};
      append_survey_coverage_segment(grid, route, route_count, room_zone_index, a, b);
    }
    upward = !upward;
  }
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
  survey_get_coverage_bounds(grid, room_zone_index, &min_x, &max_x, &min_y, &max_y);

  const SurveyPoint current =
      *route_count > 0 ? route[*route_count - 1] : (SurveyPoint){min_x, min_y};
  int left_upward = 1;
  int right_upward = 1;
  double left_distance = 1e30;
  double right_distance = 1e30;
  const int has_left = choose_vertical_snake_start(
      grid,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      current,
      0,
      &left_upward,
      &left_distance);
  const int has_right = choose_vertical_snake_start(
      grid,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      current,
      1,
      &right_upward,
      &right_distance);
  const int sweep_from_right = has_right && (!has_left || right_distance < left_distance);
  const int start_upward = sweep_from_right ? right_upward : left_upward;

  append_vertical_coverage_phase_for_bounds(
      grid,
      route,
      route_count,
      room_zone_index,
      min_x,
      max_x,
      min_y,
      max_y,
      start_upward,
      sweep_from_right);
}

static void write_mapping_survey_route_file(const char *path, const SurveyPoint *route, int route_count) {
  FILE *file = fopen(path, "w");
  if (!file) {
    set_error("Cannot write mapping survey route to route.csv");
    return;
  }

  fprintf(file,
          "# Auto-generated mapping survey route: perimeter first, then %s coverage\n",
          controller_mapping_survey_mode_to_string(mapping_survey_mode));
  fprintf(file, "# x,y\n");
  for (int i = 0; i < route_count; ++i) {
    fprintf(file, "%.3f,%.3f\n", route[i].x, route[i].y);
  }
  fclose(file);
}

static int generate_mapping_survey_route(
    const char *path,
    int clear_map_before_start,
    const RuntimeCommand *command) {
  if (clear_map_before_start) clear_persistent_map();
  const MappingSurveyMode survey_mode =
      command ? command->survey_mode : mapping_survey_mode;
  controller_mapping_survey_state_prepare(&mapping_survey_state, survey_mode);

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  const int room_zone_index = find_room_zone_index(robot_x, robot_y);
  mapping_survey_room_zone_index = room_zone_index;
  static SurveyGrid grid;
  if (!survey_build_grid(
          &grid,
          room_zone_index,
          robot_x,
          robot_y,
          MAPPING_SURVEY_INTERIOR_OFFSET,
          command)) {
    set_error("Cannot build mapping survey occupancy grid");
    return 0;
  }
  if (survey_flood_component(&grid, robot_x, robot_y) <= 0) {
    set_error("Cannot find connected free room for mapping survey");
    return 0;
  }

  SurveyPoint route[MAX_WAYPOINTS];
  int route_count = 0;
  if (survey_point_safe(robot_x, robot_y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET * 0.45)) {
    survey_route_add(route, &route_count, robot_x, robot_y);
  }
  if (!append_room_contour_phase(route, &route_count, room_zone_index, robot_x, robot_y)) {
    append_grid_boundary_contour_phase(&grid, route, &route_count, robot_x, robot_y);
  }
  mapping_survey_interior_start_index = route_count;

  switch (mapping_survey_mode) {
    case MAPPING_SURVEY_MODE_DOUBLE:
      append_scanline_coverage_phase(&grid, route, &route_count, room_zone_index);
      append_vertical_coverage_phase(&grid, route, &route_count, room_zone_index);
      break;
    case MAPPING_SURVEY_MODE_SNAKE:
    default:
      append_scanline_coverage_phase(&grid, route, &route_count, room_zone_index);
      break;
  }

  if (route_count < 2) {
    set_error("Mapping survey route is empty");
    return 0;
  }

  write_mapping_survey_route_file(path, route, route_count);
  return 1;
}

static int same_zone_data(const ZoneData *left, const ZoneData *right) {
  return controller_zone_data_equal(left, right);
}

static int same_surface_zone_data(const SurfaceZoneData *left, const SurfaceZoneData *right) {
  return controller_surface_zone_data_equal(left, right);
}

static int load_surface_zones(SurfaceZoneData *zones) {
  const ControllerZoneLoadResult result =
      controller_surface_zones_load_file(SURFACE_ZONE_PATH, zones);
  if (result == CONTROLLER_ZONE_LOAD_NO_DATA) return 1;
  if (result == CONTROLLER_ZONE_LOAD_INVALID_HEADER) set_error("Cannot parse surface_zones.txt");
  if (result == CONTROLLER_ZONE_LOAD_INVALID_ENTRY) set_error("Cannot parse surface zone entry");
  if (result == CONTROLLER_ZONE_LOAD_UNEXPECTED_END) set_error("Unexpected end of surface_zones.txt");
  if (result == CONTROLLER_ZONE_LOAD_INVALID_POINT) set_error("Cannot parse surface zone point");
  if (result != CONTROLLER_ZONE_LOAD_OK) return 0;
  clear_error();
  return 1;
}

static int load_zones(ZoneData *zones) {
  const ControllerZoneLoadResult result =
      controller_limit_zones_load_file(ZONE_PATH, zones);
  if (result == CONTROLLER_ZONE_LOAD_NO_DATA) return 1;
  if (result == CONTROLLER_ZONE_LOAD_INVALID_HEADER) set_error("Cannot parse limit_zones.txt");
  if (result == CONTROLLER_ZONE_LOAD_INVALID_ENTRY) set_error("Cannot parse limit zone entry");
  if (result == CONTROLLER_ZONE_LOAD_UNEXPECTED_END) set_error("Unexpected end of limit_zones.txt");
  if (result == CONTROLLER_ZONE_LOAD_INVALID_POINT) set_error("Cannot parse limit zone point");
  if (result != CONTROLLER_ZONE_LOAD_OK) return 0;
  clear_error();
  return 1;
}

static void remove_zone_nodes() {
  for (int i = 0; i < zone_node_count; ++i) {
    if (zone_node_defs[i][0] != '\0') {
      WbNodeRef node = wb_supervisor_node_get_from_def(zone_node_defs[i]);
      if (node) {
        wb_supervisor_node_remove(node);
      }
    }
    zone_nodes[i] = 0;
    zone_node_defs[i][0] = '\0';
  }
  zone_node_count = 0;
}

static void sync_zone_nodes() {
  remove_zone_nodes();
  if (!root_children_field) return;

  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    const LimitZone *zone = &zone_data.zones[zone_index];
    for (int point_index = 0; point_index < zone->point_count; ++point_index) {
      const int next = (point_index + 1) % zone->point_count;
      const double ax = zone->points[point_index].x;
      const double ay = zone->points[point_index].y;
      const double bx = zone->points[next].x;
      const double by = zone->points[next].y;
      const double dx = bx - ax;
      const double dy = by - ay;
      const double length = hypot2(dx, dy);
      if (length <= 0.05) continue;

      const double center_x = (ax + bx) * 0.5;
      const double center_y = (ay + by) * 0.5;
      const double angle = atan2(dy, dx);

      char def_name[64];
      char node_string[1024];
      snprintf(def_name, sizeof(def_name), "WEB_LIMIT_%d_%d", zone_index, point_index);
      snprintf(
          node_string,
          sizeof(node_string),
          "DEF %s Solid { "
          "translation %.6f %.6f %.6f "
          "rotation 0 0 1 %.6f "
          "name \"dynamic_zone_wall\" "
          "children [ "
          "Shape { "
          "appearance PBRAppearance { baseColor 0.1176 0.4510 0.9725 roughness 1 metalness 0 transparency 0.45 } "
          "geometry Box { size %.6f %.6f %.6f } "
          "} "
          "] "
          "boundingObject Box { size %.6f %.6f %.6f } "
          "locked TRUE "
          "}",
          def_name,
          center_x,
          center_y,
          WALL_HEIGHT * 0.5,
          angle,
          length,
          WALL_THICKNESS,
          WALL_HEIGHT,
          length,
          WALL_THICKNESS,
          WALL_HEIGHT);

      const int insert_at = wb_supervisor_field_get_count(root_children_field);
      wb_supervisor_field_import_mf_node_from_string(root_children_field, insert_at, node_string);
      if (zone_node_count < MAX_ZONE_NODES) {
        zone_nodes[zone_node_count] = wb_supervisor_node_get_from_def(def_name);
        strncpy(zone_node_defs[zone_node_count], def_name, sizeof(zone_node_defs[zone_node_count]) - 1);
        zone_node_defs[zone_node_count][sizeof(zone_node_defs[zone_node_count]) - 1] = '\0';
        zone_node_count += 1;
      }
    }
  }
}

static void remove_surface_zone_nodes() {
  for (int i = 0; i < surface_zone_count; ++i) {
    if (surface_zone_defs[i][0] != '\0') {
      WbNodeRef node = wb_supervisor_node_get_from_def(surface_zone_defs[i]);
      if (node) {
        wb_supervisor_node_remove(node);
      }
    }
    surface_zone_nodes[i] = 0;
    surface_zone_defs[i][0] = '\0';
  }
  surface_zone_count = 0;
}

static void get_surface_zone_color(const char *surface_key, double *red, double *green, double *blue) {
  if (strcmp(surface_key, "rough") == 0) {
    *red = 0.93;
    *green = 0.54;
    *blue = 0.08;
    return;
  }
  if (strcmp(surface_key, "slippery") == 0) {
    *red = 0.14;
    *green = 0.66;
    *blue = 0.88;
    return;
  }
  *red = 0.52;
  *green = 0.60;
  *blue = 0.70;
}

static void append_text(char *buffer, size_t buffer_size, const char *text) {
  const size_t used = strlen(buffer);
  if (used >= buffer_size - 1) return;
  strncat(buffer, text, buffer_size - used - 1);
}

static void sync_surface_zone_nodes() {
  remove_surface_zone_nodes();
  if (!root_children_field) return;

  for (int zone_index = 0; zone_index < surface_zone_data.count; ++zone_index) {
    const SurfaceZone *zone = &surface_zone_data.zones[zone_index];
    if (zone->point_count < 3) continue;

    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    get_surface_zone_color(zone->surface_key, &red, &green, &blue);

    char coord_buffer[2048] = "";
    char index_buffer[512] = "";
    char chunk[96];
    for (int point_index = 0; point_index < zone->point_count; ++point_index) {
      snprintf(
          chunk,
          sizeof(chunk),
          "%.6f %.6f %.6f, ",
          zone->points[point_index].x,
          zone->points[point_index].y,
          0.012 + zone_index * 0.001);
      append_text(coord_buffer, sizeof(coord_buffer), chunk);

      snprintf(chunk, sizeof(chunk), "%d ", point_index);
      append_text(index_buffer, sizeof(index_buffer), chunk);
    }
    append_text(index_buffer, sizeof(index_buffer), "-1");

    char def_name[64];
    char node_string[4096];
    snprintf(def_name, sizeof(def_name), "WEB_SURFACE_%d", zone_index);
    snprintf(
        node_string,
        sizeof(node_string),
        "DEF %s Pose { "
        "children [ "
        "Shape { "
        "appearance PBRAppearance { baseColor %.4f %.4f %.4f roughness 1 metalness 0 transparency 0.58 } "
        "geometry IndexedFaceSet { "
        "coord Coordinate { point [ %s ] } "
        "coordIndex [ %s ] "
        "solid FALSE "
        "} "
        "} "
        "] "
        "}",
        def_name,
        red,
        green,
        blue,
        coord_buffer,
        index_buffer);

    const int insert_at = wb_supervisor_field_get_count(root_children_field);
    wb_supervisor_field_import_mf_node_from_string(root_children_field, insert_at, node_string);
    if (surface_zone_count < MAX_SURFACE_ZONE_NODES) {
      surface_zone_nodes[surface_zone_count] = wb_supervisor_node_get_from_def(def_name);
      strncpy(surface_zone_defs[surface_zone_count], def_name, sizeof(surface_zone_defs[surface_zone_count]) - 1);
      surface_zone_defs[surface_zone_count][sizeof(surface_zone_defs[surface_zone_count]) - 1] = '\0';
      surface_zone_count += 1;
    }
  }
}

static void remove_runtime_obstacle_at(int index) {
  if (index < 0 || index >= runtime_obstacle_count) return;

  if (runtime_obstacle_defs[index][0] != '\0') {
    WbNodeRef node = wb_supervisor_node_get_from_def(runtime_obstacle_defs[index]);
    if (node) {
      wb_supervisor_node_remove(node);
    }
  }

  for (int i = index + 1; i < runtime_obstacle_count; ++i) {
    runtime_obstacle_nodes[i - 1] = runtime_obstacle_nodes[i];
    strncpy(runtime_obstacle_defs[i - 1], runtime_obstacle_defs[i], sizeof(runtime_obstacle_defs[i - 1]) - 1);
    runtime_obstacle_defs[i - 1][sizeof(runtime_obstacle_defs[i - 1]) - 1] = '\0';
  }

  runtime_obstacle_count -= 1;
  if (runtime_obstacle_count < 0) runtime_obstacle_count = 0;
  if (runtime_obstacle_count < MAX_RUNTIME_OBSTACLE_NODES) {
    runtime_obstacle_nodes[runtime_obstacle_count] = 0;
    runtime_obstacle_defs[runtime_obstacle_count][0] = '\0';
  }
}

static void remove_runtime_obstacle_nodes() {
  while (runtime_obstacle_count > 0) {
    remove_runtime_obstacle_at(0);
  }
}

static void spawn_runtime_obstacle_from_command(const RuntimeCommand *command) {
  if (!command || !command->has_spawn_obstacle || !root_children_field) return;

  const double x = clamp_value(command->x, -21.5, 21.5);
  const double y = clamp_value(command->y, -16.5, 16.5);
  const double size_x = clamp_value(command->size_x, 0.2, 3.5);
  const double size_y = clamp_value(command->size_y, 0.2, 3.5);
  const double height = clamp_value(command->height, 0.12, 2.8);
  const double half_height = height * 0.5;

  if (runtime_obstacle_count >= MAX_RUNTIME_OBSTACLE_NODES) {
    remove_runtime_obstacle_at(0);
  }

  const long long compact_id = command->id >= 0 ? command->id : 0;
  char def_name[64];
  char node_string[1024];
  snprintf(def_name, sizeof(def_name), "WEB_OBS_%lld", compact_id);
  WbNodeRef existing = wb_supervisor_node_get_from_def(def_name);
  if (existing) {
    wb_supervisor_node_remove(existing);
  }
  snprintf(
      node_string,
      sizeof(node_string),
      "DEF %s Solid { "
      "translation %.6f %.6f %.6f "
      "name \"runtime_obstacle\" "
      "children [ "
      "Shape { "
      "appearance PBRAppearance { baseColor 0.7608 0.2549 0.1451 roughness 0.95 metalness 0.0 } "
      "geometry Box { size %.6f %.6f %.6f } "
      "} "
      "] "
      "boundingObject Box { size %.6f %.6f %.6f } "
      "locked TRUE "
      "}",
      def_name,
      x,
      y,
      half_height,
      size_x,
      size_y,
      height,
      size_x,
      size_y,
      height);

  const int insert_at = wb_supervisor_field_get_count(root_children_field);
  wb_supervisor_field_import_mf_node_from_string(root_children_field, insert_at, node_string);
  if (runtime_obstacle_count < MAX_RUNTIME_OBSTACLE_NODES) {
    runtime_obstacle_nodes[runtime_obstacle_count] = wb_supervisor_node_get_from_def(def_name);
    strncpy(runtime_obstacle_defs[runtime_obstacle_count], def_name, sizeof(runtime_obstacle_defs[runtime_obstacle_count]) - 1);
    runtime_obstacle_defs[runtime_obstacle_count][sizeof(runtime_obstacle_defs[runtime_obstacle_count]) - 1] = '\0';
    runtime_obstacle_count += 1;
  }
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
    clear_error();
    if (command.survey_speed_mps > 0.0) {
      configured_cruise_speed_mps =
          clamp_value(command.survey_speed_mps, MIN_CRUISE_SPEED_MPS, MAX_CRUISE_SPEED_MPS);
      apply_motion_profile();
    }
    if (generate_mapping_survey_route(ROUTE_PATH, command.clear_map, &command)) {
      RouteData next_route = {0};
      if (load_route(&next_route)) {
        route_data = next_route;
        current_waypoint_index = 0;
        route_finished = 0;
        route_source_mapping_survey = 1;
        mapping_survey_mode = command.survey_mode;
        reset_route_avoidance_metrics();
        reset_navigation_mode();
        set_status("mapping_survey_started");
      }
    }
    return;
  }

  spawn_runtime_obstacle_from_command(&command);
  clear_error();
  set_status("runtime_obstacle_spawned");
}

static void maybe_reload_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;

  ZoneData next_zones = {0};
  if (!load_zones(&next_zones)) return;

  if (!same_zone_data(&zone_data, &next_zones)) {
    zone_data = next_zones;
    sync_zone_nodes();
    set_status(zone_data.count > 0 ? "zones_synced" : "zones_cleared");
  }
}

static void maybe_reload_surface_zones(void) {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;

  SurfaceZoneData next_zones = {0};
  if (!load_surface_zones(&next_zones)) return;

  if (!same_surface_zone_data(&surface_zone_data, &next_zones)) {
    surface_zone_data = next_zones;
    sync_surface_zone_nodes();
    set_status(surface_zone_data.count > 0 ? "surfaces_synced" : "surfaces_cleared");
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

  const long long mtime = get_file_mtime(ROUTE_PATH);
  if (mtime < 0) return;
  if (mtime == route_data.last_modified) return;

  RouteData next_route = {0};
  if (load_route(&next_route)) {
    route_data = next_route;
    current_waypoint_index = 0;
    route_finished = 0;
    route_source_mapping_survey = 0;
    controller_mapping_survey_state_reset_route(&mapping_survey_state);
    reset_route_avoidance_metrics();
    reset_navigation_mode();
    set_status("route_reloaded");
  }
}

static int replan_mapping_survey_route_from_current_map() {
  if (!route_source_mapping_survey) return 0;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  merge_trace_into_map(wb_robot_get_time());
  if (!generate_mapping_survey_route(ROUTE_PATH, 0, NULL)) {
    mapping_survey_replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
    return 0;
  }

  RouteData next_route = {0};
  if (!load_route(&next_route)) {
    mapping_survey_replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
    return 0;
  }

  route_data = next_route;
  const int escape_index = find_mapping_survey_escape_waypoint(robot_x, robot_y, 0);
  current_waypoint_index = escape_index >= 0 ? escape_index : 0;
  route_finished = 0;
  route_source_mapping_survey = 1;
  reset_navigation_mode();
  mapping_survey_replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
  clear_error();
  set_status("mapping_survey_replanned_around_obstacle");
  return 1;
}

static int escape_mapping_survey_orbit(double x, double y) {
  if (!route_source_mapping_survey) return 0;

  merge_trace_into_map(wb_robot_get_time());
  int escape_index = find_mapping_survey_escape_waypoint(x, y, current_waypoint_index);
  if (escape_index < 0 && current_waypoint_index + 1 < route_data.count) {
    escape_index = find_mapping_survey_escape_waypoint(x, y, current_waypoint_index + 1);
  }

  if (escape_index >= 0) {
    current_waypoint_index = escape_index;
    clear_local_navigation_state();
    begin_navigation_for_waypoint(current_waypoint_index, x, y);
    mapping_survey_replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
    distance_to_target = hypot2(route_data.waypoints[current_waypoint_index].x - x,
                                route_data.waypoints[current_waypoint_index].z - y);
    clear_error();
    set_status("mapping_survey_skipped_orbiting_obstacle");
    return 1;
  }

  return replan_mapping_survey_route_from_current_map();
}

static void wait_for_fresh_route() {
  route_data.count = 0;
  route_data.last_modified = get_file_mtime(ROUTE_PATH);
  current_waypoint_index = 0;
  route_finished = 0;
  route_source_mapping_survey = 0;
  controller_mapping_survey_state_reset_route(&mapping_survey_state);
  reset_route_avoidance_metrics();
  reset_navigation_mode();
  distance_to_target = 0.0;
  clear_error();
  set_status("waiting_for_route");
  stop_robot();
}

static int waypoint_reached(
    double x,
    double z,
    double heading,
    const Waypoint *waypoint,
    int is_final_waypoint) {
  const double dx = waypoint->x - x;
  const double dz = waypoint->z - z;
  const double distance = hypot2(dx, dz);

  if (distance > POSITION_TOLERANCE) return 0;
  if (!is_final_waypoint || !waypoint->has_heading) return 1;

  const double heading_error = wrap_angle(waypoint->heading_rad - heading);
  return fabs(heading_error) <= HEADING_TOLERANCE_RAD;
}

static int mapping_survey_scan_point_allowed(double x, double y) {
  if (mapping_survey_room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[mapping_survey_room_zone_index];
    if (!point_in_zone(x, y, room)) return 0;
    if (point_near_zone_boundary(x, y, room, ZONE_CLEARANCE * 0.72)) return 0;
  } else if (x < -MAPPING_SURVEY_MAX_EXTENT_X || x > MAPPING_SURVEY_MAX_EXTENT_X ||
             y < -MAPPING_SURVEY_MAX_EXTENT_Y || y > MAPPING_SURVEY_MAX_EXTENT_Y) {
    return 0;
  }

  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (zone_index == mapping_survey_room_zone_index) continue;
    if (point_near_zone(x, y, &zone_data.zones[zone_index], ZONE_CLEARANCE)) return 0;
  }

  return !survey_map_obstacle_near(x, y, ZONE_CLEARANCE * 0.68);
}

static void mapping_survey_scan_add_point(
    SurveyPoint *points,
    int *count,
    int max_count,
    double x,
    double y) {
  if (*count >= max_count) return;
  if (!mapping_survey_scan_point_allowed(x, y)) return;
  if (*count > 0 && hypot2(points[*count - 1].x - x, points[*count - 1].y - y) < 0.18) {
    points[*count - 1] = (SurveyPoint){x, y};
    return;
  }
  points[*count] = (SurveyPoint){x, y};
  *count += 1;
}

static int insert_mapping_survey_obstacle_scan_route(
    double x,
    double z,
    double heading,
    const Waypoint *target,
    const LidarObstacleContext *lidar_context,
    double turn_sign) {
  if (!route_source_mapping_survey || !target || !lidar_context) return 0;
  if (current_waypoint_index < mapping_survey_interior_start_index) return 0;
  if (mapping_survey_obstacle_scan_active) return 0;
  if (mapping_survey_obstacle_scan_cooldown_steps > 0) return 0;
  if (!lidar_context->has_closest_unexpected) return 0;

  const double sensor_origin_x =
      x + cos(heading) * LIDAR_LOCAL_X - sin(heading) * LIDAR_LOCAL_Y;
  const double sensor_origin_y =
      z + sin(heading) * LIDAR_LOCAL_X + cos(heading) * LIDAR_LOCAL_Y;
  const double obstacle_range = clamp_value(
      lidar_context->closest_unexpected_range,
      LIDAR_MIN_TRACE_RANGE,
      LIDAR_MAX_TRACE_RANGE);
  const double obstacle_angle = heading - lidar_context->closest_unexpected_beam_angle;
  const double obstacle_x = sensor_origin_x + cos(obstacle_angle) * obstacle_range;
  const double obstacle_y = sensor_origin_y + sin(obstacle_angle) * obstacle_range;

  if (hypot2(obstacle_x - mapping_survey_last_scan_x,
             obstacle_y - mapping_survey_last_scan_y) <
      MAPPING_SURVEY_OBSTACLE_SCAN_MIN_REPEAT_DISTANCE) {
    return 0;
  }

  const int max_scan_points = MAPPING_SURVEY_OBSTACLE_SCAN_POINTS + 3;
  if (route_data.count + max_scan_points > MAX_WAYPOINTS) return 0;

  SurveyPoint scan_points[MAPPING_SURVEY_OBSTACLE_SCAN_POINTS + 3];
  int scan_count = 0;
  double start_angle = atan2(z - obstacle_y, x - obstacle_x);
  if (!is_finite_double(start_angle) || hypot2(x - obstacle_x, z - obstacle_y) < 0.12) {
    start_angle = wrap_angle(heading + PI);
  }
  const double radius = MAPPING_SURVEY_OBSTACLE_SCAN_RADIUS;
  const double scan_turn = turn_sign >= 0.0 ? 1.0 : -1.0;

  mapping_survey_scan_add_point(
      scan_points,
      &scan_count,
      max_scan_points,
      obstacle_x + cos(start_angle) * radius,
      obstacle_y + sin(start_angle) * radius);

  for (int step = 1; step <= MAPPING_SURVEY_OBSTACLE_SCAN_POINTS; ++step) {
    const double angle =
        start_angle + scan_turn * 2.0 * PI * (double)step / (double)MAPPING_SURVEY_OBSTACLE_SCAN_POINTS;
    mapping_survey_scan_add_point(
        scan_points,
        &scan_count,
        max_scan_points,
        obstacle_x + cos(angle) * radius,
        obstacle_y + sin(angle) * radius);
  }

  const double seg_dx = target->x - navigation_segment_start_x;
  const double seg_dz = target->z - navigation_segment_start_z;
  const double seg_len = hypot2(seg_dx, seg_dz);
  if (seg_len > EPS) {
    const double ux = seg_dx / seg_len;
    const double uz = seg_dz / seg_len;
    const double obstacle_along =
        dot2(obstacle_x - navigation_segment_start_x,
             obstacle_y - navigation_segment_start_z,
             ux,
             uz);
    const double resume_along = clamp_value(
        obstacle_along + radius * 1.35,
        0.0,
        seg_len);
    const double resume_x = navigation_segment_start_x + ux * resume_along;
    const double resume_y = navigation_segment_start_z + uz * resume_along;
    mapping_survey_scan_add_point(
        scan_points,
        &scan_count,
        max_scan_points,
        resume_x,
        resume_y);
  }

  if (scan_count < 5 || route_data.count + scan_count > MAX_WAYPOINTS) return 0;

  for (int i = route_data.count - 1; i >= current_waypoint_index; --i) {
    route_data.waypoints[i + scan_count] = route_data.waypoints[i];
  }
  for (int i = 0; i < scan_count; ++i) {
    route_data.waypoints[current_waypoint_index + i] = (Waypoint){
        scan_points[i].x,
        scan_points[i].y,
        0.0,
        0};
  }
  route_data.count += scan_count;

  controller_mapping_survey_state_begin_scan(
      &mapping_survey_state,
      current_waypoint_index + scan_count - 1,
      obstacle_x,
      obstacle_y);
  clear_local_navigation_state();
  begin_navigation_for_waypoint(current_waypoint_index, x, z);
  distance_to_target = hypot2(route_data.waypoints[current_waypoint_index].x - x,
                              route_data.waypoints[current_waypoint_index].z - z);
  clear_error();
  set_status("mapping_survey_circle_scan_started");
  return 1;
}

static void run_navigation_step(void) {
  double x = 0.0;
  double z = 0.0;
  double heading = 0.0;

  read_pose(&x, &z, &heading);
  int manual_relocation_detected = 0;
  if (last_pose_valid) {
    const double pose_jump = hypot2(x - last_pose_x, z - last_pose_z);
    const double heading_jump = fabs(wrap_angle(heading - last_pose_heading));
    manual_relocation_detected =
        pose_jump > POSE_RELOCATION_DISTANCE || heading_jump > POSE_RELOCATION_HEADING_RAD;
  }
  last_pose_x = x;
  last_pose_z = z;
  last_pose_heading = heading;
  last_pose_valid = 1;

  if (route_data.count == 0) {
    set_status("waiting_for_route");
    route_finished = 0;
    avoidance_state.hold_steps = 0;
    reset_navigation_mode();
    distance_to_target = 0.0;
    stop_robot();
    return;
  }

  if (route_finished) {
    set_status("finished");
    avoidance_state.hold_steps = 0;
    reset_navigation_mode();
    distance_to_target = 0.0;
    stop_robot();
    return;
  }

  controller_mapping_survey_state_tick(&mapping_survey_state);

  if (manual_relocation_detected) {
    clear_local_navigation_state();
    navigation_waypoint_index = current_waypoint_index;
    navigation_segment_start_x = x;
    navigation_segment_start_z = z;
    navigation_mode = NAV_MODE_TRACK;
    distance_to_target = hypot2(route_data.waypoints[current_waypoint_index].x - x,
                                route_data.waypoints[current_waypoint_index].z - z);
    clear_error();
    set_status("relocalized_pose");
    stop_robot();
    return;
  }

  ensure_navigation_waypoint_initialized(x, z);

  Waypoint target = route_data.waypoints[current_waypoint_index];
  int is_final_waypoint = current_waypoint_index + 1 >= route_data.count;
  if (waypoint_reached(x, z, heading, &target, is_final_waypoint)) {
    if (current_waypoint_index + 1 < route_data.count) {
      ++current_waypoint_index;
      begin_navigation_for_waypoint(current_waypoint_index, x, z);
      target = route_data.waypoints[current_waypoint_index];
      is_final_waypoint = current_waypoint_index + 1 >= route_data.count;
    } else {
      route_finished = 1;
      set_status("finished");
      reset_navigation_mode();
      distance_to_target = 0.0;
      stop_robot();
      return;
    }
  }

  if (controller_mapping_survey_state_complete_scan(
          &mapping_survey_state,
          current_waypoint_index,
          MAPPING_SURVEY_OBSTACLE_SCAN_COOLDOWN_STEPS)) {
    clear_local_navigation_state();
    begin_navigation_for_waypoint(current_waypoint_index, x, z);
    set_status("mapping_survey_resumed_route");
  }

  int target_blocked_by_zone = 0;
  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (route_source_mapping_survey && zone_index == mapping_survey_room_zone_index) continue;
    if (point_near_zone(target.x, target.z, &zone_data.zones[zone_index], ZONE_CLEARANCE)) {
      target_blocked_by_zone = 1;
      break;
    }
  }
  if (target_blocked_by_zone) {
    if (route_source_mapping_survey && current_waypoint_index + 1 < route_data.count) {
      ++current_waypoint_index;
      begin_navigation_for_waypoint(current_waypoint_index, x, z);
      clear_error();
      set_status("mapping_survey_skipped_blocked_waypoint");
      return;
    }
    set_status("blocked_by_dynamic_zone");
    set_error("Current waypoint is blocked by a dynamic zone");
    distance_to_target = hypot2(target.x - x, target.z - z);
    stop_robot();
    return;
  }

  const int segment_skip_zone =
      route_source_mapping_survey ? mapping_survey_room_zone_index : -1;
  if (segment_blocked_by_zones(x, z, target.x, target.z, ZONE_CLEARANCE, segment_skip_zone)) {
    if (route_source_mapping_survey && current_waypoint_index + 1 < route_data.count) {
      ++current_waypoint_index;
      begin_navigation_for_waypoint(current_waypoint_index, x, z);
      clear_error();
      set_status("mapping_survey_skipped_blocked_segment");
      return;
    }
    set_status("blocked_by_dynamic_zone");
    set_error("Dynamic zone blocks the current segment");
    distance_to_target = hypot2(target.x - x, target.z - z);
    stop_robot();
    return;
  }

  const double dx = target.x - x;
  const double dz = target.z - z;
  const double target_distance_now = hypot2(dx, dz);
  const double heading_to_target = atan2(dz, dx);
  const double heading_error_to_target = wrap_angle(heading_to_target - heading);

  LidarObstacleContext lidar_context;
  compute_lidar_obstacle_context(
      &lidar_context, -heading_error_to_target, avoidance_state.turn_sign);
  const int avoidance_was_active = avoidance_state.active;
  const double camera_range_for_navigation =
      camera_obstacle_range > EPS ? camera_obstacle_range : CAMERA_RANGE_FALLBACK_M;
  const int camera_visual_front_obstacle =
      camera_obstacle_visible &&
      fabs(camera_obstacle_angle) < fmax(camera_fov, 0.8) * 0.46 &&
      camera_range_for_navigation < LIDAR_TRACK_CAUTION_RANGE + 0.18 &&
      (camera_obstacle_score >= CAMERA_OBSTACLE_MIN_SCORE ||
       camera_detection_count >= 3);
  const double camera_preferred_turn_sign =
      camera_visual_front_obstacle &&
              fabs(camera_obstacle_center_offset) > CAMERA_OBSTACLE_OFFSET_DEADBAND
          ? sign_or_one(camera_obstacle_center_offset)
          : 0.0;
  const ControllerAvoidanceDetectionConfig avoidance_detection_config = {
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
  };
  ControllerAvoidanceDetection avoidance_detection;
  controller_avoidance_detect(
      &lidar_context,
      lidar_available,
      camera_visual_front_obstacle,
      &avoidance_detection_config,
      &avoidance_detection);
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
          fabs(lidar_priority_turn_sign) > 0.0 ? lidar_priority_turn_sign : sign_or_one(heading_error_to_target))) {
    stop_robot();
    return;
  }

  if (!avoidance_state.active && should_start_avoidance) {
    const double left_clearance = fmax(left_obstacle_range, left_front_corner_range);
    const double right_clearance = fmax(right_obstacle_range, right_front_corner_range);
    const ControllerAvoidanceTurnInput turn_input = {
        .left_score = lidar_context.unexpected_left_score,
        .right_score = lidar_context.unexpected_right_score,
        .left_clearance = left_clearance,
        .right_clearance = right_clearance,
        .switch_margin = LIDAR_REFLEX_SWITCH_MARGIN,
        .priority_turn_sign = lidar_priority_turn_sign,
        .camera_turn_sign = camera_preferred_turn_sign,
        .heading_error = heading_error_to_target,
    };
    const double selected_turn_sign = controller_avoidance_choose_turn_sign(&turn_input);
    controller_avoidance_state_begin(
        &avoidance_state,
        x,
        z,
        heading,
        target_distance_now,
        selected_turn_sign,
        LIDAR_AVOID_MIN_CONTOUR_STEPS);
    lidar_priority_turn_sign = avoidance_state.turn_sign;
    lidar_priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS;
    const ControllerAvoidanceDetourConfig detour_config = {
        .forward_distance = LIDAR_DETOUR_FORWARD_M,
        .max_distance = LIDAR_DETOUR_MAX_RANGE_M,
        .side_distance = LIDAR_DETOUR_SIDE_M,
    };
    controller_avoidance_set_detour(
        &avoidance_state,
        x,
        z,
        heading,
        &lidar_context,
        avoidance_state.turn_sign,
        &detour_config);
  }

  if (avoidance_state.active) {
    const double detour_dx = avoidance_state.detour_x - x;
    const double detour_dz = avoidance_state.detour_z - z;
    const double detour_distance =
        avoidance_state.detour_active ? hypot2(detour_dx, detour_dz) : 0.0;
    const double detour_heading_error =
        avoidance_state.detour_active
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
    controller_avoidance_update_progress(
        &avoidance_state,
        &progress_input,
        &progress_config);

    const int mapping_survey_can_rejoin_route =
        route_source_mapping_survey &&
        avoidance_state.hold_steps <= 0 &&
        avoidance_state.contour_steps >= LIDAR_AVOID_MIN_CONTOUR_STEPS &&
        (avoidance_state.hit_target_distance - target_distance_now) > LIDAR_AVOID_LEAVE_PROGRESS &&
        fabs(heading_error_to_target) < LIDAR_AVOID_LEAVE_HEADING_RAD &&
        near_front_range > (LIDAR_AVOID_STOP_RANGE + 0.10) &&
        center_obstacle_range > (LIDAR_AVOID_STOP_RANGE + 0.14);

    if (mapping_survey_can_rejoin_route) {
      clear_local_navigation_state();
      navigation_waypoint_index = current_waypoint_index;
      navigation_segment_start_x = x;
      navigation_segment_start_z = z;
      navigation_mode = NAV_MODE_TRACK;
      distance_to_target = target_distance_now;
      clear_error();
      set_status("mapping_survey_rejoined_route");
    }

    const int mapping_survey_orbiting_same_object =
        route_source_mapping_survey &&
        avoidance_state.heading_accum_rad > MAPPING_SURVEY_AVOID_ORBIT_HEADING_RAD &&
        avoidance_state.no_progress_steps > MAPPING_SURVEY_AVOID_NO_PROGRESS_STEPS / 3;

    if (route_source_mapping_survey &&
        mapping_survey_replan_cooldown_steps <= 0 &&
        (avoidance_state.contour_steps > MAPPING_SURVEY_AVOID_REPLAN_STEPS ||
         mapping_survey_orbiting_same_object)) {
      const double distance_from_avoidance_start =
          hypot2(x - avoidance_state.start_x, z - avoidance_state.start_z);
      const int looped_near_entry =
          distance_from_avoidance_start < MAPPING_SURVEY_AVOID_LOOP_RADIUS &&
          avoidance_state.no_progress_steps > MAPPING_SURVEY_AVOID_NO_PROGRESS_STEPS / 2;
      const int lost_progress =
          avoidance_state.no_progress_steps > MAPPING_SURVEY_AVOID_NO_PROGRESS_STEPS;
      const int avoidance_timed_out =
          avoidance_state.contour_steps > MAPPING_SURVEY_AVOID_MAX_STEPS;

      if (looped_near_entry ||
          lost_progress ||
          avoidance_timed_out ||
          mapping_survey_orbiting_same_object) {
        if (escape_mapping_survey_orbit(x, z)) {
          distance_to_target = 0.0;
          stop_robot();
          return;
        }

        if (current_waypoint_index + 1 < route_data.count) {
          ++current_waypoint_index;
          clear_local_navigation_state();
          begin_navigation_for_waypoint(current_waypoint_index, x, z);
          mapping_survey_replan_cooldown_steps = MAPPING_SURVEY_REPLAN_COOLDOWN_STEPS;
          distance_to_target = hypot2(route_data.waypoints[current_waypoint_index].x - x,
                                      route_data.waypoints[current_waypoint_index].z - z);
          clear_error();
          set_status("mapping_survey_skipped_looped_obstacle");
          stop_robot();
          return;
        }
      }
    }

    if (avoidance_state.no_obstacle_steps > FREE_SPACE_RECOVERY_STEPS &&
        avoidance_state.hold_steps <= 0 &&
        avoidance_state.clear_steps >= LIDAR_AVOID_CLEAR_STEPS &&
        !avoidance_state.detour_active) {
      clear_local_navigation_state();
      navigation_waypoint_index = current_waypoint_index;
      navigation_segment_start_x = x;
      navigation_segment_start_z = z;
      navigation_mode = NAV_MODE_TRACK;
      distance_to_target = target_distance_now;
      clear_error();
      set_status("reacquired_free_space");
    }

    if (avoidance_state.active) {
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
          .turn_sign = avoidance_state.turn_sign,
          .stuck_steps = avoidance_state.stuck_steps,
          .detour_active = avoidance_state.detour_active,
          .detour_heading_error = detour_heading_error,
          .heading_error_to_target = heading_error_to_target,
          .runtime_linear_speed_limit = runtime_linear_speed_limit,
          .runtime_angular_speed_limit = runtime_angular_speed_limit,
          .pass_min_speed = scaled_linear_floor(0.35),
          .pass_max_speed = scaled_linear_cap(LIDAR_PASS_MAX_SPEED_FACTOR),
          .avoid_min_speed = scaled_linear_floor(0.30),
          .avoid_max_speed = scaled_linear_cap(LIDAR_AVOID_DRIVE_MAX_SPEED_FACTOR),
          .reverse_speed = scaled_linear_cap(0.16),
      };
      ControllerAvoidanceCommand command;
      controller_avoidance_compute_command(&command_input, &command_config, &command);

      if (command.clear_detour) avoidance_state.detour_active = 0;
      if (command.clear_hold) avoidance_state.hold_steps = 0;
      if (command.turn_sign_changed) {
        avoidance_state.turn_sign = command.turn_sign;
        avoidance_state.obstacle_side = avoidance_state.turn_sign > 0.0 ? -1 : 1;
        lidar_priority_turn_sign = avoidance_state.turn_sign;
        lidar_priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS;
      }
      if (command.reset_stuck) {
        avoidance_state.hold_steps = LIDAR_AVOID_HOLD_STEPS;
        avoidance_state.stuck_steps = 0;
      }

      if (command.mode == CONTROLLER_AVOIDANCE_COMMAND_PASS_GAP) {
        set_status("passing_lidar_gap");
      } else if (command.mode == CONTROLLER_AVOIDANCE_COMMAND_HARD_TURN) {
        set_status(lidar_context.has_best_gap ? "avoiding_gap_turn" : "avoiding_committed_turn");
      } else if (command.mode == CONTROLLER_AVOIDANCE_COMMAND_GAP_DRIVE) {
        set_status("avoiding_gap_drive");
      } else if (command.mode == CONTROLLER_AVOIDANCE_COMMAND_COMMITTED_DRIVE) {
        set_status("avoiding_committed_drive");
      } else {
        set_status("avoiding_committed_escape");
      }

      distance_to_target = target_distance_now;
      clear_error();
      set_base_velocity(command.linear_speed, 0.0, command.angular_speed);
      return;
    }
  }

  avoidance_state.active = 0;
  avoidance_state.mode = AVOID_MODE_NONE;
  avoidance_state.obstacle_side = 0;
  avoidance_state.release_steps = 0;
  avoidance_state.contour_steps = 0;
  avoidance_state.clear_steps = 0;
  avoidance_state.escape_steps = 0;
  avoidance_state.stuck_steps = 0;
  avoidance_state.hit_target_distance = 0.0;
  avoidance_state.state_heading = 0.0;
  avoidance_state.start_x = START_X;
  avoidance_state.start_z = START_Z;
  avoidance_state.best_target_distance = 0.0;
  avoidance_state.no_progress_steps = 0;
  avoidance_state.prev_heading = 0.0;
  avoidance_state.heading_accum_rad = 0.0;
  avoidance_state.detour_active = 0;
  avoidance_state.detour_x = START_X;
  avoidance_state.detour_z = START_Z;
  if (avoidance_was_active) {
    navigation_segment_start_x = x;
    navigation_segment_start_z = z;
    navigation_mode = NAV_MODE_TRACK;
  }
  distance_to_target = target_distance_now;
  navigation_segment_start_x = x;
  navigation_segment_start_z = z;
  const int route_relaxed_mode_base = 1;

  if (navigation_mode == NAV_MODE_IDLE) {
    navigation_mode = NAV_MODE_TRACK;
  }

  if (is_final_waypoint && target.has_heading && distance_to_target <= FINAL_ALIGN_DISTANCE) {
    navigation_mode = NAV_MODE_FINAL_ALIGN;
  } else if (!is_final_waypoint && navigation_mode == NAV_MODE_FINAL_ALIGN) {
    navigation_mode = NAV_MODE_TRACK;
  }

  double linear_speed = 0.0;
  double angular_speed = 0.0;

  if (navigation_mode == NAV_MODE_FINAL_ALIGN) {
    if (!is_final_waypoint || !target.has_heading) {
      navigation_mode = NAV_MODE_TRACK;
    } else if (distance_to_target > FINAL_ALIGN_DISTANCE * 1.35) {
      navigation_mode = NAV_MODE_TURN;
    } else {
      const double heading_error = wrap_angle(target.heading_rad - heading);
      angular_speed = clamp_value(
          heading_error * FINAL_ALIGN_GAIN,
          -runtime_angular_speed_limit,
          runtime_angular_speed_limit);
      if (fabs(heading_error) > HEADING_TOLERANCE_RAD && fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
        angular_speed = sign_or_one(heading_error) * MIN_ANGULAR_COMMAND;
      }
      clear_error();
      set_status("aligning_final_heading");
      apply_kinematic_step(x, z, heading, 0.0, angular_speed);
      return;
    }
  }

  const double updated_track_heading = heading_to_target;
  const double direct_heading_error = wrap_angle(heading_to_target - heading);
  double updated_heading_error = wrap_angle(updated_track_heading - heading);
  double lidar_heading_bias = 0.0;
  double lidar_speed_scale = 1.0;
  double expected_wall_speed_scale = 1.0;
  int lidar_caution_active = 0;
  int lidar_hard_priority = 0;

  if (lidar_available) {
    const double tracking_front_range =
        center_passage_available ? center_obstacle_range : near_front_range;
    const double front_caution = clamp_value(
        (LIDAR_TRACK_CAUTION_RANGE - tracking_front_range) /
            fmax(LIDAR_TRACK_CAUTION_RANGE - LIDAR_AVOID_STOP_RANGE, 0.05),
        0.0,
        1.0);
    const double left_pressure = clamp_value(
        (LIDAR_TRACK_SIDE_BIAS_RANGE - left_lidar_context) /
            fmax(LIDAR_TRACK_SIDE_BIAS_RANGE - LIDAR_AVOID_SIDE_DANGER_RANGE, 0.05),
        0.0,
        1.0);
    const double right_pressure = clamp_value(
        (LIDAR_TRACK_SIDE_BIAS_RANGE - right_lidar_context) /
            fmax(LIDAR_TRACK_SIDE_BIAS_RANGE - LIDAR_AVOID_SIDE_DANGER_RANGE, 0.05),
        0.0,
        1.0);
    const double asymmetric_pressure = right_pressure - left_pressure;
    double desired_turn_sign = 0.0;

    lidar_speed_scale = clamp_value(
        1.0 -
            front_caution * (center_passage_available ? 0.28 : 0.72) -
            fmax(left_pressure, right_pressure) * (center_passage_available ? 0.08 : 0.20),
        center_passage_available ? 0.52 : 0.16,
        1.0);
    lidar_hard_priority =
        tracking_front_range < LIDAR_TRACK_HARD_PRIORITY_RANGE ||
        (!center_passage_available &&
         (left_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10) ||
          right_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10)));
    lidar_caution_active =
        tracking_front_range < LIDAR_TRACK_CAUTION_RANGE ||
        left_lidar_context < LIDAR_TRACK_SIDE_BIAS_RANGE ||
        right_lidar_context < LIDAR_TRACK_SIDE_BIAS_RANGE;

    if (center_passage_available) {
      const double gap_heading_error = wrap_angle(-lidar_context.best_gap_beam_angle);
      lidar_heading_bias = clamp_value(
          gap_heading_error * 0.82 + asymmetric_pressure * 0.24,
          -LIDAR_TRACK_MAX_HEADING_BIAS,
          LIDAR_TRACK_MAX_HEADING_BIAS);
      lidar_priority_turn_sign = 0.0;
      lidar_priority_hold_steps = 0;
    } else if (fabs(asymmetric_pressure) > LIDAR_PRIORITY_SWITCH_MARGIN) {
      desired_turn_sign = sign_or_one(asymmetric_pressure);
    } else if (lidar_hard_priority) {
      if (fabs(heading_error_to_target) > 0.12) {
        desired_turn_sign = sign_or_one(heading_error_to_target);
      } else {
        desired_turn_sign = left_lidar_context <= right_lidar_context ? -1.0 : 1.0;
      }
    }

    if (!center_passage_available && lidar_caution_active) {
      if (desired_turn_sign != 0.0) {
        if (lidar_priority_turn_sign == 0.0 ||
            desired_turn_sign == lidar_priority_turn_sign ||
            fabs(asymmetric_pressure) > (LIDAR_PRIORITY_SWITCH_MARGIN + 0.10) ||
            lidar_priority_hold_steps <= 0) {
          lidar_priority_turn_sign = desired_turn_sign;
          lidar_priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS;
        }
      } else if (lidar_priority_hold_steps <= 0) {
        lidar_priority_turn_sign = 0.0;
      }

      if (lidar_priority_hold_steps > 0) {
        lidar_priority_hold_steps -= 1;
      }

      if (fabs(lidar_priority_turn_sign) > 0.0) {
        const double bias_magnitude = clamp_value(
            (lidar_hard_priority ? 0.34 : 0.18) +
                front_caution * 0.22 +
                fmax(left_pressure, right_pressure) * 0.14,
            0.0,
            LIDAR_TRACK_MAX_HEADING_BIAS);
        lidar_heading_bias = lidar_priority_turn_sign * bias_magnitude;
      } else if (fabs(asymmetric_pressure) > LIDAR_PRIORITY_CENTER_MARGIN) {
        lidar_heading_bias = clamp_value(
            asymmetric_pressure * (0.22 + 0.34 * front_caution),
            -LIDAR_TRACK_MAX_HEADING_BIAS * 0.6,
            LIDAR_TRACK_MAX_HEADING_BIAS * 0.6);
      }
    } else {
      lidar_priority_turn_sign = 0.0;
      lidar_priority_hold_steps = 0;
    }

    if (camera_visual_front_obstacle && lidar_caution_active && !center_passage_available) {
      lidar_heading_bias = clamp_value(
          lidar_heading_bias + camera_obstacle_center_offset * 0.24,
          -LIDAR_TRACK_MAX_HEADING_BIAS,
          LIDAR_TRACK_MAX_HEADING_BIAS);
      lidar_speed_scale = fmin(lidar_speed_scale, 0.78);
    }

    updated_heading_error = wrap_angle(
        (lidar_hard_priority && !center_passage_available ? direct_heading_error : updated_heading_error) +
        lidar_heading_bias);
    if (lidar_hard_priority) {
      lidar_speed_scale = fmin(lidar_speed_scale, center_passage_available ? 0.82 : 0.58);
    }
  }

  if (!lidar_caution_active && expected_zone_wall_slowdown) {
    expected_wall_speed_scale = clamp_value(
        (expected_front_range - EXPECTED_WALL_SOFT_STOP_RANGE) /
            fmax(EXPECTED_WALL_SLOWDOWN_RANGE - EXPECTED_WALL_SOFT_STOP_RANGE, 0.05),
        0.32,
        1.0);
    if (expected_zone_wall_close && fabs(heading_error_to_target) < 0.24) {
      expected_wall_speed_scale = fmin(expected_wall_speed_scale, 0.26);
    }
  }

  const int route_relaxed_mode = route_relaxed_mode_base || lidar_caution_active;
  if (route_relaxed_mode && navigation_mode == NAV_MODE_TURN) {
    navigation_mode = NAV_MODE_TRACK;
  }

  if (navigation_mode == NAV_MODE_TURN) {
    const double preferred_turn_error =
        (!avoidance_state.active && lidar_hard_priority && fabs(lidar_priority_turn_sign) > 0.0)
            ? wrap_angle(heading_error_to_target + lidar_heading_bias)
            : updated_heading_error;
    if (fabs(preferred_turn_error) <= TURN_EXIT_ERROR_RAD ||
        distance_to_target <= TRACK_SLOW_RADIUS * 0.5) {
      navigation_mode = NAV_MODE_TRACK;
    } else {
      angular_speed = clamp_value(
          preferred_turn_error * TURN_HEADING_GAIN,
          -runtime_angular_speed_limit,
          runtime_angular_speed_limit);
      if (fabs(preferred_turn_error) > HEADING_TOLERANCE_RAD &&
          fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
        angular_speed = sign_or_one(preferred_turn_error) * MIN_ANGULAR_COMMAND;
      }
      clear_error();
      set_status(
          (!avoidance_state.active && lidar_hard_priority && fabs(lidar_priority_turn_sign) > 0.0)
              ? "turning_lidar_priority"
              : "turning_to_path");
      apply_kinematic_step(x, z, heading, 0.0, angular_speed);
      return;
    }
  }

  if (!route_relaxed_mode &&
      fabs(updated_heading_error) > TRACK_REENTER_TURN_RAD &&
      distance_to_target > TRACK_SLOW_RADIUS) {
    navigation_mode = NAV_MODE_TURN;
    angular_speed = clamp_value(
        updated_heading_error * TURN_HEADING_GAIN,
        -runtime_angular_speed_limit,
        runtime_angular_speed_limit);
    if (fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
      angular_speed = sign_or_one(updated_heading_error) * MIN_ANGULAR_COMMAND;
    }
    clear_error();
    set_status("turning_to_path");
    apply_kinematic_step(x, z, heading, 0.0, angular_speed);
    return;
  }

  angular_speed = clamp_value(
      updated_heading_error * TRACK_HEADING_GAIN,
      -runtime_angular_speed_limit,
      runtime_angular_speed_limit);

  double base_speed = clamp_value(distance_to_target * 1.18, TRACK_MIN_LINEAR_SPEED, runtime_linear_speed_limit);
  if (distance_to_target < TRACK_SLOW_RADIUS) {
    const double near_target_speed_cap = clamp_value(
        runtime_linear_speed_limit * 0.36,
        0.14,
        0.24);
    base_speed = clamp_value(distance_to_target * 1.05, TRACK_MIN_LINEAR_SPEED, near_target_speed_cap);
  }
  base_speed *= lidar_speed_scale;
  base_speed *= expected_wall_speed_scale;

  const int clear_fast_tracking = !lidar_caution_active && !expected_zone_wall_slowdown;
  double heading_scale = clear_fast_tracking
                             ? clamp_value(1.0 - fabs(updated_heading_error) / 1.15, 0.45, 1.0)
                             : clamp_value(1.0 - fabs(updated_heading_error) / 0.85, 0.22, 1.0);
  if (fabs(updated_heading_error) > TURN_ENTER_ERROR_RAD) {
    heading_scale = fmin(heading_scale, clear_fast_tracking ? 0.62 : 0.35);
  }

  linear_speed = base_speed * heading_scale;
  if (distance_to_target <= POSITION_TOLERANCE * 1.4) {
    linear_speed = fmin(linear_speed, 0.05);
  }

  clear_error();
  set_status(lidar_caution_active ? "tracking_lidar_priority"
                                  : (expected_zone_wall_ahead ? "tracking_planned_zone_bypass"
                                                              : "tracking_path"));
  apply_kinematic_step(x, z, heading, linear_speed, angular_speed);
}

static void write_state_snapshot(void) {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  const double simulation_now = wb_robot_get_time();
  read_pose(&x, &y, &heading);

  ControllerTelemetryPoint trace_points[MAX_OBSTACLE_TRACE_POINTS];
  int trace_point_count = 0;
  for (int i = 0; i < obstacle_trace_count; ++i) {
    const double confidence = controller_lidar_trace_confidence(
        &obstacle_trace[i], simulation_now, LIDAR_TRACE_TTL_SECONDS);
    if (confidence <= LIDAR_TRACE_MIN_CONFIDENCE) continue;
    trace_points[trace_point_count++] = (ControllerTelemetryPoint){
        obstacle_trace[i].x,
        obstacle_trace[i].y,
        confidence,
    };
  }

  ControllerTelemetrySnapshot snapshot = {0};
  snapshot.simulation_time = simulation_now;
  snapshot.pose_x = x;
  snapshot.pose_y = y;
  snapshot.pose_z = START_HEIGHT;
  snapshot.pose_yaw = heading;
  snapshot.navigation = (ControllerTelemetryNavigation){
      navigation_status,
      navigation_error,
      current_waypoint_index,
      route_finished,
      distance_to_target,
      avoidance_state.active,
      route_off_route_active_now(),
      route_avoidance_time_sec,
      route_avoidance_steps,
      route_data.count > 0 && current_waypoint_index < route_data.count,
      route_data.count > 0 && current_waypoint_index < route_data.count
          ? route_data.waypoints[current_waypoint_index]
          : (Waypoint){0},
  };
  snapshot.motion_profile = (ControllerTelemetryMotionProfile){
      configured_cruise_speed_mps,
      configured_payload_kg,
      configured_battery_range_units,
      runtime_battery_speed_factor,
      runtime_linear_speed_limit,
      runtime_angular_speed_limit,
  };
  snapshot.dynamic_zone_count = zone_data.count;
  snapshot.dynamic_zone_wall_count = zone_node_count;
  snapshot.lidar = (ControllerTelemetryLidar){
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
  snapshot.camera = (ControllerTelemetryCamera){
      camera_available,
      camera_width,
      camera_height,
      camera_fov,
      camera_virtual_mode ? "virtual_lidar" : "webots_camera",
      camera_frame_file,
      camera_frame_mime,
      camera_frame_sequence,
      camera_frame_time,
      camera_obstacle_visible,
      camera_obstacle_score,
      camera_obstacle_center_offset,
      camera_obstacle_angle,
      camera_obstacle_range,
      camera_detection_count,
  };
  snapshot.trace_points = trace_points;
  snapshot.trace_point_count = trace_point_count;
  snapshot.obstacle_map_cell_count = persistent_map_count;
  snapshot.obstacle_map_cell_size = MAP_CELL_SIZE;
  snapshot.camera_map_cell_count = camera_map_count + camera_free_map_count;
  snapshot.camera_map_obstacle_cell_count = camera_map_count;
  snapshot.camera_map_free_cell_count = camera_free_map_count;
  snapshot.camera_map_cell_size = CAMERA_MAP_CELL_SIZE;
  snapshot.route_waypoints = route_data.waypoints;
  snapshot.route_waypoint_count = route_data.count;

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
    maybe_write_camera_map,
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
  init_wheels();
  init_manipulator_pose();
  init_lidar();
  init_camera();
  init_pose_tracking();
  reset_robot_pose();
  clear_persistent_map();
  remove(CAMERA_FRAME_BMP_PATH);
  remove(CAMERA_FRAME_BMP_TEMP_PATH);
  remove(CAMERA_FRAME_JPEG_PATH);
  remove(CAMERA_FRAME_JPEG_TEMP_PATH);
  apply_motion_profile();
  motion_profile_last_modified = get_file_mtime(MOTION_PROFILE_PATH);
  if (motion_profile_last_modified >= 0) {
    load_motion_profile();
  }
  maybe_reload_zones();
  maybe_reload_surface_zones();
  runtime_command_last_modified = get_file_mtime(RUNTIME_COMMAND_PATH);

  if (!translation_field || !rotation_field || !root_children_field) {
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
  maybe_write_camera_map();
  remove_runtime_obstacle_nodes();
  remove_surface_zone_nodes();
  remove_zone_nodes();
  wb_robot_cleanup();
  return 0;
}
