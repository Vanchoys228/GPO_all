#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <sys/stat.h>
#include <windows.h>
#define STAT_STRUCT struct _stat64
#define STAT_FN _stat64
#else
#include <sys/stat.h>
#define STAT_STRUCT struct stat
#define STAT_FN stat
#endif

#define TIME_STEP 16
#define PI 3.14159265358979323846
#define KINEMATIC_LINEAR_SPEED 0.60
#define KINEMATIC_ANGULAR_SPEED 1.6
#define DEFAULT_CRUISE_SPEED_MPS 0.22
#define MIN_CRUISE_SPEED_MPS 0.05
#define MAX_CRUISE_SPEED_MPS 0.8
#define DEFAULT_PAYLOAD_KG 0.0
#define MAX_PAYLOAD_KG 500.0
#define DEFAULT_BATTERY_RANGE_UNITS 100.0
#define MIN_BATTERY_RANGE_UNITS 1.0
#define MAX_BATTERY_RANGE_UNITS 100000.0
#define POSITION_TOLERANCE 0.05
#define HEADING_TOLERANCE_RAD 0.08
#define FINAL_ALIGN_DISTANCE 0.18
#define TRACK_SLOW_RADIUS 0.28
#define TURN_ENTER_ERROR_RAD 0.42
#define TURN_EXIT_ERROR_RAD 0.12
#define TRACK_REENTER_TURN_RAD 0.56
#define TURN_HEADING_GAIN 3.8
#define TRACK_HEADING_GAIN 3.0
#define FINAL_ALIGN_GAIN 3.4
#define TRACK_CROSS_TRACK_GAIN 0.22
#define TRACK_LOOKAHEAD_MIN 0.16
#define TRACK_LOOKAHEAD_MAX 0.45
#define TRACK_MIN_LINEAR_SPEED 0.035
#define TRACK_DIRECT_HEADING_CROSSTRACK 0.10
#define TRACK_REANCHOR_CROSSTRACK 0.12
#define TRACK_RELAXED_MODE_CROSSTRACK 0.035
#define MIN_ANGULAR_COMMAND 0.28
#define START_X 0.0
#define START_Z 0.0
#define START_HEIGHT 0.102838
#define MAX_WAYPOINTS 768
#define ROUTE_RELOAD_INTERVAL 20
#define MOTION_RELOAD_INTERVAL 20
#define ZONE_RELOAD_INTERVAL 10
#define RUNTIME_COMMAND_RELOAD_INTERVAL 6
#define MAX_ZONES 32
#define MAX_ZONE_POINTS 48
#define MAX_ZONE_NODES 512
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
#define LIDAR_AVOID_TRIGGER_RANGE 1.02
#define LIDAR_AVOID_STOP_RANGE 0.27
#define LIDAR_AVOID_RECOVER_RANGE 1.18
#define LIDAR_AVOID_REVERSE_RANGE 0.17
#define LIDAR_AVOID_HOLD_STEPS 28
#define LIDAR_AVOID_SIDE_TRIGGER_RANGE 0.42
#define LIDAR_AVOID_SIDE_DANGER_RANGE 0.19
#define LIDAR_AVOID_FOLLOW_RANGE 0.74
#define LIDAR_AVOID_FOLLOW_TARGET 0.29
#define LIDAR_AVOID_RELEASE_STEPS 24
#define LIDAR_AVOID_MIN_CONTOUR_STEPS 18
#define LIDAR_AVOID_CLEAR_STEPS 8
#define LIDAR_AVOID_LEAVE_PROGRESS 0.16
#define LIDAR_AVOID_LEAVE_HEADING_RAD 0.44
#define LIDAR_AVOID_TARGET_CLEAR_RANGE 1.16
#define LIDAR_AVOID_ESCAPE_STEPS 16
#define LIDAR_AVOID_STUCK_STEPS 24
#define LIDAR_AVOID_STUCK_POSE_EPS 0.004
#define LIDAR_AVOID_STUCK_PROGRESS_EPS 0.001
#define LIDAR_TRACK_CAUTION_RANGE 1.45
#define LIDAR_TRACK_HARD_PRIORITY_RANGE 1.12
#define LIDAR_TRACK_SLOW_RANGE 1.00
#define LIDAR_TRACK_SIDE_BIAS_RANGE 0.90
#define LIDAR_TRACK_MAX_HEADING_BIAS 0.52
#define LIDAR_PRIORITY_HOLD_STEPS 18
#define LIDAR_PRIORITY_SWITCH_MARGIN 0.18
#define LIDAR_PRIORITY_CENTER_MARGIN 0.08
#define POSE_RELOCATION_DISTANCE 0.45
#define POSE_RELOCATION_HEADING_RAD 1.20
#define FREE_SPACE_RECOVERY_STEPS 10
#define ZONE_WALL_EXPECTED_TOLERANCE 0.18
#define EXPECTED_WALL_SOFT_STOP_RANGE 0.24
#define EXPECTED_WALL_SLOWDOWN_RANGE 0.46
#define LIDAR_REFLEX_SIDE_RELEASE_RANGE 0.90
#define LIDAR_REFLEX_SWITCH_MARGIN 0.18
#define LIDAR_REFLEX_TARGET_GAIN 0.16
#define LIDAR_REFLEX_MAX_LINEAR_SPEED 0.18
#define LIDAR_GAP_MIN_RANGE 0.34
#define LIDAR_GAP_SWITCH_RANGE_BONUS 0.18
#define WHEEL_RADIUS 0.05
#define WHEEL_BASE_LONGITUDINAL 0.228
#define WHEEL_BASE_LATERAL 0.158
#define MAX_WHEEL_SPEED_RAD_S 12.0
#define WHEEL_ACCEL_LIMIT_RAD_S2 90.0
#define WHEEL_DECEL_LIMIT_RAD_S2 140.0
#define MAX_MAP_POINTS 4096
#define MAP_CELL_SIZE 0.06
#define MAP_MERGE_MIN_HIT_COUNT 2
#define MAP_MERGE_MAX_AGE_S 1.0
#define MAP_WRITE_INTERVAL 60
#define SURVEY_X_MIN -8.0
#define SURVEY_X_MAX 8.0
#define SURVEY_Y_MIN -8.0
#define SURVEY_Y_MAX 8.0
#define SURVEY_STRIP 1.2
#define MAPPING_SURVEY_GRID_CELL 0.25
#define MAPPING_SURVEY_MAX_GRID_CELLS 36000
#define MAPPING_SURVEY_MAX_BOUNDARY_POINTS 4096
#define MAPPING_SURVEY_CONTOUR_OFFSET 0.45
#define MAPPING_SURVEY_INTERIOR_OFFSET 0.70
#define MAPPING_SURVEY_STRIP 1.10
#define MAPPING_SURVEY_MIN_CONTOUR_STEP 1.15
#define MAPPING_SURVEY_MAX_CONTOUR_STEP 1.45
#define MAPPING_SURVEY_MIN_STRIP_LENGTH 0.65
#define MAPPING_SURVEY_RDP_EPS 0.34
#define MAPPING_SURVEY_MAP_OBSTACLE_CLEARANCE 0.45
#define MAPPING_SURVEY_MAX_EXTENT_X 21.5
#define MAPPING_SURVEY_MAX_EXTENT_Y 16.5
#define EPS 1e-9

typedef struct {
  double x;
  // Legacy field name: this stores the world-plane Y coordinate.
  double z;
  double heading_rad;
  int has_heading;
} Waypoint;

typedef struct {
  Waypoint waypoints[MAX_WAYPOINTS];
  int count;
  long long last_modified;
} RouteData;

typedef enum {
  NAV_MODE_IDLE = 0,
  NAV_MODE_TURN = 1,
  NAV_MODE_TRACK = 2,
  NAV_MODE_FINAL_ALIGN = 3,
} NavigationMode;

typedef enum {
  AVOID_MODE_NONE = 0,
  AVOID_MODE_FACE_CLEAR = 1,
  AVOID_MODE_FOLLOW_EDGE = 2,
  AVOID_MODE_SEARCH_EDGE = 3,
  AVOID_MODE_RECOVER_TARGET = 4,
  AVOID_MODE_ESCAPE = 5,
} AvoidanceMode;

typedef struct {
  char id[64];
  int point_count;
  struct {
    double x;
    double y;
  } points[MAX_ZONE_POINTS];
} LimitZone;

typedef struct {
  LimitZone zones[MAX_ZONES];
  int count;
} ZoneData;

typedef struct {
  double x;
  double y;
  double last_seen_time;
  int hit_count;
} ObstacleTracePoint;

typedef struct {
  long long id;
  int has_spawn_obstacle;
  int has_start_mapping_survey;
  int clear_map;
  double x;
  double y;
  double size_x;
  double size_y;
  double height;
} RuntimeCommand;

typedef struct {
  double x;
  double y;
  int confidence;
} MapCell;

typedef struct {
  double x;
  double y;
} SurveyPoint;

typedef struct {
  double start;
  double end;
} SurveyInterval;

typedef struct {
  double min_x;
  double min_y;
  double cell;
  int width;
  int height;
  int count;
  unsigned char free_cell[MAPPING_SURVEY_MAX_GRID_CELLS];
  unsigned char component_cell[MAPPING_SURVEY_MAX_GRID_CELLS];
  unsigned char visited_cell[MAPPING_SURVEY_MAX_GRID_CELLS];
  int parent[MAPPING_SURVEY_MAX_GRID_CELLS];
  int queue[MAPPING_SURVEY_MAX_GRID_CELLS];
} SurveyGrid;

typedef struct {
  double expected_front_min_range;
  double unexpected_front_min_range;
  double unexpected_center_min_range;
  double unexpected_left_front_min_range;
  double unexpected_right_front_min_range;
  double unexpected_left_min_range;
  double unexpected_right_min_range;
  double expected_front_score;
  double unexpected_front_score;
  double unexpected_left_score;
  double unexpected_right_score;
  double best_gap_beam_angle;
  double best_gap_range;
  double best_gap_score;
  int has_best_gap;
  int unexpected_front_hit_count;
} LidarObstacleContext;

static WbDeviceTag wheels[4];
static WbDeviceTag arm_joints[5];
static WbDeviceTag gripper_fingers[2];
static WbDeviceTag front_lidar = 0;
static WbNodeRef self_node;
static WbFieldRef translation_field;
static WbFieldRef rotation_field;
static WbFieldRef root_children_field;
static double applied_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

static RouteData route_data = {0};
static ZoneData zone_data = {0};
static WbNodeRef zone_nodes[MAX_ZONE_NODES];
static char zone_node_defs[MAX_ZONE_NODES][64];
static int zone_node_count = 0;
static WbNodeRef runtime_obstacle_nodes[MAX_RUNTIME_OBSTACLE_NODES];
static char runtime_obstacle_defs[MAX_RUNTIME_OBSTACLE_NODES][64];
static int runtime_obstacle_count = 0;
static ObstacleTracePoint obstacle_trace[MAX_OBSTACLE_TRACE_POINTS];
static int obstacle_trace_count = 0;
static MapCell persistent_map[MAX_MAP_POINTS];
static int persistent_map_count = 0;
static int map_dirty = 0;
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
static AvoidanceMode avoidance_mode = AVOID_MODE_NONE;
static int avoidance_hold_steps = 0;
static double avoidance_turn_sign = 1.0;
static int avoidance_active = 0;
static int avoidance_obstacle_side = 0;
static int avoidance_release_steps = 0;
static int avoidance_contour_steps = 0;
static int avoidance_clear_steps = 0;
static int avoidance_escape_steps = 0;
static int avoidance_stuck_steps = 0;
static int avoidance_no_obstacle_steps = 0;
static double avoidance_prev_x = START_X;
static double avoidance_prev_z = START_Z;
static double avoidance_prev_target_distance = 0.0;
static double avoidance_hit_target_distance = 0.0;
static double avoidance_state_heading = 0.0;
static double last_pose_x = START_X;
static double last_pose_z = START_Z;
static double last_pose_heading = 0.0;
static int last_pose_valid = 0;
static long long motion_profile_last_modified = -1;
static long long runtime_command_last_modified = -1;
static long long last_processed_runtime_command_id = -1;
static int route_source_mapping_survey = 0;
static int mapping_survey_room_zone_index = -1;

static const char *ROUTE_PATH = "..\\..\\..\\web_state\\route.csv";
static const char *ZONE_PATH = "..\\..\\..\\web_state\\limit_zones.txt";
static const char *STATE_PATH = "..\\..\\..\\web_state\\robot_state.json";
static const char *STATE_TEMP_PATH = "..\\..\\..\\web_state\\robot_state.tmp";
static const char *MOTION_PROFILE_PATH = "..\\..\\..\\web_state\\motion_profile.txt";
static const char *RUNTIME_COMMAND_PATH = "..\\..\\..\\web_state\\runtime_command.txt";
static const char *MAP_PATH = "..\\..\\..\\web_state\\obstacle_map.json";
static const char *MAP_TEMP_PATH = "..\\..\\..\\web_state\\obstacle_map.tmp";
static const char *MAP_CSV_PATH = "..\\..\\..\\web_state\\obstacle_map.csv";
static const char *MAP_CSV_TEMP_PATH = "..\\..\\..\\web_state\\obstacle_map_csv.tmp";

static int point_near_zone(double x, double y, const LimitZone *zone, double clearance);
static int point_near_zone_boundary(double x, double y, const LimitZone *zone, double tolerance);
static int load_route(RouteData *route);
static void reset_navigation_mode();
static void set_status(const char *status);

static double clamp_value(double value, double min_value, double max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static double hypot2(double x, double y) {
  return sqrt(x * x + y * y);
}

static double dot2(double ax, double ay, double bx, double by) {
  return ax * bx + ay * by;
}

static double wrap_angle(double angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

static double blend_angle(double from_angle, double to_angle, double weight_to) {
  const double weight = clamp_value(weight_to, 0.0, 1.0);
  const double x = (1.0 - weight) * cos(from_angle) + weight * cos(to_angle);
  const double y = (1.0 - weight) * sin(from_angle) + weight * sin(to_angle);
  if (fabs(x) <= EPS && fabs(y) <= EPS) return to_angle;
  return atan2(y, x);
}

static long long get_file_mtime(const char *path) {
  STAT_STRUCT file_stat;
  if (STAT_FN(path, &file_stat) != 0) return -1;
#ifdef _WIN32
  return (long long)file_stat.st_mtime;
#else
  return (long long)file_stat.st_mtime;
#endif
}

static void apply_motion_profile() {
  configured_cruise_speed_mps = clamp_value(
      configured_cruise_speed_mps,
      MIN_CRUISE_SPEED_MPS,
      MAX_CRUISE_SPEED_MPS);
  configured_payload_kg = clamp_value(configured_payload_kg, 0.0, MAX_PAYLOAD_KG);
  configured_battery_range_units = clamp_value(
      configured_battery_range_units,
      MIN_BATTERY_RANGE_UNITS,
      MAX_BATTERY_RANGE_UNITS);

  // Payload emulation: the heavier the load, the lower achievable linear/turn speed.
  const double payload_factor =
      clamp_value(1.0 - configured_payload_kg * 0.0011, 0.55, 1.0);
  // Low battery mode: when remaining range is lower than baseline 100 units,
  // the controller becomes more conservative with speed to reduce energy usage.
  const double battery_ratio = configured_battery_range_units / DEFAULT_BATTERY_RANGE_UNITS;
  runtime_battery_speed_factor = clamp_value(battery_ratio, 0.6, 1.0);

  runtime_linear_speed_limit = clamp_value(
      configured_cruise_speed_mps,
      TRACK_MIN_LINEAR_SPEED,
      KINEMATIC_LINEAR_SPEED);
  runtime_angular_speed_limit = clamp_value(
      KINEMATIC_ANGULAR_SPEED *
          (0.88 + 0.12 * payload_factor),
      0.75,
      KINEMATIC_ANGULAR_SPEED);
}

static int load_motion_profile() {
  FILE *file = fopen(MOTION_PROFILE_PATH, "r");
  if (!file) return 1;

  double cruise_speed = configured_cruise_speed_mps;
  double payload_kg = configured_payload_kg;
  double battery_range = configured_battery_range_units;
  char key[64];
  double value = 0.0;

  while (fscanf(file, "%63s %lf", key, &value) == 2) {
    if (strcmp(key, "cruise_speed_mps") == 0) {
      cruise_speed = value;
    } else if (strcmp(key, "payload_kg") == 0) {
      payload_kg = value;
    } else if (strcmp(key, "battery_range") == 0) {
      battery_range = value;
    }
  }

  fclose(file);
  configured_cruise_speed_mps = cruise_speed;
  configured_payload_kg = payload_kg;
  configured_battery_range_units = battery_range;
  apply_motion_profile();
  return 1;
}

static void maybe_reload_motion_profile() {
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

static int replace_file(const char *from_path, const char *to_path) {
#ifdef _WIN32
  return MoveFileExA(from_path, to_path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) ? 0 : -1;
#else
  remove(to_path);
  return rename(from_path, to_path);
#endif
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

static void set_base_velocity(double vx, double vy, double omega) {
  const double coupling = WHEEL_BASE_LONGITUDINAL + WHEEL_BASE_LATERAL;
  const double dt = TIME_STEP / 1000.0;
  double wheel_speeds[4];
  wheel_speeds[0] = (vx + vy + coupling * omega) / WHEEL_RADIUS;
  wheel_speeds[1] = (vx - vy - coupling * omega) / WHEEL_RADIUS;
  wheel_speeds[2] = (vx - vy + coupling * omega) / WHEEL_RADIUS;
  wheel_speeds[3] = (vx + vy - coupling * omega) / WHEEL_RADIUS;

  for (int i = 0; i < 4; ++i) {
    if (!wheels[i]) continue;
    wheel_speeds[i] = clamp_value(wheel_speeds[i], -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S);
    const double speed_diff = wheel_speeds[i] - applied_wheel_speeds[i];
    const double max_delta =
        ((fabs(wheel_speeds[i]) >= fabs(applied_wheel_speeds[i]) &&
          applied_wheel_speeds[i] * wheel_speeds[i] >= 0.0)
             ? WHEEL_ACCEL_LIMIT_RAD_S2
             : WHEEL_DECEL_LIMIT_RAD_S2) *
        dt;
    applied_wheel_speeds[i] += clamp_value(speed_diff, -max_delta, max_delta);
    wb_motor_set_velocity(wheels[i], applied_wheel_speeds[i]);
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
  avoidance_hold_steps = 0;
  avoidance_turn_sign = 1.0;
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
  avoidance_mode = AVOID_MODE_NONE;
  avoidance_hold_steps = 0;
  avoidance_turn_sign = 1.0;
  avoidance_active = 0;
  avoidance_obstacle_side = 0;
  avoidance_release_steps = 0;
  avoidance_contour_steps = 0;
  avoidance_clear_steps = 0;
  avoidance_escape_steps = 0;
  avoidance_stuck_steps = 0;
  avoidance_no_obstacle_steps = 0;
  avoidance_prev_x = START_X;
  avoidance_prev_z = START_Z;
  avoidance_prev_target_distance = 0.0;
  avoidance_hit_target_distance = 0.0;
  avoidance_state_heading = 0.0;
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
  avoidance_mode = AVOID_MODE_NONE;
  avoidance_hold_steps = 0;
  avoidance_turn_sign = 1.0;
  avoidance_active = 0;
  avoidance_obstacle_side = 0;
  avoidance_release_steps = 0;
  avoidance_contour_steps = 0;
  avoidance_clear_steps = 0;
  avoidance_escape_steps = 0;
  avoidance_stuck_steps = 0;
  avoidance_no_obstacle_steps = 0;
  avoidance_prev_target_distance = 0.0;
  avoidance_hit_target_distance = 0.0;
  avoidance_state_heading = 0.0;
}

static double compute_range_pressure(double range, double clear_range, double blocked_range) {
  if (clear_range <= blocked_range + EPS) {
    return range <= blocked_range ? 1.0 : 0.0;
  }

  return clamp_value((clear_range - range) / (clear_range - blocked_range), 0.0, 1.0);
}

static void compute_reflex_avoidance_command(double front_range,
                                             double center_range,
                                             double left_front_range,
                                             double right_front_range,
                                             double left_side_range,
                                             double right_side_range,
                                             double heading_error_to_target,
                                             double *linear_speed,
                                             double *angular_speed) {
  const double front_pressure =
      compute_range_pressure(front_range, LIDAR_AVOID_RECOVER_RANGE, LIDAR_AVOID_STOP_RANGE);
  const double center_pressure = compute_range_pressure(center_range,
                                                        LIDAR_AVOID_RECOVER_RANGE,
                                                        LIDAR_AVOID_REVERSE_RANGE);
  const double left_front_pressure =
      compute_range_pressure(left_front_range, LIDAR_AVOID_RECOVER_RANGE, LIDAR_AVOID_STOP_RANGE);
  const double right_front_pressure =
      compute_range_pressure(right_front_range, LIDAR_AVOID_RECOVER_RANGE, LIDAR_AVOID_STOP_RANGE);
  const double left_side_pressure = compute_range_pressure(left_side_range,
                                                           LIDAR_REFLEX_SIDE_RELEASE_RANGE,
                                                           LIDAR_AVOID_SIDE_DANGER_RANGE);
  const double right_side_pressure = compute_range_pressure(right_side_range,
                                                            LIDAR_REFLEX_SIDE_RELEASE_RANGE,
                                                            LIDAR_AVOID_SIDE_DANGER_RANGE);

  double left_drive = 0.58;
  double right_drive = 0.58;

  // e-puck-like Braitenberg idea: opposite-side sensors speed up the other wheel,
  // while the front sensors brake both wheels and bias the robot away from the obstacle.
  left_drive += right_front_pressure * 0.95 + right_side_pressure * 0.72;
  right_drive += left_front_pressure * 0.95 + left_side_pressure * 0.72;

  left_drive -= front_pressure * 0.95 + center_pressure * 1.28 + left_front_pressure * 0.12;
  right_drive -= front_pressure * 0.95 + center_pressure * 1.28 + right_front_pressure * 0.12;

  // The route remains only a weak attraction term while we are near the obstacle.
  const double target_bias =
      clamp_value(heading_error_to_target * LIDAR_REFLEX_TARGET_GAIN, -0.18, 0.18);
  left_drive -= target_bias;
  right_drive += target_bias;

  // Keep the chosen обход side stable: if we already committed to one side,
  // add a small persistent turn bias so the robot doesn't ping-pong left/right.
  const double committed_turn_bias =
      (center_pressure > 0.20 || front_pressure > 0.24 || left_front_pressure > 0.20 ||
       right_front_pressure > 0.20)
          ? 0.26
          : 0.12;
  left_drive += avoidance_turn_sign < 0.0 ? committed_turn_bias : -committed_turn_bias;
  right_drive += avoidance_turn_sign > 0.0 ? committed_turn_bias : -committed_turn_bias;

  if (center_pressure > 0.92) {
    *linear_speed = -0.06;
    *angular_speed = avoidance_turn_sign * runtime_angular_speed_limit;
    return;
  }

  if (front_pressure > 0.82 && fabs(left_drive - right_drive) < 0.18) {
    *linear_speed = 0.0;
    *angular_speed = avoidance_turn_sign * runtime_angular_speed_limit;
    return;
  }

  const double max_reflex_linear = fmin(runtime_linear_speed_limit, LIDAR_REFLEX_MAX_LINEAR_SPEED);
  const double drive_average = clamp_value(0.5 * (left_drive + right_drive), -0.60, 1.0);
  const double drive_delta = clamp_value(0.5 * (right_drive - left_drive), -1.0, 1.0);
  const double min_linear = (front_pressure > 0.60 || center_pressure > 0.55) ? -0.05 : 0.02;

  *linear_speed = clamp_value(drive_average * max_reflex_linear, min_linear, max_reflex_linear);
  *angular_speed = clamp_value(drive_delta * runtime_angular_speed_limit * 1.45,
                               -runtime_angular_speed_limit,
                               runtime_angular_speed_limit);

  if ((front_pressure > 0.22 || center_pressure > 0.22 || left_front_pressure > 0.18 ||
       right_front_pressure > 0.18) &&
      fabs(*angular_speed) < MIN_ANGULAR_COMMAND) {
    *angular_speed = avoidance_turn_sign * MIN_ANGULAR_COMMAND;
  }
}

static void compute_lidar_obstacle_context(LidarObstacleContext *context,
                                           double target_beam_angle,
                                           double preferred_turn_sign) {
  if (!context) return;

  const double effective_max_range =
      lidar_max_range > EPS ? fmin(lidar_max_range, LIDAR_MAX_TRACE_RANGE) : LIDAR_MAX_TRACE_RANGE;
  memset(context, 0, sizeof(*context));
  context->expected_front_min_range = effective_max_range;
  context->unexpected_front_min_range = effective_max_range;
  context->unexpected_center_min_range = effective_max_range;
  context->unexpected_left_front_min_range = effective_max_range;
  context->unexpected_right_front_min_range = effective_max_range;
  context->unexpected_left_min_range = effective_max_range;
  context->unexpected_right_min_range = effective_max_range;
  context->best_gap_beam_angle = 0.0;
  context->best_gap_range = 0.0;
  context->best_gap_score = -1e9;
  context->has_best_gap = 0;

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

  for (int i = 0; i < lidar_resolution; i += LIDAR_SAMPLE_STRIDE) {
    const double range = (double)ranges[i];
    if (!is_finite_double(range)) continue;
    if (range < LIDAR_MIN_TRACE_RANGE || range > effective_max_range) continue;
    if (!lidar_hit_is_consistent(ranges, i, range, effective_max_range)) continue;

    const double alpha = lidar_resolution > 1 ? (double)i / (double)(lidar_resolution - 1) : 0.5;
    const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
    const double world_angle = heading - beam_angle;
    const double hit_x = sensor_origin_x + cos(world_angle) * range;
    const double hit_y = sensor_origin_y + sin(world_angle) * range;

    int expected_zone_wall = 0;
    for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
      if (point_near_zone_boundary(hit_x,
                                   hit_y,
                                   &zone_data.zones[zone_index],
                                   ZONE_WALL_EXPECTED_TOLERANCE)) {
        expected_zone_wall = 1;
        break;
      }
    }

    const double pressure =
        compute_range_pressure(range, LIDAR_TRACK_CAUTION_RANGE, LIDAR_AVOID_STOP_RANGE);
    const double center_weight = exp(-(beam_angle * beam_angle) / (2.0 * sigma * sigma));

    if (expected_zone_wall) {
      if (fabs(beam_angle) <= (LIDAR_FRONT_SECTOR_RAD + 0.16)) {
        if (range < context->expected_front_min_range) {
          context->expected_front_min_range = range;
        }
        context->expected_front_score += pressure * center_weight;
      }
      continue;
    }

    if (fabs(beam_angle) <= (LIDAR_FRONT_SECTOR_RAD + 0.16)) {
      context->unexpected_front_hit_count += 1;
      if (range < context->unexpected_front_min_range) {
        context->unexpected_front_min_range = range;
      }
      context->unexpected_front_score += pressure * center_weight;

      if (fabs(beam_angle) <= LIDAR_CENTER_SECTOR_RAD && range < context->unexpected_center_min_range) {
        context->unexpected_center_min_range = range;
      }
      if (beam_angle <= -LIDAR_FRONT_CORNER_MIN_RAD &&
          beam_angle >= -LIDAR_FRONT_CORNER_MAX_RAD &&
          range < context->unexpected_left_front_min_range) {
        context->unexpected_left_front_min_range = range;
      }
      if (beam_angle >= LIDAR_FRONT_CORNER_MIN_RAD &&
          beam_angle <= LIDAR_FRONT_CORNER_MAX_RAD &&
          range < context->unexpected_right_front_min_range) {
        context->unexpected_right_front_min_range = range;
      }
    } else if (beam_angle < 0.0) {
      if (range < context->unexpected_left_min_range) {
        context->unexpected_left_min_range = range;
      }
    } else {
      if (range < context->unexpected_right_min_range) {
        context->unexpected_right_min_range = range;
      }
    }

    if (beam_angle < 0.0) {
      context->unexpected_left_score += pressure * (0.40 + center_weight * 0.60);
    } else {
      context->unexpected_right_score += pressure * (0.40 + center_weight * 0.60);
    }

    if (range >= LIDAR_GAP_MIN_RANGE) {
      const double target_alignment =
          1.0 - clamp_value(fabs(wrap_angle(beam_angle - target_beam_angle)) / 1.25, 0.0, 1.0);
      const double gap_side_sign = beam_angle < 0.0 ? 1.0 : -1.0;
      const double commitment_bonus =
          preferred_turn_sign == 0.0
              ? 0.0
              : (gap_side_sign == preferred_turn_sign ? 0.16 : -0.10);
      const double frontal_bonus = 1.0 - clamp_value(fabs(beam_angle) / 1.35, 0.0, 1.0);
      const double range_score =
          clamp_value((range - LIDAR_GAP_MIN_RANGE) / fmax(effective_max_range - LIDAR_GAP_MIN_RANGE, 0.1),
                      0.0,
                      1.0);
      const double gap_score =
          range_score * 0.72 + target_alignment * 0.18 + frontal_bonus * 0.08 + commitment_bonus;

      if (!context->has_best_gap ||
          gap_score > context->best_gap_score + 1e-6 ||
          (fabs(gap_score - context->best_gap_score) <= 1e-6 && range > context->best_gap_range)) {
        context->best_gap_beam_angle = beam_angle;
        context->best_gap_range = range;
        context->best_gap_score = gap_score;
        context->has_best_gap = 1;
      }
    }
  }
}

static double compute_cross_track_error(double x, double z, const Waypoint *target) {
  const double segment_dx = target->x - navigation_segment_start_x;
  const double segment_dz = target->z - navigation_segment_start_z;
  const double segment_length = hypot2(segment_dx, segment_dz);
  if (segment_length <= EPS) return 0.0;

  const double ux = segment_dx / segment_length;
  const double uz = segment_dz / segment_length;
  const double rel_x = x - navigation_segment_start_x;
  const double rel_z = z - navigation_segment_start_z;
  return ux * rel_z - uz * rel_x;
}

static double compute_track_heading(double x, double z, const Waypoint *target, double distance_to_target) {
  const double target_heading = atan2(target->z - z, target->x - x);
  const double segment_dx = target->x - navigation_segment_start_x;
  const double segment_dz = target->z - navigation_segment_start_z;
  const double segment_length = hypot2(segment_dx, segment_dz);
  if (segment_length <= EPS) {
    return target_heading;
  }

  const double ux = segment_dx / segment_length;
  const double uz = segment_dz / segment_length;
  const double rel_x = x - navigation_segment_start_x;
  const double rel_z = z - navigation_segment_start_z;
  const double along = clamp_value(dot2(rel_x, rel_z, ux, uz), 0.0, segment_length);
  const double lookahead = clamp_value(distance_to_target * 0.75, TRACK_LOOKAHEAD_MIN, TRACK_LOOKAHEAD_MAX);
  const double aim_along = clamp_value(along + lookahead, 0.0, segment_length);
  const double aim_x = navigation_segment_start_x + ux * aim_along;
  const double aim_z = navigation_segment_start_z + uz * aim_along;
  const double cross_track = ux * rel_z - uz * rel_x;
  if (fabs(cross_track) > TRACK_DIRECT_HEADING_CROSSTRACK) {
    return target_heading;
  }
  const double cross_correction = clamp_value(-cross_track * TRACK_CROSS_TRACK_GAIN, -0.16, 0.16);
  const double line_heading = wrap_angle(atan2(aim_z - z, aim_x - x) + cross_correction);
  const double direct_bias = clamp_value(
      0.32 + fabs(cross_track) / fmax(TRACK_DIRECT_HEADING_CROSSTRACK, 0.01) * 0.68,
      0.32,
      1.0);
  const double distance_bias = distance_to_target < 0.70 ? 0.75 : 0.0;
  return wrap_angle(blend_angle(line_heading, target_heading, fmax(direct_bias, distance_bias)));
}

static int point_near_known_dynamic_zone(double x, double y) {
  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (point_near_zone(x, y, &zone_data.zones[zone_index], ZONE_CLEARANCE)) {
      return 1;
    }
  }

  return 0;
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

static double lidar_trace_confidence(const ObstacleTracePoint *point, double now_time) {
  if (!point) return 0.0;
  const double age = fmax(0.0, now_time - point->last_seen_time);
  const double freshness = clamp_value(1.0 - age / LIDAR_TRACE_TTL_SECONDS, 0.0, 1.0);
  const double support = clamp_value((double)point->hit_count / 4.0, 0.0, 1.0);
  return support * freshness;
}

static int lidar_hit_is_consistent(
    const float *ranges,
    int index,
    double range,
    double effective_max_range) {
  int valid_neighbors = 0;
  int neighbor_support = 0;
  const int left = index - LIDAR_SAMPLE_STRIDE;
  const int right = index + LIDAR_SAMPLE_STRIDE;

  if (left >= 0) {
    const double left_range = (double)ranges[left];
    if (is_finite_double(left_range) &&
        left_range >= LIDAR_MIN_TRACE_RANGE &&
        left_range <= effective_max_range) {
      valid_neighbors += 1;
      if (fabs(left_range - range) <= LIDAR_RANGE_JUMP_TOLERANCE) {
        neighbor_support += 1;
      }
    }
  }

  if (right < lidar_resolution) {
    const double right_range = (double)ranges[right];
    if (is_finite_double(right_range) &&
        right_range >= LIDAR_MIN_TRACE_RANGE &&
        right_range <= effective_max_range) {
      valid_neighbors += 1;
      if (fabs(right_range - range) <= LIDAR_RANGE_JUMP_TOLERANCE) {
        neighbor_support += 1;
      }
    }
  }

  if (valid_neighbors == 0) return 1;
  return neighbor_support > 0;
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

static void capture_lidar_trace() {
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
    if (!lidar_hit_is_consistent(ranges, i, range, effective_max_range)) continue;

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

static void clear_persistent_map() {
  persistent_map_count = 0;
  map_dirty = 0;
  remove(MAP_PATH);
  remove(MAP_TEMP_PATH);
  remove(MAP_CSV_PATH);
  remove(MAP_CSV_TEMP_PATH);
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

static void maybe_write_map() {
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

static void survey_expand_bounds(double x, double y, double *min_x, double *max_x, double *min_y, double *max_y) {
  if (x < *min_x) *min_x = x;
  if (x > *max_x) *max_x = x;
  if (y < *min_y) *min_y = y;
  if (y > *max_y) *max_y = y;
}

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

static void survey_route_add(SurveyPoint *route, int *count, double x, double y) {
  if (*count >= MAX_WAYPOINTS) return;
  if (*count > 0) {
    SurveyPoint *last = &route[*count - 1];
    if (hypot2(last->x - x, last->y - y) < 0.18) {
      last->x = x;
      last->y = y;
      return;
    }
  }
  route[*count] = (SurveyPoint){x, y};
  *count += 1;
}

static void survey_route_add_segment(SurveyPoint *route, int *count, SurveyPoint from, SurveyPoint to) {
  const double length = hypot2(to.x - from.x, to.y - from.y);
  const int steps = (int)fmax(1.0, ceil(length / MAPPING_SURVEY_MAX_CONTOUR_STEP));
  for (int i = 1; i <= steps; ++i) {
    const double t = (double)i / (double)steps;
    survey_route_add(route, count, from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t);
  }
}

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
    double clearance) {
  double min_x = SURVEY_X_MIN;
  double max_x = SURVEY_X_MAX;
  double min_y = SURVEY_Y_MIN;
  double max_y = SURVEY_Y_MAX;

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

static int survey_grid_index_for_point(const SurveyGrid *grid, double x, double y) {
  int gx = (int)round((x - grid->min_x) / grid->cell);
  int gy = (int)round((y - grid->min_y) / grid->cell);
  gx = (int)clamp_value((double)gx, 0.0, (double)(grid->width - 1));
  gy = (int)clamp_value((double)gy, 0.0, (double)(grid->height - 1));
  return gy * grid->width + gx;
}

static SurveyPoint survey_grid_point(const SurveyGrid *grid, int index) {
  const int gx = index % grid->width;
  const int gy = index / grid->width;
  return (SurveyPoint){grid->min_x + (double)gx * grid->cell, grid->min_y + (double)gy * grid->cell};
}

static int survey_flood_component(SurveyGrid *grid, double robot_x, double robot_y) {
  int start = survey_grid_index_for_point(grid, robot_x, robot_y);
  if (!grid->free_cell[start]) {
    double best_dist = 1e30;
    for (int i = 0; i < grid->count; ++i) {
      if (!grid->free_cell[i]) continue;
      SurveyPoint p = survey_grid_point(grid, i);
      const double dist = hypot2(p.x - robot_x, p.y - robot_y);
      if (dist < best_dist) {
        best_dist = dist;
        start = i;
      }
    }
    if (!grid->free_cell[start]) return 0;
  }

  int head = 0;
  int tail = 0;
  grid->queue[tail++] = start;
  grid->component_cell[start] = 1;
  int component_count = 0;
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  while (head < tail) {
    const int current = grid->queue[head++];
    component_count += 1;
    const int cx = current % grid->width;
    const int cy = current / grid->width;
    for (int d = 0; d < 4; ++d) {
      const int nx = cx + dx[d];
      const int ny = cy + dy[d];
      if (nx < 0 || nx >= grid->width || ny < 0 || ny >= grid->height) continue;
      const int ni = ny * grid->width + nx;
      if (!grid->free_cell[ni] || grid->component_cell[ni]) continue;
      grid->component_cell[ni] = 1;
      grid->queue[tail++] = ni;
    }
  }

  return component_count;
}

static int survey_cell_is_boundary(const SurveyGrid *grid, int index) {
  if (!grid->component_cell[index]) return 0;
  const int cx = index % grid->width;
  const int cy = index / grid->width;
  for (int oy = -1; oy <= 1; ++oy) {
    for (int ox = -1; ox <= 1; ++ox) {
      if (ox == 0 && oy == 0) continue;
      const int nx = cx + ox;
      const int ny = cy + oy;
      if (nx < 0 || nx >= grid->width || ny < 0 || ny >= grid->height) return 1;
      if (!grid->component_cell[ny * grid->width + nx]) return 1;
    }
  }
  return 0;
}

static double point_line_distance(SurveyPoint p, SurveyPoint a, SurveyPoint b) {
  return distance_point_to_segment(p.x, p.y, a.x, a.y, b.x, b.y);
}

static void survey_rdp_keep(const SurveyPoint *points, int first, int last, unsigned char *keep) {
  if (last <= first + 1) return;
  double best_distance = 0.0;
  int best_index = -1;
  for (int i = first + 1; i < last; ++i) {
    const double distance = point_line_distance(points[i], points[first], points[last]);
    if (distance > best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  if (best_index >= 0 && best_distance > MAPPING_SURVEY_RDP_EPS) {
    keep[best_index] = 1;
    survey_rdp_keep(points, first, best_index, keep);
    survey_rdp_keep(points, best_index, last, keep);
  }
}

static int append_grid_boundary_contour_phase(SurveyGrid *grid, SurveyPoint *route, int *route_count, double robot_x, double robot_y) {
  SurveyPoint boundary[MAPPING_SURVEY_MAX_BOUNDARY_POINTS];
  unsigned char used[MAPPING_SURVEY_MAX_BOUNDARY_POINTS];
  int boundary_count = 0;

  for (int i = 0; i < grid->count && boundary_count < MAPPING_SURVEY_MAX_BOUNDARY_POINTS; ++i) {
    if (!survey_cell_is_boundary(grid, i)) continue;
    boundary[boundary_count++] = survey_grid_point(grid, i);
  }
  if (boundary_count < 3) return 0;

  memset(used, 0, (size_t)boundary_count);
  SurveyPoint ordered[MAPPING_SURVEY_MAX_BOUNDARY_POINTS];
  int ordered_count = 0;
  int current = 0;
  double best_dist = 1e30;
  for (int i = 0; i < boundary_count; ++i) {
    const double dist = hypot2(boundary[i].x - robot_x, boundary[i].y - robot_y);
    if (dist < best_dist) {
      best_dist = dist;
      current = i;
    }
  }

  while (ordered_count < boundary_count) {
    ordered[ordered_count++] = boundary[current];
    used[current] = 1;
    int next = -1;
    double next_dist = 1e30;
    for (int i = 0; i < boundary_count; ++i) {
      if (used[i]) continue;
      const double dist = hypot2(boundary[i].x - boundary[current].x, boundary[i].y - boundary[current].y);
      if (dist < next_dist) {
        next_dist = dist;
        next = i;
      }
    }
    if (next < 0 || next_dist > grid->cell * 3.2) break;
    current = next;
  }

  if (ordered_count < 3) return 0;
  unsigned char keep[MAPPING_SURVEY_MAX_BOUNDARY_POINTS];
  memset(keep, 0, (size_t)ordered_count);
  keep[0] = 1;
  keep[ordered_count - 1] = 1;
  survey_rdp_keep(ordered, 0, ordered_count - 1, keep);

  SurveyPoint simplified[MAPPING_SURVEY_MAX_BOUNDARY_POINTS];
  int simplified_count = 0;
  for (int i = 0; i < ordered_count; ++i) {
    if (!keep[i]) continue;
    simplified[simplified_count++] = ordered[i];
  }
  if (simplified_count < 3) return 0;

  survey_route_add(route, route_count, simplified[0].x, simplified[0].y);
  for (int i = 1; i < simplified_count; ++i) {
    survey_route_add_segment(route, route_count, simplified[i - 1], simplified[i]);
  }
  survey_route_add_segment(route, route_count, simplified[simplified_count - 1], simplified[0]);
  return 1;
}

static void survey_sort_values(double *values, int count) {
  for (int i = 1; i < count; ++i) {
    const double value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
}

static void survey_subtract_interval(SurveyInterval *intervals, int *count, double block_start, double block_end) {
  SurveyInterval next[64];
  int next_count = 0;
  if (block_end < block_start) {
    const double tmp = block_start;
    block_start = block_end;
    block_end = tmp;
  }
  for (int i = 0; i < *count && next_count < 64; ++i) {
    const SurveyInterval current = intervals[i];
    if (block_end <= current.start || block_start >= current.end) {
      next[next_count++] = current;
      continue;
    }
    if (block_start > current.start) {
      next[next_count++] = (SurveyInterval){current.start, fmin(block_start, current.end)};
    }
    if (block_end < current.end && next_count < 64) {
      next[next_count++] = (SurveyInterval){fmax(block_end, current.start), current.end};
    }
  }
  for (int i = 0; i < next_count; ++i) intervals[i] = next[i];
  *count = next_count;
}

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

static int survey_find_grid_path(
    SurveyGrid *grid,
    SurveyPoint from,
    SurveyPoint to,
    SurveyPoint *path,
    int *path_count,
    int max_path_count) {
  const int start = survey_grid_index_for_point(grid, from.x, from.y);
  const int goal = survey_grid_index_for_point(grid, to.x, to.y);
  if (!grid->component_cell[start] || !grid->component_cell[goal]) return 0;

  memset(grid->visited_cell, 0, (size_t)grid->count);
  for (int i = 0; i < grid->count; ++i) grid->parent[i] = -1;

  int head = 0;
  int tail = 0;
  grid->queue[tail++] = start;
  grid->visited_cell[start] = 1;
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (head < tail && !grid->visited_cell[goal]) {
    const int current = grid->queue[head++];
    const int cx = current % grid->width;
    const int cy = current / grid->width;
    for (int d = 0; d < 8; ++d) {
      const int nx = cx + dx[d];
      const int ny = cy + dy[d];
      if (nx < 0 || nx >= grid->width || ny < 0 || ny >= grid->height) continue;
      const int ni = ny * grid->width + nx;
      if (!grid->component_cell[ni] || grid->visited_cell[ni]) continue;
      grid->visited_cell[ni] = 1;
      grid->parent[ni] = current;
      grid->queue[tail++] = ni;
    }
  }

  if (!grid->visited_cell[goal]) return 0;
  int reverse[1024];
  int reverse_count = 0;
  for (int cur = goal; cur >= 0 && reverse_count < 1024; cur = grid->parent[cur]) {
    reverse[reverse_count++] = cur;
    if (cur == start) break;
  }
  if (reverse_count <= 0) return 0;

  *path_count = 0;
  for (int i = reverse_count - 1; i >= 0 && *path_count < max_path_count; i -= 3) {
    path[*path_count] = survey_grid_point(grid, reverse[i]);
    *path_count += 1;
  }
  if (*path_count < max_path_count) {
    path[*path_count] = to;
    *path_count += 1;
  }
  return *path_count > 0;
}

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

static void append_scanline_coverage_phase(
    SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int room_zone_index) {
  double min_y = grid->min_y + MAPPING_SURVEY_INTERIOR_OFFSET;
  double max_y = grid->min_y + (double)(grid->height - 1) * grid->cell - MAPPING_SURVEY_INTERIOR_OFFSET;

  if (room_zone_index >= 0) {
    const LimitZone *room = &zone_data.zones[room_zone_index];
    min_y = max_y = room->points[0].y;
    for (int i = 1; i < room->point_count; ++i) {
      if (room->points[i].y < min_y) min_y = room->points[i].y;
      if (room->points[i].y > max_y) max_y = room->points[i].y;
    }
    min_y += MAPPING_SURVEY_INTERIOR_OFFSET;
    max_y -= MAPPING_SURVEY_INTERIOR_OFFSET;
  }

  int forward = 1;
  for (double y = min_y; y <= max_y + 0.01 && *route_count < MAX_WAYPOINTS - 4; y += MAPPING_SURVEY_STRIP) {
    SurveyInterval intervals[64];
    int interval_count = build_scanline_intervals(y, room_zone_index, grid, intervals, 64);
    if (!forward) {
      for (int left = 0, right = interval_count - 1; left < right; ++left, --right) {
        const SurveyInterval tmp = intervals[left];
        intervals[left] = intervals[right];
        intervals[right] = tmp;
      }
    }

    for (int i = 0; i < interval_count && *route_count < MAX_WAYPOINTS - 2; ++i) {
      SurveyPoint a = forward
                          ? (SurveyPoint){intervals[i].start, y}
                          : (SurveyPoint){intervals[i].end, y};
      SurveyPoint b = forward
                          ? (SurveyPoint){intervals[i].end, y}
                          : (SurveyPoint){intervals[i].start, y};
      if (!survey_point_safe(a.x, a.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET) ||
          !survey_point_safe(b.x, b.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET)) {
        continue;
      }
      append_safe_transition(grid, route, route_count, a, room_zone_index);
      if (survey_segment_safe(a.x, a.y, b.x, b.y, room_zone_index, MAPPING_SURVEY_INTERIOR_OFFSET)) {
        survey_route_add(route, route_count, b.x, b.y);
      } else {
        append_safe_transition(grid, route, route_count, b, room_zone_index);
      }
    }
    forward = !forward;
  }
}

static void write_mapping_survey_route_file(const char *path, const SurveyPoint *route, int route_count) {
  FILE *file = fopen(path, "w");
  if (!file) {
    set_error("Cannot write mapping survey route to route.csv");
    return;
  }

  fprintf(file, "# Auto-generated mapping survey route: perimeter first, then scanline coverage\n");
  fprintf(file, "# x,y\n");
  for (int i = 0; i < route_count; ++i) {
    fprintf(file, "%.3f,%.3f\n", route[i].x, route[i].y);
  }
  fclose(file);
}

static int generate_mapping_survey_route(const char *path, int clear_map_before_start) {
  if (clear_map_before_start) clear_persistent_map();

  double robot_x = 0.0;
  double robot_y = 0.0;
  double heading = 0.0;
  read_pose(&robot_x, &robot_y, &heading);

  const int room_zone_index = find_room_zone_index(robot_x, robot_y);
  mapping_survey_room_zone_index = room_zone_index;
  static SurveyGrid grid;
  if (!survey_build_grid(&grid, room_zone_index, robot_x, robot_y, MAPPING_SURVEY_INTERIOR_OFFSET)) {
    set_error("Cannot build mapping survey occupancy grid");
    return 0;
  }
  if (survey_flood_component(&grid, robot_x, robot_y) <= 0) {
    set_error("Cannot find connected free room for mapping survey");
    return 0;
  }

  SurveyPoint route[MAX_WAYPOINTS];
  int route_count = 0;
  if (!append_room_contour_phase(route, &route_count, room_zone_index, robot_x, robot_y)) {
    append_grid_boundary_contour_phase(&grid, route, &route_count, robot_x, robot_y);
  }
  append_scanline_coverage_phase(&grid, route, &route_count, room_zone_index);

  if (route_count < 2) {
    set_error("Mapping survey route is empty");
    return 0;
  }

  write_mapping_survey_route_file(path, route, route_count);
  return 1;
}

static int same_zone_data(const ZoneData *left, const ZoneData *right) {
  if (left->count != right->count) return 0;

  for (int zone_index = 0; zone_index < left->count; ++zone_index) {
    if (left->zones[zone_index].point_count != right->zones[zone_index].point_count) {
      return 0;
    }

    for (int point_index = 0; point_index < left->zones[zone_index].point_count; ++point_index) {
      if (fabs(left->zones[zone_index].points[point_index].x - right->zones[zone_index].points[point_index].x) > 1e-6 ||
          fabs(left->zones[zone_index].points[point_index].y - right->zones[zone_index].points[point_index].y) > 1e-6) {
        return 0;
      }
    }
  }

  return 1;
}

static int load_zones(ZoneData *zones) {
  FILE *file = fopen(ZONE_PATH, "r");
  zones->count = 0;
  if (!file) return 1;

  char line[256];
  if (!fgets(line, sizeof(line), file)) {
    fclose(file);
    return 1;
  }

  int declared_zone_count = 0;
  if (sscanf(line, " zone_count %d", &declared_zone_count) != 1) {
    fclose(file);
    set_error("Cannot parse limit_zones.txt");
    return 0;
  }

  declared_zone_count = (int)clamp_value((double)declared_zone_count, 0.0, (double)MAX_ZONES);
  for (int zone_index = 0; zone_index < declared_zone_count; ++zone_index) {
    if (!fgets(line, sizeof(line), file)) break;

    int declared_point_count = 0;
    if (sscanf(line, " zone %d", &declared_point_count) != 1) {
      fclose(file);
      set_error("Cannot parse limit zone entry");
      return 0;
    }

    LimitZone *zone = &zones->zones[zones->count];
    snprintf(zone->id, sizeof(zone->id), "zone-%d", zones->count + 1);
    zone->point_count = 0;

    for (int point_index = 0; point_index < declared_point_count; ++point_index) {
      if (!fgets(line, sizeof(line), file)) {
        fclose(file);
        set_error("Unexpected end of limit_zones.txt");
        return 0;
      }

      double x = 0.0;
      double y = 0.0;
      if (sscanf(line, " %lf %lf", &x, &y) != 2) {
        fclose(file);
        set_error("Cannot parse limit zone point");
        return 0;
      }

      if (zone->point_count < MAX_ZONE_POINTS) {
        zone->points[zone->point_count].x = x;
        zone->points[zone->point_count].y = y;
        zone->point_count += 1;
      }
    }

    if (zone->point_count >= 3) {
      zones->count += 1;
    }
  }

  fclose(file);
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
  if (!command) return 0;
  FILE *file = fopen(RUNTIME_COMMAND_PATH, "r");
  if (!file) return 0;

  RuntimeCommand parsed = {0};
  parsed.id = -1;
  parsed.clear_map = 1;
  parsed.size_x = 0.8;
  parsed.size_y = 0.8;
  parsed.height = 0.6;

  char line[256];
  while (fgets(line, sizeof(line), file)) {
    long long id = 0;
    double numeric = 0.0;
    char token[64];

    if (sscanf(line, " id %lld", &id) == 1) {
      parsed.id = id;
      continue;
    }
    if (sscanf(line, " type %63s", token) == 1) {
      parsed.has_spawn_obstacle = strcmp(token, "spawn_obstacle") == 0 ? 1 : 0;
      parsed.has_start_mapping_survey = strcmp(token, "start_mapping_survey") == 0 ? 1 : 0;
      continue;
    }
    if (sscanf(line, " clear_map %lf", &numeric) == 1) {
      parsed.clear_map = fabs(numeric) > EPS ? 1 : 0;
      continue;
    }
    if (sscanf(line, " x %lf", &numeric) == 1) {
      parsed.x = numeric;
      continue;
    }
    if (sscanf(line, " y %lf", &numeric) == 1) {
      parsed.y = numeric;
      continue;
    }
    if (sscanf(line, " size_x %lf", &numeric) == 1) {
      parsed.size_x = numeric;
      continue;
    }
    if (sscanf(line, " size_y %lf", &numeric) == 1) {
      parsed.size_y = numeric;
      continue;
    }
    if (sscanf(line, " height %lf", &numeric) == 1) {
      parsed.height = numeric;
      continue;
    }
  }

  fclose(file);
  if (parsed.id < 0 || (!parsed.has_spawn_obstacle && !parsed.has_start_mapping_survey)) return 0;
  *command = parsed;
  return 1;
}

static void maybe_reload_runtime_command() {
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
    if (generate_mapping_survey_route(ROUTE_PATH, command.clear_map)) {
      RouteData next_route = {0};
      if (load_route(&next_route)) {
        route_data = next_route;
        current_waypoint_index = 0;
        route_finished = 0;
        route_source_mapping_survey = 1;
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

static void maybe_reload_zones() {
  if ((step_counter % ZONE_RELOAD_INTERVAL) != 0) return;

  ZoneData next_zones = {0};
  if (!load_zones(&next_zones)) return;

  if (!same_zone_data(&zone_data, &next_zones)) {
    zone_data = next_zones;
    sync_zone_nodes();
    set_status(zone_data.count > 0 ? "zones_synced" : "zones_cleared");
  }
}

static int load_route(RouteData *route) {
  FILE *file = fopen(ROUTE_PATH, "r");
  if (!file) {
    set_error("Cannot open route.csv");
    return 0;
  }

  char line[256];
  int count = 0;

  while (fgets(line, sizeof(line), file)) {
    double x = 0.0;
    double z = 0.0;
    double heading_deg = 0.0;

    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#') continue;
    if ((line[0] >= 'A' && line[0] <= 'z') || strncmp(line, "x,", 2) == 0 || strncmp(line, "x ", 2) == 0) continue;

    if (sscanf(line, " %lf , %lf , %lf", &x, &z, &heading_deg) == 3) {
      if (count < MAX_WAYPOINTS) {
        route->waypoints[count].x = x;
        route->waypoints[count].z = z;
        route->waypoints[count].heading_rad = heading_deg * PI / 180.0;
        route->waypoints[count].has_heading = 1;
        ++count;
      }
      continue;
    }

    if (sscanf(line, " %lf , %lf", &x, &z) == 2) {
      if (count < MAX_WAYPOINTS) {
        route->waypoints[count].x = x;
        route->waypoints[count].z = z;
        route->waypoints[count].heading_rad = 0.0;
        route->waypoints[count].has_heading = 0;
        ++count;
      }
    }
  }

  fclose(file);

  if (count == 0) {
    set_error("Route file is empty");
    return 0;
  }

  route->count = count;
  route->last_modified = get_file_mtime(ROUTE_PATH);
  clear_error();
  return 1;
}

static void maybe_reload_route() {
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
    mapping_survey_room_zone_index = -1;
    reset_navigation_mode();
    set_status("route_reloaded");
  }
}

static void wait_for_fresh_route() {
  route_data.count = 0;
  route_data.last_modified = get_file_mtime(ROUTE_PATH);
  current_waypoint_index = 0;
  route_finished = 0;
  route_source_mapping_survey = 0;
  mapping_survey_room_zone_index = -1;
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

static void run_navigation_step() {
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
    avoidance_hold_steps = 0;
    reset_navigation_mode();
    distance_to_target = 0.0;
    stop_robot();
    return;
  }

  if (route_finished) {
    set_status("finished");
    avoidance_hold_steps = 0;
    reset_navigation_mode();
    distance_to_target = 0.0;
    stop_robot();
    return;
  }

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
  compute_lidar_obstacle_context(&lidar_context, -heading_error_to_target, avoidance_turn_sign);
  const int front_obstacle_detected = lidar_available && lidar_context.unexpected_front_hit_count > 0;
  const double front_obstacle_range =
      front_obstacle_detected ? lidar_context.unexpected_front_min_range : LIDAR_MAX_TRACE_RANGE;
  const double center_obstacle_range =
      front_obstacle_detected ? lidar_context.unexpected_center_min_range : LIDAR_MAX_TRACE_RANGE;
  const double left_front_corner_range =
      lidar_available ? lidar_context.unexpected_left_front_min_range : LIDAR_MAX_TRACE_RANGE;
  const double right_front_corner_range =
      lidar_available ? lidar_context.unexpected_right_front_min_range : LIDAR_MAX_TRACE_RANGE;
  const double left_obstacle_range =
      lidar_available ? lidar_context.unexpected_left_min_range : LIDAR_MAX_TRACE_RANGE;
  const double right_obstacle_range =
      lidar_available ? lidar_context.unexpected_right_min_range : LIDAR_MAX_TRACE_RANGE;
  const double expected_front_range =
      lidar_available ? lidar_context.expected_front_min_range : LIDAR_MAX_TRACE_RANGE;
  const int avoidance_was_active = avoidance_active;
  const double near_front_range = fmin(
      fmin(front_obstacle_range, center_obstacle_range),
      fmin(left_front_corner_range, right_front_corner_range));
  const double left_lidar_context = fmin(left_obstacle_range, left_front_corner_range);
  const double right_lidar_context = fmin(right_obstacle_range, right_front_corner_range);
  const int expected_zone_wall_ahead =
      lidar_available && expected_front_range < LIDAR_TRACK_CAUTION_RANGE;
  const int expected_zone_wall_close =
      lidar_available && expected_front_range < EXPECTED_WALL_SOFT_STOP_RANGE;
  const int expected_zone_wall_slowdown =
      lidar_available && expected_front_range < EXPECTED_WALL_SLOWDOWN_RANGE;
  const int side_obstacle_detected =
      lidar_available &&
      (left_lidar_context < LIDAR_AVOID_SIDE_TRIGGER_RANGE ||
       right_lidar_context < LIDAR_AVOID_SIDE_TRIGGER_RANGE);
  const int lidar_hard_priority_zone =
      lidar_available &&
      (near_front_range < LIDAR_TRACK_HARD_PRIORITY_RANGE ||
       left_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10) ||
       right_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10));
  const int front_corner_obstacle_detected =
      left_front_corner_range < (LIDAR_AVOID_TRIGGER_RANGE + 0.10) ||
      right_front_corner_range < (LIDAR_AVOID_TRIGGER_RANGE + 0.10);
  const int obstacle_context_present =
      lidar_available &&
      (near_front_range < LIDAR_AVOID_RECOVER_RANGE ||
       left_lidar_context < LIDAR_REFLEX_SIDE_RELEASE_RANGE ||
       right_lidar_context < LIDAR_REFLEX_SIDE_RELEASE_RANGE);
  const int should_start_avoidance =
      lidar_available &&
      ((front_obstacle_detected && near_front_range < LIDAR_AVOID_TRIGGER_RANGE) ||
       center_obstacle_range < (LIDAR_AVOID_STOP_RANGE + 0.08) ||
       front_corner_obstacle_detected ||
       (side_obstacle_detected && near_front_range < LIDAR_TRACK_SLOW_RANGE) ||
       (lidar_hard_priority_zone && near_front_range < (LIDAR_TRACK_HARD_PRIORITY_RANGE - 0.04)));

  if (!avoidance_active && should_start_avoidance) {
    const double left_clearance = fmax(left_obstacle_range, left_front_corner_range);
    const double right_clearance = fmax(right_obstacle_range, right_front_corner_range);
    avoidance_active = 1;
    avoidance_mode = AVOID_MODE_FOLLOW_EDGE;
    avoidance_hold_steps = LIDAR_AVOID_MIN_CONTOUR_STEPS;
    if (lidar_context.unexpected_left_score + LIDAR_REFLEX_SWITCH_MARGIN < lidar_context.unexpected_right_score ||
        left_clearance > right_clearance + LIDAR_REFLEX_SWITCH_MARGIN) {
      avoidance_turn_sign = 1.0;
    } else if (lidar_context.unexpected_right_score + LIDAR_REFLEX_SWITCH_MARGIN < lidar_context.unexpected_left_score ||
               right_clearance > left_clearance + LIDAR_REFLEX_SWITCH_MARGIN) {
      avoidance_turn_sign = -1.0;
    } else if (fabs(lidar_priority_turn_sign) > 0.0) {
      avoidance_turn_sign = lidar_priority_turn_sign;
    } else {
      avoidance_turn_sign = sign_or_one(heading_error_to_target);
    }
    avoidance_obstacle_side = avoidance_turn_sign > 0.0 ? -1 : 1;
    lidar_priority_turn_sign = avoidance_turn_sign;
    lidar_priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS;
    avoidance_release_steps = 0;
    avoidance_contour_steps = 0;
    avoidance_clear_steps = 0;
    avoidance_escape_steps = 0;
    avoidance_stuck_steps = 0;
    avoidance_no_obstacle_steps = 0;
    avoidance_prev_x = x;
    avoidance_prev_z = z;
    avoidance_prev_target_distance = target_distance_now;
    avoidance_hit_target_distance = target_distance_now;
    avoidance_state_heading = heading;
  }

  if (avoidance_active) {
    const double front_pressure =
        compute_range_pressure(front_obstacle_range, LIDAR_AVOID_RECOVER_RANGE, LIDAR_AVOID_STOP_RANGE);
    const double center_pressure =
        compute_range_pressure(center_obstacle_range, LIDAR_AVOID_RECOVER_RANGE, LIDAR_AVOID_REVERSE_RANGE);
    const double moved_since_last = hypot2(x - avoidance_prev_x, z - avoidance_prev_z);
    const double target_progress = avoidance_prev_target_distance - target_distance_now;

    avoidance_contour_steps += 1;
    if (avoidance_hold_steps > 0) avoidance_hold_steps -= 1;

    if (moved_since_last < LIDAR_AVOID_STUCK_POSE_EPS &&
        target_progress < LIDAR_AVOID_STUCK_PROGRESS_EPS) {
      avoidance_stuck_steps += 1;
    } else {
      avoidance_stuck_steps = 0;
    }
    avoidance_prev_x = x;
    avoidance_prev_z = z;
    avoidance_prev_target_distance = target_distance_now;

    if (!obstacle_context_present) {
      avoidance_no_obstacle_steps += 1;
      avoidance_clear_steps += 1;
    } else {
      avoidance_no_obstacle_steps = 0;
      avoidance_clear_steps = 0;
    }

    if (avoidance_no_obstacle_steps > FREE_SPACE_RECOVERY_STEPS &&
        avoidance_hold_steps <= 0 &&
        avoidance_clear_steps >= LIDAR_AVOID_CLEAR_STEPS) {
      clear_local_navigation_state();
      navigation_waypoint_index = current_waypoint_index;
      navigation_segment_start_x = x;
      navigation_segment_start_z = z;
      navigation_mode = NAV_MODE_TRACK;
      distance_to_target = target_distance_now;
      clear_error();
      set_status("reacquired_free_space");
    }

    if (avoidance_active) {
      double linear_speed = 0.0;
      double angular_speed = 0.0;
      const int hard_front_blocked =
          center_obstacle_range < (LIDAR_AVOID_STOP_RANGE + 0.05) ||
          front_obstacle_range < (LIDAR_AVOID_STOP_RANGE + 0.10) ||
          lidar_context.unexpected_front_score > 0.95;
      const double side_commit_score =
          avoidance_turn_sign > 0.0 ? lidar_context.unexpected_right_score : lidar_context.unexpected_left_score;
      const double opposite_side_score =
          avoidance_turn_sign > 0.0 ? lidar_context.unexpected_left_score : lidar_context.unexpected_right_score;
      const double gap_heading_error =
          lidar_context.has_best_gap ? wrap_angle(-lidar_context.best_gap_beam_angle) : 0.0;
      const double gap_turn_sign =
          lidar_context.has_best_gap ? sign_or_one(gap_heading_error) : avoidance_turn_sign;
      const double gap_speed_scale =
          lidar_context.has_best_gap
              ? clamp_value((lidar_context.best_gap_range - LIDAR_GAP_MIN_RANGE) /
                                fmax(LIDAR_TRACK_CAUTION_RANGE - LIDAR_GAP_MIN_RANGE, 0.1),
                            0.28,
                            1.0)
              : 0.42;
      const double turn_lock_bias = clamp_value(
          side_commit_score - opposite_side_score,
          -0.55,
          0.55);
      const double target_bias = clamp_value(heading_error_to_target * 0.12, -0.12, 0.12);

      if (lidar_context.has_best_gap &&
          gap_turn_sign != avoidance_turn_sign &&
          (avoidance_stuck_steps > (LIDAR_AVOID_STUCK_STEPS / 2) ||
           lidar_context.best_gap_range > (front_obstacle_range + LIDAR_GAP_SWITCH_RANGE_BONUS))) {
        avoidance_turn_sign = gap_turn_sign;
        avoidance_obstacle_side = avoidance_turn_sign > 0.0 ? -1 : 1;
        lidar_priority_turn_sign = avoidance_turn_sign;
        lidar_priority_hold_steps = LIDAR_PRIORITY_HOLD_STEPS;
      }

      if (hard_front_blocked) {
        linear_speed = center_obstacle_range < LIDAR_AVOID_REVERSE_RANGE ? -0.05 : 0.0;
        angular_speed = clamp_value(
            (lidar_context.has_best_gap ? gap_heading_error * 2.4 : avoidance_turn_sign) *
                runtime_angular_speed_limit,
            -runtime_angular_speed_limit,
            runtime_angular_speed_limit);
        if (fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
          angular_speed = avoidance_turn_sign * MIN_ANGULAR_COMMAND;
        }
        set_status(lidar_context.has_best_gap ? "avoiding_gap_turn" : "avoiding_committed_turn");
      } else {
        linear_speed = clamp_value(runtime_linear_speed_limit *
                                       (0.38 - front_pressure * 0.18 - center_pressure * 0.12) *
                                       gap_speed_scale,
                                   0.04,
                                   fmin(runtime_linear_speed_limit * 0.48, 0.15));
        if (lidar_context.has_best_gap) {
          angular_speed = clamp_value(
              gap_heading_error * 1.85 +
                  avoidance_turn_sign * (runtime_angular_speed_limit * 0.18 + front_pressure * 0.22) +
                  turn_lock_bias * 0.18 + target_bias,
              -runtime_angular_speed_limit,
              runtime_angular_speed_limit);
        } else {
          angular_speed = clamp_value(
              avoidance_turn_sign * (runtime_angular_speed_limit * 0.52 + front_pressure * 0.38) +
                  turn_lock_bias * 0.55 + target_bias,
              -runtime_angular_speed_limit,
              runtime_angular_speed_limit);
        }
        if (fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
          angular_speed = avoidance_turn_sign * MIN_ANGULAR_COMMAND;
        }
        set_status(lidar_context.has_best_gap ? "avoiding_gap_drive" : "avoiding_committed_drive");
      }

      if (avoidance_stuck_steps > LIDAR_AVOID_STUCK_STEPS) {
        linear_speed = -0.05;
        angular_speed = avoidance_turn_sign * runtime_angular_speed_limit;
        avoidance_hold_steps = LIDAR_AVOID_HOLD_STEPS;
        avoidance_stuck_steps = 0;
        set_status("avoiding_committed_escape");
      }

      distance_to_target = target_distance_now;
      clear_error();
      set_base_velocity(linear_speed, 0.0, angular_speed);
      return;
    }
  }

  avoidance_active = 0;
  avoidance_mode = AVOID_MODE_NONE;
  avoidance_obstacle_side = 0;
  avoidance_release_steps = 0;
  avoidance_contour_steps = 0;
  avoidance_clear_steps = 0;
  avoidance_escape_steps = 0;
  avoidance_stuck_steps = 0;
  avoidance_hit_target_distance = 0.0;
  avoidance_state_heading = 0.0;
  if (avoidance_was_active) {
    navigation_segment_start_x = x;
    navigation_segment_start_z = z;
    navigation_mode = NAV_MODE_TRACK;
  }
  distance_to_target = target_distance_now;
  navigation_segment_start_x = x;
  navigation_segment_start_z = z;
  const double cross_track_error = 0.0;
  const int route_relaxed_mode_base = 1;
  const double track_heading = heading_to_target;
  const double heading_error_on_track = heading_error_to_target;

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
    const double front_caution = clamp_value(
        (LIDAR_TRACK_CAUTION_RANGE - near_front_range) /
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
        1.0 - front_caution * 0.72 - fmax(left_pressure, right_pressure) * 0.20,
        0.16,
        1.0);
    lidar_hard_priority =
        near_front_range < LIDAR_TRACK_HARD_PRIORITY_RANGE ||
        left_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10) ||
        right_lidar_context < (LIDAR_AVOID_SIDE_TRIGGER_RANGE + 0.10);
    lidar_caution_active =
        near_front_range < LIDAR_TRACK_CAUTION_RANGE ||
        left_lidar_context < LIDAR_TRACK_SIDE_BIAS_RANGE ||
        right_lidar_context < LIDAR_TRACK_SIDE_BIAS_RANGE;

    if (fabs(asymmetric_pressure) > LIDAR_PRIORITY_SWITCH_MARGIN) {
      desired_turn_sign = sign_or_one(asymmetric_pressure);
    } else if (lidar_hard_priority) {
      if (fabs(heading_error_to_target) > 0.12) {
        desired_turn_sign = sign_or_one(heading_error_to_target);
      } else {
        desired_turn_sign = left_lidar_context <= right_lidar_context ? -1.0 : 1.0;
      }
    }

    if (lidar_caution_active) {
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

    updated_heading_error = wrap_angle(
        (lidar_hard_priority ? direct_heading_error : updated_heading_error) + lidar_heading_bias);
    if (lidar_hard_priority) {
      lidar_speed_scale = fmin(lidar_speed_scale, 0.58);
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
        (!avoidance_active && lidar_hard_priority && fabs(lidar_priority_turn_sign) > 0.0)
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
          (!avoidance_active && lidar_hard_priority && fabs(lidar_priority_turn_sign) > 0.0)
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

  double base_speed = clamp_value(distance_to_target * 1.1, TRACK_MIN_LINEAR_SPEED, runtime_linear_speed_limit);
  if (distance_to_target < TRACK_SLOW_RADIUS) {
    base_speed = clamp_value(distance_to_target * 0.95, TRACK_MIN_LINEAR_SPEED, 0.14);
  }
  base_speed *= lidar_speed_scale;
  base_speed *= expected_wall_speed_scale;

  double heading_scale = clamp_value(1.0 - fabs(updated_heading_error) / 0.85, 0.22, 1.0);
  if (fabs(updated_heading_error) > TURN_ENTER_ERROR_RAD) {
    heading_scale = fmin(heading_scale, 0.35);
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

static void write_state_snapshot() {
  FILE *file = fopen(STATE_TEMP_PATH, "w");
  if (!file) return;

  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  const double simulation_now = wb_robot_get_time();
  read_pose(&x, &y, &heading);

  fprintf(file, "{\n");
  fprintf(file, "  \"coordinateContractVersion\": 1,\n");
  fprintf(file, "  \"simulationTime\": %.6f,\n", simulation_now);
  fprintf(file, "  \"pose\": {\n");
  fprintf(file, "    \"x\": %.6f,\n", x);
  fprintf(file, "    \"y\": %.6f,\n", y);
  fprintf(file, "    \"z\": %.6f,\n", START_HEIGHT);
  fprintf(file, "    \"yaw\": %.6f\n", heading);
  fprintf(file, "  },\n");
  fprintf(file, "  \"robot\": {\n");
  fprintf(file, "    \"x\": %.6f,\n", x);
  fprintf(file, "    \"y\": %.6f,\n", y);
  fprintf(file, "    \"z\": %.6f,\n", START_HEIGHT);
  fprintf(file, "    \"heading\": %.6f\n", heading);
  fprintf(file, "  },\n");
  fprintf(file, "  \"navigation\": {\n");
  fprintf(file, "    \"status\": \"%s\",\n", navigation_status);
  fprintf(file, "    \"error\": \"%s\",\n", navigation_error);
  fprintf(file, "    \"currentWaypointIndex\": %d,\n", current_waypoint_index);
  fprintf(file, "    \"finished\": %s,\n", route_finished ? "true" : "false");
  fprintf(file, "    \"distanceToTarget\": %.6f,\n", distance_to_target);

  if (route_data.count > 0 && current_waypoint_index < route_data.count) {
    const Waypoint *target = &route_data.waypoints[current_waypoint_index];
    fprintf(file, "    \"target\": {\"x\": %.6f, \"y\": %.6f, \"headingDeg\": %.3f}\n",
            target->x,
            target->z,
            target->heading_rad * 180.0 / PI);
  } else {
    fprintf(file, "    \"target\": null\n");
  }

  fprintf(file, "  },\n");
  fprintf(file, "  \"motionProfile\": {\n");
  fprintf(file, "    \"cruiseSpeedMps\": %.6f,\n", configured_cruise_speed_mps);
  fprintf(file, "    \"payloadKg\": %.6f,\n", configured_payload_kg);
  fprintf(file, "    \"batteryRange\": %.6f,\n", configured_battery_range_units);
  fprintf(file, "    \"batterySpeedFactor\": %.6f,\n", runtime_battery_speed_factor);
  fprintf(file, "    \"runtimeLinearLimitMps\": %.6f,\n", runtime_linear_speed_limit);
  fprintf(file, "    \"runtimeAngularLimitRad\": %.6f\n", runtime_angular_speed_limit);
  fprintf(file, "  },\n");
  fprintf(file, "  \"dynamicZones\": {\n");
  fprintf(file, "    \"count\": %d,\n", zone_data.count);
  fprintf(file, "    \"wallCount\": %d\n", zone_node_count);
  fprintf(file, "  },\n");
  fprintf(file, "  \"perception\": {\n");
  fprintf(file, "    \"lidar\": {\n");
  fprintf(file, "      \"enabled\": %s,\n", lidar_available ? "true" : "false");
  fprintf(file, "      \"horizontalResolution\": %d,\n", lidar_resolution);
  fprintf(file, "      \"maxRange\": %.3f,\n", lidar_max_range);
  fprintf(file, "      \"lastHitCount\": %d,\n", lidar_last_hit_count);
  fprintf(file, "      \"frontHitCount\": %d,\n", lidar_front_hit_count);
  fprintf(file, "      \"frontMinRange\": %.3f,\n", lidar_front_min_range);
  fprintf(file, "      \"centerMinRange\": %.3f,\n", lidar_center_min_range);
  fprintf(file, "      \"leftFrontCornerMinRange\": %.3f,\n", lidar_left_front_min_range);
  fprintf(file, "      \"rightFrontCornerMinRange\": %.3f,\n", lidar_right_front_min_range);
  fprintf(file, "      \"leftMinRange\": %.3f,\n", lidar_left_min_range);
  fprintf(file, "      \"rightMinRange\": %.3f\n", lidar_right_min_range);
  fprintf(file, "    },\n");
  fprintf(file, "    \"obstacleTrace\": [\n");
  int emitted_trace_points = 0;
  for (int i = 0; i < obstacle_trace_count; ++i) {
    const double confidence = lidar_trace_confidence(&obstacle_trace[i], simulation_now);
    if (confidence <= LIDAR_TRACE_MIN_CONFIDENCE) continue;
    if (emitted_trace_points > 0) fprintf(file, ",\n");
    fprintf(file,
            "      {\"x\": %.6f, \"y\": %.6f, \"confidence\": %.3f}",
            obstacle_trace[i].x,
            obstacle_trace[i].y,
            confidence);
    emitted_trace_points += 1;
  }
  if (emitted_trace_points > 0) fprintf(file, "\n");
  fprintf(file, "    ]\n");
  fprintf(file, "  },\n");
  fprintf(file, "  \"obstacleMap\": {\n");
  fprintf(file, "    \"cellCount\": %d,\n", persistent_map_count);
  fprintf(file, "    \"cellSize\": %.4f,\n", MAP_CELL_SIZE);
  fprintf(file, "    \"mapFile\": \"obstacle_map.json\",\n");
  fprintf(file, "    \"jsonFile\": \"obstacle_map.json\",\n");
  fprintf(file, "    \"excelCsvFile\": \"obstacle_map.csv\"\n");
  fprintf(file, "  },\n");
  fprintf(file, "  \"route\": {\n");
  fprintf(file, "    \"source\": \"route.csv\",\n");
  fprintf(file, "    \"waypoints\": [\n");
  for (int i = 0; i < route_data.count; ++i) {
    fprintf(file,
            "      {\"x\": %.6f, \"y\": %.6f, \"headingDeg\": %.3f, \"hasHeading\": %s}%s\n",
            route_data.waypoints[i].x,
            route_data.waypoints[i].z,
            route_data.waypoints[i].heading_rad * 180.0 / PI,
            route_data.waypoints[i].has_heading ? "true" : "false",
            i + 1 < route_data.count ? "," : "");
  }
  fprintf(file, "    ]\n");
  fprintf(file, "  }\n");
  fprintf(file, "}\n");
  fclose(file);

  replace_file(STATE_TEMP_PATH, STATE_PATH);
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  wb_robot_init();
  init_wheels();
  init_manipulator_pose();
  init_lidar();
  init_pose_tracking();
  reset_robot_pose();
  clear_persistent_map();
  apply_motion_profile();
  motion_profile_last_modified = get_file_mtime(MOTION_PROFILE_PATH);
  if (motion_profile_last_modified >= 0) {
    load_motion_profile();
  }
  maybe_reload_zones();
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

  while (wb_robot_step(TIME_STEP) != -1) {
    ++step_counter;
    maybe_reload_zones();
    maybe_reload_route();
    maybe_reload_motion_profile();
    maybe_reload_runtime_command();
    capture_lidar_trace();
    merge_trace_into_map(wb_robot_get_time());
    maybe_write_map();
    run_navigation_step();
    write_state_snapshot();
  }

  maybe_write_map();
  remove_runtime_obstacle_nodes();
  remove_zone_nodes();
  wb_robot_cleanup();
  return 0;
}
