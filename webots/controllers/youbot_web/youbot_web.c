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
#define KINEMATIC_LINEAR_SPEED 0.22
#define KINEMATIC_ANGULAR_SPEED 1.6
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
#define TRACK_CROSS_TRACK_GAIN 1.25
#define TRACK_LOOKAHEAD_MIN 0.16
#define TRACK_LOOKAHEAD_MAX 0.45
#define TRACK_MIN_LINEAR_SPEED 0.035
#define MIN_ANGULAR_COMMAND 0.28
#define START_X 0.0
#define START_Z 0.0
#define START_HEIGHT 0.102838
#define MAX_WAYPOINTS 256
#define ROUTE_RELOAD_INTERVAL 20
#define ZONE_RELOAD_INTERVAL 10
#define MAX_ZONES 32
#define MAX_ZONE_POINTS 48
#define MAX_ZONE_NODES 512
#define MAX_OBSTACLE_TRACE_POINTS 768
#define WALL_THICKNESS 0.08
#define WALL_HEIGHT 0.45
#define ZONE_CLEARANCE 0.32
#define LIDAR_SAMPLE_STRIDE 3
#define LIDAR_TRACE_SPACING 0.2
#define LIDAR_MIN_TRACE_RANGE 0.12
#define LIDAR_MAX_TRACE_RANGE 6.0
#define LIDAR_LOCAL_X 0.24
#define LIDAR_LOCAL_Y 0.0
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
} ObstacleTracePoint;

static WbDeviceTag wheels[4];
static WbDeviceTag arm_joints[5];
static WbDeviceTag gripper_fingers[2];
static WbDeviceTag front_lidar = 0;
static WbNodeRef self_node;
static WbFieldRef translation_field;
static WbFieldRef rotation_field;
static WbFieldRef root_children_field;

static RouteData route_data = {0};
static ZoneData zone_data = {0};
static WbNodeRef zone_nodes[MAX_ZONE_NODES];
static char zone_node_defs[MAX_ZONE_NODES][64];
static int zone_node_count = 0;
static ObstacleTracePoint obstacle_trace[MAX_OBSTACLE_TRACE_POINTS];
static int obstacle_trace_count = 0;
static int lidar_available = 0;
static int lidar_resolution = 0;
static double lidar_fov = 0.0;
static double lidar_max_range = 0.0;
static int lidar_last_hit_count = 0;
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

static const char *ROUTE_PATH = "..\\..\\..\\web_state\\route.csv";
static const char *ZONE_PATH = "..\\..\\..\\web_state\\limit_zones.txt";
static const char *STATE_PATH = "..\\..\\..\\web_state\\robot_state.json";
static const char *STATE_TEMP_PATH = "..\\..\\..\\web_state\\robot_state.tmp";

static int point_near_zone(double x, double y, const LimitZone *zone, double clearance);
static void reset_navigation_mode();

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

static long long get_file_mtime(const char *path) {
  STAT_STRUCT file_stat;
  if (STAT_FN(path, &file_stat) != 0) return -1;
#ifdef _WIN32
  return (long long)file_stat.st_mtime;
#else
  return (long long)file_stat.st_mtime;
#endif
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
      0.9,
      -1.55,
      1.78,
      0.0,
  };
  static const char *finger_names[2] = {
      "finger::left",
      "finger::right",
  };
  const double finger_opening = 0.01;

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

static void stop_robot() {
  for (int i = 0; i < 4; ++i) {
    wb_motor_set_velocity(wheels[i], 0.0);
  }
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
  const double dt = (double)TIME_STEP / 1000.0;
  const double heading_step = clamp_value(
      angular_speed * dt,
      -KINEMATIC_ANGULAR_SPEED * dt,
      KINEMATIC_ANGULAR_SPEED * dt);
  const double next_heading = wrap_angle(heading + heading_step);
  const double step_distance = clamp_value(
      linear_speed * dt,
      -KINEMATIC_LINEAR_SPEED * dt,
      KINEMATIC_LINEAR_SPEED * dt);
  const double move_heading = heading + heading_step * 0.5;

  const double translation[3] = {
      x + cos(move_heading) * step_distance,
      z + sin(move_heading) * step_distance,
      START_HEIGHT,
  };
  const double rotation[4] = {0.0, 0.0, 1.0, next_heading};

  wb_supervisor_field_set_sf_rotation(rotation_field, rotation);
  wb_supervisor_field_set_sf_vec3f(translation_field, translation);
  wb_supervisor_node_reset_physics(self_node);
}

static double sign_or_one(double value) {
  return value < 0.0 ? -1.0 : 1.0;
}

static void reset_navigation_mode() {
  navigation_mode = NAV_MODE_IDLE;
  navigation_waypoint_index = -1;
  navigation_segment_start_x = START_X;
  navigation_segment_start_z = START_Z;
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

static double compute_track_heading(double x, double z, const Waypoint *target, double distance_to_target) {
  const double segment_dx = target->x - navigation_segment_start_x;
  const double segment_dz = target->z - navigation_segment_start_z;
  const double segment_length = hypot2(segment_dx, segment_dz);
  if (segment_length <= EPS) {
    return atan2(target->z - z, target->x - x);
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
  const double cross_correction = clamp_value(-cross_track * TRACK_CROSS_TRACK_GAIN, -0.40, 0.40);

  return wrap_angle(atan2(aim_z - z, aim_x - x) + cross_correction);
}

static int point_near_known_dynamic_zone(double x, double y) {
  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (point_near_zone(x, y, &zone_data.zones[zone_index], ZONE_CLEARANCE)) {
      return 1;
    }
  }

  return 0;
}

static void append_obstacle_trace_point(double x, double y) {
  if (!is_finite_double(x) || !is_finite_double(y)) return;
  if (point_near_known_dynamic_zone(x, y)) return;

  for (int i = 0; i < obstacle_trace_count; ++i) {
    const double dx = obstacle_trace[i].x - x;
    const double dy = obstacle_trace[i].y - y;
    if (hypot2(dx, dy) <= LIDAR_TRACE_SPACING) return;
  }

  if (obstacle_trace_count >= MAX_OBSTACLE_TRACE_POINTS) {
    memmove(
        &obstacle_trace[0],
        &obstacle_trace[1],
        sizeof(ObstacleTracePoint) * (MAX_OBSTACLE_TRACE_POINTS - 1));
    obstacle_trace_count = MAX_OBSTACLE_TRACE_POINTS - 1;
  }

  obstacle_trace[obstacle_trace_count].x = x;
  obstacle_trace[obstacle_trace_count].y = y;
  obstacle_trace_count += 1;
}

static void capture_lidar_trace() {
  lidar_last_hit_count = 0;
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
  const double effective_max_range =
      lidar_max_range > EPS ? fmin(lidar_max_range, LIDAR_MAX_TRACE_RANGE) : LIDAR_MAX_TRACE_RANGE;

  for (int i = 0; i < lidar_resolution; i += LIDAR_SAMPLE_STRIDE) {
    const double range = (double)ranges[i];
    if (!is_finite_double(range)) continue;
    if (range < LIDAR_MIN_TRACE_RANGE || range > effective_max_range) continue;

    const double alpha =
        lidar_resolution > 1 ? (double)i / (double)(lidar_resolution - 1) : 0.5;
    const double beam_angle = -0.5 * lidar_fov + alpha * lidar_fov;
    const double world_angle = heading + beam_angle;
    const double hit_x = sensor_origin_x + cos(world_angle) * range;
    const double hit_y = sensor_origin_y + sin(world_angle) * range;

    append_obstacle_trace_point(hit_x, hit_y);
    lidar_last_hit_count += 1;
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

static int segment_blocked_by_zones(
    double ax,
    double ay,
    double bx,
    double by,
    double clearance) {
  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
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
    reset_navigation_mode();
    set_status("route_reloaded");
  }
}

static void wait_for_fresh_route() {
  route_data.count = 0;
  route_data.last_modified = get_file_mtime(ROUTE_PATH);
  current_waypoint_index = 0;
  route_finished = 0;
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

  if (route_data.count == 0) {
    set_status("waiting_for_route");
    route_finished = 0;
    reset_navigation_mode();
    distance_to_target = 0.0;
    stop_robot();
    return;
  }

  if (route_finished) {
    set_status("finished");
    reset_navigation_mode();
    distance_to_target = 0.0;
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

  for (int zone_index = 0; zone_index < zone_data.count; ++zone_index) {
    if (point_near_zone(target.x, target.z, &zone_data.zones[zone_index], ZONE_CLEARANCE)) {
      set_status("blocked_by_dynamic_zone");
      set_error("Current waypoint is blocked by a dynamic zone");
      distance_to_target = hypot2(target.x - x, target.z - z);
      stop_robot();
      return;
    }
  }

  if (segment_blocked_by_zones(x, z, target.x, target.z, ZONE_CLEARANCE)) {
    set_status("blocked_by_dynamic_zone");
    set_error("Dynamic zone blocks the current segment");
    distance_to_target = hypot2(target.x - x, target.z - z);
    stop_robot();
    return;
  }

  const double dx = target.x - x;
  const double dz = target.z - z;
  distance_to_target = hypot2(dx, dz);
  const double heading_to_target = atan2(dz, dx);
  const double track_heading = compute_track_heading(x, z, &target, distance_to_target);
  const double heading_error_to_target = wrap_angle(heading_to_target - heading);
  const double heading_error_on_track = wrap_angle(track_heading - heading);

  if (navigation_mode == NAV_MODE_IDLE) {
    navigation_mode = NAV_MODE_TURN;
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
          -KINEMATIC_ANGULAR_SPEED,
          KINEMATIC_ANGULAR_SPEED);
      if (fabs(heading_error) > HEADING_TOLERANCE_RAD && fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
        angular_speed = sign_or_one(heading_error) * MIN_ANGULAR_COMMAND;
      }
      clear_error();
      set_status("aligning_final_heading");
      apply_kinematic_step(x, z, heading, 0.0, angular_speed);
      return;
    }
  }

  if (navigation_mode == NAV_MODE_TRACK &&
      fabs(heading_error_on_track) > TRACK_REENTER_TURN_RAD &&
      distance_to_target > TRACK_SLOW_RADIUS) {
    navigation_mode = NAV_MODE_TURN;
  }

  if (navigation_mode == NAV_MODE_TURN) {
    if (fabs(heading_error_to_target) <= TURN_EXIT_ERROR_RAD ||
        distance_to_target <= TRACK_SLOW_RADIUS * 0.5) {
      navigation_mode = NAV_MODE_TRACK;
    } else {
      angular_speed = clamp_value(
          heading_error_to_target * TURN_HEADING_GAIN,
          -KINEMATIC_ANGULAR_SPEED,
          KINEMATIC_ANGULAR_SPEED);
      if (fabs(heading_error_to_target) > HEADING_TOLERANCE_RAD &&
          fabs(angular_speed) < MIN_ANGULAR_COMMAND) {
        angular_speed = sign_or_one(heading_error_to_target) * MIN_ANGULAR_COMMAND;
      }
      clear_error();
      set_status("turning_to_path");
      apply_kinematic_step(x, z, heading, 0.0, angular_speed);
      return;
    }
  }

  const double updated_track_heading = compute_track_heading(x, z, &target, distance_to_target);
  const double updated_heading_error = wrap_angle(updated_track_heading - heading);

  if (fabs(updated_heading_error) > TRACK_REENTER_TURN_RAD &&
      distance_to_target > TRACK_SLOW_RADIUS) {
    navigation_mode = NAV_MODE_TURN;
    angular_speed = clamp_value(
        updated_heading_error * TURN_HEADING_GAIN,
        -KINEMATIC_ANGULAR_SPEED,
        KINEMATIC_ANGULAR_SPEED);
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
      -KINEMATIC_ANGULAR_SPEED,
      KINEMATIC_ANGULAR_SPEED);

  double base_speed = clamp_value(distance_to_target * 1.1, TRACK_MIN_LINEAR_SPEED, KINEMATIC_LINEAR_SPEED);
  if (distance_to_target < TRACK_SLOW_RADIUS) {
    base_speed = clamp_value(distance_to_target * 0.95, TRACK_MIN_LINEAR_SPEED, 0.14);
  }

  double heading_scale = clamp_value(1.0 - fabs(updated_heading_error) / 0.85, 0.22, 1.0);
  if (fabs(updated_heading_error) > TURN_ENTER_ERROR_RAD) {
    heading_scale = fmin(heading_scale, 0.35);
  }

  linear_speed = base_speed * heading_scale;
  if (distance_to_target <= POSITION_TOLERANCE * 1.4) {
    linear_speed = fmin(linear_speed, 0.05);
  }

  clear_error();
  set_status("tracking_path");
  apply_kinematic_step(x, z, heading, linear_speed, angular_speed);
}

static void write_state_snapshot() {
  FILE *file = fopen(STATE_TEMP_PATH, "w");
  if (!file) return;

  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  read_pose(&x, &y, &heading);

  fprintf(file, "{\n");
  fprintf(file, "  \"coordinateContractVersion\": 1,\n");
  fprintf(file, "  \"simulationTime\": %.6f,\n", wb_robot_get_time());
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
  fprintf(file, "  \"dynamicZones\": {\n");
  fprintf(file, "    \"count\": %d,\n", zone_data.count);
  fprintf(file, "    \"wallCount\": %d\n", zone_node_count);
  fprintf(file, "  },\n");
  fprintf(file, "  \"perception\": {\n");
  fprintf(file, "    \"lidar\": {\n");
  fprintf(file, "      \"enabled\": %s,\n", lidar_available ? "true" : "false");
  fprintf(file, "      \"horizontalResolution\": %d,\n", lidar_resolution);
  fprintf(file, "      \"maxRange\": %.3f,\n", lidar_max_range);
  fprintf(file, "      \"lastHitCount\": %d\n", lidar_last_hit_count);
  fprintf(file, "    },\n");
  fprintf(file, "    \"obstacleTrace\": [\n");
  for (int i = 0; i < obstacle_trace_count; ++i) {
    fprintf(file,
            "      {\"x\": %.6f, \"y\": %.6f}%s\n",
            obstacle_trace[i].x,
            obstacle_trace[i].y,
            i + 1 < obstacle_trace_count ? "," : "");
  }
  fprintf(file, "    ]\n");
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
  wb_robot_init();
  init_wheels();
  init_manipulator_pose();
  init_lidar();
  init_pose_tracking();
  reset_robot_pose();
  maybe_reload_zones();

  if (!translation_field || !rotation_field || !root_children_field) {
    set_status("error");
    set_error("Supervisor fields are not available");
  } else {
    wait_for_fresh_route();
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    ++step_counter;
    maybe_reload_zones();
    maybe_reload_route();
    capture_lidar_trace();
    run_navigation_step();
    write_state_snapshot();
  }

  remove_zone_nodes();
  wb_robot_cleanup();
  return 0;
}
