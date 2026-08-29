#include "controller_survey_geometry.h"
#include "controller_survey_intervals.h"

#include "controller_zone_geometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

int controller_survey_nearest_point_index(
    const SurveyPoint *points,
    int count,
    double x,
    double y) {
  if (!points || count <= 0) return -1;
  int nearest = 0;
  double best_distance = 1e30;
  for (int i = 0; i < count; ++i) {
    const double distance = hypot(points[i].x - x, points[i].y - y);
    if (distance < best_distance) {
      best_distance = distance;
      nearest = i;
    }
  }
  return nearest;
}

void controller_survey_expand_bounds(
    double x, double y, double *min_x, double *max_x, double *min_y, double *max_y) {
  if (!min_x || !max_x || !min_y || !max_y) return;
  if (x < *min_x) *min_x = x;
  if (x > *max_x) *max_x = x;
  if (y < *min_y) *min_y = y;
  if (y > *max_y) *max_y = y;
}

void controller_survey_route_add(
    SurveyPoint *route, int *count, int capacity, double min_spacing, double x, double y) {
  if (!route || !count || *count < 0 || *count >= capacity) return;
  if (*count > 0) {
    SurveyPoint *last = &route[*count - 1];
    if (hypot(last->x - x, last->y - y) < min_spacing) {
      last->x = x;
      last->y = y;
      return;
    }
  }
  route[*count] = (SurveyPoint){x, y};
  *count += 1;
}

void controller_survey_route_add_segment(
    SurveyPoint *route,
    int *count,
    int capacity,
    double min_spacing,
    double max_step,
    SurveyPoint from,
    SurveyPoint to) {
  if (max_step <= 0.0) return;
  const double length = hypot(to.x - from.x, to.y - from.y);
  const int steps = (int)fmax(1.0, ceil(length / max_step));
  for (int i = 1; i <= steps; ++i) {
    const double t = (double)i / (double)steps;
    controller_survey_route_add(
        route,
        count,
        capacity,
        min_spacing,
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t);
  }
}

static double zone_primary(const LimitZone *zone, int index, int vertical) {
  return vertical ? zone->points[index].x : zone->points[index].y;
}

static double zone_secondary(const LimitZone *zone, int index, int vertical) {
  return vertical ? zone->points[index].y : zone->points[index].x;
}

static void subtract_zone_band(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    const LimitZone *zone,
    double coordinate,
    double clearance,
    double epsilon,
    int vertical) {
  if (!intervals || !count || !zone || zone->point_count < 3 || *count <= 0) return;

  double min_primary = zone_primary(zone, 0, vertical);
  double max_primary = min_primary;
  double near_min = 1e30;
  double near_max = -1e30;
  int near_band_hit = 0;
  for (int i = 0; i < zone->point_count; ++i) {
    const double primary = zone_primary(zone, i, vertical);
    const double secondary = zone_secondary(zone, i, vertical);
    if (primary < min_primary) min_primary = primary;
    if (primary > max_primary) max_primary = primary;
    if (fabs(primary - coordinate) <= clearance) {
      if (secondary < near_min) near_min = secondary;
      if (secondary > near_max) near_max = secondary;
      near_band_hit = 1;
    }
  }
  if (coordinate < min_primary - clearance || coordinate > max_primary + clearance) return;

  static const double sample_offsets[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    const double sample = coordinate + sample_offsets[sample_index] * clearance;
    double intersections[MAX_ZONE_POINTS];
    int intersection_count = 0;
    for (int i = 0; i < zone->point_count; ++i) {
      const int next = (i + 1) % zone->point_count;
      const double primary_a = zone_primary(zone, i, vertical);
      const double secondary_a = zone_secondary(zone, i, vertical);
      const double primary_b = zone_primary(zone, next, vertical);
      const double secondary_b = zone_secondary(zone, next, vertical);
      if (fabs(primary_a - primary_b) < epsilon) {
        if (fabs(primary_a - coordinate) <= clearance) {
          if (fmin(secondary_a, secondary_b) < near_min) near_min = fmin(secondary_a, secondary_b);
          if (fmax(secondary_a, secondary_b) > near_max) near_max = fmax(secondary_a, secondary_b);
          near_band_hit = 1;
        }
        continue;
      }
      if ((primary_a <= sample && primary_b > sample) ||
          (primary_b <= sample && primary_a > sample)) {
        intersections[intersection_count++] =
            secondary_a + (sample - primary_a) * (secondary_b - secondary_a) /
                (primary_b - primary_a);
      }
    }
    controller_survey_sort_values(intersections, intersection_count);
    for (int i = 0; i + 1 < intersection_count; i += 2) {
      controller_survey_subtract_interval(
          intervals,
          count,
          capacity,
          intersections[i] - clearance,
          intersections[i + 1] + clearance);
    }
  }
  if (near_band_hit && near_max >= near_min) {
    controller_survey_subtract_interval(
        intervals, count, capacity, near_min - clearance, near_max + clearance);
  }
}

void controller_survey_subtract_zone_horizontal_band(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    const LimitZone *zone,
    double y,
    double clearance,
    double epsilon) {
  subtract_zone_band(intervals, count, capacity, zone, y, clearance, epsilon, 0);
}

void controller_survey_subtract_zone_vertical_band(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    const LimitZone *zone,
    double x,
    double clearance,
    double epsilon) {
  subtract_zone_band(intervals, count, capacity, zone, x, clearance, epsilon, 1);
}

static int build_axis_intervals(
    double coordinate,
    int room_zone_index,
    const SurveyGrid *grid,
    const ZoneData *zones,
    const MapCell *map,
    int map_count,
    double interior_offset,
    double min_strip_length,
    double epsilon,
    SurveyInterval *intervals,
    int capacity,
    int vertical) {
  if (!intervals || capacity <= 0) return 0;
  int count = 0;
  if (zones && room_zone_index >= 0 && room_zone_index < zones->count) {
    const LimitZone *room = &zones->zones[room_zone_index];
    double intersections[MAX_ZONE_POINTS];
    int intersection_count = 0;
    for (int i = 0; i < room->point_count; ++i) {
      const int next = (i + 1) % room->point_count;
      const double primary_a = zone_primary(room, i, vertical);
      const double secondary_a = zone_secondary(room, i, vertical);
      const double primary_b = zone_primary(room, next, vertical);
      const double secondary_b = zone_secondary(room, next, vertical);
      if (fabs(primary_a - primary_b) < epsilon) continue;
      if ((primary_a <= coordinate && primary_b > coordinate) ||
          (primary_b <= coordinate && primary_a > coordinate)) {
        intersections[intersection_count++] =
            secondary_a + (coordinate - primary_a) * (secondary_b - secondary_a) /
                (primary_b - primary_a);
      }
    }
    controller_survey_sort_values(intersections, intersection_count);
    for (int i = 0; i + 1 < intersection_count && count < capacity; i += 2) {
      const double start = intersections[i] + interior_offset;
      const double end = intersections[i + 1] - interior_offset;
      if (end - start >= min_strip_length) intervals[count++] = (SurveyInterval){start, end};
    }
  } else if (grid) {
    const double grid_origin = vertical ? grid->min_x : grid->min_y;
    const int fixed = (int)round((coordinate - grid_origin) / grid->cell);
    const int fixed_limit = vertical ? grid->width : grid->height;
    const int run_limit = vertical ? grid->height : grid->width;
    const double run_origin = vertical ? grid->min_y : grid->min_x;
    if (fixed >= 0 && fixed < fixed_limit) {
      int run_start = -1;
      for (int run = 0; run <= run_limit; ++run) {
        const int index = vertical ? run * grid->width + fixed : fixed * grid->width + run;
        const int in_component = run < run_limit && grid->component_cell[index];
        if (in_component && run_start < 0) run_start = run;
        if ((!in_component || run == run_limit) && run_start >= 0) {
          const int run_end = run - 1;
          const double start = run_origin + (double)run_start * grid->cell + interior_offset;
          const double end = run_origin + (double)run_end * grid->cell - interior_offset;
          if (end - start >= min_strip_length && count < capacity) {
            intervals[count++] = (SurveyInterval){start, end};
          }
          run_start = -1;
        }
      }
    }
  }

  if (zones) {
    for (int zone_index = 0; zone_index < zones->count; ++zone_index) {
      if (zone_index == room_zone_index) continue;
      subtract_zone_band(
          intervals,
          &count,
          capacity,
          &zones->zones[zone_index],
          coordinate,
          interior_offset,
          epsilon,
          vertical);
    }
  }
  for (int i = 0; map && i < map_count; ++i) {
    const double primary = vertical ? map[i].x : map[i].y;
    const double secondary = vertical ? map[i].y : map[i].x;
    if (fabs(primary - coordinate) > interior_offset) continue;
    controller_survey_subtract_interval(
        intervals,
        &count,
        capacity,
        secondary - interior_offset,
        secondary + interior_offset);
  }

  int write = 0;
  for (int i = 0; i < count; ++i) {
    if (intervals[i].end - intervals[i].start < min_strip_length) continue;
    intervals[write++] = intervals[i];
  }
  return write;
}

int controller_survey_build_horizontal_intervals(
    double y,
    int room_zone_index,
    const SurveyGrid *grid,
    const ZoneData *zones,
    const MapCell *map,
    int map_count,
    double interior_offset,
    double min_strip_length,
    double epsilon,
    SurveyInterval *intervals,
    int capacity) {
  return build_axis_intervals(
      y, room_zone_index, grid, zones, map, map_count, interior_offset,
      min_strip_length, epsilon, intervals, capacity, 0);
}

int controller_survey_build_vertical_intervals(
    double x,
    int room_zone_index,
    const SurveyGrid *grid,
    const ZoneData *zones,
    const MapCell *map,
    int map_count,
    double interior_offset,
    double min_strip_length,
    double epsilon,
    SurveyInterval *intervals,
    int capacity) {
  return build_axis_intervals(
      x, room_zone_index, grid, zones, map, map_count, interior_offset,
      min_strip_length, epsilon, intervals, capacity, 1);
}

static double point_segment_distance(SurveyPoint point, SurveyPoint start, SurveyPoint end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-12) return hypot(point.x - start.x, point.y - start.y);
  double t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  return hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy));
}

void controller_survey_rdp_keep(
    const SurveyPoint *points, int first, int last, double epsilon, unsigned char *keep) {
  if (!points || !keep || first < 0 || last <= first + 1) return;
  double best_distance = 0.0;
  int best_index = -1;
  for (int i = first + 1; i < last; ++i) {
    const double distance = point_segment_distance(points[i], points[first], points[last]);
    if (distance > best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  if (best_index >= 0 && best_distance > epsilon) {
    keep[best_index] = 1;
    controller_survey_rdp_keep(points, first, best_index, epsilon, keep);
    controller_survey_rdp_keep(points, best_index, last, epsilon, keep);
  }
}

int controller_survey_find_grid_path(
    SurveyGrid *grid,
    SurveyPoint from,
    SurveyPoint to,
    SurveyPoint *path,
    int *path_count,
    int max_path_count) {
  if (!grid || !path || !path_count || max_path_count <= 0) return 0;
  const int start = controller_survey_grid_index_for_point(grid, from.x, from.y);
  const int goal = controller_survey_grid_index_for_point(grid, to.x, to.y);
  if (start < 0 || goal < 0 || !grid->component_cell[start] || !grid->component_cell[goal]) return 0;

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
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = cx + dx[direction];
      const int ny = cy + dy[direction];
      if (nx < 0 || nx >= grid->width || ny < 0 || ny >= grid->height) continue;
      const int neighbor = ny * grid->width + nx;
      if (!grid->component_cell[neighbor] || grid->visited_cell[neighbor]) continue;
      grid->visited_cell[neighbor] = 1;
      grid->parent[neighbor] = current;
      grid->queue[tail++] = neighbor;
    }
  }
  if (!grid->visited_cell[goal]) return 0;

  int *reverse = grid->queue;
  int reverse_count = 0;
  int reached_start = 0;
  for (int current = goal;
       current >= 0 && reverse_count < MAPPING_SURVEY_MAX_GRID_CELLS;
       current = grid->parent[current]) {
    reverse[reverse_count++] = current;
    if (current == start) {
      reached_start = 1;
      break;
    }
  }
  if (reverse_count <= 0 || !reached_start) return 0;

  *path_count = 0;
  for (int i = reverse_count - 1; i >= 0; i -= 3) {
    if (*path_count >= max_path_count) {
      *path_count = 0;
      return 0;
    }
    path[*path_count] = controller_survey_grid_point(grid, reverse[i]);
    *path_count += 1;
  }
  if (path[*path_count - 1].x != to.x || path[*path_count - 1].y != to.y) {
    if (*path_count >= max_path_count) {
      *path_count = 0;
      return 0;
    }
    path[*path_count] = to;
    *path_count += 1;
  }
  return 1;
}

int controller_survey_append_boundary_contour(
    const SurveyGrid *grid,
    SurveyPoint *route,
    int *route_count,
    int route_capacity,
    int boundary_capacity,
    double robot_x,
    double robot_y,
    double min_route_spacing,
    double max_route_step,
    double max_join_cell_factor,
    double rdp_epsilon) {
  if (!grid || !route || !route_count || boundary_capacity < 3) return 0;
  if (boundary_capacity > CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS) {
    boundary_capacity = CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS;
  }
  SurveyPoint boundary[CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS];
  unsigned char used[CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS];
  int boundary_count = 0;
  for (int i = 0; i < grid->count && boundary_count < boundary_capacity; ++i) {
    if (!controller_survey_cell_is_boundary(grid, i)) continue;
    boundary[boundary_count++] = controller_survey_grid_point(grid, i);
  }
  if (boundary_count < 3) return 0;

  memset(used, 0, (size_t)boundary_count);
  SurveyPoint ordered[CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS];
  int ordered_count = 0;
  int current = 0;
  double best_dist = 1e30;
  for (int i = 0; i < boundary_count; ++i) {
    const double distance = hypot(boundary[i].x - robot_x, boundary[i].y - robot_y);
    if (distance < best_dist) {
      best_dist = distance;
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
      const double distance = hypot(
          boundary[i].x - boundary[current].x, boundary[i].y - boundary[current].y);
      if (distance < next_dist) {
        next_dist = distance;
        next = i;
      }
    }
    if (next < 0 || next_dist > grid->cell * max_join_cell_factor) break;
    current = next;
  }
  if (ordered_count < 3) return 0;

  unsigned char keep[CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS];
  memset(keep, 0, (size_t)ordered_count);
  keep[0] = 1;
  keep[ordered_count - 1] = 1;
  controller_survey_rdp_keep(ordered, 0, ordered_count - 1, rdp_epsilon, keep);
  SurveyPoint simplified[CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS];
  int simplified_count = 0;
  for (int i = 0; i < ordered_count; ++i) {
    if (keep[i]) simplified[simplified_count++] = ordered[i];
  }
  if (simplified_count < 3) return 0;

  controller_survey_route_add(
      route, route_count, route_capacity, min_route_spacing,
      simplified[0].x, simplified[0].y);
  for (int i = 1; i < simplified_count; ++i) {
    controller_survey_route_add_segment(
        route, route_count, route_capacity, min_route_spacing, max_route_step,
        simplified[i - 1], simplified[i]);
  }
  controller_survey_route_add_segment(
      route, route_count, route_capacity, min_route_spacing, max_route_step,
      simplified[simplified_count - 1], simplified[0]);
  return 1;
}

int controller_survey_clip_intervals(
    const SurveyInterval *raw,
    int raw_count,
    double min_value,
    double max_value,
    double min_length,
    SurveyInterval *clipped,
    int capacity) {
  if (!raw || raw_count <= 0 || !clipped || capacity <= 0) return 0;
  int count = 0;
  for (int i = 0; i < raw_count && count < capacity; ++i) {
    const double start = fmax(raw[i].start, min_value);
    const double end = fmin(raw[i].end, max_value);
    if (end - start < min_length) continue;
    clipped[count++] = (SurveyInterval){start, end};
  }
  return count;
}

void controller_survey_reverse_intervals(SurveyInterval *intervals, int count) {
  if (!intervals || count <= 1) return;
  for (int low = 0, high = count - 1; low < high; ++low, --high) {
    const SurveyInterval temporary = intervals[low];
    intervals[low] = intervals[high];
    intervals[high] = temporary;
  }
}

void controller_survey_choose_snake_start(
    SurveyPoint current,
    SurveyPoint low_endpoint,
    SurveyPoint high_endpoint,
    int *start_from_low,
    double *best_distance) {
  if (!start_from_low || !best_distance) return;
  const double low_distance = hypot(low_endpoint.x - current.x, low_endpoint.y - current.y);
  const double high_distance = hypot(high_endpoint.x - current.x, high_endpoint.y - current.y);
  *start_from_low = low_distance <= high_distance ? 1 : 0;
  *best_distance = fmin(low_distance, high_distance);
}
