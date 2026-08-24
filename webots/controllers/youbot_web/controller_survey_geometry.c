#include "controller_survey_geometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

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

void controller_survey_sort_values(double *values, int count) {
  if (!values || count <= 1) return;
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

void controller_survey_subtract_interval(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    double block_start,
    double block_end) {
  if (!intervals || !count || *count <= 0 || capacity <= 0) return;
  SurveyInterval next[64];
  const int effective_capacity = capacity < 64 ? capacity : 64;
  int next_count = 0;
  if (block_end < block_start) {
    const double tmp = block_start;
    block_start = block_end;
    block_end = tmp;
  }
  for (int i = 0; i < *count && next_count < effective_capacity; ++i) {
    const SurveyInterval current = intervals[i];
    if (block_end <= current.start || block_start >= current.end) {
      next[next_count++] = current;
      continue;
    }
    if (block_start > current.start) {
      next[next_count++] = (SurveyInterval){current.start, fmin(block_start, current.end)};
    }
    if (block_end < current.end && next_count < effective_capacity) {
      next[next_count++] = (SurveyInterval){fmax(block_end, current.start), current.end};
    }
  }
  for (int i = 0; i < next_count; ++i) intervals[i] = next[i];
  *count = next_count;
}

int controller_survey_grid_index_for_point(const SurveyGrid *grid, double x, double y) {
  if (!grid || grid->cell <= 0.0 || grid->width <= 0 || grid->height <= 0) return -1;
  int gx = (int)round((x - grid->min_x) / grid->cell);
  int gy = (int)round((y - grid->min_y) / grid->cell);
  if (gx < 0) gx = 0;
  if (gx >= grid->width) gx = grid->width - 1;
  if (gy < 0) gy = 0;
  if (gy >= grid->height) gy = grid->height - 1;
  return gy * grid->width + gx;
}

SurveyPoint controller_survey_grid_point(const SurveyGrid *grid, int index) {
  if (!grid || grid->width <= 0 || index < 0 || index >= grid->count) {
    return (SurveyPoint){0.0, 0.0};
  }
  const int gx = index % grid->width;
  const int gy = index / grid->width;
  return (SurveyPoint){
      grid->min_x + (double)gx * grid->cell,
      grid->min_y + (double)gy * grid->cell,
  };
}

int controller_survey_flood_component(SurveyGrid *grid, double robot_x, double robot_y) {
  if (!grid || grid->count <= 0) return 0;
  int start = controller_survey_grid_index_for_point(grid, robot_x, robot_y);
  if (start < 0) return 0;
  if (!grid->free_cell[start]) {
    double best_dist = 1e30;
    for (int i = 0; i < grid->count; ++i) {
      if (!grid->free_cell[i]) continue;
      const SurveyPoint point = controller_survey_grid_point(grid, i);
      const double dist = hypot(point.x - robot_x, point.y - robot_y);
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
    for (int direction = 0; direction < 4; ++direction) {
      const int nx = cx + dx[direction];
      const int ny = cy + dy[direction];
      if (nx < 0 || nx >= grid->width || ny < 0 || ny >= grid->height) continue;
      const int neighbor = ny * grid->width + nx;
      if (!grid->free_cell[neighbor] || grid->component_cell[neighbor]) continue;
      grid->component_cell[neighbor] = 1;
      grid->queue[tail++] = neighbor;
    }
  }
  return component_count;
}

int controller_survey_cell_is_boundary(const SurveyGrid *grid, int index) {
  if (!grid || index < 0 || index >= grid->count || !grid->component_cell[index]) return 0;
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
