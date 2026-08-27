#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_GEOMETRY_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_GEOMETRY_H

#include "controller_types.h"

#define CONTROLLER_SURVEY_MAX_BOUNDARY_POINTS 4096

int controller_survey_build_offset_contour(
    const LimitZone *room,
    double offset,
    SurveyPoint *out,
    int capacity,
    int *out_count);
int controller_survey_nearest_point_index(
    const SurveyPoint *points,
    int count,
    double x,
    double y);

void controller_survey_expand_bounds(
    double x, double y, double *min_x, double *max_x, double *min_y, double *max_y);
void controller_survey_route_add(
    SurveyPoint *route, int *count, int capacity, double min_spacing, double x, double y);
void controller_survey_route_add_segment(
    SurveyPoint *route,
    int *count,
    int capacity,
    double min_spacing,
    double max_step,
    SurveyPoint from,
    SurveyPoint to);
void controller_survey_sort_values(double *values, int count);
void controller_survey_subtract_interval(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    double block_start,
    double block_end);
void controller_survey_subtract_zone_horizontal_band(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    const LimitZone *zone,
    double y,
    double clearance,
    double epsilon);
void controller_survey_subtract_zone_vertical_band(
    SurveyInterval *intervals,
    int *count,
    int capacity,
    const LimitZone *zone,
    double x,
    double clearance,
    double epsilon);
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
    int capacity);
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
    int capacity);
void controller_survey_get_coverage_bounds(
    const SurveyGrid *grid,
    const ZoneData *zones,
    int room_zone_index,
    double interior_offset,
    double *min_x,
    double *max_x,
    double *min_y,
    double *max_y);
void controller_survey_select_sweep_start(
    int has_low,
    int low_positive,
    double low_distance,
    int has_high,
    int high_positive,
    double high_distance,
    int *sweep_from_high,
    int *start_positive);
int controller_survey_grid_index_for_point(const SurveyGrid *grid, double x, double y);
SurveyPoint controller_survey_grid_point(const SurveyGrid *grid, int index);
int controller_survey_flood_component(SurveyGrid *grid, double robot_x, double robot_y);
int controller_survey_cell_is_boundary(const SurveyGrid *grid, int index);
void controller_survey_rdp_keep(
    const SurveyPoint *points, int first, int last, double epsilon, unsigned char *keep);
int controller_survey_find_grid_path(
    SurveyGrid *grid,
    SurveyPoint from,
    SurveyPoint to,
    SurveyPoint *path,
    int *path_count,
    int max_path_count);
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
    double rdp_epsilon);
int controller_survey_clip_intervals(
    const SurveyInterval *raw,
    int raw_count,
    double min_value,
    double max_value,
    double min_length,
    SurveyInterval *clipped,
    int capacity);
void controller_survey_reverse_intervals(SurveyInterval *intervals, int count);
void controller_survey_choose_snake_start(
    SurveyPoint current,
    SurveyPoint low_endpoint,
    SurveyPoint high_endpoint,
    int *start_from_low,
    double *best_distance);

#endif
