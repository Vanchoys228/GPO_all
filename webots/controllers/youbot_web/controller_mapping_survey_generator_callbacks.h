#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_GENERATOR_CALLBACKS_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_SURVEY_GENERATOR_CALLBACKS_H

#include "controller_survey_generator.h"

typedef struct {
  void (*clear_map)(void *context);
  void (*prepare)(void *context, MappingSurveyMode mode);
  SurveyPoint (*read_robot)(void *context);
  int (*find_room)(void *context, SurveyPoint robot);
  int (*build_grid)(
      void *context,
      SurveyGrid *grid,
      SurveyPoint robot,
      int room_zone_index,
      const RuntimeCommand *command);
  int (*flood_grid)(void *context, SurveyGrid *grid, SurveyPoint robot);
  int (*start_is_safe)(
      void *context,
      SurveyPoint start,
      int room_zone_index,
      double clearance);
  void (*add_route_point)(
      void *context,
      SurveyPoint *route,
      int *route_count,
      SurveyPoint point);
  int (*append_room_contour)(
      void *context,
      SurveyGrid *grid,
      SurveyPoint *route,
      int *route_count,
      int room_zone_index,
      SurveyPoint robot);
  void (*append_grid_boundary)(
      void *context,
      SurveyGrid *grid,
      SurveyPoint *route,
      int *route_count,
      SurveyPoint robot);
  void (*append_horizontal_coverage)(
      void *context,
      SurveyGrid *grid,
      SurveyPoint *route,
      int *route_count,
      int room_zone_index);
  void (*append_vertical_coverage)(
      void *context,
      SurveyGrid *grid,
      SurveyPoint *route,
      int *route_count,
      int room_zone_index);
  void (*write_route)(
      void *context,
      const char *path,
      MappingSurveyMode mode,
      const SurveyPoint *route,
      int route_count);
} ControllerMappingSurveyGeneratorCallbackOperations;

typedef struct {
  const ControllerMappingSurveyGeneratorCallbackOperations *operations;
  void *operations_context;
  const char *path;
  const RuntimeCommand *command;
  double start_clearance;
  SurveyGrid grid;
} ControllerMappingSurveyGeneratorCallbacksAdapter;

void controller_mapping_survey_generator_callbacks_adapter_init(
    ControllerMappingSurveyGeneratorCallbacksAdapter *adapter,
    const ControllerMappingSurveyGeneratorCallbackOperations *operations,
    void *operations_context,
    const char *path,
    const RuntimeCommand *command,
    double start_clearance);
const ControllerSurveyGeneratorCallbacks *
controller_mapping_survey_generator_callbacks_adapter_callbacks(
    const ControllerMappingSurveyGeneratorCallbacksAdapter *adapter);

#endif
