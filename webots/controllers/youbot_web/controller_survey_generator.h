#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_GENERATOR_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_GENERATOR_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_SURVEY_GENERATE_OK = 0,
  CONTROLLER_SURVEY_GENERATE_GRID_FAILED,
  CONTROLLER_SURVEY_GENERATE_NO_COMPONENT,
  CONTROLLER_SURVEY_GENERATE_EMPTY_ROUTE,
} ControllerSurveyGenerateResult;

typedef struct {
  void (*clear_map)(void *context);
  void (*prepare)(void *context, MappingSurveyMode mode);
  SurveyPoint (*read_robot)(void *context);
  int (*find_room)(void *context, SurveyPoint robot);
  int (*build_grid)(void *context, SurveyPoint robot, int room_zone_index);
  int (*flood_grid)(void *context, SurveyPoint robot);
  int (*build_route)(
      void *context,
      MappingSurveyMode mode,
      SurveyPoint robot,
      int room_zone_index,
      SurveyPoint *route,
      int *route_count,
      int *interior_start_index);
  void (*write_route)(
      void *context,
      MappingSurveyMode mode,
      const SurveyPoint *route,
      int route_count);
} ControllerSurveyGeneratorCallbacks;

ControllerSurveyGenerateResult controller_survey_generate(
    int clear_map_before_start,
    MappingSurveyMode mode,
    int *room_zone_index,
    int *interior_start_index,
    const ControllerSurveyGeneratorCallbacks *callbacks,
    void *context);

#endif
