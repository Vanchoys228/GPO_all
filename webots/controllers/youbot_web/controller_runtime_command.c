#include "controller_runtime_command.h"

#include "controller_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CONTROLLER_RUNTIME_EPS 1e-9

int controller_runtime_command_load_file(
    const char *path,
    const ControllerRuntimeCommandLimits *limits,
    RuntimeCommand *command) {
  if (!path || !limits || !command) return 0;
  FILE *file = fopen(path, "r");
  if (!file) return 0;

  RuntimeCommand parsed = {0};
  parsed.id = -1;
  parsed.clear_map = 1;
  parsed.survey_mode = MAPPING_SURVEY_MODE_SNAKE;
  parsed.field_min_x = limits->default_min_x;
  parsed.field_max_x = limits->default_max_x;
  parsed.field_min_y = limits->default_min_y;
  parsed.field_max_y = limits->default_max_y;
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
    } else if (sscanf(line, " type %63s", token) == 1) {
      parsed.has_spawn_obstacle = strcmp(token, "spawn_obstacle") == 0;
      parsed.has_start_mapping_survey = strcmp(token, "start_mapping_survey") == 0;
    } else if (sscanf(line, " clear_map %lf", &numeric) == 1) {
      parsed.clear_map = fabs(numeric) > CONTROLLER_RUNTIME_EPS;
    } else if (sscanf(line, " mode %63s", token) == 1) {
      parsed.survey_mode = controller_parse_mapping_survey_mode(token);
    } else if (sscanf(line, " survey_speed_mps %lf", &numeric) == 1) {
      parsed.survey_speed_mps = numeric;
    } else if (sscanf(line, " field_min_x %lf", &numeric) == 1) {
      parsed.field_min_x = numeric;
      parsed.has_field_bounds = 1;
    } else if (sscanf(line, " field_max_x %lf", &numeric) == 1) {
      parsed.field_max_x = numeric;
      parsed.has_field_bounds = 1;
    } else if (sscanf(line, " field_min_y %lf", &numeric) == 1) {
      parsed.field_min_y = numeric;
      parsed.has_field_bounds = 1;
    } else if (sscanf(line, " field_max_y %lf", &numeric) == 1) {
      parsed.field_max_y = numeric;
      parsed.has_field_bounds = 1;
    } else if (sscanf(line, " x %lf", &numeric) == 1) {
      parsed.x = numeric;
    } else if (sscanf(line, " y %lf", &numeric) == 1) {
      parsed.y = numeric;
    } else if (sscanf(line, " size_x %lf", &numeric) == 1) {
      parsed.size_x = numeric;
    } else if (sscanf(line, " size_y %lf", &numeric) == 1) {
      parsed.size_y = numeric;
    } else if (sscanf(line, " height %lf", &numeric) == 1) {
      parsed.height = numeric;
    }
  }
  fclose(file);

  if (parsed.has_field_bounds) {
    if (parsed.field_min_x > parsed.field_max_x) {
      const double tmp = parsed.field_min_x;
      parsed.field_min_x = parsed.field_max_x;
      parsed.field_max_x = tmp;
    }
    if (parsed.field_min_y > parsed.field_max_y) {
      const double tmp = parsed.field_min_y;
      parsed.field_min_y = parsed.field_max_y;
      parsed.field_max_y = tmp;
    }
    parsed.field_min_x = clamp_value(parsed.field_min_x, -limits->max_extent_x, limits->max_extent_x);
    parsed.field_max_x = clamp_value(parsed.field_max_x, -limits->max_extent_x, limits->max_extent_x);
    parsed.field_min_y = clamp_value(parsed.field_min_y, -limits->max_extent_y, limits->max_extent_y);
    parsed.field_max_y = clamp_value(parsed.field_max_y, -limits->max_extent_y, limits->max_extent_y);
    if (parsed.field_max_x - parsed.field_min_x < 1.0 ||
        parsed.field_max_y - parsed.field_min_y < 1.0) {
      parsed.has_field_bounds = 0;
    }
  }

  if (parsed.id < 0 ||
      (!parsed.has_spawn_obstacle && !parsed.has_start_mapping_survey)) {
    return 0;
  }
  *command = parsed;
  return 1;
}
