#ifndef YOUBOT_WEB_CONTROLLER_SURVEY_DEFAULT_ROUTE_H
#define YOUBOT_WEB_CONTROLLER_SURVEY_DEFAULT_ROUTE_H

typedef struct {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double strip;
} ControllerSurveyDefaultRouteConfig;

int controller_survey_default_route_write(
    const char *path,
    const ControllerSurveyDefaultRouteConfig *config);

#endif
