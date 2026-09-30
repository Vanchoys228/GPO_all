#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_METRICS_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_METRICS_H
int controller_navigation_metrics_off_route(int route_finished, int route_count, int avoidance_active, const char *status);
void controller_navigation_metrics_tick(int active, double step_seconds, double *time_seconds, int *steps);
#endif
