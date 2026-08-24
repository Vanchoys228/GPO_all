#ifndef YOUBOT_WEB_CONTROLLER_MATH_H
#define YOUBOT_WEB_CONTROLLER_MATH_H

double clamp_value(double value, double min_value, double max_value);
double hypot2(double x, double y);
double dot2(double ax, double ay, double bx, double by);
double wrap_angle(double angle);
double blend_angle(double from_angle, double to_angle, double weight_to);

#endif
