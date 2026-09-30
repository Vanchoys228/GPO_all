#ifndef CONTROLLER_MANIPULATOR_SERVICE_H
#define CONTROLLER_MANIPULATOR_SERVICE_H

typedef enum {
  CONTROLLER_MANIPULATOR_TRANSPORT = 0,
  CONTROLLER_MANIPULATOR_PRE_GRASP,
  CONTROLLER_MANIPULATOR_GRASP,
  CONTROLLER_MANIPULATOR_LIFT,
  CONTROLLER_MANIPULATOR_PLACE,
} ControllerManipulatorPose;

typedef enum {
  CONTROLLER_MANIPULATOR_IDLE = 0,
  CONTROLLER_MANIPULATOR_MOVING,
  CONTROLLER_MANIPULATOR_REACHED,
  CONTROLLER_MANIPULATOR_TIMED_OUT,
} ControllerManipulatorStep;

typedef struct {
  double joints[5];
  double finger_opening;
} ControllerManipulatorTarget;

typedef struct {
  ControllerManipulatorPose pose;
  ControllerManipulatorTarget target;
  ControllerManipulatorStep state;
  double started_at;
  double timeout_seconds;
  double joint_tolerance;
  double finger_tolerance;
} ControllerManipulatorService;

void controller_manipulator_service_init(ControllerManipulatorService *service);
ControllerManipulatorTarget controller_manipulator_service_target(
    ControllerManipulatorPose pose);
void controller_manipulator_service_start(
    ControllerManipulatorService *service,
    ControllerManipulatorPose pose,
    double now);
ControllerManipulatorStep controller_manipulator_service_step(
    ControllerManipulatorService *service,
    const double joints[5],
    const double fingers[2],
    double now);

#endif
