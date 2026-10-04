#include "controller_manipulator_service.h"

#include <assert.h>
#include <math.h>
#include <string.h>

int main() {
  ControllerManipulatorService service;
  controller_manipulator_service_init(&service);

  const ControllerManipulatorTarget pre_grasp =
      controller_manipulator_service_target(CONTROLLER_MANIPULATOR_PRE_GRASP);
  assert(pre_grasp.finger_opening > 0.01);
  assert(pre_grasp.joints[1] > 0.0);

  controller_manipulator_service_start(
      &service, CONTROLLER_MANIPULATOR_PRE_GRASP, 10.0);
  double joints[5] = {};
  double fingers[2] = {};
  ControllerManipulatorStep step =
      controller_manipulator_service_step(&service, joints, fingers, 10.5);
  assert(step == CONTROLLER_MANIPULATOR_MOVING);

  for (int i = 0; i < 5; ++i) joints[i] = pre_grasp.joints[i];
  fingers[0] = pre_grasp.finger_opening;
  fingers[1] = pre_grasp.finger_opening;
  step = controller_manipulator_service_step(&service, joints, fingers, 11.0);
  assert(step == CONTROLLER_MANIPULATOR_REACHED);

  controller_manipulator_service_start(
      &service, CONTROLLER_MANIPULATOR_GRASP, 20.0);
  step = controller_manipulator_service_step(&service, joints, fingers, 29.0);
  assert(step == CONTROLLER_MANIPULATOR_TIMED_OUT);

  const ControllerManipulatorTarget transport =
      controller_manipulator_service_target(CONTROLLER_MANIPULATOR_TRANSPORT);
  assert(fabs(transport.joints[2] - pre_grasp.joints[2]) > 0.1);
  ControllerManipulatorPose parsed_pose = CONTROLLER_MANIPULATOR_TRANSPORT;
  assert(controller_manipulator_pose_parse("lift", &parsed_pose));
  assert(parsed_pose == CONTROLLER_MANIPULATOR_LIFT);
  assert(strcmp(controller_manipulator_pose_name(parsed_pose), "lift") == 0);
  assert(!controller_manipulator_pose_parse("unsafe", &parsed_pose));
  assert(strcmp(controller_manipulator_step_name(CONTROLLER_MANIPULATOR_REACHED),
                "reached") == 0);
  return 0;
}
