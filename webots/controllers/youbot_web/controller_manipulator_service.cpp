#include "controller_manipulator_service.h"

#include <cmath>
#include <cstring>

namespace {
const ControllerManipulatorTarget kTargets[] = {
    {{0.0, 1.57, -2.635, 1.78, 0.0}, 0.011},
    {{0.0, 0.95, -1.85, 1.65, 0.0}, 0.011},
    {{0.0, 0.65, -1.25, 1.45, 0.0}, 0.001},
    {{0.0, 1.15, -1.95, 1.65, 0.0}, 0.001},
    {{0.0, 0.72, -1.35, 1.48, 0.0}, 0.001},
};
}

void controller_manipulator_service_init(ControllerManipulatorService *service) {
  if (!service) return;
  std::memset(service, 0, sizeof(*service));
  service->timeout_seconds = 8.0;
  service->joint_tolerance = 0.035;
  service->finger_tolerance = 0.0025;
  service->target = kTargets[CONTROLLER_MANIPULATOR_TRANSPORT];
}

ControllerManipulatorTarget controller_manipulator_service_target(
    ControllerManipulatorPose pose) {
  if (pose < CONTROLLER_MANIPULATOR_TRANSPORT || pose > CONTROLLER_MANIPULATOR_PLACE)
    pose = CONTROLLER_MANIPULATOR_TRANSPORT;
  return kTargets[pose];
}

const char *controller_manipulator_pose_name(ControllerManipulatorPose pose) {
  static const char *names[] = {"transport", "pre_grasp", "grasp", "lift", "place"};
  return pose >= CONTROLLER_MANIPULATOR_TRANSPORT && pose <= CONTROLLER_MANIPULATOR_PLACE
      ? names[pose] : "transport";
}

int controller_manipulator_pose_parse(
    const char *name, ControllerManipulatorPose *pose) {
  if (!name || !pose) return 0;
  for (int index = CONTROLLER_MANIPULATOR_TRANSPORT;
       index <= CONTROLLER_MANIPULATOR_PLACE; ++index) {
    const ControllerManipulatorPose candidate =
        static_cast<ControllerManipulatorPose>(index);
    if (std::strcmp(name, controller_manipulator_pose_name(candidate)) == 0) {
      *pose = candidate;
      return 1;
    }
  }
  return 0;
}

const char *controller_manipulator_step_name(ControllerManipulatorStep state) {
  static const char *names[] = {"idle", "moving", "reached", "timed_out"};
  return state >= CONTROLLER_MANIPULATOR_IDLE && state <= CONTROLLER_MANIPULATOR_TIMED_OUT
      ? names[state] : "idle";
}

void controller_manipulator_service_start(
    ControllerManipulatorService *service,
    ControllerManipulatorPose pose,
    double now) {
  if (!service) return;
  service->pose = pose;
  service->target = controller_manipulator_service_target(pose);
  service->started_at = now;
  service->state = CONTROLLER_MANIPULATOR_MOVING;
}

ControllerManipulatorStep controller_manipulator_service_step(
    ControllerManipulatorService *service,
    const double joints[5],
    const double fingers[2],
    double now) {
  if (!service || !joints || !fingers) return CONTROLLER_MANIPULATOR_IDLE;
  if (service->state != CONTROLLER_MANIPULATOR_MOVING) return service->state;
  if (now - service->started_at > service->timeout_seconds) {
    service->state = CONTROLLER_MANIPULATOR_TIMED_OUT;
    return service->state;
  }
  for (int i = 0; i < 5; ++i) {
    if (std::fabs(joints[i] - service->target.joints[i]) > service->joint_tolerance)
      return service->state;
  }
  for (int i = 0; i < 2; ++i) {
    if (std::fabs(fingers[i] - service->target.finger_opening) > service->finger_tolerance)
      return service->state;
  }
  service->state = CONTROLLER_MANIPULATOR_REACHED;
  return service->state;
}
