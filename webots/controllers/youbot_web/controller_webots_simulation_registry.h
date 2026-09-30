#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_REGISTRY_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_REGISTRY_H

#define CONTROLLER_WEBOTS_SIMULATION_MAX_NODE_REGISTRY 512

typedef struct {
  char defs[CONTROLLER_WEBOTS_SIMULATION_MAX_NODE_REGISTRY][64];
  int count;
} ControllerWebotsSimulationNodeRegistry;

int controller_webots_simulation_registry_track(
    ControllerWebotsSimulationNodeRegistry *registry,
    int capacity,
    const char *def_name);
void controller_webots_simulation_registry_forget(
    ControllerWebotsSimulationNodeRegistry *registry,
    int index);

#endif
