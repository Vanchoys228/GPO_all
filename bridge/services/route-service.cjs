const createRouteService = ({ artifactStore }) => {
  const handle = async (payload) => {
    if (payload?.type === "route") {
      await artifactStore.writeRoute(payload);
      return { handled: true };
    }
    if (payload?.type === "limit_zones") {
      await artifactStore.writeLimitZones(payload);
      return { handled: true };
    }
    if (payload?.type === "surface_zones") {
      await artifactStore.writeSurfaceZones(payload);
      return { handled: true };
    }
    if (payload?.type === "motion_profile") {
      await artifactStore.writeMotionProfile(payload.motion);
      return { handled: true };
    }
    if (payload?.type === "spawn_random_obstacle" || payload?.type === "start_mapping_survey") {
      await artifactStore.writeRuntimeCommand(payload);
      return { handled: true };
    }
    return { handled: false };
  };

  return { handle };
};

module.exports = { createRouteService };
