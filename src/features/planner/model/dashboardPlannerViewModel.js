import { getAlgorithmFields, getDefaultAlgorithmParams } from "../../../lib/routeAlgorithms";

export const createDashboardPlannerViewModel = ({
  algorithmKey,
  algorithmParams,
  activeSurfaceZoneId,
  surfaceZones,
}) => ({
  activeSurfaceZone:
    surfaceZones.find((zone) => zone.id === activeSurfaceZoneId) || surfaceZones[0] || null,
  algorithmFields: getAlgorithmFields(algorithmKey),
  selectedAlgorithmParams:
    algorithmParams[algorithmKey] || getDefaultAlgorithmParams(algorithmKey),
});
