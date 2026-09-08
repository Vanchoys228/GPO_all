import { normalizeSurfaceZones } from "../../../../shared/planning/energySurfaceZones.js";

// Snapshot DTO shared by planning and mission submission; editor drafts stay local.
export const buildSceneSnapshot = ({plannerModel,batteryRangeMeters,energyOptions,preview = false}) => ({
  polygons: (preview ? plannerModel.previewPolygons : plannerModel.polygons) || [],
  surfaceZones: normalizeSurfaceZones(plannerModel.surfaceZones),
  chargingStations: plannerModel.chargePoints || [],
  motion: {batteryRange:batteryRangeMeters,cruiseSpeedMps:energyOptions?.speedMps ?? 0.22,payloadKg:energyOptions?.payloadKg ?? 0},
});
