import { buildSurfaceZonePayload } from "./surfaceZones";

const roundPoints = (points) =>
  points.map((point) => ({
    x: Number(point.x.toFixed(4)),
    y: Number(point.y.toFixed(4)),
  }));

const roundZones = (zones) =>
  zones.map((zone) => ({ ...zone, points: roundPoints(zone.points) }));

export const buildPlannerSyncPayloads = ({
  chargePoints,
  polygons,
  previewPolygons,
  surfaceZones,
}) => ({
  zoneSyncPayloadText: JSON.stringify({
    type: "limit_zones",
    zones: roundZones(polygons).map((zone) => ({
      id: zone.id,
      name: zone.name,
      points: zone.points,
    })),
  }),
  previewPolygonRoutingText: JSON.stringify(roundZones(previewPolygons)),
  chargePointsRoutingText: JSON.stringify(roundPoints(chargePoints)),
  surfaceSyncPayloadText: JSON.stringify(
    buildSurfaceZonePayload(roundZones(surfaceZones))
  ),
});
