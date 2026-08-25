import { SURFACE_PROFILE_OPTIONS } from "../../../lib/energyModel";
import { DEFAULT_SURFACE_ZONES, isInsideMap } from "../../../lib/zonePlanner";

const surfaceProfileKeys = new Set(SURFACE_PROFILE_OPTIONS.map((profile) => profile.key));

export const isSurfaceProfileKey = (surfaceKey) => surfaceProfileKeys.has(surfaceKey);

export const DEFAULT_SURFACE_PROFILE_KEY =
  SURFACE_PROFILE_OPTIONS.find((profile) => profile.key !== "neutral")?.key ||
  SURFACE_PROFILE_OPTIONS[0]?.key ||
  "neutral";

const normalizeSurfacePoint = (point) => {
  const x = Number(point?.x);
  const y = Number(point?.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  const normalized = { x, y };
  return isInsideMap(normalized) ? normalized : null;
};

export const createSurfaceZoneDraft = (
  number,
  surfaceKey = DEFAULT_SURFACE_PROFILE_KEY
) => ({
  id: `surface-zone-${number}`,
  name: `Покрытие ${number}`,
  surfaceKey,
  closed: false,
  points: [],
});

export const normalizeSurfaceZoneForState = (zone, index) => {
  const points = (Array.isArray(zone?.points) ? zone.points : [])
    .map(normalizeSurfacePoint)
    .filter(Boolean);
  const surfaceKey = surfaceProfileKeys.has(zone?.surfaceKey)
    ? zone.surfaceKey
    : DEFAULT_SURFACE_PROFILE_KEY;

  return {
    id:
      typeof zone?.id === "string" && zone.id.trim()
        ? zone.id.trim()
        : `surface-zone-${index + 1}`,
    name:
      typeof zone?.name === "string" && zone.name.trim()
        ? zone.name.trim()
        : `Покрытие ${index + 1}`,
    surfaceKey,
    closed: zone?.closed === undefined ? points.length >= 3 : Boolean(zone.closed),
    points,
  };
};

export const createInitialSurfaceZones = () => {
  const presetZones = DEFAULT_SURFACE_ZONES.map((zone, index) =>
    normalizeSurfaceZoneForState({ ...zone, closed: true }, index)
  );
  return [...presetZones, createSurfaceZoneDraft(presetZones.length + 1)];
};

export const deriveNextSurfaceZoneNumber = (zones) => {
  const maxNumber = zones.reduce((best, zone) => {
    const idMatch = String(zone?.id ?? "").match(/surface-zone-(\d+)/i);
    const nameMatch = String(zone?.name ?? "").match(/(\d+)/);
    const candidates = [idMatch?.[1], nameMatch?.[1]]
      .map((value) => Number(value))
      .filter(Number.isFinite);
    return candidates.length ? Math.max(best, ...candidates) : best;
  }, 0);
  return Math.max(1, maxNumber + 1);
};

export const setSurfaceZoneProfile = (zones, zoneId, surfaceKey) =>
  zones.map((zone) =>
    zone.id === zoneId ? { ...zone, surfaceKey } : zone
  );

export const setSurfaceZoneClosed = (zones, zoneId, closed) =>
  zones.map((zone) => (zone.id === zoneId ? { ...zone, closed } : zone));

export const removeSurfaceZoneState = ({
  zones,
  zoneId,
  activeZoneId,
  fallbackSurfaceKey,
}) => {
  const remaining = zones.filter((zone) => zone.id !== zoneId);
  if (!remaining.length) {
    const fallback = createSurfaceZoneDraft(1, fallbackSurfaceKey);
    return {
      zones: [fallback],
      activeZoneId: fallback.id,
      activeSurfaceKey: fallback.surfaceKey,
      nextZoneNumber: 2,
    };
  }
  const activeZone =
    remaining.find((zone) => zone.id === activeZoneId) || remaining[0];
  return {
    zones: remaining,
    activeZoneId: activeZone.id,
    activeSurfaceKey: activeZone.surfaceKey,
    nextZoneNumber: deriveNextSurfaceZoneNumber(remaining),
  };
};

export const normalizeSurfaceZonesForImport = (surfaceZones) => {
  if (!Array.isArray(surfaceZones)) return null;
  const normalized = surfaceZones
    .map((zone, index) => normalizeSurfaceZoneForState(zone, index))
    .filter((zone) => zone.points.length > 0);
  return normalized.length ? normalized : null;
};

export const buildSurfaceZonePayload = (surfaceZones) => ({
  type: "surface_zones",
  zones: (Array.isArray(surfaceZones) ? surfaceZones : [])
    .filter(
      (zone) =>
        zone?.closed !== false && Array.isArray(zone?.points) && zone.points.length >= 3
    )
    .map((zone) => ({
      id: zone.id,
      name: zone.name,
      surfaceKey: surfaceProfileKeys.has(zone.surfaceKey)
        ? zone.surfaceKey
        : DEFAULT_SURFACE_PROFILE_KEY,
      points: zone.points.map((point) => ({ x: point.x, y: point.y })),
    })),
});
