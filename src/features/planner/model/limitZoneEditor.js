export const createLimitZoneDraft = (number) => ({
  id: `zone-${number}`,
  name: `Зона ${number}`,
  closed: false,
});

export const setLimitZoneClosed = (zones, zoneId, closed) =>
  zones.map((zone) => (zone.id === zoneId ? { ...zone, closed } : zone));

export const removeLimitZoneState = ({
  zones,
  points,
  zoneId,
  activeZoneId,
}) => {
  const nextZones = zones.filter((zone) => zone.id !== zoneId);
  return {
    zones: nextZones,
    points: points.filter(
      (point) => point.kind !== "limit" || point.zoneId !== zoneId
    ),
    activeZoneId:
      activeZoneId === zoneId ? nextZones[0]?.id || "" : activeZoneId,
  };
};
