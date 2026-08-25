import {
  DEFAULT_SURFACE_PROFILE_KEY,
  createSurfaceZoneDraft,
  isSurfaceProfileKey,
  removeSurfaceZoneState,
  setSurfaceZoneClosed,
  setSurfaceZoneProfile,
} from "../model/surfaceZones";

export const usePlannerSurfaceZoneEditor = ({ activeSurfaceProfileKey, activeSurfaceZoneId, clearRouteState, nextSurfaceZoneNumber, setActivePointKind, setActiveSurfaceProfileKey, setActiveSurfaceZoneId, setNextSurfaceZoneNumber, setStatus, setSurfaceZones, surfaceZones }) => {
  const createSurfaceZone = () => {
    const zone = createSurfaceZoneDraft(nextSurfaceZoneNumber, activeSurfaceProfileKey);
    setSurfaceZones((previous) => [...previous, zone]);
    setActiveSurfaceZoneId(zone.id);
    setNextSurfaceZoneNumber((previous) => previous + 1);
    setActivePointKind("surface");
    setStatus(`Создана зона покрытия: ${zone.name}.`);
  };
  const selectSurfaceZone = (zoneId) => {
    const target = surfaceZones.find((zone) => zone.id === zoneId);
    if (!target) return;
    setActiveSurfaceZoneId(zoneId);
    setActiveSurfaceProfileKey(target.surfaceKey);
    setActivePointKind("surface");
  };
  const updateActiveSurfaceProfile = (surfaceKey) => {
    const nextKey = isSurfaceProfileKey(surfaceKey) ? surfaceKey : DEFAULT_SURFACE_PROFILE_KEY;
    setActiveSurfaceProfileKey(nextKey);
    if (!activeSurfaceZoneId) return;
    setSurfaceZones((previous) => setSurfaceZoneProfile(previous, activeSurfaceZoneId, nextKey));
    clearRouteState({ dropSolvedRoute: false });
  };
  const toggleSurfaceZoneClosed = (zoneId) => {
    const target = surfaceZones.find((zone) => zone.id === zoneId);
    if (!target) return;
    if (!target.closed && target.points.length < 3) {
      setStatus("Чтобы замкнуть покрытие, нужно минимум три точки.");
      return;
    }
    setSurfaceZones((previous) => setSurfaceZoneClosed(previous, zoneId, !target.closed));
    setActiveSurfaceZoneId(zoneId);
    setActivePointKind("surface");
    clearRouteState({ dropSolvedRoute: false });
    setStatus(target.closed ? `${target.name} открыта для редактирования.` : `${target.name} замкнута и будет учитываться в расчёте.`);
  };
  const clearSurfaceZone = (zoneId) => {
    setSurfaceZones((previous) => previous.map((zone) => zone.id === zoneId ? { ...zone, points: [], closed: false } : zone));
    setActiveSurfaceZoneId(zoneId);
    setActivePointKind("surface");
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Точки выбранного покрытия очищены.");
  };
  const removeSurfaceZone = (zoneId) => {
    const next = removeSurfaceZoneState({ zones: surfaceZones, zoneId, activeZoneId: activeSurfaceZoneId, fallbackSurfaceKey: activeSurfaceProfileKey });
    setSurfaceZones(next.zones);
    setActiveSurfaceZoneId(next.activeZoneId);
    setActiveSurfaceProfileKey(next.activeSurfaceKey);
    setNextSurfaceZoneNumber(next.nextZoneNumber);
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Зона покрытия удалена.");
  };
  const clearAllSurfaceZones = () => {
    const fallback = createSurfaceZoneDraft(1, activeSurfaceProfileKey);
    setSurfaceZones([fallback]);
    setActiveSurfaceZoneId(fallback.id);
    setNextSurfaceZoneNumber(2);
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Все зоны покрытий очищены.");
  };
  return { clearAllSurfaceZones, clearSurfaceZone, createSurfaceZone, removeSurfaceZone, selectSurfaceZone, toggleSurfaceZoneClosed, updateActiveSurfaceProfile };
};
