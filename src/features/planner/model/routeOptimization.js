export const buildRouteOptimizationStatus = ({
  adjustedVisitCount = 0,
  algorithmLabel = "",
  blocked = false,
  routed,
  taskLabel = "",
}) => {
  if (blocked) {
    return "Маршрут построен, но всё ещё пересекает ограничивающий контур.";
  }
  const chargingSuffix = routed.stationStopCount
    ? ` Добавлено заездов на зарядку: ${routed.stationStopCount}.`
    : "";
  const energySuffix = ` Энергия: ${routed.routeEnergy.toFixed(1)} ед., время: ${routed.estimatedTimeSec.toFixed(1)} с.`;
  const adjustmentSuffix = adjustedVisitCount
    ? ` ${adjustedVisitCount} точек автоматически сдвинуты к безопасной позиции.`
    : "";
  return `Маршрут построен: ${taskLabel} (${algorithmLabel}).${adjustmentSuffix}${chargingSuffix}${energySuffix}`;
};
