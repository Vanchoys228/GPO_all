import { samePoint } from "./chargingPlannerGeometry.js";
export const rotateClosedRouteToNearestPoint = (route, anchor) => {
  if (!route.length) return route;

  const closed = route.length > 1 && samePoint(route[0], route[route.length - 1]);
  const cycle = closed ? route.slice(0, -1) : route.slice();
  if (!cycle.length) return route;

  let bestIndex = 0;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (let index = 0; index < cycle.length; index += 1) {
    const dx = cycle[index].x - anchor.x;
    const dy = cycle[index].y - anchor.y;
    const distance = Math.hypot(dx, dy);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index;
    }
  }

  const rotated = bestIndex === 0 ? cycle : cycle.slice(bestIndex).concat(cycle.slice(0, bestIndex));

  if (!closed) return rotated;
  return rotated.concat([rotated[0]]);
};
