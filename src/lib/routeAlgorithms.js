import { SOLVER_HEALTH_URL, SOLVER_ROUTE_URL } from "./runtimeConfig";
import { createPlanningRequest, unwrapPlanningResult } from "../../shared/contracts/index.js";

export {
  ALGORITHM_OPTIONS,
  TASK_OPTIONS,
  getAlgorithmFields,
  getAlgorithmLabel,
  getDefaultAlgorithmParams,
  getTaskLabel,
} from "./routeAlgorithmRegistry";

const normalizeRoutePoint = (point) => ({ x: Number(point?.x), y: Number(point?.y) });

export const solveRouteWithNativeAlgorithm = async (points, algorithmKey, params, taskKey = "tsp") => {
  const response = await fetch(SOLVER_ROUTE_URL, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(createPlanningRequest({
      source: "planner-frontend",
      requestId: crypto.randomUUID(),
      payload: {
        points: points.map((point) => ({ x: point.x, y: point.y })),
        algorithm: { key: algorithmKey, params },
        task: taskKey,
      },
    })),
  });
  const rawPayload = await response.json().catch(() => null);
  const payload = unwrapPlanningResult(rawPayload) || rawPayload;
  if (!response.ok) throw new Error(payload?.error || "Не удалось связаться с нативным solver.");
  if (!payload?.ok || !Array.isArray(payload.route)) {
    throw new Error(payload?.error || "Solver вернул некорректный ответ.");
  }
  return { ...payload, route: payload.route.map(normalizeRoutePoint) };
};

export const probeNativeSolver = async () => {
  const response = await fetch(SOLVER_HEALTH_URL, { method: "GET" });
  const payload = await response.json().catch(() => null);
  if (!response.ok || !payload) throw new Error("Solver API недоступен.");
  return payload;
};
