import { describe, expect, it } from "vitest";
import {
  buildAutoRouteUpdatedStatus,
  buildControllerRoutePayload,
  buildRouteSentStatus,
} from "./routeCommand";

describe("route command", () => {
  it("builds the controller payload without leaking planner-only point fields", () => {
    expect(
      buildControllerRoutePayload({
        algorithmKey: "ga_tabu",
        batteryRangeMeters: 120,
        cruiseSpeedMps: 0.4,
        payloadKg: 12,
        route: [{ x: 1, y: 2, kind: "visit" }],
        routeTaskKey: "tsp",
        selectedAlgorithmParams: { generations: 10 },
      })
    ).toEqual({
      type: "route",
      algorithm: {
        key: "ga_tabu",
        task: "tsp",
        params: { generations: 10 },
      },
      motion: {
        cruiseSpeedMps: 0.4,
        payloadKg: 12,
        batteryRange: 120,
      },
      route: [{ x: 1, y: 2 }],
    });
  });

  it("formats sent status with an optional charging stop count", () => {
    expect(buildRouteSentStatus(8, 0)).toBe("Маршрут отправлен (8 точек).");
    expect(buildRouteSentStatus(8, 2)).toBe(
      "Маршрут отправлен (8 точек, зарядок: 2)."
    );
  });

  it("formats automatic route update status", () => {
    expect(buildAutoRouteUpdatedStatus(6, 1)).toBe(
      "Маршрут обновлён после изменения ограничивающих зон (6 точек, зарядок: 1)."
    );
  });
});
