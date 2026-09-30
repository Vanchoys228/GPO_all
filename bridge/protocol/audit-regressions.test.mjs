import { readFileSync } from "node:fs";
import limits from "../../shared/controller-limits.json" with { type: "json" };
import { describe, expect, it } from "vitest";
import validation from "./route-validation.cjs";

describe("controller payload limits", () => {
  it("keeps frontend/bridge limits aligned with the C controller capacity", () => {
    const header = readFileSync(new URL("../../webots/controllers/youbot_web/controller_types.h", import.meta.url), "utf8");
    for (const [macro, value] of [["MAX_WAYPOINTS", limits.maxRoutePoints], ["MAX_ZONES", limits.maxZones], ["MAX_ZONE_POINTS", limits.maxZonePoints]]) {
      expect(header).toContain(`#define ${macro} ${value}`);
    }
  });
  it("rejects more waypoints than the controller can execute", () => {
    expect(() => validation.validatePoints(Array.from({length:769},()=>({x:0,y:0})))).toThrow();
  });
  it("rejects excess zone count and vertices", () => {
    const zone = {points:[{x:0,y:0},{x:1,y:0},{x:0,y:1}]};
    expect(() => validation.validatePolygons(Array(33).fill(zone))).toThrow();
    expect(() => validation.validatePolygons([{points:Array(49).fill({x:0,y:0})}])).toThrow();
  });
});
