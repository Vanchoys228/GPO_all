import { describe, expect, it } from "vitest";
import { estimateRouteEnergy } from "./energyEstimator";
import { planRouteWithCharging } from "../../../lib/chargingPlanner";

describe("energy feasibility", () => {
  it("rejects a battery that cannot pay for the turn between mandatory legs", () => {
    expect(planRouteWithCharging({route:[{x:0,y:0},{x:1,y:0},{x:1,y:1}],
      batteryRange:2.2,stations:[],surfaceZones:[]}).ok).toBe(false);
  });
  it("integrates surfaces independently of collinear waypoint placement", () => {
    const surfaceZones = [{surfaceKey:"rough",points:[{x:0,y:-1},{x:2,y:-1},{x:2,y:1},{x:0,y:1}]}];
    const whole = estimateRouteEnergy([{x:0,y:0},{x:10,y:0}],{surfaceZones});
    const split = estimateRouteEnergy([{x:0,y:0},{x:2,y:0},{x:10,y:0}],{surfaceZones});
    expect(whole.totalEnergy).toBeCloseTo(split.totalEnergy, 8);
    expect(whole.totalEnergy).toBeGreaterThan(10);
  });
});
