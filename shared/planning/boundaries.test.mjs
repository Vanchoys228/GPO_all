import {spawnSync} from "node:child_process";
import { describe, expect, it } from "vitest";
import { readdir, readFile } from "node:fs/promises";

describe("shared planning library", () => {
  it("loads without a bundler or browser shims", () => {
    const result=spawnSync(process.execPath,["--input-type=module","-e",`await import(${JSON.stringify(new URL("./routeEnergy.js",import.meta.url).href)})`],{encoding:"utf8",windowsHide:true});
    expect(result.stderr).toBe("");
    expect(result.status).toBe(0);
  });
  it("runs the same complete planner in plain Node without frontend dependencies", async () => {
    const { buildRouteWithEnergyStops } = await import("./routeEnergy.js");
    const result = buildRouteWithEnergyStops({seedRoute:[{x:0,y:0},{x:1,y:0}],polygons:[],surfaceZones:[],chargingStations:[],batteryRangeMeters:100,energyOptions:{}});
    expect(result.ok).toBe(true);
    expect(result.routeEnergy).toBeGreaterThan(0);
  });
  it("keeps domain modules independent of React, browser and bridge", async () => {
    const files = (await readdir(new URL("./", import.meta.url))).filter(name => name.endsWith(".js"));
    expect(files.length).toBeGreaterThan(5);
    for (const name of files) {
      const source = await readFile(new URL(name, import.meta.url), "utf8");
      expect(source).not.toMatch(/from ["'][^"']*(src|bridge|react)|\b(document|window|WebSocket)\b/);
    }
  });
});
