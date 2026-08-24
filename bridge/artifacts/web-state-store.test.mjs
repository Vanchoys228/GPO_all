import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import storeModule from "./web-state-store.cjs";
import coordinateContract from "../../shared/coordinate-contract.json" with { type: "json" };

const { createWebStateStore } = storeModule;
const temporaryDirectories = [];

afterEach(async () => {
  await Promise.all(
    temporaryDirectories.splice(0).map((directory) =>
      fs.rm(directory, { recursive: true, force: true })
    )
  );
});

const createStore = async () => {
  const stateDir = await fs.mkdtemp(path.join(os.tmpdir(), "gpo-state-"));
  temporaryDirectories.push(stateDir);
  return { stateDir, store: createWebStateStore({ coordinateContract, stateDir }) };
};

describe("web state store", () => {
  it("writes compatible route JSON, CSV and motion files", async () => {
    const { stateDir, store } = await createStore();
    await store.writeRoute({
      type: "route",
      algorithm: { key: "genetik", task: "tsp", params: { generations: 10 } },
      motion: { cruiseSpeedMps: 0.3, payloadKg: 5, batteryRange: 100 },
      route: [
        { x: 0, y: 0 },
        { x: 1, y: 0 },
      ],
    });

    const route = JSON.parse(await fs.readFile(path.join(stateDir, "route.json"), "utf8"));
    const csv = await fs.readFile(path.join(stateDir, "route.csv"), "utf8");
    const motion = await fs.readFile(path.join(stateDir, "motion_profile.txt"), "utf8");

    expect(route.algorithm.key).toBe("ga_tabu");
    expect(route.route).toEqual([{ x: 0, y: 0 }, { x: 1, y: 0 }]);
    expect(csv).toContain(coordinateContract.routeCsv.header.join(","));
    expect(csv).toContain("1,0,0");
    expect(motion).toContain("cruise_speed_mps 0.3");
  });

  it("writes limit and surface zone contracts", async () => {
    const { stateDir, store } = await createStore();
    const points = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 1 }];
    await store.writeLimitZones({ zones: [{ id: "limit", points }] });
    await store.writeSurfaceZones({ zones: [{ id: "rough", surfaceKey: "rough", points }] });

    expect(await fs.readFile(path.join(stateDir, "limit_zones.txt"), "utf8")).toContain(
      "zone_count 1"
    );
    expect(await fs.readFile(path.join(stateDir, "surface_zones.txt"), "utf8")).toContain(
      "surface_zone 3 rough rough"
    );
  });

  it("writes runtime commands with bounded obstacle values", async () => {
    const { stateDir, store } = await createStore();
    await store.writeRuntimeCommand({
      type: "spawn_random_obstacle",
      commandId: 10,
      obstacle: { x: 500, y: -500, sizeX: 100, sizeY: 0, height: 20 },
    });

    const command = await fs.readFile(path.join(stateDir, "runtime_command.txt"), "utf8");
    expect(command).toContain("id 10");
    expect(command).toContain("x 21.5");
    expect(command).toContain("y -16.5");
    expect(command).toContain("size_x 3.5");
  });
});
