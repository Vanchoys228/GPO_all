import { describe, expect, it } from "vitest";
import { interpolateCanvasTelemetry } from "./plannerCanvasTelemetry";

describe("planner canvas telemetry", () => {
  it("smooths position and rotates through the shortest yaw delta", () => {
    expect(
      interpolateCanvasTelemetry(
        { x: 0, y: 4, z: 0, yaw: 3.1 },
        { x: 10, y: 0, z: 2, yaw: -3.1 },
        0.5
      )
    ).toMatchObject({ x: 5, y: 2, z: 1 });

    const { yaw } = interpolateCanvasTelemetry(
      { x: 0, y: 0, z: 0, yaw: 3.1 },
      { x: 0, y: 0, z: 0, yaw: -3.1 },
      0.5
    );
    expect(yaw).toBeGreaterThan(3.1);
  });
});
