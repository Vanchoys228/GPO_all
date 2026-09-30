import { describe, expect, it } from "vitest";
import serializersModule from "./web-state-serializers.cjs";

const { createMotionProfileText, createRuntimeCommandText } = serializersModule;

describe("web state serializers", () => {
  it("serializes a sanitized motion profile into the controller format", () => {
    expect(createMotionProfileText({ cruiseSpeedMps: 0.3, payloadKg: 5, batteryRange: 100 }))
      .toBe("cruise_speed_mps 0.3\npayload_kg 5\nbattery_range 100\n");
  });

  it("serializes a bounded obstacle command", () => {
    expect(createRuntimeCommandText({
      type: "spawn_random_obstacle",
      commandId: 10,
      obstacle: { x: 500, y: -500, sizeX: 100, sizeY: 0, height: 20 },
    })).toContain("x 21.5");
  });
});
