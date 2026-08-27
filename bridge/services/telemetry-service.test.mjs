import { describe, expect, it } from "vitest";
import telemetryServiceModule from "./telemetry-service.cjs";
import { createTelemetryEvent } from "../../shared/contracts/index.js";

const { createTelemetryService } = telemetryServiceModule;

describe("telemetry service", () => {
  it("retains the latest state and publishes a telemetry event", async () => {
    const service = createTelemetryService({ now: () => "2026-01-01T00:00:09.000Z" });
    const state = { type: "telemetry", pose: { x: 1, y: 2, z: 0, yaw: 0 } };

    const message = await service.publish(state);

    expect(service.getLatest()).toEqual(state);
    expect(JSON.parse(message)).toMatchObject({
      type: "telemetry.event",
      requestId: "telemetry-1",
      payload: state,
    });
  });

  it("unwraps a versioned telemetry event before retaining and republishing it", async () => {
    const service = createTelemetryService({ now: () => "2026-01-01T00:00:10.000Z" });
    const state = { type: "telemetry", pose: { x: 3, y: 4, z: 0, yaw: 0 } };
    const incomingEvent = createTelemetryEvent({
      source: "webots-adapter",
      requestId: "adapter-1",
      timestamp: "2026-01-01T00:00:09.000Z",
      payload: state,
    });

    const message = await service.publish(incomingEvent);

    expect(service.getLatest()).toEqual(state);
    expect(JSON.parse(message).payload).toEqual(state);
  });
});
