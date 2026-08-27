import { describe, expect, it, vi } from "vitest";
import telemetryProcessModule from "./telemetry-process.cjs";

const { startTelemetryProcess } = telemetryProcessModule;

describe("telemetry process", () => {
  it("starts the telemetry server with an independent file source", () => {
    const server = { marker: "telemetry-server" };
    const normalizer = vi.fn();
    const fileSource = { marker: "file-source" };
    const createTelemetryNormalizer = vi.fn(() => normalizer);
    const createFileTelemetrySource = vi.fn(() => fileSource);
    const createTelemetryServer = vi.fn(() => server);
    const config = { BRIDGE_HOST: "127.0.0.1", TELEMETRY_PORT: 9001, WEB_STATE_DIR: "/state" };
    const coordinateContract = { version: 1 };

    const result = startTelemetryProcess({
      config,
      coordinateContract,
      createTelemetryNormalizer,
      createFileTelemetrySource,
      createTelemetryServer,
      enableMockTelemetry: true,
    });

    expect(createTelemetryNormalizer).toHaveBeenCalledWith(coordinateContract);
    expect(createFileTelemetrySource).toHaveBeenCalledWith({
      normalizeTelemetry: normalizer,
      stateDir: "/state",
    });
    expect(createTelemetryServer).toHaveBeenCalledWith({
      coordinateContract,
      enableMockTelemetry: true,
      fileSource,
      host: "127.0.0.1",
      port: 9001,
    });
    expect(result).toBe(server);
  });
});
