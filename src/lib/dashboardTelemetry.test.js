import { describe, expect, it } from "vitest";
import { normalizeTelemetry } from "./dashboardTelemetry";

describe("normalizeTelemetry", () => {
  it("normalizes pose payloads from the shared coordinate contract", () => {
    const result = normalizeTelemetry({
      type: "telemetry",
      pose: {
        x: 1.25,
        y: -2.5,
        z: 0.12,
        yaw: 1.57,
      },
      perception: {
        lidar: {
          enabled: true,
        },
        obstacleTrace: [
          { x: 2, y: 3 },
        ],
      },
    });

    expect(result).toEqual({
      x: 1.25,
      y: -2.5,
      z: 0.12,
      yaw: 1.57,
      obstacleTrace: [
        { x: 2, y: 3, confidence: 1 },
      ],
      obstacleMap: {
        cellSize: 0.06,
        cellCount: 0,
        mapFile: "obstacle_map.json",
        jsonFile: "obstacle_map.json",
        excelCsvFile: "obstacle_map.csv",
        imageFile: "obstacle_map.png",
        cells: [],
      },
      perception: {
        lidar: {
          enabled: true,
        },
      },
    });
  });

  it("falls back to previous z/yaw values when the message omits them", () => {
    const result = normalizeTelemetry(
      {
        type: "telemetry",
        pose: {
          x: 3,
          y: 4,
        },
      },
      { x: 0, y: 0, z: 0.5, yaw: 0.75 }
    );

    expect(result).toEqual({
      x: 3,
      y: 4,
      z: 0.5,
      yaw: 0.75,
      obstacleTrace: [],
      obstacleMap: {
        cellSize: 0.06,
        cellCount: 0,
        mapFile: "obstacle_map.json",
        jsonFile: "obstacle_map.json",
        excelCsvFile: "obstacle_map.csv",
        imageFile: "obstacle_map.png",
        cells: [],
      },
      perception: {
        lidar: null,
      },
    });
  });

  it("normalizes obstacle map payloads with cells and metadata", () => {
    const result = normalizeTelemetry({
      type: "telemetry",
      pose: {
        x: 0,
        y: 0,
      },
      obstacleMap: {
        cellSize: 0.08,
        cellCount: 2,
        imageFile: "custom-map.png",
        cells: [
          { x: 1, y: 2, confidence: 4 },
          { x: -1.5, y: 0.5, confidence: 1 },
        ],
      },
    });

    expect(result?.obstacleMap).toEqual({
      cellSize: 0.08,
      cellCount: 2,
      mapFile: "obstacle_map.json",
      jsonFile: "obstacle_map.json",
      excelCsvFile: "obstacle_map.csv",
      imageFile: "custom-map.png",
      cells: [
        { x: 1, y: 2, confidence: 4 },
        { x: -1.5, y: 0.5, confidence: 1 },
      ],
    });
  });

  it("ignores payloads with a different message type", () => {
    expect(normalizeTelemetry({ type: "route", pose: { x: 1, y: 2 } })).toBeNull();
  });
});
