import { describe, expect, it } from "vitest";
import {
  parsePlannerTableRows,
  readPlannerImportFile,
} from "./plannerFileImport";

describe("planner file import service", () => {
  it("converts localized table columns into planner points", () => {
    expect(
      parsePlannerTableRows([
        ["Координата X", "Координата Y", "Тип", "Операция"],
        ["1,5", "-2.25", "зарядка", ""],
        [3, 4, "зона", ""],
        [5, 6, "visit", "inspect"],
      ])
    ).toEqual({
      points: [
        { x: 1.5, y: -2.25, kind: "charge", zoneId: null, task: null },
        { x: 3, y: 4, kind: "limit", zoneId: "zone-1", task: null },
        { x: 5, y: 6, kind: "visit", zoneId: null, task: "inspect" },
      ],
      limitZones: [{ id: "zone-1", name: "Зона 1", closed: false }],
    });
  });

  it("rejects a table without valid coordinates", () => {
    expect(() => parsePlannerTableRows([["x", "y"], ["bad", "data"]])).toThrow(
      "В файле не найдено валидных координат."
    );
  });

  it("reads JSON without changing its graph payload", async () => {
    const graph = { points: [{ x: 1, y: 2, kind: "visit" }] };
    const result = await readPlannerImportFile({
      name: "route.json",
      text: async () => JSON.stringify(graph),
    });
    expect(result).toEqual({ graph, sourceName: "route.json" });
  });

  it("rejects unsupported file extensions", async () => {
    await expect(
      readPlannerImportFile({ name: "route.txt" })
    ).rejects.toThrow("Поддерживаются JSON, Excel и CSV.");
  });
});
