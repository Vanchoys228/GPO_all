import { describe, expect, it } from "vitest";
import packageLock from "../../../../package-lock.json";
import * as XLSX from "xlsx";
import {
  parsePlannerTableRows,
  readPlannerImportFile,
} from "./plannerFileImport";

describe("planner file import service", () => {
  it("uses a SheetJS release with the known import vulnerabilities fixed", () => {
    const installedVersion = packageLock.packages["node_modules/xlsx"]?.version;
    expect(installedVersion).toBe("0.20.3");
  });

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

  it("reads coordinates from an XLSX workbook", async () => {
    const workbook = XLSX.utils.book_new();
    const sheet = XLSX.utils.aoa_to_sheet([
      ["x", "y", "type"],
      [1.5, 2.5, "visit"],
    ]);
    XLSX.utils.book_append_sheet(workbook, sheet, "Route");
    const bytes = XLSX.write(workbook, { bookType: "xlsx", type: "array" });

    const result = await readPlannerImportFile({
      name: "route.xlsx",
      arrayBuffer: async () => bytes,
    });

    expect(result.graph.points).toEqual([
      { x: 1.5, y: 2.5, kind: "visit", zoneId: null, task: null },
    ]);
  });

  it("rejects unsupported file extensions", async () => {
    await expect(
      readPlannerImportFile({ name: "route.txt" })
    ).rejects.toThrow("Поддерживаются JSON, Excel и CSV.");
  });
});
