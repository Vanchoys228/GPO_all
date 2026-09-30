import { INITIAL_TELEMETRY } from "./dashboardTelemetryState";
import { pickNumber } from "./telemetryNumbers";

const normalizeCells = (rawCells, limit) =>
  rawCells
    .map((cell) => ({
      x: pickNumber(cell?.x),
      y: pickNumber(cell?.y),
      confidence: Math.max(0, pickNumber(cell?.confidence, 0) ?? 0),
    }))
    .filter((cell) => cell.x !== null && cell.y !== null)
    .slice(-limit);

export const normalizeTelemetryMap = (
  rawMap,
  prevMap = INITIAL_TELEMETRY.obstacleMap
) => {
  if (!rawMap || typeof rawMap !== "object") return prevMap;

  const rawCells = Array.isArray(rawMap.cells) ? rawMap.cells : prevMap?.cells || [];
  const rawFreeCells = Array.isArray(rawMap.freeCells)
    ? rawMap.freeCells
    : prevMap?.freeCells || [];
  const cells = normalizeCells(rawCells, 4096);
  const freeCells = normalizeCells(rawFreeCells, 4096);
  const cellSize = pickNumber(
    rawMap.cellSize,
    prevMap?.cellSize,
    INITIAL_TELEMETRY.obstacleMap.cellSize
  );
  const cellCount = pickNumber(
    rawMap.cellCount,
    rawMap.totalCells,
    cells.length,
    prevMap?.cellCount,
    0
  );
  const obstacleCellCount = pickNumber(
    rawMap.obstacleCellCount,
    cells.length,
    prevMap?.obstacleCellCount,
    0
  );
  const freeCellCount = pickNumber(
    rawMap.freeCellCount,
    freeCells.length,
    prevMap?.freeCellCount,
    0
  );

  return {
    cellSize: cellSize ?? INITIAL_TELEMETRY.obstacleMap.cellSize,
    cellCount: cellCount ?? cells.length,
    obstacleCellCount: obstacleCellCount ?? cells.length,
    freeCellCount: freeCellCount ?? freeCells.length,
    mapFile:
      typeof rawMap.mapFile === "string" && rawMap.mapFile.trim()
        ? rawMap.mapFile
        : prevMap?.mapFile ?? INITIAL_TELEMETRY.obstacleMap.mapFile,
    jsonFile:
      typeof rawMap.jsonFile === "string" && rawMap.jsonFile.trim()
        ? rawMap.jsonFile
        : prevMap?.jsonFile ?? INITIAL_TELEMETRY.obstacleMap.jsonFile,
    excelCsvFile:
      typeof rawMap.excelCsvFile === "string" && rawMap.excelCsvFile.trim()
        ? rawMap.excelCsvFile
        : prevMap?.excelCsvFile ?? INITIAL_TELEMETRY.obstacleMap.excelCsvFile,
    imageFile:
      typeof rawMap.imageFile === "string" && rawMap.imageFile.trim()
        ? rawMap.imageFile
        : prevMap?.imageFile ?? INITIAL_TELEMETRY.obstacleMap.imageFile,
    cells,
    freeCells,
  };
};
