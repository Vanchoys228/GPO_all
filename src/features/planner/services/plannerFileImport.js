import { INITIAL_ZONE } from "../../../lib/plannerModel";
import { DEFAULT_POINT_TASK } from "../../../lib/zonePlanner";

const parseNumber = (value) => {
  const normalized = String(value ?? "").trim().replace(",", ".");
  if (!normalized) return null;
  const parsed = Number(normalized);
  return Number.isFinite(parsed) ? parsed : null;
};

const normalizeHeader = (value) => String(value ?? "").trim().toLowerCase();

export const parsePlannerTableRows = (rows) => {
  if (!Array.isArray(rows) || !rows.length) {
    throw new Error("Файл пуст.");
  }

  const firstRow = rows[0] || [];
  const firstRowHasCoordinates =
    parseNumber(firstRow[0]) !== null && parseNumber(firstRow[1]) !== null;
  const headers = firstRowHasCoordinates ? [] : firstRow.map(normalizeHeader);
  const dataRows = firstRowHasCoordinates ? rows : rows.slice(1);
  const findColumn = (aliases, fallback) => {
    const index = headers.findIndex((header) => aliases.includes(header));
    return index >= 0 ? index : fallback;
  };

  const xCol = findColumn(["x", "х", "xcoord", "x coordinate", "координата x"], 0);
  const yCol = findColumn(["y", "у", "ycoord", "y coordinate", "координата y"], 1);
  const kindCol = findColumn(["kind", "type", "тип", "роль"], -1);
  const taskCol = findColumn(["task", "operation", "операция", "задача"], -1);

  const points = dataRows
    .map((row) => {
      const x = parseNumber(row?.[xCol]);
      const y = parseNumber(row?.[yCol]);
      if (x === null || y === null) return null;

      const kindText = kindCol >= 0 ? normalizeHeader(row?.[kindCol]) : "";
      const kind =
        kindText.includes("charge") || kindText.includes("зар")
          ? "charge"
          : kindText.includes("limit") ||
              kindText.includes("zone") ||
              kindText.includes("зон")
            ? "limit"
            : "visit";
      return {
        x,
        y,
        kind,
        zoneId: kind === "limit" ? INITIAL_ZONE.id : null,
        task:
          kind === "visit" && taskCol >= 0
            ? String(row?.[taskCol] || DEFAULT_POINT_TASK)
            : null,
      };
    })
    .filter(Boolean);

  if (!points.length) {
    throw new Error("В файле не найдено валидных координат.");
  }

  return {
    points,
    limitZones: points.some((point) => point.kind === "limit") ? [INITIAL_ZONE] : [],
  };
};

export const readPlannerImportFile = async (file) => {
  const extension = file.name.split(".").pop()?.toLowerCase() || "";
  if (extension === "json") {
    return {
      graph: JSON.parse(await file.text()),
      sourceName: file.name,
    };
  }

  if (!["xlsx", "xls", "csv"].includes(extension)) {
    throw new Error("Поддерживаются JSON, Excel и CSV.");
  }

  const XLSX = await import("xlsx");
  const workbook = XLSX.read(await file.arrayBuffer(), {
    type: "array",
    raw: false,
  });
  const firstSheetName = workbook.SheetNames[0];
  const sheet = firstSheetName ? workbook.Sheets[firstSheetName] : null;
  if (!sheet) throw new Error("В файле не найден лист с точками.");

  return {
    graph: parsePlannerTableRows(
      XLSX.utils.sheet_to_json(sheet, { header: 1, defval: "" })
    ),
    sourceName: file.name,
  };
};
