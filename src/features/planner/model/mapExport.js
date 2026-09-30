export const hasMapData = (map) =>
  Boolean(
    (Array.isArray(map?.cells) ? map.cells.length : 0) +
      (Array.isArray(map?.freeCells) ? map.freeCells.length : 0)
  );

export const normalizeMapExportVariant = (variant) =>
  variant === "camera" ? "camera" : "lidar";
