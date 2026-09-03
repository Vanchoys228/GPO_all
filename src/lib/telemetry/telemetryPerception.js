import { pickNumber } from "./telemetryNumbers";

export const normalizeCamera = (rawCamera, prevCamera = null) => {
  if (!rawCamera || typeof rawCamera !== "object") return prevCamera ?? null;

  const frameDataUrl =
    typeof rawCamera.frameDataUrl === "string" && rawCamera.frameDataUrl.startsWith("data:image/")
      ? rawCamera.frameDataUrl
      : prevCamera?.frameDataUrl ?? null;

  return {
    enabled: Boolean(rawCamera.enabled),
    width: pickNumber(rawCamera.width, prevCamera?.width, 0) ?? 0,
    height: pickNumber(rawCamera.height, prevCamera?.height, 0) ?? 0,
    fov: pickNumber(rawCamera.fov, prevCamera?.fov, 0) ?? 0,
    frameFile:
      typeof rawCamera.frameFile === "string" && rawCamera.frameFile.trim()
        ? rawCamera.frameFile
        : prevCamera?.frameFile ?? "camera_frame.bmp",
    mimeType:
      typeof rawCamera.mimeType === "string" && rawCamera.mimeType.startsWith("image/")
        ? rawCamera.mimeType
        : prevCamera?.mimeType ?? "image/bmp",
    mode:
      typeof rawCamera.mode === "string" && rawCamera.mode.trim()
        ? rawCamera.mode
        : prevCamera?.mode ?? "webots_camera",
    frameSequence: pickNumber(rawCamera.frameSequence, prevCamera?.frameSequence, 0) ?? 0,
    capturedAt: pickNumber(rawCamera.capturedAt, prevCamera?.capturedAt, 0) ?? 0,
    frameMtimeMs: pickNumber(rawCamera.frameMtimeMs, prevCamera?.frameMtimeMs, null),
    obstacleVisible: Boolean(rawCamera.obstacleVisible),
    obstacleScore: pickNumber(rawCamera.obstacleScore, prevCamera?.obstacleScore, 0) ?? 0,
    obstacleOffset: pickNumber(rawCamera.obstacleOffset, prevCamera?.obstacleOffset, 0) ?? 0,
    obstacleAngle: pickNumber(rawCamera.obstacleAngle, prevCamera?.obstacleAngle, 0) ?? 0,
    obstacleRange: pickNumber(rawCamera.obstacleRange, prevCamera?.obstacleRange, 0) ?? 0,
    detectionCount: pickNumber(rawCamera.detectionCount, prevCamera?.detectionCount, 0) ?? 0,
    frameDataUrl,
  };
};

export const normalizeObstacleTrace = (rawTrace, prevTrace = []) =>
  Array.isArray(rawTrace)
    ? rawTrace
      .map((point) => ({
        x: pickNumber(point?.x),
        y: pickNumber(point?.y),
        confidence: Math.max(0, Math.min(1, pickNumber(point?.confidence, 1) ?? 1)),
      }))
      .filter((point) => point.x !== null && point.y !== null)
      .slice(-520)
    : prevTrace || [];
