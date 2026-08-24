const fs = require("fs");
const path = require("path");
const { resolveCameraFrameMimeType } = require("./normalizer.cjs");

const safeJsonParse = (text) => {
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
};

const createFileTelemetrySource = ({ normalizeTelemetry, stateDir }) => {
  const fsp = fs.promises;
  const paths = {
    robotState: path.join(stateDir, "robot_state.json"),
    obstacleMap: path.join(stateDir, "obstacle_map.json"),
    cameraMap: path.join(stateDir, "camera_map.json"),
  };
  let lastMtimeMs = -1;
  let pollInFlight = false;
  let cachedCameraFrame = { path: "", mtimeMs: -1, dataUrl: null };

  const readOptionalText = (filePath) =>
    fsp.readFile(filePath, "utf8").catch((error) => {
      if (error?.code === "ENOENT") return null;
      throw error;
    });

  const readCameraFrame = async (camera) => {
    if (!camera?.enabled) return null;
    const frameFile = path.basename(camera.frameFile || "camera_frame.bmp");
    const framePath = path.join(stateDir, frameFile);
    const mimeType = resolveCameraFrameMimeType(frameFile, camera.mimeType);
    try {
      const stat = await fsp.stat(framePath);
      if (cachedCameraFrame.path === framePath && cachedCameraFrame.mtimeMs === stat.mtimeMs) {
        return cachedCameraFrame.dataUrl
          ? { frameDataUrl: cachedCameraFrame.dataUrl, frameMtimeMs: stat.mtimeMs }
          : null;
      }
      const bytes = await fsp.readFile(framePath);
      const dataUrl = `data:${mimeType};base64,${bytes.toString("base64")}`;
      cachedCameraFrame = { path: framePath, mtimeMs: stat.mtimeMs, dataUrl };
      return { frameDataUrl: dataUrl, frameMtimeMs: stat.mtimeMs };
    } catch {
      return null;
    }
  };

  const poll = async () => {
    if (pollInFlight) return null;
    pollInFlight = true;
    try {
      const stat = await fsp.stat(paths.robotState);
      if (!Number.isFinite(stat.mtimeMs) || stat.mtimeMs === lastMtimeMs) return null;
      const [text, obstacleMapText, cameraMapText] = await Promise.all([
        fsp.readFile(paths.robotState, "utf8"),
        readOptionalText(paths.obstacleMap),
        readOptionalText(paths.cameraMap),
      ]);
      const normalized = normalizeTelemetry(
        safeJsonParse(text),
        obstacleMapText ? safeJsonParse(obstacleMapText) : null,
        cameraMapText ? safeJsonParse(cameraMapText) : null
      );
      if (!normalized) return null;
      const cameraFrame = await readCameraFrame(normalized.perception?.camera);
      if (cameraFrame && normalized.perception?.camera) {
        normalized.perception.camera = { ...normalized.perception.camera, ...cameraFrame };
        normalized.camera = normalized.perception.camera;
      }
      lastMtimeMs = stat.mtimeMs;
      return normalized;
    } catch (error) {
      if (error?.code !== "ENOENT") throw error;
      return null;
    } finally {
      pollInFlight = false;
    }
  };

  return { paths, poll, readCameraFrame };
};

module.exports = { createFileTelemetrySource, safeJsonParse };
