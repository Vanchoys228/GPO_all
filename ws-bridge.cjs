const http = require("http");
const fs = require("fs");
const fsp = fs.promises;
const path = require("path");
const { spawn } = require("child_process");
const WebSocket = require("ws");
const coordinateContract = require("./shared/coordinate-contract.json");
const {
  BRIDGE_HOST,
  ROUTE_PORT,
  ROUTE_WS_URL,
  SOLVER_HTTP_URL,
  SOLVER_PORT,
  TELEMETRY_PORT,
  TELEMETRY_WS_URL,
} = require("./bridge-config.cjs");

const ENABLE_MOCK_TELEMETRY = process.env.MOCK_TELEMETRY === "1";
const TELEMETRY_IDLE_MS = 2500;
const FILE_TELEMETRY_POLL_MS = 120;
const SOLVER_TIMEOUT_MS = 30000;
const SOLVER_EXE_PATH = path.join(__dirname, "native", "build", "gpo_route_solver.exe");
const WEB_STATE_DIR = path.join(__dirname, "web_state");
const ROUTE_JSON_PATH = path.join(WEB_STATE_DIR, "route.json");
const ROUTE_CSV_PATH = path.join(WEB_STATE_DIR, "route.csv");
const LIMIT_ZONES_JSON_PATH = path.join(WEB_STATE_DIR, "limit_zones.json");
const LIMIT_ZONES_TXT_PATH = path.join(WEB_STATE_DIR, "limit_zones.txt");
const ROBOT_STATE_PATH = path.join(WEB_STATE_DIR, "robot_state.json");
const OBSTACLE_MAP_PATH = path.join(WEB_STATE_DIR, "obstacle_map.json");
const MOTION_PROFILE_PATH = path.join(WEB_STATE_DIR, "motion_profile.txt");
const RUNTIME_COMMAND_PATH = path.join(WEB_STATE_DIR, "runtime_command.txt");
const ROUTE_CSV_HEADER = coordinateContract.routeCsv.header.join(",");
const TELEMETRY_MESSAGE_TYPE = coordinateContract.telemetry.messageType;
const TELEMETRY_POSE_KEY = coordinateContract.telemetry.poseKey;

const DEFAULT_GA_TABU_PARAMS = {
  population_size: 90,
  generations: 260,
  mutation_rate: 0.18,
  crossover_rate: 0.9,
  tabu_iterations: 24,
};

const DEFAULT_ANNEALING_PARAMS = {
  pool_size: 48,
  max_iterations: 5000,
  minimum_temperature: 0.001,
  cooling_rate: 0.996,
  neighborhood_strength: 2.5,
};

const DEFAULT_SCATTER_PARAMS = {
  population_size: 70,
  refset_ratio: 0.18,
  max_iterations: 130,
  mutation_rate: 0.2,
  local_steps: 8,
};

const DEFAULT_CUCKOO_PARAMS = {
  nests: 30,
  discovery_probability: 0.25,
  max_iterations: 200,
  alpha: 0.12,
  beta: 1.5,
};

const DEFAULT_MOTION_PROFILE = {
  cruiseSpeedMps: 0.22,
  payloadKg: 0,
  batteryRange: 100,
};

const clamp = (value, min, max) => Math.max(min, Math.min(value, max));

const clampInt = (value, min, max) =>
  Math.round(clamp(Number.isFinite(value) ? value : min, min, max));

const normalizeNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const toDegrees = (radians) => (radians * 180) / Math.PI;

let ensureWebStateDirPromise = null;

const ensureWebStateDir = () => {
  if (!ensureWebStateDirPromise) {
    ensureWebStateDirPromise = fsp.mkdir(WEB_STATE_DIR, { recursive: true });
  }
  return ensureWebStateDirPromise;
};

const solverExists = async () => {
  try {
    await fsp.access(SOLVER_EXE_PATH, fs.constants.F_OK);
    return true;
  } catch {
    return false;
  }
};

const resolveAlgorithmKey = (algorithmKey) => {
  if (algorithmKey === "genetik") return "ga_tabu";
  if (algorithmKey === "annealing") return "otshig";
  if (algorithmKey === "scatter") return "rasseivanie";
  return algorithmKey || "ga_tabu";
};

const resolveTaskKey = (taskKey) => {
  if (
    taskKey === "tsp" ||
    taskKey === "hamiltonian_chain" ||
    taskKey === "shortest_route"
  ) {
    return taskKey;
  }
  return "tsp";
};

const sanitizeNativeParams = (algorithmKey, rawParams) => {
  const resolvedKey = resolveAlgorithmKey(algorithmKey);
  const params = rawParams || {};

  if (resolvedKey === "ga_tabu") {
    const base = { ...DEFAULT_GA_TABU_PARAMS, ...params };
    return {
      nests: clampInt(base.population_size, 8, 1200),
      pa: clamp(normalizeNumber(base.mutation_rate, DEFAULT_GA_TABU_PARAMS.mutation_rate), 0, 1),
      max_iter: clampInt(base.generations, 1, 10000),
      alpha: clamp(normalizeNumber(base.crossover_rate, DEFAULT_GA_TABU_PARAMS.crossover_rate), 0, 1),
      beta: clamp(normalizeNumber(base.tabu_iterations, DEFAULT_GA_TABU_PARAMS.tabu_iterations), 1, 80),
    };
  }

  if (resolvedKey === "otshig") {
    const base = { ...DEFAULT_ANNEALING_PARAMS, ...params };
    return {
      nests: clampInt(base.pool_size, 8, 200),
      pa: clamp(
        normalizeNumber(base.minimum_temperature, DEFAULT_ANNEALING_PARAMS.minimum_temperature),
        0.000001,
        1
      ),
      max_iter: clampInt(base.max_iterations, 1, 200000),
      alpha: clamp(
        normalizeNumber(base.cooling_rate, DEFAULT_ANNEALING_PARAMS.cooling_rate),
        0.9,
        0.99999
      ),
      beta: clamp(
        normalizeNumber(base.neighborhood_strength, DEFAULT_ANNEALING_PARAMS.neighborhood_strength),
        1,
        12
      ),
    };
  }

  if (resolvedKey === "rasseivanie") {
    const base = { ...DEFAULT_SCATTER_PARAMS, ...params };
    return {
      nests: clampInt(base.population_size, 10, 1200),
      pa: clamp(normalizeNumber(base.refset_ratio, DEFAULT_SCATTER_PARAMS.refset_ratio), 0.05, 0.5),
      max_iter: clampInt(base.max_iterations, 1, 3000),
      alpha: clamp(
        normalizeNumber(base.mutation_rate, DEFAULT_SCATTER_PARAMS.mutation_rate) * 2,
        0,
        1
      ),
      beta: clamp(normalizeNumber(base.local_steps, DEFAULT_SCATTER_PARAMS.local_steps), 2, 30),
    };
  }

  const base = { ...DEFAULT_CUCKOO_PARAMS, ...params };
  return {
    nests: clampInt(base.nests, 5, 250),
    pa: clamp(
      normalizeNumber(base.discovery_probability, DEFAULT_CUCKOO_PARAMS.discovery_probability),
      0.01,
      0.9
    ),
    max_iter: clampInt(base.max_iterations, 1, 5000),
    alpha: clamp(normalizeNumber(base.alpha, DEFAULT_CUCKOO_PARAMS.alpha), 0.001, 2),
    beta: clamp(normalizeNumber(base.beta, DEFAULT_CUCKOO_PARAMS.beta), 1.1, 1.99),
  };
};

const validatePoints = (points) => {
  if (!Array.isArray(points)) {
    throw new Error("Points payload must be an array.");
  }

  return points.map((point) => {
    const x = Number(point?.x);
    const y = Number(point?.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw new Error("Every route point must contain finite x and y.");
    }
    return { x, y };
  });
};

const validatePolygons = (zones) => {
  if (!Array.isArray(zones)) {
    throw new Error("Limit zones payload must be an array.");
  }

  return zones.map((zone, index) => {
    const id =
      typeof zone?.id === "string" && zone.id.trim()
        ? zone.id.trim()
        : `zone-${index + 1}`;
    const name =
      typeof zone?.name === "string" && zone.name.trim()
        ? zone.name.trim()
        : `Zone ${index + 1}`;
    const points = validatePoints(zone?.points || []);
    if (points.length < 3) {
      throw new Error("Every limit zone must contain at least three points.");
    }
    return { id, name, points };
  });
};

const safeJsonParse = (text) => {
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
};

const normalizeObstacleMap = (rawMap, fallback = null) => {
  const fallbackMap = fallback && typeof fallback === "object" ? fallback : {};
  const rawCells = Array.isArray(rawMap?.cells)
    ? rawMap.cells
    : Array.isArray(fallbackMap?.cells)
      ? fallbackMap.cells
      : [];
  const cells = rawCells
    .map((cell) => ({
      x: Number(cell?.x),
      y: Number(cell?.y),
      confidence: Number(cell?.confidence),
    }))
    .filter((cell) => Number.isFinite(cell.x) && Number.isFinite(cell.y))
    .map((cell) => ({
      x: cell.x,
      y: cell.y,
      confidence: Number.isFinite(cell.confidence) ? Math.max(0, cell.confidence) : 0,
    }))
    .slice(-4096);
  const cellSize = Number(rawMap?.cellSize ?? fallbackMap?.cellSize);
  const cellCount = Number(rawMap?.totalCells ?? fallbackMap?.cellCount ?? cells.length);

  return {
    cellSize: Number.isFinite(cellSize) && cellSize > 0 ? cellSize : 0.06,
    cellCount: Number.isFinite(cellCount) && cellCount >= 0 ? cellCount : cells.length,
    mapFile:
      typeof fallbackMap?.mapFile === "string" && fallbackMap.mapFile.trim()
        ? fallbackMap.mapFile
        : "obstacle_map.json",
    jsonFile:
      typeof fallbackMap?.jsonFile === "string" && fallbackMap.jsonFile.trim()
        ? fallbackMap.jsonFile
        : "obstacle_map.json",
    excelCsvFile:
      typeof fallbackMap?.excelCsvFile === "string" && fallbackMap.excelCsvFile.trim()
        ? fallbackMap.excelCsvFile
        : "obstacle_map.csv",
    imageFile:
      typeof fallbackMap?.imageFile === "string" && fallbackMap.imageFile.trim()
        ? fallbackMap.imageFile
        : "obstacle_map.png",
    cells,
  };
};

const sanitizeMotionProfile = (motion) => {
  const profile = motion || {};
  return {
    cruiseSpeedMps: clamp(
      normalizeNumber(profile.cruiseSpeedMps, DEFAULT_MOTION_PROFILE.cruiseSpeedMps),
      0.05,
      0.8
    ),
    payloadKg: clamp(
      normalizeNumber(profile.payloadKg, DEFAULT_MOTION_PROFILE.payloadKg),
      0,
      500
    ),
    batteryRange: clamp(
      normalizeNumber(profile.batteryRange, DEFAULT_MOTION_PROFILE.batteryRange),
      1,
      100000
    ),
  };
};

const sanitizeRuntimeObstacle = (rawObstacle) => {
  const obstacle = rawObstacle || {};
  return {
    x: clamp(normalizeNumber(obstacle.x, 0), -21.5, 21.5),
    y: clamp(normalizeNumber(obstacle.y, 0), -16.5, 16.5),
    sizeX: clamp(normalizeNumber(obstacle.sizeX, 0.8), 0.2, 3.5),
    sizeY: clamp(normalizeNumber(obstacle.sizeY, 0.8), 0.2, 3.5),
    height: clamp(normalizeNumber(obstacle.height, 0.6), 0.12, 2.8),
  };
};

const normalizeCommandId = (rawCommandId) => {
  const value = Number(rawCommandId);
  if (Number.isFinite(value) && value > 0) {
    return Math.trunc(value);
  }
  return Date.now();
};

const writeRouteArtifacts = async (payload) => {
  await ensureWebStateDir();

  const route = validatePoints(payload?.route || []);
  const task = resolveTaskKey(payload?.algorithm?.task);
  const algorithmKey = resolveAlgorithmKey(payload?.algorithm?.key);
  const params = payload?.algorithm?.params || {};
  const motionProfile = sanitizeMotionProfile(payload?.motion);

  const routeJson = {
    type: "route",
    coordinateContractVersion: coordinateContract.version,
    createdAt: new Date().toISOString(),
    task,
    algorithm: {
      key: algorithmKey,
      params,
    },
    motion: motionProfile,
    route,
  };

  const csvLines = [ROUTE_CSV_HEADER];
  for (let index = 0; index < route.length; index += 1) {
    const point = route[index];
    const previous =
      index === 0
        ? { x: 0, y: 0 }
        : route[index - 1];
    const headingDeg = toDegrees(
      Math.atan2(point.y - previous.y, point.x - previous.x)
    );
    csvLines.push(`${point.x},${point.y},${headingDeg}`);
  }
  const motionLines = [
    `cruise_speed_mps ${motionProfile.cruiseSpeedMps}`,
    `payload_kg ${motionProfile.payloadKg}`,
    `battery_range ${motionProfile.batteryRange}`,
  ];

  await Promise.all([
    fsp.writeFile(ROUTE_JSON_PATH, JSON.stringify(routeJson, null, 2)),
    fsp.writeFile(ROUTE_CSV_PATH, `${csvLines.join("\n")}\n`),
    fsp.writeFile(MOTION_PROFILE_PATH, `${motionLines.join("\n")}\n`),
  ]);
};

const writeLimitZoneArtifacts = async (payload) => {
  await ensureWebStateDir();

  const zones = validatePolygons(payload?.zones || []);
  const zonesJson = {
    type: "limit_zones",
    coordinateContractVersion: coordinateContract.version,
    createdAt: new Date().toISOString(),
    zones,
  };

  const textLines = [`zone_count ${zones.length}`];
  for (const zone of zones) {
    textLines.push(`zone ${zone.points.length}`);
    for (const point of zone.points) {
      textLines.push(`${point.x} ${point.y}`);
    }
  }

  await Promise.all([
    fsp.writeFile(LIMIT_ZONES_JSON_PATH, JSON.stringify(zonesJson, null, 2)),
    fsp.writeFile(LIMIT_ZONES_TXT_PATH, `${textLines.join("\n")}\n`),
  ]);
};

const writeRuntimeCommandArtifact = async (payload) => {
  await ensureWebStateDir();
  const obstacle = sanitizeRuntimeObstacle(payload?.obstacle);
  const commandId = normalizeCommandId(payload?.commandId);
  const lines = [
    `id ${commandId}`,
    "type spawn_obstacle",
    `x ${obstacle.x}`,
    `y ${obstacle.y}`,
    `size_x ${obstacle.sizeX}`,
    `size_y ${obstacle.sizeY}`,
    `height ${obstacle.height}`,
  ];
  await fsp.writeFile(RUNTIME_COMMAND_PATH, `${lines.join("\n")}\n`);
};

const normalizeFileTelemetry = (raw, rawMap = null) => {
  const pose = raw?.[TELEMETRY_POSE_KEY] || null;
  const x = Number(pose?.x ?? raw?.robot?.x ?? raw?.x);
  const y = Number(pose?.y ?? raw?.robot?.y ?? raw?.robot?.z ?? raw?.y);
  const z = Number(pose?.z ?? raw?.robot?.z ?? raw?.z ?? 0);
  const yaw = Number(pose?.yaw ?? raw?.robot?.yaw ?? raw?.robot?.heading ?? raw?.yaw);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(yaw)) {
    return null;
  }

  const obstacleTrace = Array.isArray(raw?.perception?.obstacleTrace)
    ? raw.perception.obstacleTrace
        .map((point) => ({
          x: Number(point?.x),
          y: Number(point?.y),
          confidence: Number(point?.confidence),
        }))
        .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
        .map((point) => ({
          x: point.x,
          y: point.y,
          confidence: Number.isFinite(point.confidence)
            ? clamp(point.confidence, 0, 1)
            : 1,
        }))
    : [];
  const obstacleMap = normalizeObstacleMap(rawMap, raw?.obstacleMap || null);

  return {
    type: TELEMETRY_MESSAGE_TYPE,
    coordinateContractVersion: coordinateContract.version,
    pose: {
      x,
      y,
      z,
      yaw,
    },
    x,
    y,
    z,
    yaw,
    navigation: raw?.navigation || null,
    perception: {
      lidar: raw?.perception?.lidar || null,
      obstacleTrace,
    },
    obstacleMap,
    obstacleTrace,
    simulationTime: Number(raw?.simulationTime) || 0,
  };
};

const buildSolverInput = ({ points, algorithmKey, taskKey, params, seed }) => {
  const lines = [
    `task ${taskKey}`,
    `algorithm ${algorithmKey}`,
    `seed ${seed}`,
    `params ${params.nests} ${params.pa} ${params.max_iter} ${params.alpha} ${params.beta}`,
    `count ${points.length}`,
  ];

  for (const point of points) {
    lines.push(`${point.x} ${point.y}`);
  }

  return `${lines.join("\n")}\n`;
};

const parseSolverOutput = (stdout) => {
  const lines = stdout
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean);

  if (!lines.length) {
    throw new Error("Solver returned an empty response.");
  }

  const [statusLine, ...rest] = lines;
  if (statusLine !== "status ok") {
    const errorLine = rest.find((line) => line.startsWith("message "));
    throw new Error(errorLine ? errorLine.slice("message ".length) : "Native solver failed.");
  }

  const getValueLine = (prefix) => {
    const line = rest.find((item) => item.startsWith(prefix));
    if (!line) throw new Error(`Solver response is missing "${prefix.trim()}".`);
    return line.slice(prefix.length);
  };

  const closed = getValueLine("closed ") === "1";
  const length = Number(getValueLine("length "));
  const order = getValueLine("order ")
    .split(/\s+/)
    .filter(Boolean)
    .map((value) => Number(value));
  const routeCount = Number(getValueLine("route_count "));
  if (!Number.isFinite(routeCount) || routeCount < 0) {
    throw new Error("Solver returned an invalid route count.");
  }

  const routeStart = rest.findIndex((line) => line.startsWith("route_count "));
  if (routeStart < 0) {
    throw new Error("Solver response is missing route_count.");
  }
  const route = rest
    .slice(routeStart + 1, routeStart + 1 + routeCount)
    .map((line) => {
      const [x, y] = line.split(/\s+/).map(Number);
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        throw new Error("Solver returned an invalid route point.");
      }
      return { x, y };
    });

  return {
    closed,
    length: Number.isFinite(length) ? length : 0,
    order,
    route,
  };
};

const runNativeSolver = async ({ points, algorithmKey, params, taskKey, seed }) => {
  if (!(await solverExists())) {
    throw new Error(
      "Не найден gpo_route_solver.exe. Сначала соберите native solver командой native\\build_msvc.bat."
    );
  }

  const input = buildSolverInput({
    points,
    algorithmKey,
    taskKey,
    params,
    seed,
  });

  return new Promise((resolve, reject) => {
    const child = spawn(SOLVER_EXE_PATH, [], {
      stdio: ["pipe", "pipe", "pipe"],
      windowsHide: true,
    });

    let stdout = "";
    let stderr = "";
    let settled = false;
    let timedOut = false;

    const timeoutId = setTimeout(() => {
      timedOut = true;
      child.kill();
    }, SOLVER_TIMEOUT_MS);

    const finish = (handler) => {
      if (settled) return;
      settled = true;
      clearTimeout(timeoutId);
      handler();
    };

    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      stdout += chunk;
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });

    child.on("error", (error) => {
      finish(() => {
        reject(new Error(`Не удалось запустить native solver: ${error.message}`));
      });
    });

    child.on("close", (code, signal) => {
      finish(() => {
        if (timedOut) {
          reject(new Error(`Native solver превысил лимит ${SOLVER_TIMEOUT_MS} мс.`));
          return;
        }
        if (signal) {
          reject(new Error(`Native solver был остановлен сигналом ${signal}.`));
          return;
        }
        if (code !== 0) {
          if (stdout.trim()) {
            try {
              resolve(parseSolverOutput(stdout));
              return;
            } catch (parseError) {
              reject(parseError);
              return;
            }
          }
          reject(new Error(stderr.trim() || "Native solver завершился с ошибкой."));
          return;
        }

        try {
          resolve(parseSolverOutput(stdout));
        } catch (parseError) {
          reject(parseError);
        }
      });
    });

    child.stdin.on("error", () => {});
    child.stdin.end(input, "utf8");
  });
};

const sendJson = (response, statusCode, payload) => {
  response.writeHead(statusCode, {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Headers": "Content-Type",
    "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
    "Content-Type": "application/json; charset=utf-8",
  });
  response.end(JSON.stringify(payload));
};

const readJsonBody = (request) =>
  new Promise((resolve, reject) => {
    let body = "";
    request.on("data", (chunk) => {
      body += chunk;
      if (body.length > 1024 * 1024) {
        reject(new Error("Request body is too large."));
        request.destroy();
      }
    });
    request.on("end", () => {
      try {
        resolve(body ? JSON.parse(body) : {});
      } catch {
        reject(new Error("Invalid JSON payload."));
      }
    });
    request.on("error", reject);
  });

// 9001: telemetry from controller -> broadcast to UI clients
const telemetryWss = new WebSocket.Server({ host: BRIDGE_HOST, port: TELEMETRY_PORT });
const telemetryClients = new Set();
const telemetrySenders = new Set();

let telemetryCount = 0;
let lastRealTelemetryAt = 0;
let mockPhase = 0;
let lastFileTelemetryMtime = -1;
let telemetryFilePollInFlight = false;

const broadcastTelemetry = (payload, exclude = null) => {
  for (const client of telemetryClients) {
    if (client === exclude) continue;
    if (client.readyState === WebSocket.OPEN) {
      client.send(payload);
    }
  }
};

const buildMockTelemetry = () => {
  mockPhase += 0.2;
  const radius = 3.8;
  const x = radius * Math.cos(mockPhase);
  const y = radius * Math.sin(mockPhase);
  const z = 0;
  const yaw = mockPhase + Math.PI / 2;

  return {
    type: TELEMETRY_MESSAGE_TYPE,
    coordinateContractVersion: coordinateContract.version,
    pose: {
      x,
      y,
      z,
      yaw,
    },
    x,
    y,
    z,
    yaw,
  };
};

telemetryWss.on("connection", (ws) => {
  telemetryClients.add(ws);
  console.log("[telemetry] client connected");

  ws.on("message", (data) => {
    telemetrySenders.add(ws);
    lastRealTelemetryAt = Date.now();
    const text = typeof data === "string" ? data : data.toString();
    telemetryCount += 1;
    if (telemetryCount % 20 === 1) {
      console.log(`[telemetry] msg ${telemetryCount} size=${text.length}`);
    }
    broadcastTelemetry(text, ws);
  });

  ws.on("close", () => {
    telemetryClients.delete(ws);
    telemetrySenders.delete(ws);
    console.log("[telemetry] client disconnected");
  });
});

setInterval(() => {
  if (!ENABLE_MOCK_TELEMETRY) return;
  if (telemetryClients.size === 0) return;
  if (Date.now() - lastRealTelemetryAt < TELEMETRY_IDLE_MS) return;

  const payload = JSON.stringify(buildMockTelemetry());
  for (const client of telemetryClients) {
    if (telemetrySenders.has(client)) continue;
    if (client.readyState === WebSocket.OPEN) {
      client.send(payload);
    }
  }
}, 250);

const pollFileTelemetry = async () => {
  if (telemetryFilePollInFlight) return;
  telemetryFilePollInFlight = true;

  try {
    const stat = await fsp.stat(ROBOT_STATE_PATH);
    const mtime = stat.mtimeMs;
    if (!Number.isFinite(mtime) || mtime === lastFileTelemetryMtime) return;

    const [text, obstacleMapText] = await Promise.all([
      fsp.readFile(ROBOT_STATE_PATH, "utf8"),
      fsp.readFile(OBSTACLE_MAP_PATH, "utf8").catch((error) => {
        if (error?.code === "ENOENT") return null;
        throw error;
      }),
    ]);
    const parsed = safeJsonParse(text);
    const parsedObstacleMap = obstacleMapText ? safeJsonParse(obstacleMapText) : null;
    const normalized = normalizeFileTelemetry(parsed, parsedObstacleMap);
    if (!normalized) return;

    lastFileTelemetryMtime = mtime;
    lastRealTelemetryAt = Date.now();
    broadcastTelemetry(JSON.stringify(normalized));
  } catch (error) {
    if (error?.code !== "ENOENT") {
      console.error("[telemetry] failed to poll robot_state.json:", error.message);
    }
  } finally {
    telemetryFilePollInFlight = false;
  }
};

setInterval(() => {
  void pollFileTelemetry();
}, FILE_TELEMETRY_POLL_MS);

// 9002: route from UI -> forward to controller
// UI should connect to ws://127.0.0.1:9002/ui
const routeWss = new WebSocket.Server({ host: BRIDGE_HOST, port: ROUTE_PORT });
let controllerConn = null;
const uiRouteClients = new Set();

routeWss.on("connection", (ws, req) => {
  const url = req?.url || "/";
  const isUi = url.startsWith("/ui");
  if (isUi) uiRouteClients.add(ws);
  else controllerConn = ws;

  console.log(`[route] client connected (${isUi ? "ui" : "controller"})`);

  ws.on("message", async (data) => {
    if (!isUi) return;
    const text = data.toString();
    const parsed = safeJsonParse(text);
    if (parsed?.type === "route") {
      try {
        await writeRouteArtifacts(parsed);
      } catch (error) {
        console.error("[route] failed to write route artifacts:", error.message);
      }
    } else if (parsed?.type === "limit_zones") {
      try {
        await writeLimitZoneArtifacts(parsed);
      } catch (error) {
        console.error("[route] failed to write limit zone artifacts:", error.message);
      }
    } else if (parsed?.type === "spawn_random_obstacle") {
      try {
        await writeRuntimeCommandArtifact(parsed);
      } catch (error) {
        console.error("[route] failed to write runtime obstacle command:", error.message);
      }
    }
    if (controllerConn && controllerConn.readyState === WebSocket.OPEN) {
      controllerConn.send(text);
    } else {
      console.log("[route] controller websocket not connected; route saved to web_state");
    }
  });

  ws.on("close", () => {
    if (isUi) uiRouteClients.delete(ws);
    if (controllerConn === ws) controllerConn = null;
    console.log("[route] client disconnected");
  });
});

const solverServer = http.createServer(async (request, response) => {
  if (request.method === "OPTIONS") {
    sendJson(response, 204, {});
    return;
  }

  if (request.method === "GET" && request.url === "/health") {
    sendJson(response, 200, {
      ok: true,
      coordinateContractVersion: coordinateContract.version,
      solverAvailable: await solverExists(),
      solverPath: SOLVER_EXE_PATH,
    });
    return;
  }

  if (request.method === "POST" && request.url === "/api/solve-route") {
    try {
      const body = await readJsonBody(request);
      const points = validatePoints(body.points);
      const algorithmKey = resolveAlgorithmKey(body?.algorithm?.key);
      const params = sanitizeNativeParams(algorithmKey, body?.algorithm?.params);
      const taskKey = resolveTaskKey(body?.task);
      const seed = clampInt(normalizeNumber(body?.seed, 1337), 1, 2147483647);

      const solved = await runNativeSolver({
        points,
        algorithmKey,
        params,
        taskKey,
        seed,
      });

      sendJson(response, 200, {
        ok: true,
        task: taskKey,
        algorithm: algorithmKey,
        length: solved.length,
        closed: solved.closed,
        order: solved.order,
        route: solved.route,
      });
    } catch (error) {
      sendJson(response, 500, {
        ok: false,
        error: error instanceof Error ? error.message : "Unexpected solver failure.",
      });
    }
    return;
  }

  sendJson(response, 404, {
    ok: false,
    error: "Not found.",
  });
});

solverServer.listen(SOLVER_PORT, BRIDGE_HOST, () => {
  console.log(`[solver] ${SOLVER_HTTP_URL}`);
});

console.log(`[bridge] telemetry ${TELEMETRY_WS_URL}, route ${ROUTE_WS_URL}`);
if (ENABLE_MOCK_TELEMETRY) {
  console.log("[bridge] mock telemetry is enabled (MOCK_TELEMETRY=1)");
}
