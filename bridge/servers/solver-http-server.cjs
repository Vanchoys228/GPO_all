const http = require("http");
const {
  clampInt,
  normalizeNumber,
  resolveAlgorithmKey,
  resolveTaskKey,
  sanitizeNativeParams,
  validatePoints,
} = require("../protocol/validation.cjs");

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

const createSolverHttpServer = ({
  coordinateContract,
  host,
  nativeSolver,
  port,
  solverPath,
}) => {
  const server = http.createServer(async (request, response) => {
    if (request.method === "OPTIONS") {
      sendJson(response, 204, {});
      return;
    }

    if (request.method === "GET" && request.url === "/health") {
      sendJson(response, 200, {
        ok: true,
        coordinateContractVersion: coordinateContract.version,
        solverAvailable: await nativeSolver.solverExists(),
        solverPath,
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

        const solved = await nativeSolver.run({
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

  return {
    server,
    start() {
      server.listen(port, host, () => {
        console.log(`[solver] http://${host}:${port}`);
      });
      return server;
    },
  };
};

module.exports = { createSolverHttpServer };
