const http = require("http");
const { createPlanningService } = require("../services/planning-service.cjs");

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
  const planningService = createPlanningService({ nativeSolver });
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
        const result = await planningService.solve(body);
        if (body?.type === "planning.request") {
          const { createPlanningResult } = await import("../../shared/contracts/index.js");
          sendJson(response, 200, createPlanningResult({
            source: "planning-service",
            requestId: body.requestId,
            payload: result,
          }));
        } else {
          sendJson(response, 200, result);
        }
      } catch (error) {
        const statusCode = Number.isInteger(error?.statusCode) ? error.statusCode : 500;
        sendJson(response, statusCode, {
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
