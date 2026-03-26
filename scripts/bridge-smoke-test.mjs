import { spawn } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.resolve(__dirname, "..");
const bridgePath = path.join(rootDir, "ws-bridge.cjs");

const BRIDGE_HOST = "127.0.0.1";
const TELEMETRY_PORT = "19001";
const ROUTE_PORT = "19002";
const SOLVER_PORT = "19003";
const SOLVER_URL = `http://${BRIDGE_HOST}:${SOLVER_PORT}`;

const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

const run = async () => {
  const bridge = spawn(process.execPath, [bridgePath], {
    cwd: rootDir,
    env: {
      ...process.env,
      BRIDGE_HOST,
      TELEMETRY_PORT,
      ROUTE_PORT,
      SOLVER_PORT,
    },
    stdio: ["ignore", "pipe", "pipe"],
    windowsHide: true,
  });

  let stdout = "";
  let stderr = "";
  let exitError = null;
  bridge.stdout.setEncoding("utf8");
  bridge.stderr.setEncoding("utf8");
  bridge.stdout.on("data", (chunk) => {
    stdout += chunk;
  });
  bridge.stderr.on("data", (chunk) => {
    stderr += chunk;
  });

  try {
    await wait(1200);

    const health = await fetch(`${SOLVER_URL}/health`).then((response) => response.json());
    if (!health.ok) {
      throw new Error("Bridge health check returned a failed payload.");
    }
    if (!health.solverAvailable) {
      throw new Error(
        "Native solver is unavailable. Run `npm run native:build` before bridge smoke tests."
      );
    }

    const solve = await fetch(`${SOLVER_URL}/api/solve-route`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        points: [
          { x: 0, y: 0 },
          { x: 1, y: 0 },
          { x: 0, y: 1 },
        ],
        algorithm: {
          key: "ga_tabu",
          params: {},
        },
        task: "tsp",
        seed: 1337,
      }),
    }).then((response) => response.json());

    if (!solve.ok || !Array.isArray(solve.route) || solve.route.length < 3) {
      throw new Error(`Unexpected solve payload: ${JSON.stringify(solve)}`);
    }

    console.log("bridge smoke test passed");
  } finally {
    bridge.kill();
    await wait(200);

    if (stderr.trim()) {
      console.error(stderr.trim());
    }
    if (bridge.exitCode && bridge.exitCode !== 0) {
      exitError = new Error(`Bridge exited with code ${bridge.exitCode}. Stdout: ${stdout}`);
    }
  }

  if (exitError) {
    throw exitError;
  }
};

run().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
