const fs = require("fs");
const { spawn } = require("child_process");

const DEFAULT_TIMEOUT_MS = 30000;

const createNativeSolver = ({ solverPath, timeoutMs = DEFAULT_TIMEOUT_MS }) => {
  const solverExists = async () => {
    try {
      await fs.promises.access(solverPath, fs.constants.F_OK);
      return true;
    } catch {
      return false;
    }
  };

  const buildInput = ({ points, algorithmKey, taskKey, params, seed }) => {
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

  const parseOutput = (stdout) => {
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

  const run = async ({ points, algorithmKey, params, taskKey, seed }) => {
    if (!(await solverExists())) {
      throw new Error(
        `Не найден native solver: ${solverPath}. Сначала соберите его для текущей платформы.`
      );
    }

    const input = buildInput({ points, algorithmKey, taskKey, params, seed });

    return new Promise((resolve, reject) => {
      const child = spawn(solverPath, [], {
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
      }, timeoutMs);

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
        finish(() => reject(new Error(`Не удалось запустить native solver: ${error.message}`)));
      });

      child.on("close", (code, signal) => {
        finish(() => {
          if (timedOut) {
            reject(new Error(`Native solver превысил лимит ${timeoutMs} мс.`));
            return;
          }
          if (signal) {
            reject(new Error(`Native solver был остановлен сигналом ${signal}.`));
            return;
          }
          if (code !== 0 && !stdout.trim()) {
            reject(new Error(stderr.trim() || "Native solver завершился с ошибкой."));
            return;
          }

          try {
            resolve(parseOutput(stdout));
          } catch (error) {
            reject(error);
          }
        });
      });

      child.stdin.on("error", () => {});
      child.stdin.end(input, "utf8");
    });
  };

  return {
    buildInput,
    parseOutput,
    run,
    solverExists,
  };
};

module.exports = { createNativeSolver };
