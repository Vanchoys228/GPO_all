require("dotenv").config();

const toPort = (value, fallback) => {
  const parsed = Number.parseInt(value, 10);
  if (!Number.isInteger(parsed) || parsed <= 0 || parsed > 65535) return fallback;
  return parsed;
};

const toHost = (value, fallback) => {
  const normalized = String(value || "").trim();
  return normalized || fallback;
};

const BRIDGE_HOST = toHost(process.env.BRIDGE_HOST, "127.0.0.1");
const TELEMETRY_PORT = toPort(process.env.TELEMETRY_PORT, 9001);
const ROUTE_PORT = toPort(process.env.ROUTE_PORT, 9002);
const SOLVER_PORT = toPort(process.env.SOLVER_PORT, 9003);

module.exports = {
  BRIDGE_HOST,
  TELEMETRY_PORT,
  ROUTE_PORT,
  SOLVER_PORT,
  TELEMETRY_WS_URL: `ws://${BRIDGE_HOST}:${TELEMETRY_PORT}`,
  ROUTE_WS_URL: `ws://${BRIDGE_HOST}:${ROUTE_PORT}`,
  SOLVER_HTTP_URL: `http://${BRIDGE_HOST}:${SOLVER_PORT}`,
};
