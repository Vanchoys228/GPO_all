const DEFAULT_ORIGINS = ["http://127.0.0.1:5173", "http://localhost:5173",
  "http://127.0.0.1:4173", "http://localhost:4173", "http://127.0.0.1:8080", "http://localhost:8080"];

const isAllowedOrigin = (origin) => {
  // CLI and the native local adapter have no browser Origin.
  if (origin === undefined) return true;
  const allowed = process.env.BRIDGE_ALLOWED_ORIGINS?.split(",").map(value => value.trim()) || DEFAULT_ORIGINS;
  return allowed.includes(origin);
};
const verifyWebSocketOrigin = ({ origin }) => isAllowedOrigin(origin);
module.exports = { isAllowedOrigin, verifyWebSocketOrigin };
