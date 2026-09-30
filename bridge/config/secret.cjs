const fs = require("node:fs");
const path = require("node:path");
const readSecret = ({ value = "", file = "", root }) => {
  if (value && file) throw new Error("Configure either GATEWAY_TOKEN or GATEWAY_TOKEN_FILE");
  if (!file) return value;
  let secret;
  try { secret = fs.readFileSync(path.resolve(root, file), "utf8").trim(); }
  catch { throw new Error("Cannot read GATEWAY_TOKEN_FILE"); }
  if (!secret) throw new Error("GATEWAY_TOKEN_FILE is empty");
  return secret;
};
module.exports = { readSecret };
