const fs = require("node:fs");
const {randomBytes} = require("node:crypto");
const filename = "/run/secrets/gateway_token";
try {
  fs.writeFileSync(filename, randomBytes(32).toString("hex") + "\n", {flag:"wx",mode:0o444});
} catch (error) {
  if (error.code !== "EEXIST") throw error;
}
if (!fs.readFileSync(filename,"utf8").trim()) throw new Error("Gateway token is empty");
