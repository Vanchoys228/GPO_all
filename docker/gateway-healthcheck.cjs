const {readFileSync} = require("node:fs");
const token = readFileSync(process.env.GATEWAY_TOKEN_FILE, "utf8").trim();
fetch("http://127.0.0.1:9004/ready", {
  headers: {Authorization: `Bearer ${token}`},
  signal: AbortSignal.timeout(3000),
}).then(response => {if (!response.ok) process.exitCode = 1;})
  .catch(() => {process.exitCode = 1;});
