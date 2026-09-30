const fs = require("node:fs");
const path = require("node:path");
const { format } = require("node:util");

const createLogger = ({ service, directory, secret = "", maxBytes = 5 * 1024 * 1024, write = line => process.stdout.write(line) }) => {
  const filename = directory && path.join(directory, `${service}.log`);
  if (directory) fs.mkdirSync(directory, { recursive: true });
  return (level, ...args) => {
    let message = format(...args);
    if (secret) message = message.split(secret).join("[redacted]");
    message = message.replace(/Bearer\s+\S+/gi, "Bearer [redacted]");
    const record = { time: new Date().toISOString(), service, level, message };
    let line = JSON.stringify(record) + "\n";
    while (Buffer.byteLength(line) > maxBytes && record.message.length) {
      record.message = record.message.slice(0, Math.floor(record.message.length / 2));
      record.truncated = true;
      line = JSON.stringify(record) + "\n";
    }
    write(line);
    if (!filename) return;
    try {
      const size = fs.existsSync(filename) ? fs.statSync(filename).size : 0;
      if (size + Buffer.byteLength(line) > maxBytes) {
        fs.rmSync(`${filename}.3`, { force: true });
        for (let i = 2; i >= 0; i--) {
          const source = i ? `${filename}.${i}` : filename;
          if (fs.existsSync(source)) fs.renameSync(source, `${filename}.${i + 1}`);
        }
      }
      fs.appendFileSync(filename, line, { mode: 0o600 });
    } catch {
      // Disk errors must not interrupt command processing; stdout remains available.
      write(JSON.stringify({ time: record.time, service, level: "error", message: "File logging failed" }) + "\n");
    }
  };
};

const installLogging = service => {
  const config = require("../config/runtime-config.cjs");
  const log = createLogger({ service, directory: config.LOG_DIR, secret: config.GATEWAY_TOKEN });
  for (const level of ["log", "info", "warn", "error"]) console[level] = (...args) => log(level, ...args);
};
module.exports = { createLogger, installLogging };
