import { expect, it } from "vitest";
import { mkdtempSync, readFileSync, readdirSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import logging from "./logger.cjs";
it("redacts secrets in stdout and keeps a bounded rotating file history", () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "gpo-log-"));
  const lines = [];
  try {
    const logger = logging.createLogger({ service: "route", directory, secret: "private-token", maxBytes: 256, write: line => lines.push(line) });
    for (let i = 0; i < 20; i++) logger("error", "failed private-token Bearer another-token " + "x".repeat(300));
    expect(lines.join("")).not.toContain("private-token");
    expect(lines.join("")).not.toContain("another-token");
    expect(JSON.parse(lines[0]).service).toBe("route");
    const files = readdirSync(directory);
    expect(files.length).toBeLessThanOrEqual(4);
    for (const file of files) expect(readFileSync(path.join(directory, file)).length).toBeLessThanOrEqual(256);
  } finally { rmSync(directory, { recursive: true, force: true }); }
});
