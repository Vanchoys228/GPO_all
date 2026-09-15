import { expect, it } from "vitest";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import path from "node:path";
import os from "node:os";
import { ensureToken, parseOptions } from "./stack-config.mjs";
it("creates a persistent secret without replacing it on restart", async () => {
  const root = await mkdtemp(path.join(os.tmpdir(), "stack-token-"));
  try {
    const file = path.join(root, "secrets/token");
    const first = await ensureToken(file);
    expect(first).toMatch(/^[a-f0-9]{64}$/);
    expect(await ensureToken(file)).toBe(first);
    expect((await readFile(file, "utf8")).trim()).toBe(first);
  } finally { await rm(root, { recursive: true, force: true }); }
});
it("rejects unknown launcher flags", () => {
  expect(() => parseOptions(["--unknown"])).toThrow("Unknown");
  expect(parseOptions(["--headless", "--no-build"])).toMatchObject({ headless: true, build: false });
});
