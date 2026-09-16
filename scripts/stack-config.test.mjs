import { expect, it } from "vitest";
import { mkdtemp, readFile, rm, stat } from "node:fs/promises";
import path from "node:path";
import os from "node:os";
import { ensureToken, parseOptions, stageToken } from "./stack-config.mjs";
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
it("stages a non-root-readable mount inside a private host directory", async () => {
  const staged = await stageToken("test-token");
  try {
    expect(await readFile(staged.file, "utf8")).toBe("test-token");
    if (process.platform !== "win32") {
      expect((await stat(staged.directory)).mode & 0o777).toBe(0o700);
      expect((await stat(staged.file)).mode & 0o777).toBe(0o444);
    }
  } finally { await rm(staged.directory, { recursive: true, force: true }); }
});
