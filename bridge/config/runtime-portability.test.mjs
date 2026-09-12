import { expect, it } from "vitest";
import { execFileSync } from "node:child_process";
import { mkdtemp, rm } from "node:fs/promises";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";

it("resolves relative service paths against the project from another working directory", async () => {
  const root = fileURLToPath(new URL("../../", import.meta.url));
  const directory = await mkdtemp(path.join(os.tmpdir(), "gpo-working-dir-"));
  try {
    const configPath = path.join(root, "bridge/config/runtime-config.cjs");
    const output = execFileSync(process.execPath, ["-e", "console.log(JSON.stringify(require(process.argv[1])))", configPath], {
      cwd: directory,
      env: { ...process.env, WEB_STATE_DIR: "./state", MISSION_STATE_DIR: "./missions", SOLVER_PATH: "./native/build/solver" },
      encoding: "utf8",
    });
    const config = JSON.parse(output.trim().split(/\r?\n/).at(-1));
    expect(config.WEB_STATE_DIR).toBe(path.join(root, "state"));
    expect(config.MISSION_STATE_DIR).toBe(path.join(root, "missions"));
    expect(config.SOLVER_PATH).toBe(path.join(root, "native/build/solver"));
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});
