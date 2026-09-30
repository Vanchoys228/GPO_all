import { afterEach, expect, it } from "vitest";
import { mkdtemp, rm, readFile, writeFile } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import sqlite from "../repositories/sqlite-repository.cjs";
import { exportRepository, importRepository } from "./repository-backup.mjs";
const directories = [];
afterEach(async () => { for (const directory of directories.splice(0)) await rm(directory, { recursive: true, force: true }); });
const fixture = async () => {
  const root = await mkdtemp(path.join(os.tmpdir(), "gpo-backup-")); directories.push(root);
  const source = path.join(root, "source"), backup = path.join(root, "backup.json"), target = path.join(root, "restored");
  const repository = sqlite.createSqliteRepository({ directory: source });
  await repository.save({ missionId: "mission-1", status: "completed" });
  await repository.close();
  return { source, backup, target };
};
it("round trips records without process ownership and refuses overwrite", async () => {
  const { source, backup, target } = await fixture();
  await exportRepository(source, backup);
  await importRepository(backup, target);
  const restored = sqlite.createSqliteRepository({ directory: target });
  try { expect(await restored.get("mission-1")).toMatchObject({ status: "completed" }); }
  finally { await restored.close(); }
  await expect(importRepository(backup, target)).rejects.toThrow();
  await expect(exportRepository(source, backup)).rejects.toThrow();
});
it("refuses a live repository and detects corruption before creating target", async () => {
  const { source, backup, target } = await fixture();
  const repository = sqlite.createSqliteRepository({ directory: source });
  await repository.ready();
  try { await expect(exportRepository(source, backup)).rejects.toThrow("already owned"); }
  finally { await repository.close(); }
  await exportRepository(source, backup);
  const document = JSON.parse(await readFile(backup, "utf8"));
  document.records[0].status = "prepared";
  await writeFile(backup, JSON.stringify(document));
  await expect(importRepository(backup, target)).rejects.toThrow("checksum");
  await expect(readFile(path.join(target, "records.sqlite"))).rejects.toThrow();
});
