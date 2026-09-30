import { readFile, writeFile, mkdir, stat } from "node:fs/promises";
import { createHash } from "node:crypto";
import path from "node:path";
import sqlite from "../repositories/sqlite-repository.cjs";
import contract from "../protocol/mission-contract.cjs";

const checksum = records => createHash("sha256").update(JSON.stringify(records)).digest("hex");

// The repository owner lock excludes service writers for the entire export.
export async function exportRepository(directory, destination) {
  await stat(path.join(directory, "records.sqlite"));
  const repository = sqlite.createSqliteRepository({ directory });
  try {
    const records = await repository.list();
    await writeFile(destination, JSON.stringify({ version: 1, createdAt: new Date().toISOString(), records, checksum: checksum(records) }), { flag: "wx", mode: 0o600 });
  } finally { await repository.close(); }
}

export async function importRepository(source, directory) {
  const document = JSON.parse(await readFile(source, "utf8"));
  if (document.version !== 1 || !Array.isArray(document.records)) throw new Error("Unsupported backup format");
  if (checksum(document.records) !== document.checksum) throw new Error("Backup checksum mismatch");
  const ids = new Set();
  for (const record of document.records) {
    contract.validateMissionId(record.missionId);
    if (ids.has(record.missionId)) throw new Error("Duplicate backup record");
    ids.add(record.missionId);
  }
  // Never merge restored commands into live service state.
  await mkdir(directory);
  const repository = sqlite.createSqliteRepository({ directory });
  try {
    await repository.ready();
    for (const record of document.records) await repository.save(record);
  } finally { await repository.close(); }
}
