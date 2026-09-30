import path from "node:path";
import { exportRepository, importRepository } from "../bridge/operations/repository-backup.mjs";

const [operation, source, destination, ...extra] = process.argv.slice(2);
if (!["export", "import"].includes(operation) || !source || !destination || extra.length) {
  console.error("Usage: node scripts/repository-backup.mjs export <state-directory> <new-backup.json> | import <backup.json> <new-state-directory>");
  process.exitCode = 1;
} else {
  try {
    await (operation === "export" ? exportRepository : importRepository)(path.resolve(source), path.resolve(destination));
    console.log(`Repository ${operation} completed.`);
  } catch (error) {
    console.error(`Repository ${operation} failed: ${error.message}`);
    process.exitCode = 1;
  }
}
