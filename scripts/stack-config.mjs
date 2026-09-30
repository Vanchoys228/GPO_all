import { mkdir, mkdtemp, readFile, writeFile } from "node:fs/promises";
import os from "node:os";
import { randomBytes } from "node:crypto";
import path from "node:path";

export async function ensureToken(file) {
  await mkdir(path.dirname(file), { recursive: true });
  try { await writeFile(file, randomBytes(32).toString("hex") + "\n", { flag: "wx", mode: 0o600 }); }
  catch (error) { if (error.code !== "EEXIST") throw error; }
  const token = (await readFile(file, "utf8")).trim();
  if (!token) throw new Error("Stack token file is empty");
  return token;
}

export function parseOptions(args) {
  for (const arg of args) if (!["--headless", "--no-build", "--no-webots"].includes(arg)) throw new Error(`Unknown option: ${arg}`);
  return { headless: args.includes("--headless"), build: !args.includes("--no-build"), webots: !args.includes("--no-webots") };
}

export async function stageToken(token) {
  // Private parent directory protects the host; the bind-mounted file is readable
  // by the container's non-root UID without changing the persistent secret's mode.
  const directory = await mkdtemp(path.join(os.tmpdir(), "gpo-secret-"));
  const file = path.join(directory, "gateway-token");
  await writeFile(file, token, { mode: 0o444, flag: "wx" });
  return { directory, file };
}
