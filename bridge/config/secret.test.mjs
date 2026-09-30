import { expect, it } from "vitest";
import { mkdtempSync, writeFileSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import secret from "./secret.cjs";
it("loads a secret file and rejects ambiguous or empty secrets without disclosing contents", () => {
  const root = mkdtempSync(path.join(os.tmpdir(), "gpo-secret-"));
  try {
    writeFileSync(path.join(root, "token"), "private-value\n");
    expect(secret.readSecret({ file: "token", root })).toBe("private-value");
    expect(() => secret.readSecret({ value: "other", file: "token", root })).toThrow("either");
    writeFileSync(path.join(root, "token"), "\n");
    expect(() => secret.readSecret({ file: "token", root })).toThrow("empty");
  } finally { rmSync(root, { recursive: true, force: true }); }
});
