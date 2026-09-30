import {readdir,readFile} from "node:fs/promises";
import {describe,expect,it} from "vitest";
describe("application service boundaries", () => {
  it("depends on injected ports and contracts, not transport or simulator implementations", async () => {
    for (const file of (await readdir(new URL("./",import.meta.url))).filter(name => name.endsWith(".cjs"))) {
      const source=await readFile(new URL(file,import.meta.url),"utf8");
      expect(source).not.toMatch(/require\(["'](?:node:)?(?:fs|http|https|net|ws|child_process)["']|(?:require\(|import\()["'][^"']*(?:adapters|repositories|artifacts|servers|src)\//);
    }
  });
});

it("mission and telemetry composition cannot import simulator files",async()=>{
  for(const file of ["route-process.cjs","telemetry-process.cjs"]){
    const source=await readFile(new URL(`../processes/${file}`,import.meta.url),"utf8");
    expect(source).not.toMatch(/WEB_STATE_DIR|webots-file|webots-telemetry|web-state-store/);
  }
});
