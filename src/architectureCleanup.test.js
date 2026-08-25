import { existsSync, readFileSync, readdirSync } from "node:fs";
import { describe, expect, it } from "vitest";

describe("architecture cleanup", () => {
  it("does not keep the unused collapsible section implementation", () => {
    expect(existsSync("src/components/dashboard/CollapsibleSection.jsx")).toBe(false);

    const plannerUiState = readFileSync("src/lib/plannerUiState.js", "utf8");
    expect(plannerUiState).not.toContain("getSectionOpen");
    expect(plannerUiState).not.toContain("setSectionOpenInStorage");
    expect(plannerUiState).not.toContain("sections");
  });

  it("does not keep generated binaries or the unused legacy map asset", () => {
    expect(existsSync("public/map.png")).toBe(false);
    expect(existsSync("native/build/gpo_route_solver.exe")).toBe(false);

    const controllerFiles = readdirSync("webots/controllers/youbot_web");
    expect(controllerFiles.some((file) => file.endsWith(".exe"))).toBe(false);
  });

  it("does not keep temporary implementation plan artifacts", () => {
    expect(existsSync("docs/superpowers")).toBe(false);
  });

  it("keeps the editable architecture source without a stale rendered duplicate", () => {
    expect(existsSync("PROJECT_MODULARITY.drawio")).toBe(true);
    expect(existsSync("PROJECT_MODULARITY.png")).toBe(false);
  });

  it("does not keep Vite starter styles and branding", () => {
    expect(existsSync("src/App.css")).toBe(false);
    expect(existsSync("src/assets")).toBe(false);
    expect(existsSync("public/vite.svg")).toBe(false);

    const indexHtml = readFileSync("index.html", "utf8");
    expect(indexHtml).toContain('<html lang="ru">');
    expect(indexHtml).toContain("<title>GPO — планировщик маршрута</title>");
    expect(indexHtml).not.toContain("vite.svg");
  });
});
