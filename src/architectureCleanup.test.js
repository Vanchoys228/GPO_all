import { existsSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";

describe("architecture cleanup", () => {
  it("publishes bridge ports on loopback only", () => {
    const compose = readFileSync("compose.yaml", "utf8");

    for (const port of [9001, 9002, 9003]) {
      expect(compose).toContain(`127.0.0.1:${port}:${port}`);
    }
  });

  it("does not keep the unused collapsible section implementation", () => {
    expect(existsSync("src/components/dashboard/CollapsibleSection.jsx")).toBe(false);

    const plannerUiState = readFileSync("src/lib/plannerUiState.js", "utf8");
    expect(plannerUiState).not.toContain("getSectionOpen");
    expect(plannerUiState).not.toContain("setSectionOpenInStorage");
    expect(plannerUiState).not.toContain("sections");
  });

  it("ignores generated binaries and does not keep the unused legacy map asset", () => {
    expect(existsSync("public/map.png")).toBe(false);
    const gitignore = readFileSync(".gitignore", "utf8");
    expect(gitignore).toContain("native/build");
    expect(gitignore).toContain("webots/controllers/youbot_web/*.exe");
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
