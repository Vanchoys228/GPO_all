import { existsSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";

describe("architecture cleanup", () => {
  it("publishes bridge ports on loopback only", () => {
    const compose = readFileSync("compose.yaml", "utf8");

    for (const port of [9001, 9002, 9003]) {
      expect(compose).toContain(`127.0.0.1:${port}:${port}`);
    }
  });

  it("defines production frontend and optional headless Webots containers", () => {
    const compose = readFileSync("compose.yaml", "utf8");

    expect(compose).toContain("dockerfile: docker/frontend.Dockerfile");
    expect(compose).toContain('127.0.0.1:8080:80');
    expect(compose).toContain("dockerfile: docker/webots.Dockerfile");
    expect(compose).toContain("- simulation");
    expect(existsSync("docker/nginx.conf")).toBe(true);
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

  it("does not track generated Webots UI metadata or runtime camera frames", () => {
    expect(existsSync("webots/worlds/.youbot_only.wbproj")).toBe(false);
    expect(existsSync("webots/worlds/.youbot_only.jpg")).toBe(false);
    expect(existsSync("web_state/camera_frame.jpg")).toBe(false);

    const gitignore = readFileSync(".gitignore", "utf8");
    expect(gitignore).toContain("web_state/*.jpg");
    expect(gitignore).toContain("webots/worlds/.*.wbproj");
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
