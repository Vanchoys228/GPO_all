import { readFileSync, readdirSync } from "node:fs";
import { describe, expect, it } from "vitest";
import packageJson from "../package.json";

const controllerDirectory = "webots/controllers/youbot_web";

describe("Webots controller build configuration", () => {
  it("links every production controller source", () => {
    const makefile = readFileSync(`${controllerDirectory}/Makefile`, "utf8");
    const productionSources = readdirSync(controllerDirectory)
      .filter((file) => file.endsWith(".c") && !file.endsWith("_test.c"))
      .sort();

    for (const source of productionSources) {
      expect(makefile, `${source} is missing from C_SOURCES`).toMatch(
        new RegExp(`(^|\\s)${source.replace(".", "\\.")}($|\\s)`),
      );
    }
  });

  it("uses strict void signatures for lifecycle callbacks", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");
    const callbackNames = [
      "maybe_reload_zones",
      "maybe_reload_surface_zones",
      "maybe_reload_route",
      "maybe_reload_motion_profile",
      "maybe_reload_runtime_command",
      "capture_lidar_trace",
      "maybe_write_map",
      "maybe_update_camera_perception",
      "maybe_write_camera_frame",
      "maybe_write_camera_map",
      "run_navigation_step",
      "update_route_avoidance_metrics",
      "write_state_snapshot",
    ];

    for (const name of callbackNames) {
      expect(source, `${name} must accept an explicit void parameter list`).toMatch(
        new RegExp(`static\\s+(?:void|int)\\s+${name}\\(void\\)`),
      );
    }
  });

  it("provides one command for all standalone controller tests", () => {
    expect(packageJson.scripts["test:webots"]).toBe(
      "webots\\controllers\\youbot_web\\run_controller_tests.bat",
    );
  });

  it("keeps Supervisor pose access outside the orchestration file", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).not.toContain("wb_supervisor_node_get_self");
    expect(source).not.toContain("wb_supervisor_field_get_sf_vec3f");
    expect(source).not.toContain("wb_supervisor_field_set_sf_vec3f");
    expect(source).toContain('#include "controller_webots_pose.h"');
  });

  it("keeps Webots motor access outside the orchestration file", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).not.toContain("wb_motor_set_position");
    expect(source).not.toContain("wb_motor_set_velocity");
    expect(source).not.toContain("<webots/motor.h>");
    expect(source).toContain('#include "controller_webots_devices.h"');
  });

  it("keeps Webots sensor access outside the orchestration file", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).not.toContain("wb_robot_get_device");
    expect(source).not.toContain("wb_lidar_");
    expect(source).not.toContain("wb_camera_");
    expect(source).not.toContain("<webots/lidar.h>");
    expect(source).not.toContain("<webots/camera.h>");
    expect(source).toContain('#include "controller_webots_sensors.h"');
  });
});
