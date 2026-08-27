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

  it("keeps every reusable production source in the Windows build and standalone test runner", () => {
    const productionSources = readdirSync(controllerDirectory)
      .filter((file) => file.endsWith(".c") && !file.endsWith("_test.c"))
      .filter((file) => file !== "youbot_web.c");
    const windowsBuild = readFileSync(`${controllerDirectory}/build_youbot_web.bat`, "utf8");
    const testRunner = readFileSync(`${controllerDirectory}/run_controller_tests.bat`, "utf8");

    for (const source of productionSources) {
      expect(windowsBuild, `${source} is missing from build_youbot_web.bat`).toContain(source);
      expect(testRunner, `${source} is missing from run_controller_tests.bat`).toContain(source);
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

  it("allows a single standalone controller test to be selected", () => {
    const testRunner = readFileSync(`${controllerDirectory}/run_controller_tests.bat`, "utf8");

    expect(testRunner).toContain("CONTROLLER_TEST_FILTER");
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

  it("delegates limit-zone rendering to the Simulation Adapter", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_webots_simulation_sync_limit_zones(");
    expect(source).not.toContain("name \"dynamic_zone_wall\"");
    expect(source).not.toContain("controller_webots_simulation_format_limit_wall(");
  });

  it("delegates limit-zone node synchronization to the Simulation Adapter", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_webots_simulation_sync_limit_zones(");
    expect(source).not.toContain('"WEB_LIMIT_%d_%d"');
  });

  it("delegates surface-zone presentation to the Simulation Adapter", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_webots_simulation_sync_surface_zones(");
    expect(source).not.toContain("geometry IndexedFaceSet {");
    expect(source).not.toContain("controller_webots_simulation_format_surface_zone(");
  });

  it("delegates runtime-obstacle presentation to the Simulation Adapter", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_webots_simulation_spawn_runtime_obstacle(");
    expect(source).not.toContain("name \"runtime_obstacle\"");
    expect(source).not.toContain("controller_webots_simulation_format_runtime_obstacle(");
    expect(source).not.toContain("wb_supervisor_node_get_from_def");
  });

  it("delegates camera-to-LiDAR range matching to Camera Fusion", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_camera_fusion_estimate_range(");
    expect(source).not.toContain("double best_angle_error = CAMERA_RANGE_SEARCH_WINDOW_RAD");
  });

  it("delegates virtual-camera reticle rendering to Camera Render", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_camera_render_reticle(");
    expect(source).not.toContain("center_x - 14, horizon, center_x + 14, horizon, 80, 220, 230");
  });

  it("delegates virtual-camera waypoint-marker rendering to Camera Render", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_camera_render_waypoint_marker(");
    expect(source).not.toContain("target_x - 3, horizon - 21, target_x + 3, horizon - 15");
  });

  it("delegates camera-observation decisions to Camera", () => {
    const source = readFileSync(`${controllerDirectory}/youbot_web.c`, "utf8");

    expect(source).toContain("controller_camera_observation_hint(");
    expect(source).not.toContain("camera_obstacle_center_offset * fmax(effective_fov, 0.8) * 0.5");
  });
});
