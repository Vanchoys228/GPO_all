# Controller Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix unsafe long-path reconstruction and conflicting camera-map cell classifications.

**Architecture:** Preserve the current fixed-capacity C architecture. Strengthen the existing module contracts so pathfinding fails closed and camera obstacle insertion atomically removes stale free-space evidence.

**Tech Stack:** C11, Webots MinGW GCC, standalone C test executables.

---

### Task 1: Mapping Survey path reconstruction

**Files:**
- Modify: `webots/controllers/youbot_web/controller_survey_geometry_test.c`
- Modify: `webots/controllers/youbot_web/controller_survey_geometry.c`

- [x] Add a regression test using a traversable path longer than 1024 cells.
- [x] Add a regression test proving insufficient output capacity returns failure.
- [x] Compile and run the focused test; confirm the new assertion fails.
- [x] Replace the reconstruction limit with `MAPPING_SURVEY_MAX_GRID_CELLS` and require both endpoints in the returned path.
- [x] Re-run the focused test; confirm it passes.

### Task 2: Camera map obstacle priority

**Files:**
- Modify: `webots/controllers/youbot_web/controller_camera_map_test.c`
- Modify: `webots/controllers/youbot_web/controller_camera_map.h`
- Modify: `webots/controllers/youbot_web/controller_camera_map.c`
- Modify: `webots/controllers/youbot_web/youbot_web.c`

- [x] Add a regression test that inserts free space before an obstacle at the same cell.
- [x] Compile and run the focused test; confirm the new assertion fails.
- [x] Extend obstacle insertion to remove matching free cells before adding the obstacle.
- [x] Update the controller adapter to pass both map collections.
- [x] Re-run the focused test; confirm it passes.

### Task 3: Full regression verification

**Files:**
- Modify: `REFACTORING_PLAN.md`

- [x] Run all 18 standalone C tests.
- [x] Compile and link the complete controller with strict warnings.
- [x] Run JavaScript tests, lint, Vite production build, and bridge smoke test.
- [x] Document the fixes and verification results.
