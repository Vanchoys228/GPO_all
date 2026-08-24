# Camera Map Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move camera-map coordinate calculations into the existing pure camera module without changing storage behavior.

**Architecture:** Value-only pose/config/point structures isolate geometry from Webots and map arrays. The main controller remains an adapter that reads pose and appends returned points.

**Tech Stack:** C11, Webots Controller C API, Webots MinGW GCC 14.2.

---

### Task 1: Specify geometry with failing tests

**Files:**
- Modify: `webots/controllers/youbot_web/controller_camera_test.c`

- [ ] Add obstacle-point tests for forward and rotated poses.
- [ ] Add invalid-range tests.
- [ ] Add free-ray count, coordinates, margin, and invalid-input tests.
- [ ] Compile and confirm failure because the geometry API is missing.

### Task 2: Implement pure geometry

**Files:**
- Modify: `webots/controllers/youbot_web/controller_camera.h`
- Modify: `webots/controllers/youbot_web/controller_camera.c`

- [ ] Define explicit pose, geometry config, and point value types.
- [ ] Implement obstacle-point calculation with existing transforms and guards.
- [ ] Implement bounded free-ray generation with existing step order and guards.
- [ ] Run the focused strict test and expect exit code 0.

### Task 3: Integrate and verify

**Files:**
- Modify: `webots/controllers/youbot_web/youbot_web.c`
- Modify: `REFACTORING_PLAN.md`
- Modify: `webots/README.md`

- [ ] Replace only coordinate calculations in both merge functions.
- [ ] Keep pose reading, map appends, confidence, and persistence unchanged.
- [ ] Update documentation and line counts.
- [ ] Run 12 strict C tests, full strict controller build, 54 JS tests, lint,
  frontend build, and bridge smoke test.

No Git steps apply because this workspace has no `.git` repository.
