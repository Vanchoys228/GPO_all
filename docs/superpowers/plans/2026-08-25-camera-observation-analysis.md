# Camera Observation Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract real-camera RGB obstacle analysis from `youbot_web.c` without changing detection or navigation behavior.

**Architecture:** A Webots-independent `controller_camera` module receives image dimensions and a pixel-reader callback and returns a value-only observation. `youbot_web.c` remains the adapter and orchestrator for Webots image access, lidar range refinement, map updates, scheduling, and telemetry.

**Tech Stack:** C11, Webots Controller C API, Webots MinGW GCC 14.2, standalone executable tests.

---

### Task 1: Define the camera analysis contract through tests

**Files:**
- Create: `webots/controllers/youbot_web/controller_camera_test.c`
- Create later: `webots/controllers/youbot_web/controller_camera.h`
- Create later: `webots/controllers/youbot_web/controller_camera.c`

- [ ] Write a standalone test with an in-memory RGB frame and a callback returning `ControllerCameraPixel`.
- [ ] Assert the default crop fractions (`0.14`, `0.86`, `0.18`, `0.82`), sample step `4`, and score threshold `0.025`.
- [ ] Assert invalid dimensions produce a zeroed, invisible observation.
- [ ] Assert a neutral frame has samples but no hits and is invisible.
- [ ] Assert a warm rectangle uses the existing RGB thresholds, reports exact hits/bounds/score, produces the expected normalized horizontal offset and fallback range, and crosses the visibility threshold.
- [ ] Assert a small warm region below threshold retains measurements but remains invisible.
- [ ] Compile the test before adding production files and verify the expected failure is a missing `controller_camera.h`.

Run from `webots/controllers/youbot_web`:

```powershell
gcc -std=c11 -Wall -Wextra -Werror controller_camera_test.c controller_camera.c -lm -o controller_camera_test_verify.exe
```

Expected: FAIL because the new module does not exist.

### Task 2: Implement the pure camera module

**Files:**
- Create: `webots/controllers/youbot_web/controller_camera.h`
- Create: `webots/controllers/youbot_web/controller_camera.c`
- Test: `webots/controllers/youbot_web/controller_camera_test.c`

- [ ] Define `ControllerCameraPixel`, `ControllerCameraPixelReader`, `ControllerCameraAnalysisConfig`, and `ControllerCameraObservation`.
- [ ] Add `controller_camera_default_config(width, height)` with current constants.
- [ ] Add `controller_camera_observation_reset` so every exit returns deterministic neutral values and empty bounds.
- [ ] Add `controller_camera_analyze(config, reader, context, observation)`.
- [ ] Preserve the exact crop rounding, sampling loops, warm-pixel predicate, y weighting, score, centroid, clamp, and fallback-range formulas from `update_camera_obstacle_hint`.
- [ ] Keep the module free of Webots headers, global controller state, allocation, and I/O.
- [ ] Compile and run the focused test; expect exit code 0.

### Task 3: Integrate through a Webots adapter

**Files:**
- Modify: `webots/controllers/youbot_web/youbot_web.c:1-18`
- Modify: `webots/controllers/youbot_web/youbot_web.c:540-627`
- Modify: `webots/controllers/youbot_web/build_youbot_web.bat`

- [ ] Include `controller_camera.h`.
- [ ] Add a file-local pixel-reader adapter that calls `wb_camera_image_get_red`, `wb_camera_image_get_green`, and `wb_camera_image_get_blue` with the original image pointer and width.
- [ ] Replace only the RGB scan/classification block with `controller_camera_analyze`.
- [ ] Copy observation score, offset, and hit count to the existing globals.
- [ ] Preserve the existing angle formula, lidar range refinement, free-ray update, obstacle-map update, and confidence formula, using returned bounds.
- [ ] Add `controller_camera.c` to the Windows build script.
- [ ] Compile and run all standalone C tests with every production module and `-Wall -Wextra -Werror`; expect 12 passing executables.
- [ ] Compile and link the full Webots controller with `-Wall -Wextra -Werror`; expect exit code 0 and no diagnostics.

### Task 4: Document and verify the completed slice

**Files:**
- Modify: `REFACTORING_PLAN.md`
- Modify if module list is present: `webots/README.md`

- [ ] Record the new camera module, updated `youbot_web.c` line count, and 12 standalone C tests.
- [ ] Update the next safe slice to camera-map geometry/storage, leaving virtual rendering and I/O for later.
- [ ] Run `npm test -- --run`; expect 54 passing tests.
- [ ] Run `npm run lint`; expect exit code 0.
- [ ] Run `npm run build`; expect exit code 0 (existing bundle-size advisory is allowed).
- [ ] Run `npm run test:bridge`; expect the bridge smoke test to pass.
- [ ] Re-run all 12 C tests and the full strict controller build after documentation changes.
- [ ] Inspect the final changed files and confirm Docker files are untouched.

No commit steps are included because the supplied workspace is not a Git repository.
