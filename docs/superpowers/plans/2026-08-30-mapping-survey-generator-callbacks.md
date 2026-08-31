# Mapping Survey Generator Callbacks Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove Mapping Survey generator and route-phase callbacks from `youbot_web.c` without changing route-generation behavior.

**Architecture:** Add a callback-adapter module that owns `SurveyGrid`, the generator context, the `ControllerSurveyGeneratorCallbacks` table, and the `ControllerSurveyRouteCallbacks` table. `youbot_web.c` will provide thin dependency callbacks for map/runtime access and existing Mapping Survey services; the adapter will preserve the current phase order: safe start, contour, boundary fallback, horizontal coverage, then vertical coverage.

**Tech Stack:** C11, Webots controller build, existing controller unit-test runner.

---

## Chunk 1: Callback adapter

### Task 1: Specify the adapter’s observable callback delegation

**Files:**
- Create: `webots/controllers/youbot_web/controller_mapping_survey_generator_callbacks_test.c`
- Create: `webots/controllers/youbot_web/controller_mapping_survey_generator_callbacks.h`

- [x] **Step 1: Write a failing test**

Define injectable operations that record grid construction, flood-fill, route phases, and route writing. Assert the adapter invokes them with the robot position, room index, supplied command/path, configured start-clearance factor, and the contour → boundary → horizontal → vertical ordering.

- [x] **Step 2: Run test to verify it fails**

Run: `cmd /c "set CONTROLLER_TEST_FILTER=controller_mapping_survey_generator_callbacks_test.c&& webots\\controllers\\youbot_web\\run_controller_tests.bat"`

Expected: compilation failure because the adapter module is missing.

- [x] **Step 3: Implement the minimal adapter interface and callbacks**

Create a focused module with an operations struct representing only integration dependencies: clear/prepare/read pose/find room, grid build/flood, start validation/addition, four route phases, and route write. Keep `SurveyGrid` private to the adapter context and construct the two existing controller callback tables there.

- [x] **Step 4: Run test to verify it passes**

Run: `cmd /c "set CONTROLLER_TEST_FILTER=controller_mapping_survey_generator_callbacks_test.c&& webots\\controllers\\youbot_web\\run_controller_tests.bat"`

Expected: one passing test executable.

### Task 2: Wire the Webots integration to the adapter

**Files:**
- Modify: `webots/controllers/youbot_web/youbot_web.c:949-1220`
- Modify: `webots/controllers/youbot_web/controller_sources.txt`

- [x] **Step 1: Write a failing build-level assertion**

Extend the callback adapter test only if needed to cover an integration operation absent from Task 1; otherwise rely on the full controller build as the failing check for the removed static callbacks.

- [x] **Step 2: Run the focused test/build to verify the pre-change boundary**

Run the focused callback test and record the current passing baseline before deleting the in-file callback implementations.

- [x] **Step 3: Replace the local contexts and callback tables**

Leave `generate_mapping_survey_route` as a thin orchestrator: construct Webots-backed adapter operations from the existing helpers, configure constants, invoke the adapter to obtain `ControllerSurveyGeneratorCallbacks` and opaque context, and pass them to `controller_mapping_survey_route_generation_service_generate`. Remove `SurveyRouteBuilderContext`, `SurveyGeneratorContext`, and all `survey_route_*`/`survey_generator_*` functions from `youbot_web.c`.

- [x] **Step 4: Add the adapter source to the controller source list**

Append `controller_mapping_survey_generator_callbacks.c` to `controller_sources.txt` so both Webots and unit-test builds include it.

- [x] **Step 5: Run focused and full verification**

Run:

`cmd /c "set CONTROLLER_TEST_FILTER=controller_mapping_survey_generator_callbacks_test.c&& webots\\controllers\\youbot_web\\run_controller_tests.bat"`

`cmd /c "webots\\controllers\\youbot_web\\run_controller_tests.bat"`

`npm run lint`

Expected: all controller tests compile and pass; JavaScript lint remains clean.

- [ ] **Step 6: Commit**

```powershell
git add webots/controllers/youbot_web/controller_mapping_survey_generator_callbacks.c webots/controllers/youbot_web/controller_mapping_survey_generator_callbacks.h webots/controllers/youbot_web/controller_mapping_survey_generator_callbacks_test.c webots/controllers/youbot_web/controller_sources.txt webots/controllers/youbot_web/youbot_web.c docs/superpowers/plans/2026-08-30-mapping-survey-generator-callbacks.md
git commit -m "refactor: extract mapping survey generator callbacks"
```
