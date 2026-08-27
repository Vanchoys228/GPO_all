# Robot Control Decomposition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not use subagents for this repository.

**Goal:** Turn `youbot_web.c` into a thin Webots adapter by extracting the remaining navigation orchestration into tested modules that form the future Robot Control service boundary.

**Architecture:** Preserve the current synchronous control loop and all observable statuses while moving pure decisions into C domain modules. Webots API access, device reads, and actuator writes remain in `youbot_web.c`; extracted modules accept value objects and return decisions or commands. No network transport, process extraction, Docker, or Webots container work belongs to this plan.

**Tech Stack:** C11, Webots C API, MSVC batch builds, Makefile builds, C executable tests, Vitest architecture tests.

---

## File Structure

New modules live beside the existing controller modules in
`webots/controllers/youbot_web`:

- `controller_navigation_session.{h,c}`: waiting, finished, and manual-relocation decisions.
- `controller_navigation_zone_guard.{h,c}`: waypoint and segment zone-block decisions.
- `controller_navigation_perception.{h,c}`: normalized camera/LiDAR navigation inputs.
- `controller_avoidance_start.{h,c}`: whether and how to enter avoidance.
- `controller_avoidance_recovery.{h,c}`: rejoin, orbit, skip, and free-space recovery decisions.
- `controller_avoidance_presentation.{h,c}`: translate avoidance commands to stable controller statuses and state updates.
- `controller_navigation_service.{h,c}`: service-layer operation that computes one navigation decision from a prepared input snapshot.

Every module gets a matching `*_test.c`. Add every production source to
`Makefile`, `build_youbot_web.bat`, and the `SOURCES` list in
`run_controller_tests.bat`. Keep `controllerBuild.test.js` green so a source
cannot be omitted from a build path.

## Invariants for Every Task

- Preserve status strings, error strings, tolerances, cooldowns, and command ordering.
- Write and run a failing test before creating production code.
- Keep Webots functions out of extracted domain and service modules.
- Run the new C test, all controller tests, and the native controller build.
- Do not begin the next task with a failing baseline.
- Commit only when working in a real Git worktree; this workspace currently has no `.git` metadata.

### Task 1: Navigation session and relocation decisions

**Files:**
- Create: `webots/controllers/youbot_web/controller_navigation_session.h`
- Create: `webots/controllers/youbot_web/controller_navigation_session.c`
- Test: `webots/controllers/youbot_web/controller_navigation_session_test.c`
- Modify: `webots/controllers/youbot_web/youbot_web.c` around `run_navigation_step`
- Modify: controller Makefile and both Windows build scripts

- [ ] **Step 1: Write the failing test**

Define an input containing route count, finished flag, relocation flag, current
waypoint, and pose. Assert these decisions independently:

```c
WAIT_FOR_ROUTE   /* route count == 0 */
STOP_FINISHED    /* route_finished != 0 */
RELOCALIZE       /* relocation detected with an active route */
CONTINUE         /* ordinary active navigation */
```

Assert that relocalization returns the current target distance and the current
pose as the new segment origin.

- [ ] **Step 2: Verify RED**

Run `webots\controllers\youbot_web\run_controller_tests.bat`.
Expected: compilation fails because `controller_navigation_session.h` does not exist.

- [ ] **Step 3: Implement the pure decision function**

The module must not set globals or call `stop_robot`. It returns a decision plus:

```c
typedef struct {
  ControllerNavigationSessionDecision decision;
  double distance_to_target;
  double segment_start_x;
  double segment_start_z;
} ControllerNavigationSessionOutput;
```

- [ ] **Step 4: Replace the first three branches in `run_navigation_step`**

Keep effects in `youbot_web.c`: statuses, resetting navigation, clearing errors,
and stopping the robot. Delete only logic now represented by the module.

- [ ] **Step 5: Verify GREEN**

Run all controller tests and `build_youbot_web.bat`. Expected: all tests pass
and the compiler emits no warnings.

### Task 2: Dynamic-zone navigation guard

**Files:**
- Create: `controller_navigation_zone_guard.h`
- Create: `controller_navigation_zone_guard.c`
- Test: `controller_navigation_zone_guard_test.c`
- Modify: the zone-block section of `youbot_web.c`
- Modify: all controller build lists

- [ ] **Step 1: Write failing table-driven tests**

Cover an unblocked target, target inside a dynamic zone, blocked segment,
mapping-survey room-zone exclusion, skippable survey waypoint, and non-skippable
final waypoint. The result type is:

```c
typedef enum {
  CONTROLLER_ZONE_ROUTE_CLEAR,
  CONTROLLER_ZONE_SKIP_WAYPOINT,
  CONTROLLER_ZONE_BLOCKED_TARGET,
  CONTROLLER_ZONE_BLOCKED_SEGMENT
} ControllerNavigationZoneDecision;
```

- [ ] **Step 2: Verify RED, then implement minimal geometry orchestration**

Reuse `controller_zone_geometry` operations through callbacks or prepared
boolean inputs. Do not duplicate polygon mathematics.

- [ ] **Step 3: Integrate and preserve effects**

Keep the exact statuses `mapping_survey_skipped_blocked_waypoint`,
`mapping_survey_skipped_blocked_segment`, and `blocked_by_dynamic_zone`, plus
their existing error messages.

- [ ] **Step 4: Run full C and native-build verification**

### Task 3: Navigation perception snapshot

**Files:**
- Create: `controller_navigation_perception.h`
- Create: `controller_navigation_perception.c`
- Test: `controller_navigation_perception_test.c`
- Modify: camera/LiDAR context construction in `youbot_web.c`
- Modify: all controller build lists

- [ ] **Step 1: Test normalization rules first**

Cover camera fallback range, field-of-view filtering, minimum confidence/count,
camera turn-sign deadband, and forwarding of LiDAR detection values.

- [ ] **Step 2: Implement one input-to-output operation**

```c
void controller_navigation_perception_prepare(
    const ControllerNavigationPerceptionInput *input,
    ControllerNavigationPerceptionOutput *output);
```

The output contains camera-front-obstacle state, preferred turn sign, and the
`ControllerAvoidanceDetection` used by later stages.

- [ ] **Step 3: Replace local scalar preparation in `run_navigation_step`**

Do not read Webots devices from the module. Pass the already captured sensor
snapshot into it.

- [ ] **Step 4: Run full verification**

### Task 4: Avoidance entry decision

**Files:**
- Create: `controller_avoidance_start.h`
- Create: `controller_avoidance_start.c`
- Test: `controller_avoidance_start_test.c`
- Modify: the avoidance-start branch in `youbot_web.c`
- Modify: all controller build lists

- [ ] **Step 1: Write failing tests**

Verify no-op when avoidance is active or not requested. Verify left/right
clearance selection, turn choice inputs, initial avoidance state, priority hold,
and detour initialization when avoidance begins.

- [ ] **Step 2: Implement the operation using existing domain functions**

Compose `controller_avoidance_choose_turn_sign`,
`controller_avoidance_state_begin`, and `controller_avoidance_set_detour`.
Do not duplicate those algorithms.

- [ ] **Step 3: Integrate after survey scan insertion**

Preserve the existing ordering: survey scan insertion gets the first opportunity
to consume a newly detected obstacle; ordinary avoidance begins only afterward.

- [ ] **Step 4: Run full verification**

### Task 5: Avoidance recovery orchestration

**Files:**
- Create: `controller_avoidance_recovery.h`
- Create: `controller_avoidance_recovery.c`
- Test: `controller_avoidance_recovery_test.c`
- Modify: active-avoidance lifecycle branch in `youbot_web.c`
- Modify: all controller build lists

- [ ] **Step 1: Write failing tests for each lifecycle outcome**

Cover continuing avoidance, rejoining the route, invoking mapping-loop escape,
skipping a looped waypoint when escape fails, and reacquiring free space.

- [ ] **Step 2: Implement a decision result without Webots callbacks**

Return an enum and state changes. Keep route regeneration and orbit escape behind
an integration callback owned by `youbot_web.c` until the Robot Control service
interface is introduced.

- [ ] **Step 3: Integrate while preserving stop/return behavior**

Each branch must stop or continue on exactly the same simulation step as before.

- [ ] **Step 4: Run full verification**

### Task 6: Avoidance command application and status mapping

**Files:**
- Create: `controller_avoidance_presentation.h`
- Create: `controller_avoidance_presentation.c`
- Test: `controller_avoidance_presentation_test.c`
- Modify: avoidance-command branch in `youbot_web.c`
- Modify: all controller build lists

- [ ] **Step 1: Write failing tests for command effects**

Cover detour clearing, hold clearing, turn-sign changes, stuck reset, and every
status mapping:

```text
PASS_GAP        -> passing_lidar_gap
HARD_TURN       -> avoiding_gap_turn / avoiding_committed_turn
GAP_DRIVE       -> avoiding_gap_drive
COMMITTED_DRIVE -> avoiding_committed_drive
ESCAPE          -> avoiding_committed_escape
```

- [ ] **Step 2: Implement pure state/result application**

Return the status string and updated avoidance metadata. Applying wheel velocity
remains in `youbot_web.c`.

- [ ] **Step 3: Integrate and run full verification**

### Task 7: Robot Control navigation service operation

**Files:**
- Create: `controller_navigation_service.h`
- Create: `controller_navigation_service.c`
- Test: `controller_navigation_service_test.c`
- Modify: `run_navigation_step` in `youbot_web.c`
- Modify: all controller build lists
- Modify: `docs/ARCHITECTURE_INVENTORY.md`

- [ ] **Step 1: Define the service boundary in a failing test**

The input is a prepared snapshot of pose, route, zones, sensor context,
navigation state, avoidance state, and runtime limits. The output is one of:

```c
STOP, APPLY_MOTION, ROUTE_ADVANCED, ROUTE_COMPLETED,
REQUEST_SURVEY_SCAN, REQUEST_REPLAN, STATE_ONLY
```

The output also carries linear/angular speed, status, error, target distance,
and explicit state updates.

- [ ] **Step 2: Verify RED and implement by composing extracted modules**

Do not copy algorithms into the service. It coordinates
`controller_navigation_session`, `controller_navigation_route`, zone guard,
perception, avoidance, and tracking modules.

- [ ] **Step 3: Reduce `run_navigation_step` to an adapter**

Its responsibilities become: capture Webots inputs, construct the service
input, invoke one service operation, perform requested integration callbacks,
apply motion, and publish status/error.

- [ ] **Step 4: Add architecture assertions**

Update `webots/controllerBuild.test.js` to require every new production module
in Makefile, native Windows build, and controller-test source lists. Add a
reasonable maximum-size assertion for `run_navigation_step`, based on its final
verified size rather than an arbitrary whole-file limit.

- [ ] **Step 5: Run the complete project verification**

Run:

```text
npx vitest run --maxWorkers=2
npm run lint
npm run build
npx vitest run webots/controllerBuild.test.js --maxWorkers=2
npm run test:bridge
webots\controllers\youbot_web\run_controller_tests.bat
webots\controllers\youbot_web\build_youbot_web.bat
```

Expected: all frontend and architecture tests pass, all C tests pass, bridge
smoke test passes, and both builds complete without new warnings.

- [ ] **Step 6: Document the achieved boundary**

Record which data belongs to Robot Control, which effects remain in the Webots
Adapter, and the exact interface that Stage 2 will expose through a service
layer. Do not add network transport yet.

## Completion Criteria

- `run_navigation_step` is an adapter/coordinator, not the owner of navigation algorithms.
- Extracted modules contain no Webots API calls.
- Existing status and error strings remain compatible with the frontend.
- Existing runtime files and local startup continue to work.
- Every extracted decision has direct C test coverage.
- The full regression suite and native controller build pass.
- No Docker or process-boundary implementation has begun.
