# YouBot Web Controller: final monolith decomposition

## Goal

Turn `webots/controllers/youbot_web/youbot_web.c` into a thin Webots composition root. Preserve controller behaviour, the on-disk state contract, and the public controller binary name while moving each remaining domain workflow to a focused module.

## Target architecture

`youbot_web.c` owns only Webots startup, the dependency graph, controller lifecycle, and shutdown. It initializes one long-lived `ControllerApplication` context, whose address is retained by the root for the full process lifetime.

`ControllerStepCallbacks` becomes context-aware: each callback is invoked with an opaque `void *context`, and `controller_step_run` receives that context. The application orchestration module owns construction of the callback table; the root passes `&application` as its context. This makes callback state explicit and removes the remaining runtime globals rather than recreating them in new modules.

The remaining static workflows move into modules grouped by responsibility:

- **application state**: owns the runtime state and the small Webots adapter contexts needed by all orchestrators;
- **configuration and inputs**: reloads motion profile, runtime commands, zones, and routes;
- **mapping and camera**: captures lidar traces, updates camera perception, synchronizes observations to maps, and writes map/frame artifacts;
- **survey orchestration**: generates and repairs mapping-survey routes through existing pure survey modules;
- **navigation orchestration**: runs a single navigation frame, translates module outcomes into runtime mutations, status, and drive commands;
- **telemetry orchestration**: turns the runtime state into the existing state snapshot.

Each module receives a narrowly typed context rather than reading globals. Existing pure modules stay unchanged unless their public interface needs a small adapter to express a dependency.

## Data flow

At startup, the root constructs `ControllerApplication` with the paths, control configuration, runtime, mapping/perception state, motion/navigation state, and Webots devices. The root initializes Webots adapters, then delegates initialization and reload callbacks through the application modules.

Every Webots tick follows the existing ordering: reload inputs, capture and merge perception, flush artifacts, execute navigation, update metrics, and publish telemetry. The new callback wiring preserves this order exactly.

Initialization order is also part of the contract: initialize paths; call `wb_robot_init`; initialize devices, drive adapter, sensors, navigation state, and pose tracking; construct zone-sync registries; reset robot pose; initialize runtime, mapping runtime, and route/zone service; clear maps and remove old camera frames; apply/load the motion profile and record its mtime; load zones and surface zones; record the runtime-command mtime; then either generate the default survey route when no route exists or wait for a fresh route. Supervisor unavailability preserves the current error status/error message. Shutdown always force-flushes the mapping runtime, removes synced zone nodes, and only then calls `wb_robot_cleanup`.

The input-orchestration module is the sole owner of `ControllerRouteZoneService`, all reload mtimes, the last processed command id, and the step counter. It retains the current modulo intervals and executes reloads in their current callback order, so status and error precedence stays unchanged and each input is processed once per eligible tick.

## Error handling

Modules report failures using the existing runtime status/error fields and return simple success results where the caller needs to branch. File paths, reload intervals, constants, and state-file formats remain stable. No new runtime dependencies or allocation paths are introduced.

## Verification

- Add focused unit tests for extracted orchestration seams where they can run without a Webots simulator, including callback ordering and reload de-duplication.
- Keep all current controller unit tests compiling with the complete production source list.
- Run the existing controller test runner, then perform a production `Makefile` build (or equivalent root compile/link smoke test), because the unit-test runner deliberately excludes `youbot_web.c`.
- Add contract coverage for startup when the route is absent, the forced final map flush, and the unchanged observable state JSON, map/camera-map/frame write paths.
- Run the frontend lint/build checks available in the worktree.

## Non-goals

- Changing robot navigation behaviour, route formats, telemetry schema, or Webots world setup.
- Rewriting already-extracted pure domain modules.
- Moving to a different process architecture.
