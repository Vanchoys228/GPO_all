# Microservice and Service-Layer Architecture

## Goal

Transform the current project from a collection of coupled frontend, bridge,
native-planning, and Webots components into independently runnable services.
Each developer must be able to run an isolated local copy. The final delivery
stage will package all services, including Webots, into containers. Webots will
support browser-accessible graphical operation.

Container implementation is deliberately postponed until the service
boundaries, contracts, independent processes, and non-container integration
tests are complete.

## Migration Strategy

Use an incremental two-step extraction:

1. Introduce explicit domain modules and service-layer interfaces while the
   system still runs locally in its current form.
2. Move those stable service interfaces behind process and network boundaries.

This preserves working behavior during the migration and avoids combining
business-logic refactoring with distributed-system failures.

## Target Services

### Gateway Service

The frontend's only backend entry point. It owns HTTP and WebSocket transport,
request validation, command routing, response composition, and presentation of
service availability errors. It contains no navigation or planning logic.

### Route Service

Owns routes, waypoints, limit zones, surface zones, coordinate validation,
route versions, and import/export. Existing files remain an initial storage
adapter and may later be replaced without changing consumers.

### Planning Service

Owns route construction, route optimization, native algorithms, mapping-survey
planning, and replanning. It consumes versioned planning requests and returns
validated planning results. It neither controls Webots nor owns UI state.

### Telemetry Service

Owns the latest observable robot state: pose, route progress, navigation
status, errors, LiDAR-derived data, obstacle maps, and camera state. It streams
updates through the Gateway. Intermediate telemetry frames may be dropped, but
the latest state must remain available.

### Robot Control Service

Owns navigation decisions: route tracking, obstacle avoidance, dynamic-zone
handling, mapping survey, completion decisions, motion commands, and control
status. Fast control-loop components remain in-process modules of this service
rather than separate network services.

### Webots Adapter

An infrastructure adapter located next to Webots. It reads simulator sensors,
converts them to shared contracts, sends sensor frames to Robot Control,
receives motion commands, applies wheel velocities, and performs permitted
simulation operations. It contains no planning or presentation logic.

## Internal Service Structure

Each service is divided into:

1. Transport layer: HTTP, WebSocket, or internal message handlers.
2. Service layer: application use cases and orchestration.
3. Domain layer: transport-independent business and calculation logic.
4. Infrastructure layer: files, native executables, Webots API, HTTP,
   WebSocket, and future persistent storage.

Representative service-layer operations include `StartMappingSurvey`,
`BuildRoute`, `ActivateRoute`, `UpdateRobotState`, `ProcessSensorFrame`,
`CalculateMotionCommand`, and `SpawnRuntimeObstacle`.

## Shared Contracts

Versioned contracts live under `shared/contracts` and include:

- `RobotState`
- `SensorFrame`
- `MotionCommand`
- `Route`
- `RouteCommand`
- `PlanningRequest`
- `PlanningResult`
- `TelemetryEvent`
- `ServiceError`

Every cross-service message includes a schema version, request or event ID,
timestamp, source, and payload. JSON conversion remains at transport boundaries
and does not enter C calculation modules.

## Data Flow

```text
Frontend -> Gateway -> Route Service
                    -> Planning Service -> Route Service

Webots -> Webots Adapter -> Robot Control Service
Webots <- Webots Adapter <- Robot Control Service

Robot Control Service -> Telemetry Service -> Gateway -> Frontend
```

## Failure Behavior

- Losing the frontend or Gateway does not stop an active route.
- Losing Planning does not stop the current route.
- Losing Robot Control communication causes the adapter to issue a safe stop.
- Repeated commands with the same request ID are not executed twice.
- Unsupported contract versions are rejected with explicit errors.
- Telemetry preserves the latest known state even when intermediate frames are
  discarded.
- A service restart must not silently execute stale commands.

## Testing Strategy

Each service receives domain unit tests, service-layer tests, contract tests,
transport integration tests, and health checks. Before containerization, the
complete system must pass an end-to-end local-process test. The final stage adds
Docker Compose smoke tests and Webots graphical-access verification.

## Delivery Stages

### Stage 1: Robot Control decomposition

Approximately seven slices: route state and relocation, dynamic zones, sensor
context, avoidance start, active avoidance lifecycle, avoidance commands, and
normal route tracking. `youbot_web.c` becomes a thin Webots coordinator.

### Stage 2: Service layer

Approximately five or six slices: shared contracts, Robot Control interface,
Route Service, Planning Service, Telemetry Service, and Gateway orchestration.

### Stage 3: Independent processes

Approximately five slices: extract each service process and the Webots Adapter.
Each process gets independent configuration, startup, health reporting, network
contracts, failure handling, and tests.

### Stage 4: Non-container integration

Approximately three or four slices: contract tests, process integration,
complete Webots/frontend execution, and service-disconnection recovery.

### Stage 5: Architecture cleanup

Approximately two slices: remove obsolete bridges and duplicated scripts, then
normalize directories, documentation, and launch commands.

### Stage 6: Final containerization

Approximately four to six slices: service images, Compose topology, Webots
image, browser-accessible Webots GUI, networks, volumes, health checks, startup
ordering, and complete smoke testing.

The total estimate is 26–30 slices, with roughly 20–24 occurring before the
final Docker stage.

## Explicit Non-Goals Until the Final Stage

- Do not containerize Webots yet.
- Do not modify Docker topology while service boundaries are still unstable.
- Do not split latency-sensitive LiDAR, tracking, avoidance, and wheel control
  into separate network services.
- Do not require a shared external environment between developers' local
  copies.
