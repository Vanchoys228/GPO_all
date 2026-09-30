# Automatic Object Transfer Design

## Goal

Add an optional automatic pick-and-place mission for the KUKA youBot. The robot
drives to one known box, grasps it, carries it to a destination selected on the
dashboard, releases it, and returns the arm to its transport pose.

The existing route mission remains independent. Sending a normal route never
requires an object, never starts the manipulator, and leaves the arm in its
transport pose.

## User flow

The dashboard contains a separate **Object transfer** section. It shows the
known box on the planning map and lets the user select one free destination on
the map. The section displays the source and destination coordinates, validation
feedback, the current transfer stage, and the actions **Transfer object** and
**Cancel**.

Starting a transfer creates a dedicated command. It does not modify or wrap a
normal route command. The controller progresses through nine active stages:

1. `approaching_object`
2. `aligning`
3. `lowering_arm`
4. `grasping`
5. `lifting`
6. `transporting`
7. `placing`
8. `releasing`
9. `returning_arm`

It then enters `completed`, `failed`, or `cancelled`.

Only one robot operation may be active. A normal route and an object transfer
cannot run concurrently, but either operation can be started when the robot is
idle. The box can remain in the world while the robot executes an unrelated
normal route.

## System boundaries

The existing distributed service layout remains in place:

```text
React dashboard
  -> route/mission service
  -> Webots gateway
  -> C++ Webots controller services
  -> Webots simulation
  -> telemetry service
  -> React dashboard
```

React owns interaction and presentation. The route/mission service validates
the command contract, assigns a durable operation identity, records lifecycle
state, and forwards the command through the gateway. The gateway remains the
only service that writes simulator exchange artifacts.

All robot-specific behavior is implemented in C++. JavaScript services do not
calculate joint trajectories, operate the gripper, locate the object, or decide
state-machine transitions.

## C++ controller design

The controller implementation is divided into small services with explicit
interfaces:

- `ManipulatorService` commands the five arm joints and two gripper fingers. It
  accepts named poses or a joint target and reports completion or timeout.
- `ObjectTransferService` owns the transfer state machine and its safety rules.
  It coordinates navigation, manipulation, grasping, telemetry, cancellation,
  and recovery without calling Webots APIs directly.
- `GraspService` verifies the box is within a configured grasp volume and
  creates or releases the simulated attachment.
- `ObjectLocator` reports the current box pose through a Webots Supervisor
  adapter.
- `TransferNavigationAdapter` converts approach and placement requirements into
  goals for the existing base navigation implementation.
- `TransferCommandParser` validates and decodes the simulator command artifact.
- `TransferTelemetryPublisher` contributes transfer state, progress, errors,
  and object pose to the existing robot telemetry snapshot.
- `RobotOperationCoordinator` is the only unit allowed to grant ownership of
  base motion. It grants either `route` or `object_transfer` ownership, never
  both. Transfer navigation is routed through the existing navigation runtime
  while transfer owns the base; the ordinary route loop remains paused without
  consuming or advancing its waypoints.

The services depend on narrow interfaces. Webots motor, node, field, and physics
calls terminate in adapters so the state machine and pose logic can be tested
without starting Webots.

## Scene and grasp behavior

The world adds `DEF DEMO_BOX Solid` with a 0.05 x 0.05 x 0.05 metre bounding
box, a mass of 0.08 kg, and an initial floor position. No modified youBot PROTO
is required. Its initial location is known but its current pose is always read
from the simulation, allowing repeated transfers.

The grasp is hybrid and deterministic. The robot physically approaches the
box, lowers the simulated arm, and closes both fingers. The Webots adapter reads
`DEMO_BOX` through the Supervisor API. The adapter enables `arm1sensor` through
`arm5sensor`. After the box enters the configured grasp volume, it records the
box pose relative to the gripper anchor. A pure C++ forward-kinematics function
uses the actual sensor readings and the R2025a youBot link transforms to compute
that anchor every step, including while joints interpolate between named poses.
While attached, each controller step transforms the recorded relative pose by
the current robot world pose and computed anchor, writes the box `translation`
and `rotation` fields, and calls
`wb_supervisor_node_reset_physics`. During placement, the arm lowers the box
before the adapter stops updating its pose and resets its physics one final
time. The box then participates in ordinary Webots physics again.

This is a controller-owned kinematic attachment rather than a new Webots joint.
Its complete state is held in memory. Controller startup always begins detached
and resets the box physics, so a restart cannot leave an invisible constraint.

The attachment is never created merely because a transfer command exists. It is
created only after positioning, finger closure, and the grasp-volume check pass.
This preserves a meaningful failure path while preventing an otherwise correct
demo from failing due to small friction differences.

## Commands and persistence

The browser continues using the existing `route.command` envelope and `/ui`
WebSocket acknowledgement. Its payload is discriminated by `type`. The shared
contract version remains `contractVersion: 1`; `transfer_object` is added as a
new allowed payload type:

```json
{
  "contractVersion": 1,
  "type": "route.command",
  "source": "planner-frontend",
  "requestId": "uuid",
  "timestamp": "2026-09-30T12:00:00.000Z",
  "payload": {
    "type": "transfer_object",
    "objectId": "demo-box",
    "destination": { "x": 4.0, "y": -2.0 },
    "scene": {
      "polygons": [],
      "surfaceZones": [],
      "chargingStations": [],
      "motion": {
        "batteryRange": 100,
        "cruiseSpeedMps": 0.22,
        "payloadKg": 0
      }
    },
    "sceneRevision": "sha256-of-the-included-scene"
  }
}
```

`route.command` is retained as the transport envelope name for compatibility;
the payload discriminator determines the operation. The route service returns
the existing `route.ack` with `operationType`, `missionId`, and persisted status.

The mission record adds `operationType: "route" | "object_transfer"`. Submission
dispatches to a route command validator or transfer command validator before a
common persistence and delivery path. Route-only fields such as `route`,
`planning`, and `scene` are optional on the common record and remain required by
the route validator. Transfer records persist `objectId`, `destination`, and
`sceneRevision`. Public mission history exposes `operationType` and transfer
progress without exposing command fingerprints.

The mission layer treats the request ID idempotently. Repeating the same request
with the same payload returns the existing operation. Reusing the ID for a
different payload is rejected. The gateway receives the existing versioned
`{version: 1, operation: "submit", requestId, payload}` envelope and writes a
`transfer_object` runtime command through the existing artifact boundary.

Gateway scene persistence writes limit zones, surface zones, and an atomic
`scene_revision.txt` marker before publishing the runtime command. The command
artifact includes `scene_revision`. The controller accepts a transfer only
after its zone reload has observed the same revision marker. A different or
missing marker produces `scene_revision_mismatch` without moving the robot.

Normal route commands retain their current payload and processing path.

## Validation and coordination

The dashboard performs immediate validation for usability. The transfer payload
includes the normalized scene snapshot and its revision, using the same DTO as
route planning and route submission. The mission service recomputes the revision
before validating the destination against the submitted restricted polygons;
the gateway persists that exact scene before the transfer command. JavaScript
authoritative checks cover only payload shape, finite world coordinates, known
`objectId`, scene consistency, and point exclusion from those polygons. They do
not claim that the object is present or reachable.

The authoritative planar bounds are the existing shared planning constants:
`x` from -22 to 22 metres and `y` from -17 to 17 metres. Dashboard and mission
validation import these constants rather than duplicating values in UI code.

The C++ controller owns physical validation: current object existence and pose,
approach-pose feasibility, base alignment, grasp volume, joint completion, and
safe release. It reports stable error codes: `object_not_found`,
`scene_revision_mismatch`, `approach_unreachable`, `alignment_failed`,
`grasp_failed`, `arm_timeout`, `navigation_failed`, `unsafe_release`, and
`operation_interrupted`.

A transfer is rejected when:

- another route or transfer operation is active;
- the destination is outside the traversable map or inside a restricted zone;
- the command version, object ID, coordinates, or request identity is invalid.

The C++ controller rejects after delivery when the object cannot be found or a
safe approach pose cannot be produced. These failures are returned through
operation feedback rather than guessed by the mission service.

The box does not make the transfer feature mandatory. A normal route may be
submitted without selecting the box or a transfer destination. Existing obstacle
perception may still detect the physical box like any other small obstacle.

## Failure and cancellation

Every state has a deadline. Failure stops the base immediately. Before grasping,
the arm returns to the transport pose. Grasp alignment may be retried once.

After attachment, the controller keeps the object attached if it cannot find a
safe release pose. It enters nonterminal `holding_for_recovery`; this state keeps
the shared robot-operation lock. Cancellation stops the base and attempts a
controlled placement at the current location. A successful placement produces
terminal `cancelled`. An unsafe placement remains `holding_for_recovery` with
`unsafe_release`; it never unlocks the robot while carrying the box.

The first version exposes `POST /api/missions/:id/resume`, valid only for an
object-transfer operation in `holding_for_recovery`. The mission service creates
and persists one control request ID per resume attempt and calls the existing
gateway `operation: "update"` with payload `{type: "resume_transfer",
missionId, destination}`. Gateway validation permits `recover_transfer` and
`resume_transfer` updates only when `payload.missionId` equals the current
`gateway-active.commandId`. They use the existing journal key derived from the
control request ID but never replace the active record. Their controller
artifacts carry both `request_id` and the original `mission_id`; feedback always
correlates to the original mission ID. Repeating the HTTP request reuses the
persisted control request and is idempotent. Resume retries navigation to the
original destination and placement; it does not repeat pickup. Progress remains
at the placement portion of the original scale. No general manual recovery
tooling is included.

Controller restart never silently resumes arm or base movement from an old
command. Telemetry contains a controller boot ID. When reconciliation sees a
changed boot ID for a nonterminal transfer, the mission service persists one
idempotent gateway `operation: "update"` request with payload
`{type: "recover_transfer", missionId, destination}`. The controller treats it
as synchronization only: it does not move and reports `holding_for_recovery`
with `operation_interrupted` for that mission ID. Because the attachment is
in-memory, recovery first checks whether the box lies inside the physical grasp
volume. If it does, an explicit resume may reattach it; otherwise resume is
rejected with `grasp_failed`. Cancellation remains allowed and becomes terminal
only after the controller confirms the box is detached and the base is stopped.

## Telemetry and dashboard state

Robot telemetry adds an optional `objectTransfer` object. It is absent before
the controller has observed any transfer command. Fields are:

- `missionId` and `objectId`: nonempty strings;
- `status`: `accepted | running | holding_for_recovery | completed | failed | cancelled`;
- `stage`: one of the nine active workflow stages, or `null` for terminal state;
- `progress`: integer 0-100 derived from the ordered state-machine stage;
- `errorCode`: one of the defined physical error codes, or `null`;
- `objectPose` and `destination`: `{x, y, z}` / `{x, y}` finite coordinates;
- `attached` and `gripperClosed`: booleans;
- `observedAt`: simulation time in seconds.

Allowed combinations are enforced by the publisher: `accepted` precedes base
ownership, active stages use `running`, unresolved attached failures use
`holding_for_recovery`, and terminal states have `stage: null`. The telemetry
service preserves the latest value and its existing freshness metadata.

Gateway feedback selects `objectTransfer` when its `missionId` matches the
requested operation; otherwise it selects existing `navigation` feedback. It
maps `accepted`, `running`, `holding_for_recovery`, and the three terminal states
without collapsing them. Terminal transfer feedback is cached in the gateway
journal exactly like route feedback. The mission reconciler persists progress,
stage, error code, and freshness. `holding_for_recovery` is nonterminal in the
mission service, gateway busy check, dashboard, and history API.

The dashboard derives its progress display from controller telemetry. It does
not advance stages on timers. Once completed, the map uses the reported object
pose so the same box can be transferred again.

## Verification

Verification is split along module boundaries:

- C++ unit tests cover state transitions, deadlines, one alignment retry,
  cancellation, recovery, coordinator ownership, grasp-volume checks, and named
  arm poses.
- C++ adapter tests cover navigation requests and simulated attachment behavior
  behind test doubles where Webots is unavailable.
- JavaScript tests cover command contracts, validation, serialization,
  idempotency, and telemetry normalization.
- React component tests cover selecting a destination, independent normal-route
  behavior, transfer progress, cancellation, and failures.
- A Webots smoke test starts from the known scene, transfers the box, verifies
  that the Supervisor adapter updates the box with the gripper during transport,
  verifies release and ordinary physics near the destination, then runs a normal
  route without triggering the manipulator. Additional cases cancel before and
  after attachment and restart the controller while holding the object; each
  asserts adapter cleanup, reported recovery state, and operation-lock behavior.

Named `transport`, `pre_grasp`, `grasp`, `lift`, `pre_place`, and `place` joint
targets, finger openings, stage deadlines, base offsets, and position tolerances
live in a single C++ calibration structure. The orchestration state machine does
not embed calibration numbers.

## First-version limits

The first version supports one known box, one robot, one active operation, and a
destination selected on the 2D map. It uses calibrated named arm poses instead
of general inverse kinematics or camera-based object recognition. Multiple
objects, arbitrary object shapes, visual detection, and free-form Cartesian arm
planning are outside this delivery.
