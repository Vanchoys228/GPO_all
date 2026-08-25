# Dashboard Dependency Map

`src/pages/Dashboard.jsx` is the planner composition page. Its responsibilities
are grouped below so they can be extracted without changing user-visible
behavior.

| Area | State and refs | Main callbacks/effects | Target boundary |
| --- | --- | --- | --- |
| Connections | telemetry hook, route socket ref, solver health | bridge sync, route timing | existing planner hooks |
| Import | points, zones, active selections, algorithm/task | `usePlannerGraphImport` | planner import hook and service |
| Editor | points, active point kind, drag ref, active zones | canvas mouse handlers, zone/surface CRUD | editor actions hook |
| Route planning | route seed, optimized route, algorithm parameters, energy state | optimization and rebuild hooks | planner route hooks and models |
| Route commands | route socket, motion profile, survey mode | route sender and runtime command hooks | planner command hooks |
| Export | telemetry maps, export dialog | `usePlannerMapExport` | map export hook/service |
| Layout | sidebar collapse state | `setSidebarCollapsed`, JSX composition | remains in `Dashboard.jsx` |

## Dependency direction

`Dashboard.jsx` may compose hooks and UI components. Hooks may depend on planner
model/services and shared libraries. Model and service modules must not import
the page or dashboard components.

## First safe slice

Move JSON/Excel/CSV file decoding and tabular row normalization into
`src/features/planner/services/plannerFileImport.js`. Applying the normalized
graph to React state remains in the page until editor state is moved as one
coherent unit.
