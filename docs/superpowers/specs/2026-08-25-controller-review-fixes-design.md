# Controller Review Fixes Design

## Goal

Correct the two defects found during the read-only review without changing the
controller's navigation algorithms or external file contracts.

## Mapping Survey path reconstruction

`controller_survey_find_grid_path` must only report success when the reconstructed
parent chain reaches the requested start cell and the returned path ends at the
requested target. The reverse buffer will use the existing maximum grid-cell
capacity instead of the unrelated 1024-cell limit. If a complete path cannot fit
in the caller's output buffer, the function will return failure instead of
returning a truncated transition that the caller may treat as safe.

## Camera map obstacle priority

Obstacle insertion will receive the free-cell collection as part of the same
operation. When an obstacle is observed, all matching free-cell entries will be
removed before the obstacle is inserted or reinforced. This makes obstacle
priority independent of observation order.

## Verification

Add regression cases for a path longer than 1024 grid cells, insufficient path
output capacity, and a camera cell observed first as free and then as occupied.
Run focused tests first, followed by all standalone C tests, strict controller
compilation, JavaScript tests, lint, production build, and bridge smoke tests.
