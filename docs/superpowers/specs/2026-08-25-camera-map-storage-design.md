# Camera Map Storage Design

Extract camera-map cell normalization and merging into `controller_camera_map.c/.h`.
The module receives caller-owned `MapCell` buffers, counts, capacities, cell size,
and epsilon. It returns whether storage changed. Obstacle cells merge confidence
up to 255; free cells never override obstacle cells and otherwise merge the same
way. The controller retains global allocation, dirty state, clearing, and file I/O.
Existing rounding, minimum boost, strict half-cell comparison, and capacity behavior
must remain unchanged.
