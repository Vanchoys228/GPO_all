# Survey Path and Boundary Design

Extend `controller_survey_geometry` with the existing eight-neighbor BFS grid
path reconstruction and boundary-contour assembly. Path reconstruction preserves
the parent traversal and every-third-node thinning. Boundary assembly preserves
nearest-neighbor ordering, the `cell * 3.2` join limit, RDP marking, segment
subdivision, and closure. All buffers, capacities, and thresholds are explicit.
