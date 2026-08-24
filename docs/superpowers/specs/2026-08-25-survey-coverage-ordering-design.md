# Survey Coverage Ordering Design

Extend `controller_survey_geometry` with the orientation-independent coverage
rules shared by horizontal and vertical sweeps: clipping raw intervals to bounds,
minimum-strip filtering, reversing interval traversal, and selecting the nearest
snake endpoint. Zone/map interval generation and safe segment application remain
in `youbot_web.c`. Comparisons, tie-breaking, and interval order remain unchanged.
