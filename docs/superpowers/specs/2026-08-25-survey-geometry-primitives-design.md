# Survey Geometry Primitives Design

Extract Webots-independent Mapping Survey primitives into
`controller_survey_geometry.c/.h`: bounds expansion, capacity-aware route point
insertion with the existing minimum-spacing replacement, segment subdivision,
numeric sorting, and interval subtraction. All thresholds and capacities are
explicit inputs. Grid construction, zones, obstacle queries, route-file I/O, and
survey state remain in `youbot_web.c`. Existing comparison and interpolation
behavior must remain unchanged.
