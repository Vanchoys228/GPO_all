# Survey Grid Geometry Design

Extend `controller_survey_geometry` with pure grid indexing, index-to-world
conversion, four-neighbor component flood fill, eight-neighbor boundary
detection, and Ramer-Douglas-Peucker keep marking. The functions operate only on
caller-owned `SurveyGrid`/`SurveyPoint` data and explicit epsilon values. Grid
population from zones/maps and boundary ordering remain in `youbot_web.c`.
