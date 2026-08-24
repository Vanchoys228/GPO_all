# Mapping Survey State Design

Replace nine independent Mapping Survey globals with one
`ControllerMappingSurveyState`. A pure state module owns initialization, route
reset, survey-generation preparation, cooldown ticking, obstacle-scan start, and
scan completion. Navigation side effects, statuses, route mutation, map merging,
and Webots calls remain in `youbot_web.c`. Existing default values and transition
conditions remain unchanged.
