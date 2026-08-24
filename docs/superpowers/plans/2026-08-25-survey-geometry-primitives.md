# Survey Geometry Primitives Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract the first pure Mapping Survey geometry layer without behavioral changes.

**Architecture:** Stateless functions operate on caller-owned `SurveyPoint` and `SurveyInterval` buffers with explicit capacities and thresholds.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add failing tests
- [ ] Cover bounds expansion, route spacing/capacity, segment subdivision, sorting, and interval subtraction.
- [ ] Confirm the focused build fails because the module is absent.

### Task 2: Implement and integrate
- [ ] Create `controller_survey_geometry.c/.h`.
- [ ] Make the focused strict test pass.
- [ ] Replace the equivalent helpers in `youbot_web.c` and update the Windows build.

### Task 3: Verify and document
- [ ] Update docs and line counts.
- [ ] Run 15 C tests, strict controller build, 54 JS tests, lint, frontend build, and bridge smoke test.

No Git steps apply because this workspace has no `.git` repository.
