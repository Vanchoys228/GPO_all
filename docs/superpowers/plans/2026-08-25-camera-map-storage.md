# Camera Map Storage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract camera-map cell storage rules without changing runtime behavior.

**Architecture:** A Webots-independent module mutates caller-owned buffers through explicit inputs and reports whether a change occurred. The main controller owns buffers and persistence.

**Tech Stack:** C11 and standalone strict GCC tests.

---

### Task 1: Add failing storage tests

- [ ] Test obstacle rounding, insertion, confidence merging/clamping, and capacity.
- [ ] Test free insertion/merging and obstacle precedence.
- [ ] Compile and confirm failure because the module is absent.

### Task 2: Implement and integrate

- [ ] Create `controller_camera_map.c/.h` with explicit buffer APIs.
- [ ] Make focused tests pass with `-Wall -Wextra -Werror`.
- [ ] Replace the two append bodies in `youbot_web.c` and retain dirty handling.
- [ ] Add the module to the Windows build script.

### Task 3: Verify and document

- [ ] Update module documentation and line counts.
- [ ] Run 13 C tests, strict full controller build, 54 JS tests, lint, frontend build,
  and bridge smoke test.

No Git steps apply because this workspace has no `.git` repository.
