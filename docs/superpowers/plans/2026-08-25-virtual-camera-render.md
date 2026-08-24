# Virtual Camera Render Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract deterministic virtual-camera raster drawing without changing its image.

**Architecture:** A pure renderer mutates a caller-owned RGB buffer using explicit dimensions. Scene selection and I/O stay in `youbot_web.c`.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add failing raster tests
- [ ] Test pixel bounds/channel clamping, clipped rectangles, lines, background samples, and obstacle box colors.
- [ ] Confirm compilation fails because the renderer is missing.

### Task 2: Implement and integrate
- [ ] Create `controller_camera_render.c/.h` by preserving current formulas.
- [ ] Pass focused strict tests.
- [ ] Replace local raster helpers with renderer calls and add it to the Windows build.

### Task 3: Verify and document
- [ ] Update documentation and line count.
- [ ] Run 14 strict C tests, full strict controller build, 54 JS tests, lint, frontend build, and bridge smoke test.

No Git steps apply because this workspace has no `.git` repository.
