# Survey Grid Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract deterministic Mapping Survey grid geometry without behavior changes.

**Architecture:** Extend the existing pure survey module; retain environment-specific grid population and route orchestration in the controller.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add failing tests
- [ ] Test clamped grid indexing, world-point conversion, flood fill, boundary detection, and RDP marking.
- [ ] Confirm failure because the new API is absent.

### Task 2: Implement and integrate
- [ ] Add the pure functions with the existing neighbor order and comparisons.
- [ ] Pass the focused strict test.
- [ ] Remove equivalent local helpers and adapt existing calls.

### Task 3: Verify and document
- [ ] Update docs and line count.
- [ ] Run 15 C tests, strict full controller build, 54 JS tests, lint, frontend build, and bridge smoke test.

No Git steps apply because this workspace has no `.git` repository.
