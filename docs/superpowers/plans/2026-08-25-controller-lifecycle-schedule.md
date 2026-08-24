# Controller Lifecycle Schedule Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make periodic controller orchestration explicit without changing timing.

**Architecture:** A pure scheduler returns boolean task flags; Webots and I/O remain in the main adapter.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add failing schedule tests for step 0, common multiples, and disabled intervals
### Task 2: Implement scheduler and use its mask in the main loop
### Task 3: Update docs and execute the full verification matrix

No Git steps apply because this workspace has no `.git` repository.
