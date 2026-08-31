# Input Orchestration Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move configuration and input reload decisions out of `youbot_web.c` while preserving their timing and status updates.

**Architecture:** Create `controller_input_orchestration` as the owner of reload cadence and the delegation order for motion profiles, runtime commands, limit zones, surface zones, and routes. It receives narrow operations for existing reload services; the Webots root supplies adapters to those services and remains responsible only for the static step-callback binding.

**Tech Stack:** C11 and the existing Webots controller unit-test runner.

---

## Chunk 1: Input orchestration

### Task 1: Lock down reload cadence

**Files:**
- Create: `webots/controllers/youbot_web/controller_input_orchestration.h`
- Create: `webots/controllers/youbot_web/controller_input_orchestration.c`
- Create: `webots/controllers/youbot_web/controller_input_orchestration_test.c`

- [ ] **Step 1: Write a failing test** — inject traceable reload operations; prove motion emits `motion_profile_reloaded` only when changed, runtime delegates at its interval, and each route/zone callback respects its own due step.
- [ ] **Step 2: Run the focused test and observe the missing-module failure.**
- [ ] **Step 3: Implement the minimal context and five reload entry points.**
- [ ] **Step 4: Run the focused test and observe it pass.**

### Task 2: Bind the root to input orchestration

**Files:**
- Modify: `webots/controllers/youbot_web/youbot_web.c`
- Modify: `webots/controllers/youbot_web/controller_sources.txt`

- [ ] **Step 1: Replace reload decision bodies with adapters delegating to the new context.**
- [ ] **Step 2: Add the new source file to the production/test source list.**
- [ ] **Step 3: Run the complete controller tests, production controller build, and `npm run lint`.**
- [ ] **Step 4: Commit the slice with a Conventional Commit message.**
