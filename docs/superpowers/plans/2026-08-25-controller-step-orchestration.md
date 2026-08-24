# Controller Step Orchestration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract and regression-test the exact order of one controller step.

**Architecture:** A callback table isolates orchestration from Webots; lifecycle scheduling remains pure and reusable.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add a failing exact-order regression test
### Task 2: Implement callback orchestration and replace the main-loop body
### Task 3: Verify, document, and run the full project matrix

No Git steps apply because this workspace has no `.git` repository.
