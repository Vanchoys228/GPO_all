# Survey Coverage Ordering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract shared horizontal/vertical coverage ordering rules.

**Architecture:** Pure functions transform caller-owned interval buffers and select endpoints; environment queries remain adapters.

**Tech Stack:** C11 and strict standalone GCC tests.

---

### Task 1: Add failing clipping, reversal, and endpoint-selection tests
### Task 2: Implement the shared rules and replace duplicated controller code
### Task 3: Update documentation and run the full verification matrix

No Git steps apply because this workspace has no `.git` repository.
