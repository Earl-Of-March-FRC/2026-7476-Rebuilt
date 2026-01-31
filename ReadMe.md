# FRC Team 7476 – Robot Code Workflow Guide

This is FRC Team 7476's robot code repository for the 2026 game. This guide outlines the workflow we use to ensure our robot code is always stable and competition-ready.

## Core Philosophy

**The robot must always be deployable.**

To maintain this standard:
- No one works directly on competition code
- All changes are reviewed before merging
- Code must compile before it can be merged

## Branch Structure

Our repository uses four types of branches:

### `main` – Competition Branch

**Purpose:** Represents code that can be deployed to the robot immediately.

**Rules:**
- No direct commits
- No experimental code
- Only merged from `develop`
- Must always compile and pass CI
- Merged only by programming leads

### `develop` – Development Integration Branch

**Purpose:** Where completed features are combined and tested.

**Rules:**
- No direct commits (recommended)
- All changes come from pull requests
- Must compile at all times
- Used for testing and integration

### `feature/*` – Individual Work Branches

**Purpose:** Where individual members do their work on specific features or tasks.

**Examples:**
- `feature/swerve-odometry`
- `feature/swerve-heading-lock`
- `feature/auto-path-following`

**Rules:**
- Always created from `develop`
- Never merged directly into `main`
- Deleted after merge

### `fix/*` – Bug Fix Branches

**Purpose:** Small, targeted fixes for known issues.

**Examples:**
- `fix/swerve-gyro-drift`
- `fix/swerve-module-offset`

**Rules:**
- Created from `develop` (or `main` during competition)
- Merged back into `develop`
- Promoted to `main` if needed

## Development Workflow

All changes follow this path:

```
feature/* → develop → main
```

### Step-by-Step Guide

**Step 1: Update your local develop branch**
```bash
cd path/to/robot/repo
git checkout develop
git pull
```

**Step 2: Create a feature branch**
```bash
git checkout -b feature/short-description
```

Example:
```bash
git checkout -b feature/swerve-heading-lock
```

**Step 3: Make changes and commit**

Write code and commit small, clear changes:
```bash
git add .
git commit -m "Add heading lock control to swerve drive"
```

Guidelines:
- Commit messages should describe what changed
- Avoid large, unrelated commits

**Step 4: Push your branch**
```bash
git push origin feature/short-description
```

Example:
```bash
git push origin feature/swerve-heading-lock
```

**Step 5: Open a pull request**

- **Base branch:** `develop`
- **Compare branch:** your `feature/*` branch

Each pull request must explain:
- What changed
- What subsystem is affected
- How the change was tested

Example format:
```
Summary:
Adds heading lock to maintain robot orientation during translation

Subsystems:
- Swerve Drive

Testing:
- Tested on robot
- Verified rotation holds under driver input
```


## Merging Rules

**Merging into develop:**
- Requires a pull request
- Requires CI to pass
- Requires code review

**Merging into main:**
- Only done by programming leads
- Only from `develop`
- Only when the robot is stable

## Competition Mode

During competitions:

- `main` is frozen
- No experimental features
- Only critical bug fixes allowed

**Emergency fix flow:**
```
fix/issue → develop → main
```

Fixes must be small, tested, and reviewed.

---

All team members are expected to follow this workflow. Questions should be directed to a programming lead.

Testing