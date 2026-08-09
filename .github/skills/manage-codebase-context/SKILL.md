---
name: manage-codebase-context
description: Maintain and consume HySim's AI-facing codebase documentation. Use for repository onboarding, current implementation or test-status questions, architecture handoffs, dependency upgrades, documentation drift checks, and updates to AGENTS.md, CLAUDE.md, README.md, docs/README.md, or docs/development_status.md.
---

# Manage Codebase Context

Keep AI-facing documentation concise, source-backed, and safe to update while
development is in progress.

## Establish Current State

Read, in order:

1. `AGENTS.md` for architecture, invariants, commands, and project conventions.
2. `docs/development_status.md` for the last verified baseline and active work.
3. `docs/README.md` for canonical implementation and theory documentation.
4. `cmake/Dependencies.cmake` for the required MIPSolvers revision.

Then verify transient state instead of trusting prose:

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
git diff --check
```

Inspect the relevant `include/`, `src/`, `tests/`, and CMake declarations before
describing behavior. Treat them as authoritative when documentation disagrees.

## Update The Right Surface

| Change | Update |
|---|---|
| Active work, last verified build, sanitizer boundary | `docs/development_status.md` |
| Stable architecture, invariants, commands, recurring pitfalls | `AGENTS.md` |
| User-visible capabilities or dependency baseline | `README.md` |
| Documentation navigation or policy | `docs/README.md` |
| AI skill list | `AGENTS.md` and `CLAUDE.md` together |
| Detailed implementation contract | Existing topic document linked by `docs/README.md` |

Update existing living documents in place. Do not add dated audit snapshots.
Do not duplicate detailed contracts in root context files.

## Record Evidence Honestly

- Separate Release/full-dev regression results from sanitizer results.
- Name the build preset, exact test scope, failures, and skips.
- Mark a claim as pending when the relevant target was not rebuilt after a change.
- Distinguish committed baselines from dirty-worktree experiments in both repos.
- Record fallback, approximation, time-limit, and unsupported model coverage.
- Never convert a roadmap, hypothesis, or partial test into current behavior.
- Remove obsolete active-work entries once the replacement result is verified.

## Keep Context Efficient

- Keep `AGENTS.md` focused on facts needed in most tasks.
- Put volatile status only in `docs/development_status.md`.
- Link to existing contracts instead of restating them.
- Prefer exact paths, symbols, presets, and test targets over narrative history.
- Preserve unrelated worktree changes and verify dates only when content changes.

## Finish A Documentation Pass

1. Re-read every changed claim against source or test output.
2. Check links and keep the skills tables in `AGENTS.md` and `CLAUDE.md` aligned.
3. Run `git diff --check` and scan changed files for placeholders.
4. Report what is verified, what remains active, and what was not tested.
