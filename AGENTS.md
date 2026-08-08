# MIPSolvers — Agent Working Rules

These rules are **mandatory for every code change** in this repository. They
are not suggestions and do not depend on task phrasing. The full workflow
(rationale template, citation format, mismatch protocol) is in the
`theory-guided-coding` skill (`~/.agents/skills/theory-guided-coding/SKILL.md`);
follow it for all algorithmic/numerical work.

## Theory-guided development (non-negotiable)

1. **Theory leads, code follows.** Every implementation or modification must
   be guided by an explicit model or algorithm (paper, textbook, or a
   derivation document in `docs/`). No code-level guessing.
2. **References at the site.** Nontrivial formulas, update rules, tolerances,
   and constants in code carry a citation (paper + section/equation, or the
   internal derivation doc).
3. **No TODOs or placeholders.** `TODO`/`FIXME`/stubs/dummy values are treated
   as deception. Deliver complete implementations; unhandled paths must fail
   loudly, never return plausible numbers. Check your diff before finishing:
   `git diff | grep -inE '^\+.*(TODO|FIXME|XXX|HACK|placeholder|not.?implemented|stub)'`
   must return nothing.
4. **Rationale before implementation.** Before changing code, state the
   theoretical reason: the model/algorithm, the cost model, a *quantitative*
   predicted effect, assumptions, references, and the validation protocol
   (fixed before seeing results).
5. **Mismatch triggers re-derivation.** If measured performance is far from
   the prediction (wrong sign, or deviation > ~50% of the predicted effect),
   stop and re-investigate in order: implementation infidelity → machine/
   cost-model error → assumption violation → theory error. Write the finding
   back into the derivation document before touching code again.

## Repository specifics

- Performance work on the native dual simplex is additionally bound by the
  acceptance contracts in `docs/` (bit-identical pivot paths for
  pure-performance changes; full-suite accuracy gates for algorithmic ones).
- Report benchmark results as measured-vs-predicted, with the command,
  build flags, and commit hash.
