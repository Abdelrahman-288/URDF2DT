# Contributing

URDF2DT includes the serial parser/solver, constrained editor, native and notebook
interfaces, sampled FK validation, persistence and hosted CI. Start with
[HANDOFF.md](HANDOFF.md) for integration contracts, fixture development, dependency
maintenance and release procedures. Use the README for setup; stage notes retain
historical evidence and unresolved inputs.

## Changes

Keep changes focused on the current development stage. Include a description of
the problem, resulting behavior, relevant validation, and any remaining limits.
Update the stage record alongside code. Do not describe planned capabilities as
implemented or reference comparisons as passing without measured evidence.

## Architecture

- Keep geometry, editor state, and FK validation independent of notebook widgets.
- Preserve automatic DH output as an immutable baseline.
- Put robot-specific reference data in fixtures, with its source and license.
- Centralize numerical thresholds and record the values used in validation.
- Test meaningful mathematical and state-transition invariants, including failure
  behavior and preservation of accepted state after rejected edits.
- Validate URDF input with an XML parser that rejects external entities.

## Research evidence

Record reference provenance, joint/frame conventions, configuration, sampling
seed, and validation results. Resolve reference disagreements before dependent
work. Do not copy third-party implementations without checking their licenses.

## Current limitations

The source release candidate is verified; standalone Windows packaging, reference
reconciliation and advisor acceptance remain open. Global FK is sampled evidence,
not a formal proof. The project owner selected MIT licensing in Stage 23; see
LICENSE. Preserve third-party fixture/runtime attribution and complete packaged
dependency and clean-PC checks before final distribution.

## Git identity and contribution workflow

Use an email verified on your own GitHub account. Check the effective identity
(including environment overrides) before committing:

```console
git var GIT_AUTHOR_IDENT
python scripts/contribution_audit.py --expected-email YOUR_VERIFIED_EMAIL
```

The audit is read-only and accepts several repository paths. It reports dirty
files, effective author identity, local-only commits, upstream divergence, and
branches with work outside the remote default branch. Run `git fetch origin`
first for current remote information; the audit itself never accesses credentials
or changes refs. A missing upstream is reported as unknown, not zero unpushed work.
Email counts are potential attribution clues, not proof of account ownership.

Commit only relevant files with explicit paths. Preserve feature branches, push
completed changes, open a pull request against `main`, and wait for all three CI
jobs and owner approval before merging. Work on a feature branch does not yet
qualify for default-branch commit credit. Use a merge that preserves original
commits when retaining their individual history is important. Never rewrite
published authors or dates to change the contribution graph.

AI-assisted changes use the contributing human's configured identity when that
human is responsible for the work; do not copy another author's identity or
change historical authorship. Repository administration and authenticated push
identity do not determine commit attribution.

[GitHub contribution requirements](https://docs.github.com/en/account-and-profile/how-tos/contribution-settings/troubleshooting-missing-contributions)
require an associated author email and eligible repository/branch. Graph updates
can take up to 24 hours. Another account's GitHub-provided noreply address cannot
be treated as an email belonging to your current account.
