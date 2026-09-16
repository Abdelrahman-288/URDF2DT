# Stage 21: Final integration and release candidate

The source candidate `23512a99e0fba1e25ed68cdd332d8534fff80c44` was installed
in a newly created Python 3.12.14 environment from an isolated Git worktree.
No system site packages were included. Existing user edits and robot folders in
the original checkout were not part of the candidate.

- Full tests: 314 passed; no skipped tests or xfails; unexpected xpasses fail the run.
- Mypy: no issues in 38 package files; pip check passed.
- UR5 notebook: 14 code cells passed in a fresh kernel, including scene rendering.
- SCARA: 200-pose analytic verification, accepted edits, native control/actor checks,
  light/dark screenshots, and exact state reload passed.
- Both archives use Standard DH and validated session schema 1.0; package version
  remains 0.1.0. Candidate commit provenance is checked by the runner.
- Full and desktop Windows Python 3.12 dependency pins and environment versions
  are retained; an offline-index dry run confirmed the full installed pin set.
- Versioned reports, archives, logs, JUnit and executed notebook are retained in
  [stage21-rc1](../../outputs/releases/stage21-rc1/verification.json).

[Draft release notes](../releases/stage21-rc1.md) specify the exact candidate,
commands, results and remaining gates. The candidate source and later evidence-only
commit are distinguished explicitly. Hosted checks run on the candidate and the
integration commit; graphics verification remains local.

This fulfills the requested release-candidate stage, not the older plan's final
v1.0 tagging step. No final release tag or standalone executable is published.
The user's revised plan remains preserved as a local change. Stage 22 handoff and
Stage 23 standalone Windows packaging still require their respective work.
