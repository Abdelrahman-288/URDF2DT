# Stage 17: GitHub continuous integration

The `Tests` workflow runs on pushes and pull requests containing the workflow,
and supports manual dispatch once available on the default branch. Its matrix is:

| Runner | Python |
|---|---|
| ubuntu-latest | 3.10 (minimum supported Python) |
| ubuntu-latest | 3.12 |
| windows-latest | 3.12 (desktop development baseline) |

Each job installs the package with `dev,research` extras plus `ipywidgets`, runs
`pip check`, records dependency versions, runs the complete pytest suite, and
type-checks all production package files with mypy. Widget callback tests are
included instead of being silently skipped because the optional dependency is
missing. The tested suite uses no live renderer; Xvfb, Qt and a display server are
not needed for these tests. Native graphics verification remains a separate local
check, not a claim of this headless workflow.

Jobs time out after fifteen minutes. A failure does not cancel the other matrix
jobs. New commits cancel obsolete runs of the same branch. Checkout uses read-only
repository permissions and does not persist credentials. Actions are pinned to
verified commit SHAs; pip dependencies remain version ranges, with their resolved
versions retained alongside JUnit results as fourteen-day artifacts. This detects
compatibility drift but is not a fully locked reproducible environment.

The workflow follows [GitHub's Python testing guidance](https://docs.github.com/en/actions/tutorials/build-and-test-code/python).

## Failure detection check

A temporary test deliberately failed on the Stage 17 branch. The [failure run](https://github.com/Abdelrahman-288/URDF2DT/actions/runs/34907969632)
used commit `53fe593`. Both Ubuntu jobs reported exactly `1 failed, 300 passed`,
and the sole failure was `test_intentional_ci_failure`. The test was then removed.
This verifies the actual hosted test gate, rather than assuming workflow syntax
alone is enough. The temporary test is not part of the final suite.

## Branch and integration scope

This workflow is introduced on `codex/stage17-github-ci`, which includes the earlier
stages. The README badge is scoped to this branch until the project changes are
merged into the default branch. Existing older branches do not gain this file
automatically. No default-branch merge or branch-protection policy is changed here.
After merging, the workflow will apply to default-branch pushes and pull requests;
maintainers can choose to require its matrix checks in repository protection rules.

## Reproduce locally

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev,research]" ipywidgets
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pytest -q -ra
.\.venv\Scripts\python.exe -m mypy
```

Stage 18 is the next planned development stage; CI provides continuing regression
coverage rather than replacing the remaining research and release work.
