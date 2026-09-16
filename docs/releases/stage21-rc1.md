# Stage 21 release candidate: rc1 (draft release notes)

This source candidate integrates the worked notebook and consolidated research
record with the native editor. The package version remains **0.1.0** and validated
session schema **1.0**, using Standard DH. `stage21-rc1` identifies the verification
record; it is not a published v1.0 tag or standalone binary release.

## Included behavior

- Serial URDF validation, automatic Standard-DH generation, and immutable baseline.
- Native robot joint sliders, constrained compensated DH edits, and light/dark UI.
- Sequential acceptance, sampled global FK, JSON-only export and checked reload.
- Full worked UR5 notebook and independent analytic SCARA verification.
- Documented geometric freedoms, research ablations, limitations and reproduction.

## Candidate verification procedure

The exact tested commit and outcomes will be retained in
`outputs/releases/stage21-rc1/verification.json`, alongside JUnit, command logs,
executed notebook, and UR5/SCARA validation archives. A later evidence-only commit
may contain those outputs; the tested source commit stays explicitly recorded.

The Windows Python 3.12 version pins are in `requirements/windows-py312-full.lock`
and `requirements/windows-py312-desktop.lock`. The full file covers verification
and research tools; the desktop file covers runtime dependencies only. They
exclude the local package, which is installed from the chosen checkout. Pins are
platform-specific resolved versions, not wheel hashes or a cross-platform lock.
PyInstaller and its build configuration belong to the later packaging stage.

To reproduce from a clean checkout with Python 3.12:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements/windows-py312-full.lock
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
.\.venv\Scripts\python.exe scripts/verify_release_candidate.py --output outputs/tmp/rc-rerun --desktop
```

Choose a new output directory. The runner refuses tracked modifications, requires
zero skipped/xfail tests, runs mypy and the full notebook in a fresh kernel, checks
SCARA with its analytic oracle, and confirms the exported schema/version. Native
checks require a working Windows display/VTK graphics stack. The original fresh
candidate environment is installed through the README's extras workflow before
recording these pins.

## Remaining release gates

This is readiness for packaging work, not proof of standalone distribution.
Clean-machine testing without Python, executable packaging, licence/runtime
inventory and download verification remain required. MATLAB comparison,
report-specific R1–R3 reconciliation and advisor acceptance of provisional
tolerances are also still open. Sampled FK is not continuous-space proof or a
measurement of physical robot accuracy. See the research record for scope.

No final v1.0 release tag is created in Stage 21. Stage 22 covers handoff; the
requested later Windows distribution stage must validate the packaged artifact.
