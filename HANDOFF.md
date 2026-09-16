# URDF2DT handoff and maintenance

Start here when taking over development or integrating the module. This guide
describes the source candidate integrated at `07f9f33cce74bd7e242114e98e05162b8ba9ec5a`.
The independently tested source commit is `23512a99e0fba1e25ed68cdd332d8534fff80c44`;
the later commits add evidence and merge a README formatting change.
Package version is **0.1.0**; validated session schema is **1.0**.

## Read and run

- [Architecture and state transitions](docs/architecture.md)
- [Research record and measured limitations](docs/research_record.md)
- [Environment, reproduction and failure recovery](docs/reproducibility.md)
- [Candidate notes](docs/releases/stage21-rc1.md) and
  [versioned verification](outputs/releases/stage21-rc1/verification.json)
- [Worked UR5 notebook](examples/ur5_full_pipeline.ipynb)

From a checkout root on Windows with Python 3.12 available:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements/windows-py312-full.lock
.\.venv\Scripts\python.exe -m pip install --no-deps -e .
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/scara/scara_rrpr.urdf
```

Create the environment only when needed. Desktop-only dependencies are in
`requirements/windows-py312-desktop.lock`; the full lock includes test/notebook
tools. These are Windows Python 3.12 version pins, not cross-platform or wheel-hash
locks. Dependencies used by pip's isolated build backend are not fully frozen.
The app can open without a source argument and offer **Open URDF**.
The source launcher requires Python; no standalone Windows release is available yet.

## Integration surfaces

These are the supported integration points for this 0.1.0 candidate. Preserve
their contracts during maintenance; no long-term API compatibility guarantee is
implied by the pre-1.0 package version. Avoid underscore-prefixed helpers and
direct mutation of UI or session internals.

| Import / entry point | Contract |
|---|---|
| `urdf2dt.config.load_config(path=None)` | Typed defaults or explicitly named complete YAML configuration |
| `urdf2dt.parser.urdf_input.URDFInput` | Owned source bytes; `from_path` / `from_upload`; SHA-256 identity |
| `urdf2dt.parser.urdf_validator.validate_urdf` | Structured issues; `require_valid()` gates parsing |
| `urdf2dt.parser.robot_document.RobotDocument` | Rooted-tree load and zero-based serial-path selection |
| `urdf2dt.pipeline.generate_automatic_model` | Validated source, chain, immutable baseline and effective configuration |
| `urdf2dt.interfaces.URDFParser` / `DHSolver` | `parse(validated_source, config)` / `solve(chain, config)` adapters |
| `urdf2dt.app.Application` | Owns run/session; `validate`, `export`, `resume` |
| `urdf2dt.dh.editor_session.EditorSession` | Single-owner editing, immutable snapshots, one pending proposal |
| `urdf2dt.dh.recompute.FrameEdit` | Axial translation in metres and rotation in radians, subject to classification |
| `urdf2dt.dh.classification.classify_dh_model` / `get_editable_params` | Current geometry, review locks and permitted continuous freedoms |
| `urdf2dt.dh.global_validation.validate_global_fk` | Deterministic sampled comparison, `passed`, `to_dict`, `markdown` |
| `urdf2dt.export.persistence.save_session` / `load_session` | Revalidated JSON bundle export and checked reconstruction |
| `urdf2dt.kinematics.urdf_fk` / `dh_fk` | Base-relative end-effector transform for ordered q |
| `python -m urdf2dt.ui.desktop` | Native interface; optional source path |

DH frame indices are one-based; document chain indices are zero-based. q follows
`chain.joint_names`. Revolute/continuous q is radians, prismatic q is metres.
Transforms are 4x4 homogeneous matrices acting on column vectors. The complete
Standard-DH model includes base and tool transforms and joint signs; the row table
alone is insufficient. Fixed joints are absorbed into geometry/alignment.

Parser/solver adapters must preserve source identity, joint order/types and chain
endpoints; the pipeline rejects mismatches. Keep UI imports outside core paths.
Raw changed DH rows require an injected validator; ordinary integrations should
use `FrameEdit`, which runs built-in compensation and local checks.

```python
from pathlib import Path
from urdf2dt.app import Application
from urdf2dt.dh.recompute import FrameEdit

app = Application("robots/scara/scara_rrpr.urdf")
for index in range(1, len(app.run.automatic_model.rows) + 1):
    app.session.unlock_next()
    app.session.propose_edit(index, FrameEdit())  # unchanged confirmation
    decision = app.session.accept()
    assert decision.valid, decision.reason
assert app.validate().passed
archive = app.export(Path("outputs/sessions/handoff-example"))  # must not exist
restored = Application.resume(archive)
assert restored.session.state == app.session.state
```

For a real nonzero edit and rejection example, use the worked notebook. A locally
accepted frame does not certify global FK. Upstream edits invalidate successor
acceptance; resolve every preview and accept all frames before validation/export.

## Downstream output contract

Use `load_session` / `Application.resume` for complete trusted-by-validation
reconstruction. JSON Schema checks the envelope; typed decoders and fresh FK
checks enforce additional semantic constraints. Schema validation alone is not
equivalent to successful session reload.

| Bundle file | Content and consumer guidance |
|---|---|
| `session.json` | Authoritative source snapshot, configuration, editor state, events, validation and provenance |
| `dh_model.json` | `schema_version`, `dh_convention`, `model`, `reproducibility`; useful downstream model envelope |
| `edit_history.json` | Records, events and provenance; audit evidence rather than executable instructions |
| `validation_report.json` | `sampled-global-fk-v1` report, candidate/hash, source digest, q samples, errors and diagnostics |
| `validation_report.md` | Human-readable validation summary |
| `configuration.json` | Effective configuration snapshot |

Session top-level fields are exactly `schema_version`, `dh_convention`, `source`,
`configuration`, `state`, `events`, `geometric_frames`, `validation`, and
`reproducibility`. Source contains a name, optional path and base64 XML bytes.
State contains automatic/working models, frame statuses and edit history.
Each model contains rows, source/provenance, base/tool transforms and metadata.
Each row contains `a`, `alpha`, `d`, `theta_offset`, `joint_name`, `joint_type`,
and `joint_sign`. JSON tuples become arrays; enum values become strings.

The authoritative schema/decoders are in [export/schemas.py](urdf2dt/export/schemas.py).
The earlier `schemas/dh-model-0.1.schema.json` is a provisional model envelope,
not the validated-session schema. Do not use it to certify an exported session.
Exports require a new directory and never overwrite an existing bundle. Reload
may fail on numerical differences across dependency versions; preserve the original
bundle and its environment. Hashes are provenance checks, not digital signatures.
Visual meshes and generated collision parts are not embedded in the session.

For schema changes, update the version and loader policy explicitly, retain an
old checked-in archive regression, and document compatibility/migration behavior.
Do not silently reinterpret old fields or convert Standard DH to Modified DH.

## Add a robot fixture

1. Add source under `robots/<name>/` with a README recording origin, licence,
   exact revision/hash, units, endpoints, joint order and any edited assumptions.
   Keep external asset attribution; do not assume public download grants reuse.
2. Provide a connected serial path with supported joints. Use desktop selection
   for a branching tree and record which path was selected.
3. Run structural validation and automatic generation. Record warnings and
   confirm movable joint identities and fixed base/tool alignment.
4. Test zero, limits and representative poses plus deterministic random samples.
   Prefer an independent analytic/manufacturer reference over two shared code paths.
5. Exercise actual permitted edits, upstream invalidation, reject/restore,
   complete acceptance, export and reload. Add meaningful regression fixtures.
6. Run the native view if claiming UI support, checking joint units/limits and
   missing mesh notices. Keep kinematic and visual claims distinct.

Use `robots/scara/README.md`, `urdf2dt/research/scara_oracle.py`, and
`scripts/verify_stage15.py` as a worked pattern. Fixture-specific constants belong
in reference tests, never as fallbacks in the general solver. New local robot files
are not verified merely because they are present or can be opened.

## Change tolerances safely

Start with a copied complete YAML file and load it explicitly. Record why a value
changes, its units, source and confirmation status. Built-in defaults and
`config/default.yaml` must stay synchronized if defaults change. Do not alter
representation guards in `_numeric.py` just to turn a research failure into a pass.

Run boundary classification tests, legal/forbidden edits, negative controls,
sample-density experiments and both robots. Recheck review locks, not just category
names. Re-export into new directories with the changed configuration; retain old
reports for comparison. Raising sample counts increases coverage, not strictness
of geometric tolerances. Set `reference_status=confirmed` only with an actual
recorded reference/advisor decision.

## Maintenance and release procedure

For each change, record the concrete defect/behavior, run relevant checks and
update the current documentation. Core changes require the full pytest suite and
mypy; renderer changes additionally need native smoke evidence. Preserve failure
cases and immutable-state regressions. The CI workflow is headless and does not
exercise a native graphics stack.

Before a candidate, run `scripts/verify_release_candidate.py` as described in
[the candidate notes](docs/releases/stage21-rc1.md). It requires committed tracked
changes and a new output directory. Record the exact tested commit and dependency
versions. For dependency updates, resolve in a separate environment, regenerate
version pins, then recheck notebook, exports and native rendering. Do not update
locks from an unrelated daily-use environment.

When diagnosing a report, request the commit, Python/OS/dependencies, source hash
and permissible source/asset files, configuration, reproduction steps, and error
log. Never request credentials. Reproduce before changing tolerances; retain failed
reports as evidence. Roll back through a reviewed revert or previous known-good
checkout; keep user archives and uncommitted work intact.

The project owner selected MIT for URDF2DT in Stage 23; see LICENSE. Upstream
fixture and runtime licences remain separate. Complete bundled runtime/asset
attributions before final distribution. Maintainer succession, review ownership and advisor sign-off should be
recorded in [the acceptance record](docs/handoff_acceptance.md).

## Limits and deferred work

Only serial kinematics are solved: no closed loops, floating/planar joints,
mimic coupling or simultaneous whole-tree editing. Xacro expansion is external.
Sampled FK is not continuous-space proof, physical calibration, dynamics validation
or collision-free motion planning. R1–R3 report mapping, MATLAB comparison and
advisor confirmation of tolerances remain pending.

Stage 23 must package `URDF2DT.exe`, resolve packaged assets/configuration, audit
licences, and test on clean Windows without Python before publishing a ZIP and
checksums. Browser UI, richer CAD appearance and broader topology support remain
deferred. Do not label the current source candidate as a verified standalone app.

## Access and acknowledgement

The repository, candidate source and committed verification material are public:
[repository](https://github.com/Abdelrahman-288/URDF2DT),
[candidate notes](https://github.com/Abdelrahman-288/URDF2DT/blob/main/docs/releases/stage21-rc1.md),
[evidence](https://github.com/Abdelrahman-288/URDF2DT/tree/main/outputs/releases/stage21-rc1).
There is no published GitHub Release as of the Stage 22 check.
Public access was verified without authentication; this does not establish that
an advisor/team member has received, reviewed or accepted the work.
See [the dated acknowledgement record](docs/handoff_acceptance.md).
