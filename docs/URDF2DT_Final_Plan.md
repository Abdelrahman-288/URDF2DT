# URDF2DT — Interactive DH-Frame Editor
## Revised Full Project Specification and End-to-End Development Plan

**Module scope:** guided, constraint-aware Standard-DH frame assignment editor with 3-D visualization, validation, persistence, and a clear URDF-driven user workflow inside the larger URDF2DT toolbox.

**Primary reference platform:** Universal Robots UR5 (6-DOF serial manipulator).

**Primary user input:** a robot `.urdf` file.

**Primary user output:** a validated Standard-DH model, edit history, and FK-validation report suitable for downstream URDF2DT modules.

---

# 0. Briefing for an AI Assistant or Developer Working on This Project

**This is the final merged edition of the project plan.** It combines the revised
specification with the following additional improvements identified during review:
XML/XXE input security (§12.4), per-frame error attribution for failed global
validation (§20.8), reproducibility via recorded git commit hash (§10.1), a human
usability check added to the generalization stage (§23.5, Stage 15), and explicit
resolution steps for the two previously open decisions — Module 1/2 interface status
and threshold reconciliation — folded into Stage 3 (§2.2). Treat this document as the
single source of truth going forward.

Read this section first.

This is one module of a larger academic robotics research project. The goal is not merely to display or manually manipulate robot frames. The research contribution is a **constraint-aware DH-frame editor** that allows a researcher to modify automatically generated Standard-DH frame assignments **only within geometrically legal bounds**, while continuously preserving or proving kinematic equivalence to the source URDF.

The implementation must therefore preserve these principles:

1. The robot begins from a `.urdf` file supplied by the user or a bundled test fixture.
2. The URDF must be validated before parsing or DH generation.
3. The system consumes or integrates with the existing URDF parser and automatic DH solver if they already exist.
4. The automatic DH solution is immutable during an editing session.
5. The editor never exposes unrestricted 6-DOF frame dragging.
6. Every editable parameter is derived from the geometric relationship between consecutive joint axes.
7. Every proposed edit is locally validated against the DH geometric rules.
8. Editing an already accepted upstream frame invalidates every dependent downstream frame.
9. A global validator must prove that the edited DH model remains forward-kinematically equivalent to the URDF model.
10. The accepted model, edit history, configuration, and validation results must be reproducible and exportable.
11. The core editor/session/validation logic must remain independent from the UI so it can be unit-tested and run headlessly.
12. No robot-specific constants may appear in production editor logic.

When executing this plan, work through the Master Development Plan in order. Do not skip a stage merely because a later stage looks more interesting. Each stage has a **Done when** gate.

When code is requested, write working code and tests for the current stage rather than only describing what should exist.

---

# 1. Project Definition

## 1.1 Problem Statement

URDF is widely used to describe robot links, joints, origins, orientations, limits, and kinematic structure. Standard Denavit-Hartenberg (DH) parameters provide a compact analytical representation of serial robot kinematics.

Automatic URDF-to-DH conversion can produce a mathematically correct DH representation, but the automatically selected frames may be inconvenient for a researcher to inspect, explain, compare with literature, or use in later modeling steps.

A naive manual editor that allows arbitrary translation and rotation of DH frames is unsafe because the researcher can easily violate the geometric rules of the DH convention and silently create a model that no longer matches the original URDF.

URDF2DT addresses this problem by providing a guided editor that:

- automatically obtains an initial Standard-DH solution from the URDF,
- classifies the geometric relationship between consecutive joint axes,
- exposes only the legal edit degrees of freedom for the current geometric case,
- validates each edit locally,
- invalidates dependent downstream frames when necessary,
- validates the entire edited robot globally using forward kinematics,
- and exports the validated result in a reproducible format.

## 1.2 Research Question

> Can an interactive, constraint-aware tool make automatically generated DH frames understandable, editable, and verifiable without breaking kinematic equivalence to the source URDF?

## 1.3 Research Contribution

The contribution is not URDF parsing alone, DH extraction alone, or 3-D visualization alone.

The contribution is the combination of:

- geometric case classification,
- legal-edit-space derivation,
- guided sequential frame editing,
- local DH-rule validation,
- downstream dependency invalidation,
- immutable automatic reference state,
- and global URDF-FK versus DH-FK verification.

## 1.4 Primary User Workflow

The final application should behave conceptually as follows:

```text
Start URDF2DT
    ↓
Select / upload .urdf file
    ↓
Validate URDF structure and supported robot topology
    ↓
Parse URDF → KinematicChain
    ↓
Generate automatic Standard-DH solution
    ↓
Freeze automatic DH solution as immutable reference
    ↓
Render robot + joint axes + URDF frames + DH frames
    ↓
Classify each consecutive axis pair
    ↓
Unlock Frame 1
    ↓
Expose only legal edit parameters
    ↓
Propose edit → preview → local validation
    ↓
Accept / reject / restore
    ↓
Unlock next frame in kinematic order
    ↓
Repeat until all frames are accepted
    ↓
Run global URDF-FK ↔ DH-FK validation
    ↓
PASS? ── No → return to editor / restore
  │
 Yes
  ↓
Export validated DH model + edit history + validation report
```

## 1.5 Supported Scope for v1

Supported:

- serial-chain robots,
- fixed, revolute, continuous, and supported prismatic joints as permitted by the parser/solver,
- Standard-DH representation,
- CPU execution,
- Windows/Linux/macOS development,
- Jupyter-based interactive UI for v1,
- headless validation in CI.

Out of scope for v1:

- closed-loop robots,
- general branching kinematic trees,
- arbitrary CAD mesh rendering,
- real-time hardware-in-the-loop editing,
- unconstrained 6-DOF manual frame dragging,
- machine-learning-based frame placement.

---

# 2. Required Reference Material and Open Dependencies

The following reference material should be available or requested before the corresponding implementation stage:

- `universalUR5.urdf` — primary robot fixture.
- `GenerateDhParams.m` — MATLAB reference implementation.
- `TestURDF_Code.m` — reference invocation / expected UR5 output.
- DH Parameters technical report v2 — P–Q common-normal algorithm, thresholds, DH axis rules, UR5 case table, published UR5 DH table.
- `main.pdf` — parent URDF2DT architecture and module interfaces.
- Independent reference implementations such as `trzy/urdf_to_dh` and `takeshiD/urdf2dh` for cross-checking.

## 2.1 Mandatory Early Question

Before assuming the parser or automatic DH solver interface, determine whether the shared project already contains:

- Module 1: URDF parser → `KinematicChain`
- Module 2: `dh_solver.py` → automatic DH solution

If both exist, use their real interfaces and data types.

If they do not yet exist, use the published UR5 DH table from the MATLAB reference as a **temporary fixture** so editor development can continue without blocking.

The temporary fixture must be clearly marked and replaced once the real Module 1/2 interfaces become available.

## 2.2 Decisions to Resolve Before Stage 3

Two items in this plan were left open during earlier review and must be closed out
explicitly, not carried forward indefinitely:

1. **Module 1/2 interface status.** Confirm with the advisor or shared repository
   whether the URDF parser and automatic DH solver already exist. Record the answer
   (and the real interface, if it exists) in `docs/stages/02_dh_editor.md` before
   writing `dh/types.py`.
2. **Threshold reconciliation.** The default geometry and validation tolerances in
   `config/default.yaml` (parallel threshold `1e-5`, position tolerance `1e-4` m, etc.)
   are provisional. Confirm them against the DH technical report and the advisor
   before Stage 8 (case classification) depends on them — a threshold changed later
   invalidates any classification or validation results already produced with the old
   value.

Both are one short conversation with the advisor. Do not guess and proceed silently on
either.

---

# 3. Requirements Specification

## 3.1 Functional Requirements

| ID | Requirement |
|---|---|
| FR-1 | Let the user select a `.urdf` file as the starting input of the application. |
| FR-2 | Validate the selected URDF before running the parser or DH solver. |
| FR-3 | Parse a supported serial-chain URDF and obtain a `KinematicChain`. |
| FR-4 | Obtain the automatic Standard-DH solution from Module 2 or the temporary reference fixture. |
| FR-5 | Keep the automatic DH solution immutable for the entire editing session. |
| FR-6 | Render URDF link frames, joint axes, DH frames, and simple robot bones in one 3-D scene using visually distinguishable styles. |
| FR-7 | Classify each consecutive joint-axis pair using the project’s geometric cases and expose the user-facing interpretation parallel / intersecting / skew where appropriate. |
| FR-8 | Unlock DH frames strictly in kinematic order. Frame `i` may be accepted only after frame `i-1` is accepted. |
| FR-9 | Expose only geometrically legal edit parameters for the current case. Never allow unrestricted 6-DOF dragging. |
| FR-10 | Recompute only the affected DH row immediately when an edit is proposed or accepted. |
| FR-11 | Validate every proposed frame against DH geometric rules R1–R3 and return a clear reason on rejection. |
| FR-12 | Cascade-invalidate every accepted downstream frame when a previously accepted upstream frame changes. |
| FR-13 | Restore one frame or the entire session to the original automatic solution. |
| FR-14 | Compare URDF forward kinematics and DH forward kinematics for a fixed set of sampled joint configurations. |
| FR-15 | Validate both the automatic model and the final edited model. |
| FR-16 | Persist the validated DH model, configuration snapshot, edit history, validation samples, and validation report. |
| FR-17 | Provide a clear application entry point so the user can start the workflow without manually calling internal modules. |
| FR-18 | Provide readable status/error messages for invalid URDF input, unsupported topology, rejected edits, failed validation, and export errors. |

## 3.2 Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-1 | Axis classification and single-row recomputation should normally complete in <100 ms at UR5 scale. |
| NFR-2 | Numerical thresholds must be configurable and never scattered as unexplained magic numbers. |
| NFR-3 | The automatic solution must remain immutable during the session. |
| NFR-4 | The core state machine and global validator must run without Jupyter/PyVista. |
| NFR-5 | UR5 DH values must agree with the published reference within `1e-4` where the reference defines comparable values. |
| NFR-6 | Production editor logic must contain no UR5-specific joint names or constants. |
| NFR-7 | Validation must be reproducible from the same URDF, configuration, sample set, and seed. |
| NFR-8 | Export formats must be versioned with an explicit schema version. |
| NFR-9 | Core public APIs must use type hints and documented data contracts. |
| NFR-10 | Logging should provide enough information to reconstruct the sequence of major editor actions without relying on print statements. |
| NFR-11 | UI state must never disagree with the accepted `EditorSession` state after an operation completes. |
| NFR-12 | Orientation error must use an explicitly documented mathematical metric and tolerance rather than the phrase “small angular error.” |

---

# 4. Improved System Architecture

```text
                         USER
                           │
                           ▼
                ┌───────────────────┐
                │ Application Entry │
                │ app.py / notebook │
                └─────────┬─────────┘
                          ▼
                ┌───────────────────┐
                │ URDF File Input   │
                └─────────┬─────────┘
                          ▼
                ┌───────────────────┐
                │ URDF Validator    │
                │ syntax/topology   │
                └─────────┬─────────┘
                          ▼
                ┌───────────────────┐
                │ Module 1 Parser   │
                │ → KinematicChain  │
                └─────────┬─────────┘
                          ▼
                ┌───────────────────┐
                │ Module 2 DH       │
                │ Solver            │
                └─────────┬─────────┘
                          ▼
                ┌───────────────────┐
                │ Immutable Auto DH │
                └─────────┬─────────┘
                          ▼
       ┌────────────────────────────────────┐
       │          DH EDITOR CORE            │
       │                                    │
       │ case_dispatcher.py                 │
       │ geometry.py / recompute.py         │
       │ dh_validator.py                    │
       │ editor_session.py                  │
       │ global_validator.py                │
       └───────────────┬────────────────────┘
                       │
              ┌────────┴─────────┐
              ▼                  ▼
     ┌─────────────────┐  ┌──────────────────┐
     │ UI Layer        │  │ Visualization    │
     │ ipywidgets      │  │ PyVista          │
     │ dh_editor.py    │  │ scene.py         │
     └────────┬────────┘  └────────┬─────────┘
              └──────────┬─────────┘
                         ▼
               Guided editing session
                         │
                         ▼
                ┌───────────────────┐
                │ Global FK         │
                │ Validator         │
                └─────────┬─────────┘
                          ▼
                   PASS / FAIL
                          │
                    PASS  ▼
                ┌───────────────────┐
                │ Export Layer      │
                │ versioned schema  │
                └─────────┬─────────┘
                          ▼
       ┌────────────────────────────────────┐
       │ DH model + edit history + report   │
       │ + config + validation sample set   │
       └────────────────────────────────────┘
```

## 4.1 Architectural Rules

### Rule A — Core logic must not depend on the UI

`EditorSession`, geometric classification, DH recomputation, local validation, and global validation must be ordinary Python modules callable from tests and scripts.

### Rule B — Visualization must not own editor truth

PyVista objects are views of state. They are not the authoritative state.

### Rule C — `EditorSession` is the authoritative working state

Widgets may request transitions. Only successful session transitions update accepted state.

### Rule D — Automatic and working solutions are separate

Maintain conceptually:

```text
automatic_solution  # immutable baseline
working_solution    # current accepted/editable state
```

Never overwrite the automatic baseline.

### Rule E — Validation is layered

1. Input validation — is the URDF structurally supported?
2. Local geometric validation — is a proposed frame legal under the DH rules?
3. Global kinematic validation — does the complete DH model still match URDF FK?

---

# 5. Technology Selection

| Purpose | Technology | Reason |
|---|---|---|
| Language | Python 3.10+ | Project integration, scientific ecosystem, testability |
| URDF parsing | Existing Module 1 / reused project code | Avoid duplicating parser work |
| Automatic DH solving | Existing Module 2 / URDFly-derived logic | Reuse validated project component |
| Numerics | NumPy, SciPy | Vector/matrix/frame computation |
| Symbolic FK | CasADi | Reusable, deterministic symbolic/numeric validation |
| 3-D visualization | PyVista | Existing project direction and convenient frame visualization |
| Interactive controls | ipywidgets + JupyterLab | Thin notebook-native UI for v1 |
| Data structures | `dataclasses`, often `frozen=True` | Explicit contracts and immutability |
| Configuration | YAML + typed Python config object | Reproducible, centralized thresholds |
| Tests | pytest | Project-wide automated testing |
| Type checking | mypy or pyright | Catch data-contract mistakes |
| Logging | Python `logging` | Reproducible debug/action records |
| Export | JSON + Markdown report | Machine-readable + human-readable outputs |
| CI | GitHub Actions | Test every push/PR |

## 5.1 Explicitly Rejected v1 Alternatives

- Full desktop GUI with PyQt as the primary interface.
- Free six-degree-of-freedom frame manipulation.
- GPU dependency.
- ML-based frame recommendation.
- Silent automatic correction of invalid edits.

---

# 6. Infrastructure Requirements

- **Compute:** CPU only; no GPU required.
- **OS:** Windows 11 primary development environment; Linux/macOS supported where dependencies permit.
- **IDE:** VS Code recommended.
- **Notebook environment:** JupyterLab recommended for ipywidgets/PyVista integration.
- **CI:** GitHub Actions on Linux.
- **Headless rendering:** use Xvfb only where visual tests require it on Linux CI.
- **Storage:** small text-based files: URDF, JSON, YAML, Markdown, notebooks.
- **Reproducibility:** configuration snapshot and random seed must accompany validation reports.

---

# 7. Environment Setup — From Zero

## 7.1 Required Software

Install:

- Python 3.10+
- Git for Windows
- VS Code

Recommended VS Code extensions:

- Python
- Pylance
- Jupyter
- GitLens
- autoDocstring
- Even Better TOML

## 7.2 Create Virtual Environment

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.venv\Scripts\Activate.ps1
```

Select `.venv` through **Python: Select Interpreter** in VS Code.

## 7.3 Install Dependencies

```powershell
python -m pip install --upgrade pip
pip install numpy scipy casadi pandas
pip install pyvista ipywidgets pythreejs jupyterlab
pip install pyyaml pytest mypy
```

Depending on PyVista/Jupyter backend requirements:

```powershell
pip install trame trame-vtk trame-vuetify
```

## 7.4 Verify CasADi

```powershell
python -c "import casadi as ca; print(ca.__version__)"
```

## 7.5 Verify PyVista on Windows

```python
import pyvista
pyvista.Sphere().plot()
```

A 3-D sphere window should appear.

Do not call `pyvista.start_xvfb()` on ordinary Windows development. Xvfb is intended for headless Linux environments.

---

# 8. Repository and Branching Strategy

Create or clone the repository, then use a visible stage-oriented Git history.

Recommended permanent branches:

```text
main
integration/dh-editor   # optional integration branch
```

Recommended working branch pattern:

```text
feature/stage-03-core-types
feature/stage-04-urdf-input
feature/stage-05-parser-integration
...
```

Minimum `.gitignore`:

```gitignore
.venv/
__pycache__/
*.pyc
*.egg-info/
.ipynb_checkpoints/
.pytest_cache/
.mypy_cache/
outputs/tmp/
```

Decide intentionally whether `.vscode/` is tracked. A good compromise is to track only shared recommendations such as `.vscode/extensions.json` and project test settings if useful.

Every completed stage must be committed, pushed, tested, and documented.

---

# 9. Revised Complete Project Structure

```text
URDF2DT/
│
├── README.md
├── pyproject.toml
├── .gitignore
│
├── config/
│   └── default.yaml
│
├── robots/
│   ├── ur5/
│   │   └── universalUR5.urdf
│   └── examples/
│
├── outputs/
│   ├── dh_models/
│   ├── edit_history/
│   ├── validation_reports/
│   └── tmp/
│
├── urdf2dt/
│   ├── __init__.py
│   ├── app.py
│   ├── config.py
│   ├── logging_config.py
│   │
│   ├── vendor/
│   │   ├── __init__.py
│   │   ├── urdf2dh.py
│   │   ├── kinematics_helpers.py
│   │   ├── geometry_helpers.py
│   │   └── urdf_helpers.py
│   │
│   ├── parser/
│   │   ├── __init__.py
│   │   ├── urdf_validator.py
│   │   ├── urdf_parser.py
│   │   └── kinematic_graph.py
│   │
│   ├── dh/
│   │   ├── __init__.py
│   │   ├── types.py
│   │   ├── geometry.py
│   │   ├── dh_solver.py
│   │   ├── case_dispatcher.py
│   │   ├── recompute.py
│   │   ├── dh_validator.py
│   │   ├── editor_session.py
│   │   └── global_validator.py
│   │
│   ├── ui/
│   │   ├── __init__.py
│   │   └── dh_editor.py
│   │
│   ├── visualization/
│   │   ├── __init__.py
│   │   └── scene.py
│   │
│   ├── export/
│   │   ├── __init__.py
│   │   ├── schemas.py
│   │   └── model_exporter.py
│   │
│   └── simulation/
│       ├── __init__.py
│       ├── simulator.py
│       ├── trajectory.py
│       └── visualizer.py
│
├── examples/
│   ├── ur5_full_pipeline.ipynb
│   └── two_link_tutorial.ipynb
│
├── docs/
│   └── stages/
│       ├── 00_environment.md
│       ├── 01_repository.md
│       ├── 02_scaffolding.md
│       └── 03_dh_editor.md
│
├── tests/
│   ├── fixtures/
│   │   ├── ur5_automatic_dh.py
│   │   └── invalid_urdf/
│   ├── test_types.py
│   ├── test_config.py
│   ├── test_urdf_validator.py
│   ├── test_scene.py
│   ├── test_dh.py
│   ├── test_editor_session.py
│   ├── test_global_validator.py
│   └── test_export.py
│
└── .github/
    └── workflows/
        └── tests.yml
```

## 9.1 Structure Rationale

- `parser/` owns URDF structural handling.
- `dh/` owns kinematic/DH domain logic.
- `ui/` owns widgets only.
- `visualization/` owns PyVista rendering only.
- `export/` owns persistent schemas and serialization.
- `config/` centralizes thresholds and validation settings.
- `robots/` holds bundled fixtures, not arbitrary user-upload history.
- `outputs/` holds generated artifacts and reports.
- `tests/fixtures/` keeps robot-specific reference constants out of production code.

---

# 10. Configuration and Reproducibility Contract

Create `config/default.yaml`.

Example:

```yaml
geometry:
  parallel_threshold: 1.0e-5
  intersection_threshold: 1.0e-6
  common_normal_threshold: 1.0e-6

validation:
  position_tolerance_m: 1.0e-4
  orientation_tolerance_rad: 1.0e-4
  samples: 50
  random_seed: 42

ui:
  live_preview: true

logging:
  level: INFO
```

Important:

- The exact defaults must be reconciled with the DH technical report and advisor guidance.
- The file should be loaded into an immutable typed configuration object.
- Every validation report must record the effective configuration values.
- A report generated with one threshold set must not be presented as equivalent to a report generated with another threshold set without saying so.

## 10.1 Reproducibility Metadata (Git Commit Hash)

Configuration and sample seed alone do not guarantee reproducibility — the same
config run against two different versions of the editor's code can legitimately
produce different results. Every validation report must additionally record:

- the git commit hash of the code that produced it (`git rev-parse HEAD`, captured
  programmatically at report-generation time, not typed in by hand),
- whether the working tree was clean or had uncommitted changes at that time.

A report generated from a dirty working tree should be clearly marked as such — it is
not a claim of full reproducibility until the corresponding code is actually committed.

---

# 11. Core Data Contracts

Create `urdf2dt/dh/types.py`.

Expected typed concepts include:

- `AxisCase`
- `DHRow`
- `DHModel`
- `FrameState`
- `EditorState`
- `EditorConfig`
- `ValidationSample`
- `ValidationResult`
- `EditRecord`

Use immutable dataclasses for values that should not change after construction.

Example conceptual definition:

```python
@dataclass(frozen=True)
class DHRow:
    a: float
    alpha: float
    d: float
    theta_offset: float
```

The exact names must match the automatic DH solver’s representation once that interface is known.

---

# 12. URDF Input and Pre-Validation

## 12.1 User Input

The normal user starts by selecting a `.urdf` file.

The system should not ask the user to manually type DH parameters at startup.

## 12.2 Validation Pipeline

Before full parsing/DH generation, check at minimum:

1. File exists and is readable.
2. Extension/content is plausible URDF/XML.
3. XML parses successfully.
4. A `<robot>` root exists.
5. Links exist.
6. Joints exist.
7. Parent/child link references are valid.
8. The kinematic graph is connected where required.
9. v1 topology is a supported serial chain.
10. Joint types are supported by the parser/solver.
11. No invalid cycles are present.
12. Required axis/origin fields have sensible defaults or are handled explicitly.

## 12.3 Error Messages

Bad:

```text
Failed.
```

Good:

```text
URDF rejected: branching kinematic structure detected.
URDF2DT v1 currently supports serial chains only.
```

or:

```text
URDF rejected: joint 'joint_4' references missing child link 'link_4'.
```

The validator should return structured error information so the UI can display it without parsing raw exception strings.

## 12.4 XML Security (XXE Protection)

URDF files are XML, and parsing untrusted XML with the Python standard library
(`xml.etree.ElementTree`, `xml.dom.minidom`) is vulnerable to XML External Entity
(XXE) attacks — a crafted file can reference external entities to read local files or
trigger network requests during parsing, before any structural validation runs.

Requirements:

1. Parse URDF files using `defusedxml` (a drop-in replacement for the standard
   library's XML parsers) instead of the raw `xml` module, or explicitly disable
   external entity and DTD resolution if a different parser is required.
2. Treat every uploaded/selected `.urdf` file as untrusted input, regardless of
   whether it came from a bundled fixture or a user's own machine — the security
   guard should not depend on trusting the source.
3. Add a test fixture containing a deliberately malicious XXE payload and confirm the
   parser rejects or safely neutralizes it rather than resolving the entity.

This check belongs in Stage 4 (URDF Input and Structural Validation), immediately
alongside the other XML/topology checks, not as a separate later pass.

---

# 13. Automatic DH Solution Contract

After URDF validation and parsing:

```text
URDF → KinematicChain → automatic Standard-DH solution
```

The output must become the immutable baseline for the session.

Do not let UI controls directly modify the automatic object.

Recommended conceptual model:

```text
automatic_model : immutable DHModel
working_model   : current accepted DHModel
```

The automatic model serves four purposes:

- restore source,
- audit baseline,
- automatic-model FK validation,
- before/after comparison.

---

# 14. Axis-Case Classification

Every consecutive joint-axis pair must be classified according to the DH report’s geometric logic.

The user-facing conceptual categories are:

- parallel,
- intersecting,
- skew.

If the reference implementation uses more specific labels such as `A`, `B1`, `B2`, preserve those exact internal labels and map them to explanatory user-facing text.

For UR5, the reference case sequence expected by the existing plan is:

```text
B2, A, A, B1, B1, A
```

This must be verified against:

1. DH report Table 3,
2. MATLAB/reference logic,
3. independent open-source implementations where comparison is meaningful.

Thresholds must come from configuration.

---

# 15. Legal Edit Space

The editor must never expose arbitrary XYZ + roll/pitch/yaw dragging.

For each axis case, implement:

```python
get_editable_params(case)
```

This returns exactly the degrees of freedom allowed by the Standard-DH geometric construction for that case.

The UI is generated from this legal-edit description.

Conceptual rule:

```text
Geometry determines legal parameters.
Legal parameters determine controls.
Controls do not determine geometry.
```

This direction is important: the UI must not invent degrees of freedom merely because a slider is easy to build.

---

# 16. Editor State Machine

`EditorSession` is the authoritative state-transition system.

Required behaviors:

```python
unlock_next()
propose_edit(...)
accept(...)
reject(...)
restore_frame(...)
restore_automatic()
```

The exact API can evolve, but the semantics must remain stable.

## 16.1 Sequential Unlocking

Example:

```text
Frame 1 unlocked
Frame 2 locked
Frame 3 locked
...
```

After Frame 1 is accepted:

```text
Frame 1 accepted
Frame 2 unlocked
Frame 3 locked
...
```

## 16.2 Cascade Invalidation

If all six UR5 frames have been accepted and Frame 2 changes:

```text
Frame 1: accepted and unchanged
Frame 2: edited / revalidated
Frame 3: invalidated
Frame 4: invalidated
Frame 5: invalidated
Frame 6: invalidated
```

If Frame 1 changes, Frames 2–6 must all invalidate.

## 16.3 Restore Behavior

The user must be able to:

- restore the current frame,
- restore a previously accepted frame,
- restore the entire editing session to the automatic solution.

Restoring must use the immutable automatic baseline, not a chain of reverse edits.

---

# 17. Local DH Recompute and Validation

## 17.1 Recompute

Implement `recompute_dh_row()` using the P–Q/common-normal mathematics defined in the technical report.

Only the affected row should be recomputed for live editing when possible.

## 17.2 Local Validation

Implement `validate_frame()` to verify the required DH axis/frame rules, including the project’s R1–R3 conditions.

At minimum this should cover the intended meanings of:

- axis collinearity requirements,
- common-normal correctness,
- frame orientation/right-handedness,
- numerical tolerance handling.

Do not silently clamp or “fix” invalid user edits unless such behavior is explicitly defined in the report.

Return a structured validation result such as:

```text
valid: false
rule: R2
message: "x_i is not aligned with the common normal between z_(i-1) and z_i"
```

---

# 18. 3-D Visualization

Implement in `urdf2dt/visualization/scene.py`:

```python
draw_frame_triad(...)
draw_joint_axis(...)
draw_bone(...)
```

and a `StaticScene`/scene-controller abstraction.

The scene must make it possible to distinguish:

- URDF frames,
- DH frames,
- joint axes,
- simplified robot links/bones,
- currently selected frame,
- accepted vs unlocked/invalidated state if helpful.

For the UR5 reference scene, retain automated numeric checks for known zero-pose origins, including the existing ground-truth values used by the plan:

```text
shoulder_link  = [0, 0, 0.089159]
upper_arm_link = [0, 0.13585, 0.089159]
```

Visual inspection is not enough; numeric tests are required.

---

# 19. Interactive UI

The v1 UI is a thin Jupyter/ipython widget layer.

## 19.1 Start Screen

Conceptual layout:

```text
┌────────────────────────────────────────┐
│                URDF2DT                 │
│                                        │
│ Robot URDF                             │
│ [ Choose .urdf File ]                  │
│                                        │
│ Selected: universalUR5.urdf            │
│                                        │
│ [ Load Robot ]                         │
└────────────────────────────────────────┘
```

After successful input validation:

```text
✓ URDF valid
✓ Serial chain detected
✓ 6 movable joints
✓ Automatic Standard-DH solution generated

[ Open DH Editor ]
```

## 19.2 Editor Screen

Conceptual layout:

```text
┌────────────────────────────────────────────────────┐
│                   3-D ROBOT VIEW                   │
│          URDF + axes + DH frame overlay            │
├────────────────────────────────────────────────────┤
│ Current frame: 3                                   │
│ Geometric case: Parallel / A                       │
│                                                    │
│ Allowed edit parameter                             │
│ origin offset [---------○------------]             │
│                                                    │
│ Preview status: Valid                              │
│                                                    │
│ [ Accept ] [ Restore Frame ] [ Restore All ]       │
└────────────────────────────────────────────────────┘
```

## 19.3 UI State Rule

A slider value is not accepted model state merely because it is visible.

The flow must be:

```text
widget input
   ↓
propose edit
   ↓
recompute
   ↓
local validate
   ↓
preview
   ↓
accept
   ↓
EditorSession state changes
```

Rejected values must not leave the visual widgets pretending that the underlying accepted session changed.

---

# 20. Global URDF-FK vs DH-FK Validator

Implement `global_validator.py` independently of PyVista/Jupyter.

## 20.1 Inputs

- validated source `KinematicChain`,
- candidate `DHModel`,
- configuration snapshot,
- deterministic set of joint samples.

## 20.2 Sample Generation

Use:

- zero configuration,
- 20–50 default sampled configurations,
- fixed random seed,
- valid joint-limit-aware sampling where limits are available.

Store the actual sample set or enough information to reproduce it exactly.

## 20.3 Forward Kinematics

Build:

```python
urdf_fk(q)
dh_fk(q)
```

using CasADi or the project’s common computational representation.

## 20.4 Position Error

For end-effector positions:

```math
e_p = \|p_{URDF} - p_{DH}\|_2
```

Default target from the existing project plan:

```text
position error < 1e-4 m
```

unless the technical report defines a stricter or different criterion.

## 20.5 Orientation Error

Use an explicit SO(3) geodesic angle metric:

```math
e_R = \cos^{-1}\left(\operatorname{clip}\left(\frac{\operatorname{tr}(R_{URDF}^{T}R_{DH}) - 1}{2}, -1, 1\right)\right)
```

The `clip` is important numerically because floating-point roundoff can push the cosine argument slightly outside `[-1, 1]`.

The orientation tolerance must be configurable and confirmed against the project’s reference methodology/advisor rather than left as “small angular error.”

## 20.6 Validation Result

Report at minimum:

- pass/fail,
- number of samples,
- maximum position error,
- mean position error,
- maximum orientation error,
- mean orientation error,
- worst-case sample index/configuration,
- tolerance values,
- configuration hash or snapshot,
- candidate model version.

## 20.7 Required Runs

At minimum validate:

1. automatic DH model,
2. one fully edited valid UR5 model.

The automatic model must pass before claiming the editor is correct.

## 20.8 Per-Frame Error Attribution (Failure Diagnosis)

A bare pass/fail result is not enough to act on when a multi-frame edited session
fails global validation — with six accepted frames, the user has no way to tell which
one is responsible without re-checking each individually by hand.

Requirements:

1. When global validation fails, compute the FK chain incrementally up to each joint
   (`T_urdf(q, i)` vs `T_dh(q, i)` for `i = 1..n`) rather than only at the end
   effector, and report the first joint index at which the per-joint error exceeds
   tolerance.
2. Surface this as a specific diagnostic message, e.g. `"Deviation first exceeds
   tolerance at frame 3 (position error 0.0021 m); frames 1-2 remain within
   tolerance."` — not just `"validation failed."`
3. Expose this diagnostic in both the machine-readable JSON report and the
   human-readable Markdown report (§21).
4. In the UI, allow the user to jump directly back to the flagged frame for
   correction rather than re-opening frames one by one.

This turns global validation from a final gate into a usable debugging tool during
editing, not just a pass/fail check at the very end.

---

# 21. Export and Persistence Contract

The system must persist outputs in a format that downstream modules can consume.

## 21.1 Versioned DH Model JSON

Conceptual schema:

```json
{
  "schema_version": "1.0",
  "robot": {
    "name": "UR5",
    "source_urdf": "universalUR5.urdf"
  },
  "dh_convention": "standard",
  "dh_parameters": [
    {
      "index": 1,
      "joint_name": "...",
      "a": 0.0,
      "alpha": 1.5707963268,
      "d": 0.089159,
      "theta_offset": 0.0
    }
  ],
  "validation": {
    "passed": true,
    "position_tolerance_m": 0.0001,
    "orientation_tolerance_rad": 0.0001,
    "max_position_error_m": 0.000001,
    "max_orientation_error_rad": 0.000002
  },
  "reproducibility": {
    "git_commit": "a1b2c3d4",
    "working_tree_clean": true,
    "random_seed": 42
  }
}
```

This is illustrative. Final field names must match downstream module contracts.

## 21.2 Edit History

Store a chronological record containing information such as:

- timestamp or sequence number,
- frame index,
- previous accepted values,
- proposed values,
- validation result,
- accepted/rejected/restored action,
- downstream frames invalidated.

## 21.3 Validation Report

Produce both:

```text
validation_report.json  # machine-readable
validation_report.md    # human/paper-friendly
```

## 21.4 Configuration Snapshot

A validation artifact must preserve the thresholds and sample-generation settings used for the run.

---

# 22. Logging

Use Python `logging`, not scattered `print()` statements.

Log significant events such as:

```text
URDF selected
URDF validation started
URDF validation passed/failed
Kinematic chain created
Automatic DH generated
Axis case classified
Edit proposed
Edit rejected
Edit accepted
Cascade invalidation triggered
Frame restored
Global validation started
Global validation passed/failed
Export completed
```

Example:

```text
2026-09-13 14:22:10 INFO  URDF validated: UR5, serial chain, 6 movable joints
2026-09-13 14:22:11 INFO  Automatic Standard-DH model generated
2026-09-13 14:23:04 INFO  Frame 2 edit accepted
2026-09-13 14:23:04 INFO  Frames 3-6 invalidated
```

Do not log excessively noisy slider events at INFO level if they make the research logs unusable; use DEBUG where appropriate.

---

# 23. Data and Testing Strategy

This project does not use a machine-learning dataset.

## 23.1 Input Data

- URDF files.
- UR5 primary fixture.
- second structurally different robot before final generality claim.

## 23.2 Reference Data

- published UR5 DH table,
- UR5 geometric case table,
- known link-frame origins,
- MATLAB expected output.

These belong in tests/fixtures or documentation, not production editor logic.

## 23.3 Generated Data

- sampled joint configurations,
- validation result tables,
- edit histories,
- ablation results.

## 23.4 Test Layers

### Unit tests

- vector/frame geometry,
- classification,
- editable-parameter mapping,
- immutable dataclasses,
- row recomputation,
- local validation,
- export schema.

### State-machine tests

- unlock order,
- accept/reject behavior,
- cascade invalidation,
- single-frame restore,
- session restore.

### Integration tests

- URDF → parser → DH solution,
- editor-session + recompute + validator,
- candidate model → global validator,
- export → reload.

### Invariant/property-style tests

Verify that:

- rotation matrices remain orthonormal within tolerance,
- accepted frames satisfy R1–R3,
- automatic model is unchanged after arbitrary legal edit sequences,
- editing frame `i` never changes accepted upstream frames `< i`,
- editing frame `i` invalidates accepted downstream frames `> i`,
- restore reproduces the exact automatic state,
- fixed seed produces identical sample sets,
- serialization round-trips without changing the DH model.

## 23.5 Human Usability Check

Automated tests confirm the editor is *correct*; they do not confirm it is
*understandable* — and understandability is part of the research claim (§1.2's
research question is about making frames "understandable," not just mathematically
valid). Before finalizing the module:

1. Have someone other than the module's author — a labmate, the advisor, or another
   student unfamiliar with the implementation details — load a robot and attempt to
   accept a single frame using only what the UI shows them, without walkthrough or
   explanation beforehand.
2. Note where they hesitate, misinterpret a control, or need to ask a question the UI
   should have answered on its own.
3. Record findings in `docs/stages/02_dh_editor.md` alongside the automated test
   results — this is qualitative evidence, not a pass/fail gate, but it belongs in the
   research record.

This check is folded into Stage 15 (Generalization to a Second Robot) as an
additional step, so it happens once the tool is stable enough to be worth watching
someone else use.

---

# 24. Evaluation Metrics

| Metric | Target | Method |
|---|---|---|
| UR5 DH numeric agreement | `<1e-4` where directly comparable | Published table / MATLAB output |
| UR5 axis-case classification | 100% agreement with accepted reference | Table 3 + references |
| Automatic model FK position error | `<1e-4 m` default | Sampled CasADi FK comparison |
| Edited model FK position error | Same configured tolerance | Sampled FK comparison |
| Orientation error | Below configured angular tolerance | SO(3) geodesic angle |
| State-machine correctness | All defined transition tests pass | pytest |
| Immutability | Automatic state never mutates | pytest + frozen types |
| UI/session synchronization | No accepted-state divergence | integration test |
| Generalization | second robot passes full supported pipeline | end-to-end run |

---

# 25. Ablation Studies

Even though this is not an ML project, controlled ablations provide evidence for design choices.

## 25.1 Threshold Sensitivity

Vary the parallelism threshold around the default by approximately an order of magnitude and record whether classifications change.

Questions:

- Does the UR5 sit close to a classification boundary?
- Are outcomes stable across reasonable threshold values?
- Do reference implementations use different conventions?

## 25.2 Legal Edit-Range Sweep

For a case with a continuous legal edit degree of freedom, sweep the parameter across its complete allowed interval.

At every sampled edit:

1. recompute the affected frame,
2. validate R1–R3,
3. run global FK validation,
4. record max error.

This is one of the strongest experiments supporting the research claim.

## 25.3 Sample-Density Ablation

Run global validation with:

```text
10 samples
50 samples
200 samples
```

Compare error statistics and worst cases.

## 25.4 Near-Boundary Synthetic Geometry

Construct an axis pair close to the configured parallel/intersection threshold and document classifier behavior.

The purpose is not necessarily to declare one threshold philosophically “correct,” but to demonstrate that the implementation is explicit, deterministic, configurable, and understood.

---

# 26. Expected Failure Modes and Error Analysis

Explicitly test and document:

1. malformed XML,
2. missing link references,
3. cycles,
4. branching topology,
5. unsupported joint types,
6. near-parallel numeric classification,
7. near-zero common normal,
8. floating-point drift in chained transforms,
9. joint order mismatch between URDF and DH model,
10. incorrect theta-offset convention,
11. widget/session desynchronization,
12. incomplete cascade invalidation,
13. restore not reproducing exact baseline,
14. orientation metric `acos` domain issues without clipping,
15. inconsistent thresholds between local and global validation,
16. export schema drift,
17. downstream module incompatibility.

---

# 27. Performance Strategy

Performance is not expected to be difficult for six joints, but structure the code correctly:

- cache CasADi FK `Function` objects,
- use `map()`/batch evaluation where useful,
- recompute one DH row rather than the whole table on every preview,
- avoid rebuilding the PyVista scene from zero when an actor update is sufficient,
- keep UI callbacks thin,
- precompute immutable geometry where possible,
- benchmark classification/recompute to enforce the interactive target.

---

# 28. Documentation Strategy

Maintain documentation during development.

Primary research record:

```text
docs/stages/03_dh_editor.md
```

or retain the parent project’s expected `02_dh_editor.md` naming if module numbering requires it. Pick one naming convention and use it consistently.

The document should eventually contain:

- objective,
- architecture,
- external interfaces,
- DH geometry assumptions,
- case mapping,
- legal edit rules,
- state-machine behavior,
- validation methodology,
- configuration values,
- test results,
- ablation results,
- second-robot result,
- limitations,
- known issues,
- final validation numbers.

Every public function/class should have a docstring explaining geometric meaning where relevant, not merely argument types.

---

# 29. Naming and Consistency Rules

The previous project material mixed Stage A–E language with numeric Stage 0–14 language.

This revised plan uses **numeric stages only** for execution.

Use:

```text
Stage 0
Stage 1
Stage 2
...
```

Do not create Git branches called “Stage B” while the documentation calls the same work “Stage 7.”

Likewise, distinguish clearly between:

- parent-project module number,
- source folder name,
- development stage number.

Before finalizing README/documentation, confirm the official Module 1 / Module 2 / DH-editor module numbering from `main.pdf` and use it consistently throughout the repository.

---

# 30. Project Management Rules

1. Work top to bottom through the Master Development Plan.
2. One stage is not complete until tests pass.
3. Update documentation in the same stage as code.
4. Commit and push at the end of every stage.
5. Resolve interface ambiguity before building code that depends on the ambiguous interface.
6. Preserve research fixtures separately from production constants.
7. Do not postpone FK equivalence testing until the very end.
8. Hold advisor checkpoints after geometry/case classification and after global FK validation.
9. Prefer a correct narrow implementation over a broad unvalidated implementation.

---

# 31. Definition of Done

## 31.1 Stage Definition of Done

A stage is complete only when:

- the stage’s code exists,
- relevant tests pass,
- no stage-specific TODO remains unresolved,
- documentation is updated,
- changes are committed and pushed,
- any required reference comparison has been performed.

## 31.2 Module Definition of Done

The module is complete when:

- user can start from a `.urdf` file,
- invalid/unsupported URDFs are rejected clearly,
- automatic DH generation/integration works,
- automatic solution is immutable,
- full UR5 scene renders,
- all UR5 axis cases are correct,
- guided legal editing works for every supported case,
- local R1–R3 validation works,
- cascade invalidation works,
- restore works,
- automatic and edited UR5 models pass global FK validation,
- ablation studies are recorded,
- a second supported robot passes end to end,
- outputs export using a versioned schema,
- test suite passes with no unexplained skips/xfails,
- mypy/pyright is clean to the agreed project level,
- worked example notebook runs top to bottom,
- CI passes,
- documentation and handoff notes are complete.

---

# 32. Final Deliverables

1. **Application entry point** — starts the URDF-driven workflow.
2. **URDF validator** — readable rejection of unsupported inputs.
3. **Parser/DH integration** — consumes the project’s real Module 1/2 outputs.
4. **DH editor core** — classification, recompute, validation, state machine.
5. **Interactive UI** — legal controls only.
6. **3-D visualization** — URDF + axes + DH frames.
7. **Global validator** — deterministic URDF-FK vs DH-FK comparison.
8. **Configuration system** — versioned/reproducible thresholds/settings.
9. **Export layer** — DH model, history, config, report.
10. **Logging** — significant actions recorded.
11. **Tests** — unit, state-machine, integration, invariant checks.
12. **UR5 worked notebook** — complete demonstration.
13. **Second robot result** — evidence for generalization.
14. **Research record** — implementation and experiment documentation.
15. **Versioned validation report** — paper/table-ready metrics.
16. **Paper-ready summary paragraph** — contribution + final numbers.
17. **GitHub CI** — automated pytest/type-check checks.
18. **Tagged release** — final reproducible state.
19. **Handoff note** — downstream integration contract and limitations.

---

# 33. Master Development Plan — Start to Finish

The following stage order is mandatory unless the advisor explicitly changes dependencies.

---

## Stage 0 — Environment Setup

1. Install Python 3.10+.
2. Verify `python --version`.
3. Install Git and configure identity.
4. Install VS Code and recommended extensions.
5. Create/open the project directory.
6. Create `.venv`.
7. Activate `.venv`.
8. Select `.venv` interpreter in VS Code.
9. Install NumPy/SciPy/CasADi/Pandas.
10. Install PyVista/ipywidgets/pythreejs/JupyterLab.
11. Install PyYAML/pytest/mypy.
12. Verify CasADi.
13. Verify PyVista renders on Windows.
14. Configure pytest discovery in VS Code.
15. Record local setup notes.

**Done when:** Python environment, CasADi, PyVista, pytest, and VS Code interpreter all work without error.

---

## Stage 1 — GitHub Repository and Version Control

1. Create GitHub repository after confirming public/private status.
2. Initialize Git or clone the new repository.
3. Add remote.
4. Create `.gitignore`.
5. Create minimal `README.md`.
6. Commit initial setup.
7. Push `main`.
8. Create first feature branch.
9. Push feature branch.
10. Record repository URL in project documentation.

Suggested commit:

```text
chore: initial repository setup
```

**Done when:** remote repository, initial commit, and working branch are visible on GitHub.

---

## Stage 2 — Package Scaffolding

1. Create the full revised folder structure from Section 9.
2. Add `__init__.py` files.
3. Create `pyproject.toml`.
4. Configure editable package installation.
5. Run:

```powershell
pip install -e .
```

6. Verify:

```powershell
python -c "import urdf2dt; print('ok')"
```

7. Create documentation stubs.
8. Commit and push.

Suggested commit:

```text
stage2: package scaffolding
```

**Done when:** package imports cleanly and tree matches the agreed structure.

---

## Stage 3 — Requirements, Interfaces, Configuration, and Core Types

1. Re-read requirements and architecture.
2. Confirm official parent-project module numbering.
3. Determine whether Module 1 parser exists.
4. Determine whether Module 2 DH solver exists.
5. Record their real interfaces if available.
6. Resolve threshold reconciliation with the advisor (§2.2) before proceeding.
7. Create `config/default.yaml` with the confirmed thresholds.
8. Implement typed immutable configuration loading.
9. Implement core types in `dh/types.py`.
10. Add immutability tests.
11. If Module 1/2 is unavailable, add temporary UR5 DH fixture.
12. Define provisional versioned output schema.
13. Commit, push, document.

Suggested commit:

```text
stage3: interfaces, configuration, core dataclasses
```

**Done when:** interfaces are documented, config loads, immutable types pass tests, and there is either real automatic-DH output or a marked fixture.

---

## Stage 4 — URDF Input and Structural Validation

1. Implement file-path/file-selection input abstraction.
2. Implement `urdf_validator.py` using `defusedxml` (or equivalent XXE-safe parsing) per §12.4.
3. Validate XML syntax.
4. Validate robot root, links, joints.
5. Validate parent/child references.
6. Detect disconnected structures, cycles, and branches.
7. Enforce v1 supported topology.
8. Validate supported joint types.
9. Return structured validation errors.
10. Add invalid URDF fixtures, including a deliberately malicious XXE payload.
11. Test valid UR5 acceptance.
12. Test malformed/branching/missing-link rejection and confirm the XXE fixture is safely rejected.
13. Commit, push, document.

Suggested commit:

```text
stage4: URDF input validation
```

**Done when:** UR5 passes and deliberately invalid fixtures fail with specific reasons.

---

## Stage 5 — Parser and Automatic DH Integration

1. Connect validated URDF to real Module 1 parser if available.
2. Verify kinematic chain order.
3. Connect `KinematicChain` to real Module 2 solver if available.
4. Otherwise connect the temporary UR5 fixture through the same adapter interface.
5. Normalize automatic output into `DHModel`/agreed internal types.
6. Freeze automatic model as immutable baseline.
7. Add tests proving automatic baseline cannot be mutated.
8. Compare UR5 automatic values against MATLAB/published reference.
9. Commit, push, document.

Suggested commit:

```text
stage5: parser and automatic DH integration
```

**Done when:** UR5 produces a reference-compatible immutable automatic DH model through the project interface.

---

## Stage 6 — Static Scene Renderer

1. Check whether visualization code already exists elsewhere in URDF2DT.
2. Implement/reuse `draw_frame_triad`.
3. Implement/reuse `draw_joint_axis`.
4. Implement/reuse `draw_bone`.
5. Build `StaticScene`.
6. Render URDF frames.
7. Render joint axes.
8. Render automatic DH frames.
9. Use distinguishable visual styles.
10. Verify known UR5 zero-pose link origins numerically.
11. Add `test_scene.py`.
12. Commit, push, document.

Suggested commit:

```text
stage6: static URDF and DH scene renderer
```

**Done when:** scene renders and numeric origin tests pass.

---

## Stage 7 — Axis-Case Classifier

1. Implement `classify_axis_pair()`.
2. Load thresholds from config.
3. Reproduce UR5 expected case sequence.
4. Compare with DH report Table 3.
5. Compare with MATLAB/reference implementation.
6. Compare with independent repositories where practical.
7. Investigate every mismatch before continuing.
8. Implement user-facing case descriptions.
9. Implement `get_editable_params(case)`.
10. Add classification and legal-parameter tests.
11. Commit, push, document.
12. Advisor checkpoint.

Suggested commit:

```text
stage7: axis-case classification and legal edit mapping
```

**Done when:** classifier agrees with accepted references and legal-edit mapping tests pass.

---

## Stage 8 — EditorSession State Machine

1. Implement session initialization from immutable automatic model.
2. Implement frame lock/unlock state.
3. Implement `unlock_next()`.
4. Implement proposed-edit representation.
5. Implement `accept()` semantics.
6. Implement rejection semantics without corrupting accepted state.
7. Implement single-frame restore.
8. Implement whole-session restore.
9. Implement cascade invalidation.
10. Test frames 1–3 accepted then Frame 2 changed → Frame 3 invalid.
11. Test all six accepted then Frame 1 changed → Frames 2–6 invalid.
12. Test restore exactly matches baseline.
13. Commit, push, document.

Suggested commit:

```text
stage8: editor state machine and cascade invalidation
```

**Done when:** all state-transition tests pass without any UI dependency.

---

## Stage 9 — DH Recompute and Local Validation

1. Implement P–Q/common-normal geometry helpers.
2. Implement case-specific `recompute_dh_row()`.
3. Implement R1–R3 `validate_frame()`.
4. Return structured rule-specific validation errors.
5. Connect recompute to `EditorSession.propose_edit()`.
6. Ensure rejected edits do not mutate accepted state.
7. Add numeric edge-case tests.
8. Add right-handedness/rotation validity tests.
9. Add legal edit-range local tests.
10. Benchmark classification/recompute against interactive target.
11. Commit, push, document.

Suggested commit:

```text
stage9: DH row recomputation and local validation
```

**Done when:** legal edits pass, illegal edits fail with specific reasons, and accepted state remains consistent.

---

## Stage 10 — Interactive Editor UI

1. Implement start/load screen in `ui/dh_editor.py`.
2. Connect file selection to URDF validator.
3. Display robot summary after loading.
4. Add 3-D scene panel.
5. Display current frame and geometric case.
6. Dynamically build controls from `get_editable_params()`.
7. Connect controls to propose → preview flow.
8. Display local validation status.
9. Connect Accept/Restore Frame/Restore All actions.
10. Display lock/accepted/invalidated frame state.
11. Ensure UI refreshes after cascade invalidation.
12. Add UI/session synchronization test where feasible.
13. Run full UR5 manual notebook test.
14. Commit, push, document.

Suggested commit:

```text
stage10: guided DH editor UI
```

**Done when:** a user can load UR5 and complete a guided session without manually calling internal functions, while session state remains authoritative.

---

## Stage 11 — Global FK Validator

1. Define exact URDF FK joint order.
2. Define exact DH FK convention and offset handling.
3. Implement/call `urdf_fk(q)`.
4. Implement `dh_fk(q)`.
5. Build reusable CasADi functions.
6. Implement deterministic sample generation.
7. Include zero pose.
8. Sample 20–50 configurations by default.
9. Implement position error.
10. Implement SO(3) geodesic orientation error with clipping.
11. Define configurable tolerances.
12. Produce per-sample metrics.
13. Produce summary metrics.
14. Validate automatic UR5 model.
15. Diagnose any mismatch before validating edited sessions.
16. Validate one complete edited UR5 session.
17. Implement per-frame error attribution per §20.8 so failures point to a specific frame.
18. Commit, push, document.
19. Advisor checkpoint.

Suggested commit:

```text
stage11: global URDF-FK vs DH-FK validator
```

**Done when:** automatic and one fully edited UR5 model pass agreed tolerances with a reproducible report.

---

## Stage 12 — Export, Persistence, and Logging

1. Finalize output schema version `1.0`.
2. Implement `schemas.py`.
3. Implement DH model export.
4. Implement edit-history export.
5. Implement validation JSON export.
6. Implement validation Markdown report.
7. Save configuration snapshot.
8. Save/reproduce validation sample set.
9. Capture git commit hash and working-tree-clean status per §10.1 in every report.
10. Implement schema/version checks on reload.
11. Implement Python logging configuration.
12. Add meaningful session and validation log events.
13. Add serialization round-trip tests.
14. Commit, push, document.

Suggested commit:

```text
stage12: versioned export and structured logging
```

**Done when:** a validated session can be saved, reloaded, audited, and supplied to downstream code without loss of essential information.

---

## Stage 13 — End-to-End UR5 Application

1. Implement/finish `app.py` orchestration.
2. Ensure user starts from `.urdf` selection.
3. Run input validation.
4. Run parser.
5. Run automatic DH solution.
6. Open editor.
7. Complete sequential edit session.
8. Run global validation.
9. Block final export if validation fails unless explicitly exporting a marked failed report for debugging.
10. Export validated result.
11. Add end-to-end integration test for non-UI core path.
12. Commit, push, document.

Suggested commit:

```text
stage13: end-to-end UR5 workflow
```

**Done when:** UR5 can travel from `.urdf` input to validated exported DH model in one coherent workflow.

---

## Stage 14 — Research Ablation Studies

1. Run threshold-sensitivity ablation.
2. Record classification changes, if any.
3. Run legal edit-range sweep.
4. Record FK error across the complete tested legal range.
5. Run sample-density study at 10, 50, 200 samples.
6. Build near-boundary synthetic case.
7. Record results and interpretation.
8. Save plots/tables if useful for the paper.
9. Commit, push, document.

Suggested commit:

```text
stage14: ablation and error-analysis results
```

**Done when:** all planned ablations have reproducible recorded results.

---

## Stage 15 — Generalization to a Second Robot

1. Select a structurally different supported serial robot.
2. Prefer a 7-DOF arm or a robot exercising a different supported joint pattern.
3. Run URDF validation.
4. Run parser and automatic DH solver.
5. Run visualization.
6. Run classifier.
7. Run editor.
8. Run global validation.
9. Export result.
10. Identify any UR5-specific assumptions uncovered.
11. Remove those assumptions from production code.
12. Add second-robot fixture/tests where appropriate.
13. Run the human usability check per §23.5 with someone other than yourself.
14. Record usability findings in `docs/stages/02_dh_editor.md`.
15. Commit, push, document.

Suggested commit:

```text
stage15: second-robot generalization validation
```

**Done when:** second robot passes the supported full pipeline and no hidden UR5-specific production assumptions remain.

---

## Stage 16 — Stronger Invariant and Regression Testing

1. Add orthonormal-rotation invariant tests.
2. Add accepted-frame R1–R3 invariant tests.
3. Add automatic-model immutability fuzz sequence.
4. Add upstream-stability/downstream-invalidation tests.
5. Add deterministic-seed test.
6. Add export/reload round-trip test.
7. Add regression fixtures for every bug discovered so far.
8. Commit, push, document.

Suggested commit:

```text
stage16: invariant and regression test suite
```

**Done when:** key correctness claims are directly represented as automated tests.

---

## Stage 17 — Continuous Integration

1. Create `.github/workflows/tests.yml`.
2. Install package/dependencies on `ubuntu-latest`.
3. Run pytest.
4. Run mypy/pyright according to project policy.
5. Run headless global-validator tests.
6. Use Xvfb only for tests that truly require rendering.
7. Add CI badge to README.
8. Confirm failing test causes failing workflow.
9. Fix it and confirm green workflow.
10. Commit, push, document.

Suggested commit:

```text
stage17: continuous integration pipeline
```

**Done when:** every push/PR automatically reports meaningful project health.

---

## Stage 18 — Code Quality and Interface Review

1. Run full type checker.
2. Resolve type errors.
3. Add missing public docstrings.
4. Audit production code for UR5 constants.
5. Audit duplicated threshold values.
6. Audit UI/domain coupling.
7. Audit logging quality.
8. Audit export schema stability.
9. Review entire diff as if reviewing another developer’s PR.
10. Merge integration branch after review.
11. Commit/push any final fixes.

Suggested commit:

```text
stage18: code quality and interface review
```

**Done when:** type checks are clean to agreed standard, documentation is complete, and architecture still matches this specification.

---

## Stage 19 — Final Worked Notebook

Create/finalize:

```text
examples/ur5_full_pipeline.ipynb
```

It must demonstrate, in order:

1. imports/config,
2. URDF selection/path,
3. URDF validation,
4. parsing,
5. automatic DH generation,
6. automatic DH display,
7. 3-D scene,
8. case classification,
9. guided editing,
10. final DH table,
11. global FK validation,
12. validation metrics,
13. export paths/artifacts.

Run it top to bottom in a fresh environment/kernel.

Commit, push, document.

Suggested commit:

```text
stage19: complete UR5 example notebook
```

**Done when:** notebook runs top to bottom without manual code repair.

---

## Stage 20 — Final Documentation and Research Record

1. Finalize architecture diagram.
2. Document all geometric cases.
3. Document legal edit parameters.
4. Document R1–R3 validation.
5. Document state-machine rules.
6. Document configuration defaults.
7. Document FK metrics/tolerances.
8. Add UR5 results.
9. Add ablation results.
10. Add second-robot results.
11. Add limitations.
12. Add known failure modes.
13. Add reproducibility instructions.
14. Write paper-ready contribution paragraph with actual measured numbers.
15. Commit, push.

Suggested commit:

```text
stage20: final research documentation
```

**Done when:** another researcher can understand what was built, why it is correct, how it was validated, and how to reproduce the results.

---

## Stage 21 — Final Integration and Release

1. Create a fresh virtual environment if practical.
2. Install project from repository.
3. Run full tests with zero unexplained skip/xfail.
4. Run type checker.
5. Run UR5 example notebook.
6. Run second-robot validation.
7. Generate final versioned validation reports.
8. Confirm exported schema/version.
9. Verify README setup instructions.
10. Verify CI green on release commit.
11. Tag release:

```powershell
git tag v1.0-dh-editor
git push --tags
```

12. Create release notes.
13. Commit/push final documentation if needed.

**Done when:** release tag corresponds exactly to the documented, tested project state.

---

## Stage 22 — Handoff and Maintenance Plan

1. Write `HANDOFF.md` or equivalent section.
2. Document stable public interfaces.
3. Document output schema consumed by downstream modules.
4. Document known limitations.
5. Document unsupported robot topologies.
6. Document deferred improvements such as desktop GUI/CAD meshes.
7. Document how to add a new robot fixture.
8. Document how to change tolerances safely.
9. Confirm advisor/team can access repository and release.
10. Record final acknowledgement/acceptance.
11. Commit and push final handoff state.

Suggested commit:

```text
stage22: handoff and maintenance documentation
```

**Done when:** another developer can integrate and maintain the module without relying on undocumented chat history.

---

# 34. Final User Experience

The completed v1 should feel like this:

```text
1. User launches URDF2DT.
2. User selects robot .urdf.
3. Program validates the URDF.
4. Program parses the robot.
5. Program automatically generates Standard-DH parameters.
6. Program shows robot and DH frames in 3-D.
7. Program classifies each axis pair.
8. Program unlocks the first DH frame.
9. User sees only legal edit controls.
10. User previews and accepts an edit.
11. Program validates the edit.
12. Program continues frame by frame.
13. Program globally compares URDF FK and DH FK.
14. If validation fails, user returns to the editor/restores.
15. If validation passes, program exports the validated model and report.
```

The user should **not** be required to understand or manually enter every DH parameter just to start the application. Their primary initial responsibility is to provide the robot URDF; the system handles extraction, constraint discovery, guidance, and verification.

---

# 35. Final Research Success Criteria

The project can claim success only if the evidence supports all of the following:

1. The automatic UR5 DH representation matches accepted reference results.
2. Geometric case classification matches the project reference methodology.
3. The editor exposes only legal frame modifications.
4. Local DH constraints reject illegal modifications reliably.
5. The automatic baseline remains immutable.
6. Cascade invalidation preserves dependency correctness.
7. Legal edits remain globally FK-equivalent to the source URDF within the configured tolerance.
8. The result is reproducible from recorded configuration/sample data.
9. The approach works on at least one structurally different supported robot.
10. Another developer/researcher can reproduce and inspect the results from the repository and exported artifacts.

---

# 36. Deferred v2 Opportunities

These are explicitly not required to finish v1 but may be considered later:

- native desktop GUI,
- browser/web application frontend,
- full CAD mesh visualization,
- branching-tree support,
- closed-loop mechanisms,
- hardware-in-the-loop editing,
- richer downstream dynamics integration,
- multi-end-effector support,
- interactive comparison of Standard vs Modified DH conventions,
- session replay visualization,
- automatic report figures for publications.

Do not allow these v2 ideas to delay completion and validation of the v1 research contribution.

---

# 37. One-Sentence Project Summary

> **URDF2DT’s Interactive DH-Frame Editor takes a robot URDF, automatically derives a Standard-DH model, lets the researcher modify each DH frame only within mathematically legal geometric constraints, and proves through local DH-rule checks and global forward-kinematics validation that the edited model remains equivalent to the original robot.**

