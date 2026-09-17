# URDF2DT — Interactive DH-Frame Editor
## Revised Full Project Specification and End-to-End Development Plan

**Module scope:** guided, constraint-aware Standard-DH frame assignment editor with 3-D visualization, validation, persistence, and a clear URDF-driven user workflow inside the larger URDF2DT toolbox.

**Primary reference platform:** Universal Robots UR5 (6-DOF serial manipulator).

**Primary user input:** a robot `.urdf` file.

**Primary user output:** a validated Standard-DH model, edit history, and FK-validation report suitable for downstream URDF2DT modules.

**Full project scope:** mandatory Stages 0–31 extend that kinematic foundation through validated dynamics, parameter identification, trajectories, control, simulation, hardware validation, simulation-to-real comparison, and a physically synchronized digital twin. Stage 23 is the initial standalone kinematic release; Stage 31 is the final integrated `URDF2DT.exe` release and project completion gate. These are planned requirements, not claims of existing implementation.

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

Automatic Standard-DH generation, constraint-aware frame editing, geometric validation, and FK-equivalence validation remain the primary research contribution. Mesh, color, and collision support improve usability, robot understanding, visual quality, and future extensibility; appearance fidelity is not evidence of kinematic correctness.

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
- native desktop UI as the supported end-user interface for the Windows release,
- Jupyter-based interactive UI as an optional research/development workflow,
- headless validation in CI.

Also required for v1: independent extraction of visual, collision, inertial, material, and joint-metadata domains; URDF primitive and supported mesh rendering with link-local origins, mesh scale, and colors; independently toggled collision overlays; and basic rendering fallback for unavailable assets. Inertial extraction and storage do not imply dynamic-model generation or validation.

Out of scope for v1:

- closed-loop robots,
- general branching kinematic trees,
- arbitrary CAD authoring/import beyond supported URDF-referenced mesh formats,
- full collision detection, motion planning, and mandatory texture rendering,
- real-time hardware-in-the-loop editing,
- unconstrained 6-DOF manual frame dragging,
- machine-learning-based frame placement.

---

# 2. Required Reference Material and Open Dependencies

Full project completion also requires access to a supported physical robot, its documented control/sensor interfaces, laboratory operating procedures, and suitable measurement/calibration equipment for Stages 29–31. Resolve hardware access and required signals early; unavailable hardware prevents full completion but does not prevent delivery of the Stage 23 milestone.

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
| FR-19 | Extract every supported `<visual>` element into typed data associated with its owning link, including multiple visuals per link. |
| FR-20 | Resolve relative, configured `package://`, and local absolute mesh references through one asset resolver; report unresolved references by link and element. |
| FR-21 | Support mesh, box, cylinder, and sphere visual geometry; apply mesh scale once and compose each visual origin with its owning link pose. |
| FR-22 | Extract inline and globally named materials and RGBA colors; use neutral defaults for missing/unsupported appearance information. Textures are optional. |
| FR-23 | Render an optional realistic robot geometry layer that follows link FK while retaining frames, axes, and bone fallback. |
| FR-24 | Extract and store collision geometry independently of visuals, preserving link association, multiple elements, local origins, and mesh scale. |
| FR-25 | Offer a separately toggled transparent/wireframe collision overlay; never silently substitute collision geometry for normal visuals. |
| FR-26 | Extract mass, inertial origin, center-of-mass offset, and all six independent inertia-tensor fields without running DH generation. |
| FR-27 | Separate fatal structural errors from asset warnings and incomplete inertial data; continue valid kinematic editing and DH/FK validation without optional assets or inertia. |
| FR-28 | Independently toggle visual geometry, collision geometry, URDF frames, DH frames, and joint axes; support link-name toggles if labels are provided. |
| FR-29 | Extract joint limits and metadata through a dedicated component and make reusable domain contracts available independently to downstream consumers. |
| FR-30 | Generate and validate inverse/forward dynamics from validated kinematics and independent inertial properties (Stage 24). |
| FR-31 | Implement and evaluate dynamic parameter identification, retaining a trusted-parameter execution mode (Stage 25). |
| FR-32 | Generate constrained joint-space and Cartesian trajectory references (Stage 26). |
| FR-33 | Implement and test the required feedback and model-based controllers (Stage 27). |
| FR-34 | Integrate a reproducible simulation environment and controller evaluation workflow (Stage 28). |
| FR-35 | Validate approved trajectories on a physical robot with calibrated, timestamped measurements and operating limits (Stage 29). |
| FR-36 | Quantify simulation-to-real deviation and evaluate parameter/model updates on held-out physical data (Stage 30). |
| FR-37 | Demonstrate live physical-to-virtual state synchronization, monitoring, validated model updating, and the final integrated application release (Stage 31). |

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
| NFR-13 | Extraction responsibilities must be logically independent and have no circular subsystem dependencies; adapters must reuse existing contracts where possible. |
| NFR-14 | The DH solver and validation core must be able to execute without loading any mesh, color, or collision asset, and without requiring inertial data. |
| NFR-15 | Typed domain contracts must be usable headlessly and must not contain renderer actors, GUI state, or required filesystem operations. |
| NFR-16 | Unavailable optional assets must cause structured, actionable diagnostics and graceful degradation, without changing valid kinematic results. |
| NFR-17 | Asset resolution and scale handling must have one owner and deterministic behavior across source and packaged execution. |
| NFR-18 | Cache immutable loaded mesh data where useful; do not reload files on each FK update or mutate shared cached geometry per actor. |
| NFR-19 | Rendering must consume read-only model/pose snapshots and leave kinematic, inertial, and accepted editor state unchanged. |

---

# 4. Improved System Architecture

The following diagram shows the existing kinematic/editor path. Its Module 1 parser is the kinematic adapter of the modular URDF architecture in §4.2; visual, collision, material, and inertial extraction are independent branches, not prerequisites of DH generation. The UI layer includes the native desktop interface and optional research notebook controls.

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

### Rule F — Extraction modules are independent

Kinematics, visual geometry, collision geometry, inertial properties, joint metadata, and material definitions must have logically independent extractors over the same safely parsed URDF snapshot. They may share pure numeric/transform utilities and stable link/joint identifiers. They must not require another domain to finish successfully unless a specific data dependency is declared, such as visual appearance resolving a material name.

### Rule G — No circular subsystem dependencies

The DH module must not import visualization or asset loading. Inertial extraction must not depend on DH generation. Collision extraction must not depend on visualization. Renderers must not import or query `EditorSession` as their data source: application orchestration supplies immutable geometry, link poses, and DH display snapshots after session transitions. `EditorSession` remains authoritative for edits, while source kinematics remain authoritative for physical link motion.

### Rule H — Graceful partial data

A robot with valid supported kinematics remains usable when visual meshes, collision meshes, or inertial properties are unavailable. Report kinematic, visual, collision, material, and inertia availability separately. Domain warnings must not become a single global failure flag that blocks DH work. Missing inertia is not a zero-mass model, and a missing mesh is not invalid kinematics.

### Rule I — Single ownership of responsibilities

Mesh path resolution and asset preparation belong to `assets/`; DH mathematics to `dh/`; inertia parsing to the inertial extractor; material definitions to the material extractor; and rendering to `visualization/`. Extractors preserve data and references without loading meshes. The renderer consumes resolved assets without implementing filesystem search. A shared core XML snapshot is allowed; a mandatory monolithic robot object passed to every subsystem is not.

## 4.2 Modular URDF Processing and Data Flow

```text
URDF File → Safe Core Parser / Structural Validation → Read-only URDF snapshot
                                                        │
                           Independent extractors       │
              ┌─────────────────────────────────────────┤
              ├─ Kinematic Structure → KinematicChain → Automatic DH Solver
              ├─ Joint Limits / Metadata → JointMetadata → FK sampling / UI
              ├─ Visual Geometry → LinkVisual / VisualGeometry ───────┐
              ├─ Material / Color → Material ────────────────────────┤
              ├─ Collision Geometry → CollisionGeometry ───────┐     │
              └─ Inertial Parameters → InertialModel           │     │
                                         │                    │     │
                                 Dynamics (Stage 24)              │     │
                       (validated kinematics + inertia)       │     │
                                                              ▼     ▼
Mesh references → Asset Resolver / Cache → Resolved Assets → Layered Renderer
KinematicChain → FK link poses ────────────────────────────→ Layered Renderer
DH display snapshots ─────────────────────────────────────→ Frame Overlays

CollisionGeometry → Optional Collision Overlay / Future Collision Engine
```

The core parser owns safe XML decoding and source provenance. The kinematic adapter returns `KinematicChain` independently; each other extractor returns its own typed collection and diagnostics. `RobotDescription` may group these results for application orchestration, but is an optional aggregate rather than a mandatory subsystem input. Existing parser/solver protocols should remain usable through adapters. Asset resolution may run after kinematic validation without blocking the core chain-to-DH path.

```text
KinematicExtractor(URDF snapshot) → KinematicChain
DHGenerator(KinematicChain) → DHModel
InertialExtractor(URDF snapshot) → InertialModel
Dynamics(kinematics=validated KinematicChain,
               inertia=InertialModel,
               dh=optional validated DHModel with explicit link-frame mapping)
```

The DH solver must not parse or own inertia. The Stage 24 dynamics module must receive independent inertial properties alongside validated kinematics; a DH table alone does not contain link masses or inertial-frame mappings. This architecture specifies planned responsibilities, not a claim that all extractors already exist.

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
| Desktop controls | PySide6 + PyVista/pyvistaqt | Native, packageable interface for the final Windows application |
| Research controls | ipywidgets + JupyterLab | Optional notebook workflow for experiments and reproducible demonstrations |
| Data structures | `dataclasses`, often `frozen=True` | Explicit contracts and immutability |
| Configuration | YAML + typed Python config object | Reproducible, centralized thresholds |
| Tests | pytest | Project-wide automated testing |
| Type checking | mypy or pyright | Catch data-contract mistakes |
| Logging | Python `logging` | Reproducible debug/action records |
| Export | JSON + Markdown report | Machine-readable + human-readable outputs |
| CI | GitHub Actions | Test every push/PR |

## 5.1 Explicitly Rejected v1 Alternatives

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
- **Storage:** text-based URDF, JSON, YAML, Markdown, and notebooks, plus potentially larger external mesh assets; cache usage should be bounded and documented.
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

This is the target organization, not an inventory of completed modules. Preserve existing `KinematicChain`/`DHModel` contracts in `dh/types.py`, parser/solver protocols in `interfaces.py`, FK in `kinematics.py`, and source handling in `parser/urdf_input.py`. Adapt existing `parser/robot_document.py` handling into these responsibilities instead of maintaining a second resolver or duplicate robot model. The conceptual `RobotDescription` may be an adapter around existing `RobotDocument` orchestration; neither should become a mandatory input to every extractor. The simulation folder supports required Stage 28; add modular dynamics, identification, trajectory, control, hardware, and twin integration packages for Stages 24–31, reusing the existing shared contracts.

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
│   ├── interfaces.py
│   ├── kinematics.py
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
│   │   ├── urdf_input.py
│   │   ├── robot_document.py
│   │   ├── kinematic_extractor.py
│   │   ├── visual_extractor.py
│   │   ├── collision_extractor.py
│   │   ├── inertial_extractor.py
│   │   ├── material_extractor.py
│   │   ├── joint_extractor.py
│   │   └── kinematic_graph.py
│   │
│   ├── assets/
│   │   ├── __init__.py
│   │   ├── mesh_resolver.py
│   │   └── asset_cache.py
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   ├── robot_description.py
│   │   ├── geometry.py
│   │   ├── visual.py
│   │   ├── collision.py
│   │   ├── inertial.py
│   │   ├── material.py
│   │   └── joint_metadata.py
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
│   │   ├── scene.py
│   │   ├── robot_renderer.py
│   │   ├── mesh_renderer.py
│   │   ├── collision_renderer.py
│   │   └── frame_renderer.py
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
│   ├── test_visual_extractor.py
│   ├── test_collision_extractor.py
│   ├── test_inertial_extractor.py
│   ├── test_mesh_resolver.py
│   ├── test_modular_extraction.py
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

- `parser/` owns safe source handling and independent domain extraction.
- `models/` owns new reusable geometry, material, inertia, and metadata contracts; reuse existing kinematic/DH types rather than creating competing versions.
- `assets/` owns reference resolution, scale preparation, diagnostics, and mesh caching.
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

## 11.1 Independent URDF Domain Contracts

Extend rather than duplicate existing contracts. Use immutable values/read-only collections and shared stable link/joint identifiers; represent absence explicitly rather than inventing required values for every link.

| Concept | Required meaning and fields when present |
|---|---|
| `KinematicChain` | Existing ordered topology, link/joint identities, joint types, axes, and transforms; usable without geometry or inertia. |
| `MeshReference` | Original URI/filename and three-component scale, defaulting to `(1, 1, 1)` when omitted; keep resolution results separate from source references. |
| `VisualGeometry` | Tagged mesh, box, cylinder, or sphere geometry; mesh reference or primitive dimensions, with no renderer actors. |
| `Material` | Optional name, RGBA color, and optional texture reference; global definitions and inline values retain source provenance. |
| `LinkVisual` | Owning link name, stable element index/ID, link-local origin transform, `VisualGeometry`, and optional material name/inline material. Store a collection per link. |
| `CollisionGeometry` | Owning link, element index/ID, independent local origin, tagged primitive or mesh reference and its scale. Store a collection per link independently of visuals. |
| `InertialProperties` | Owning link, mass, inertial-origin translation and rotation, COM offset, and symmetric 3-by-3 tensor assembled from `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`. Preserve the tensor's reference frame. |
| `InertialModel` | Link-keyed inertial properties plus missing/incomplete/invalid status; does not require a `DHModel`. |
| `JointMetadata` | Joint name/type and available position, velocity, effort limits and other supported metadata. Retain absence and use the existing chain's documented joint order for consumers. |
| `RobotDescription` | Optional orchestration aggregate of domain results, source provenance, and per-domain diagnostics; never the mandatory DH/renderer input contract. |

Use the URDF inertial-origin translation as the COM offset in the link frame, and retain inertial-origin rotation for interpreting tensor axes. Distinguish absent inertia from malformed supplied values; report both without disabling otherwise valid kinematics. Geometry collections may be empty. Apply documented URDF origin defaults when an element omits its origin, but do not fabricate missing mass or tensor values. Reuse shared numeric transform utilities without importing GUI or DH-generation code into the extractors.

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

## 12.5 Fatal Errors, Domain Availability, and Asset Warnings

Return separate structured `fatal_errors`, `warnings`, and `domain_status` results. Each diagnostic should identify severity, domain, code, link/joint name, element ID, original asset reference where relevant, and an actionable message. Structural checks run before optional external assets are loaded.

| Condition | Required result |
|---|---|
| Malformed XML, invalid link/joint references, unsupported kinematic topology or required joint data | Fatal for the affected kinematic workflow; do not generate DH. |
| Missing/unreadable visual mesh, unknown package mapping, unsupported mesh format or texture | Non-fatal visual/asset warning; skip the affected geometry and use the documented fallback. |
| Missing collision mesh | Non-fatal collision warning; retain other geometry and continue DH/FK work. |
| Absent, incomplete, or invalid inertia | Mark inertia availability separately; do not block the kinematic editor or present the data as ready for dynamics. |
| Invalid optional visual/collision origin, dimensions, scale, or material values | Warn and exclude the invalid element or use the documented neutral appearance; do not silently corrupt kinematics. |

Example: `Visual asset unavailable: link 'link_1', visual[0], 'package://demo_robot/meshes/link_1.stl'. Configure package 'demo_robot'; using basic link rendering.` A successful kinematic result may coexist with asset warnings. Optional-data errors must not bypass existing XML security or topology requirements.

---

# 13. Automatic DH Solution Contract

The DH solver accepts kinematics and its existing configuration only. It must neither parse inertial data nor load visual/collision assets or colors; missing meshes, collision geometry, or inertia must not change its numerical output for the same kinematic chain.

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

## 18.1 Layered Robot Rendering and Geometry Transforms

Retain frame triads, joint axes, DH frames, selection highlights, and simple link/bone rendering for debugging. Add URDF `<visual>` geometry as a separately toggled realistic robot layer, supporting `<mesh filename="..." scale="...">`, `<box>`, `<cylinder>`, and `<sphere>`. Support multiple visual elements per link with independent origins and materials. A link without usable visuals should have a basic link/bone fallback where geometrically meaningful.

Geometry must attach to its owning URDF link, never to a DH frame selected for editing. For a local mesh point, the displayed transform is conceptually:

```math
p_{world} = T_{world,link}(q)\,T_{link,visual}\,S_{mesh}\,p_{mesh}
```

Here `T_link,visual` comes from `<origin xyz="..." rpy="...">` and `S_mesh` is the homogeneous mesh scale. Primitive dimensions are used directly. Scale applies to local mesh coordinates before the visual origin, not to the link pose or origin translation. The asset layer prepares scaled mesh data once; the renderer applies the link and element transforms without applying scale a second time. Recompute actor poses from immutable geometry and current FK rather than accumulating incremental transforms.

Joint motion updates every visual and collision element attached to the moving link and downstream links. A legal DH-frame edit changes DH overlays, not the physical URDF geometry at the same joint configuration. Independent display controls cover visual robot geometry, collision geometry, URDF frames, DH frames, joint axes, and link names if provided. Layer visibility must not alter model state or validation results.

## 18.2 Mesh Asset Resolution and Caching

Use a dedicated `urdf2dt/assets/mesh_resolver.py` interface, adapting existing asset-resolution code into a single owner. It receives the original URDF location, original reference, explicit package/project mappings, and geometry scale. It resolves and prepares assets for the renderer; extraction itself only records references.

- Resolve relative paths against the source URDF directory first. If configured project/package-root fallback is supported, use a documented ordered list of explicit roots and record which resolved the asset; never depend on the process working directory.
- Resolve `package://package_name/path` using an explicit `package_name → package_directory` mapping. Allow the user to select a package directory and retain the mapping; unknown packages produce warnings rather than guessed matches.
- Accept readable local absolute paths and preserve their original references for diagnostics. Do not assume these paths remain portable across machines.
- Do not require ROS installation or assume meshes are embedded in `URDF2DT.exe`. Remote asset fetching is outside this v1 resolver contract.
- Define and test supported formats during Stage 6; STL is the minimum mesh baseline. Other formats, such as OBJ or DAE, are supported only when a tested loader is available in both source and packaged execution. Unsupported formats produce a clear warning and fallback.
- Apply finite, valid mesh scales once through the asset-preparation interface; record scale policy and test nonuniform scaling. Reject invalid optional geometry through domain warnings.
- Cache immutable geometry using resolved asset identity, file version/fingerprint, loader settings, and scale as appropriate. Keep link-specific transforms and materials on separate actors so shared meshes cannot leak changes between links. Invalidate cache entries after file or mapping changes and bound cache memory.
- Return structured load status, resolved path, and provenance; catch missing, unreadable, corrupt, and unsupported asset failures. Identify the owning link, element, and original URI in warnings. Continue the DH/kinematic workflow and use basic rendering for unavailable visuals.

The renderer consumes resolved assets and diagnostics, never performs its own filesystem search. Development and standalone execution must use the same resolution rules, including when a robot directory is moved or is outside the application directory.

## 18.3 URDF Materials and Colors

The material extractor parses inline `<material>` definitions, `<color rgba="r g b a">`, and globally named materials referenced by visual elements. An explicit inline definition supplies that visual's appearance; a name-only reference uses the global definition. Diagnose unknown names or conflicting global definitions and use a neutral fallback when appearance cannot be resolved. Validate finite RGBA components in the supported range and honor alpha where supported by the renderer.

Retain texture references if present, but texture loading/rendering may remain deferred. A missing or unsupported texture must not hide otherwise usable geometry. Default to a neutral material when neither supported color nor appearance information exists. Colors, transparency, and material lookup must have no effect on DH generation or FK validation.

## 18.4 Independent Collision Geometry

Parse `<collision>` independently of `<visual>` and retain multiple elements per link. Support mesh, box, cylinder, and sphere families through the shared geometry/asset utilities while retaining separate collision records, origins, and actors. For collision rendering use `T_world,link(q) T_link,collision` and the collision mesh's own scale; do not reuse a visual element's transform or dimensions.

Provide an optional transparent or wireframe overlay, distinguishable from normal visual geometry, with its own toggle. Collision geometry must not become the normal robot visual model unless the link has no usable visual geometry and the user explicitly enables that fallback. The default missing-visual fallback remains basic link/bone rendering. Hiding visuals must not automatically substitute collision geometry.

Current v1 covers collision parsing, storage, correct link association, an optional display layer, and reusable downstream data. Full collision detection, contact dynamics, and motion planning are future work; displaying collision shapes does not certify collision-free motion.

## 18.5 Example URDF and Independent Routing

The following illustrative link fragment includes all four domains; it is not a complete robot. A documented executable demonstration should embed it in a supported serial-chain URDF with a locally supplied `meshes/link_1.stl` asset. The values are illustrative, not manufacturer measurements.

```xml
<link name="link_1">
  <visual>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <mesh filename="meshes/link_1.stl" scale="1 1 1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.10"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.0003"/>
  </inertial>
</link>
```

```text
visual → VisualExtractor → LinkVisual → Asset Resolver → Renderer
material/color → MaterialExtractor → Material → Renderer
collision → CollisionExtractor → Collision Model → Optional Collision Overlay
inertial → InertialExtractor → InertialModel → Dynamics (Stage 24)
link/joint structure → KinematicExtractor → KinematicChain → DH Solver
```

The visual mesh and collision box intentionally differ. The renderer uses each element's own link-local origin; mass/tensor data never pass through the mesh loader or DH solver. Include a name-only global-material variant, nonidentity origins, nonuniform mesh scale, and multiple geometry elements in the associated demonstration fixtures. Removing the mesh must demonstrate warning/fallback behavior without changing the DH/FK result.

---

# 19. Interactive UI

The v1 UI is a thin native desktop layer for the Windows release, with an optional Jupyter/ipython widget layer for research. Both consume the same domain and scene interfaces.

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

## 19.4 Geometry Layers and Asset Status

Expose separate toggles for visual geometry, collision overlay, URDF frames, DH frames, and joint axes, plus link names where supported. Show domain availability and actionable asset warnings with the affected link and path. Provide package-directory mapping controls without requiring development tools. Keep warnings visible without blocking valid DH editing, FK validation, or export. Display controls must only change the view; the application supplies scene snapshots from authoritative model/session state.

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

## 21.5 Independent Domain Persistence and Asset Provenance

Preserve reusable visual, collision, material, joint-metadata, and inertial records through versioned optional sidecar artifacts or a backward-compatible documented schema extension. Record element-to-link associations, origins, mesh scales, original references, resolution mappings, and domain warnings. Keep DH exports valid without those sidecars or asset files; do not silently embed meshes or require a local absolute path to reload the kinematic model. Re-resolve relocated assets explicitly and report changed availability. Stored inertial data is extracted information, not a validated dynamic model.

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

## 23.6 Modular Extraction, Asset, and Rendering Tests

Add deterministic fixtures for the example in §18.5 and variants with missing assets, primitive-only visuals, global materials, and multiple visual/collision elements. Test domain extraction headlessly; use rendering integration checks only where a graphics context is necessary.

| Area | Required checks |
|---|---|
| Visual extraction | Mesh and each primitive assigned to the correct link; multiple visuals retained; local translation/rotation, default and nonuniform mesh scale, inline colors and global material references parsed correctly. |
| Collision extraction | Correct owning link, independent origin/scale, multiple collision elements, and mesh/box/cylinder/sphere support; visual changes must not alter collision records. |
| Inertia extraction | Correct mass and inertial translation/rotation; COM offset; all six tensor fields including nonzero off-diagonal entries; missing/incomplete data reported without crashing kinematics. |
| Asset resolution | Relative paths, explicit package mappings, absolute local paths, moved robot directories, spaces in filenames, missing/corrupt assets, unsupported formats, and unknown packages; results independent of working directory. |
| Scale and cache | Nonuniform scale applied once before element origin; repeated FK updates do not compound transforms; two links sharing a mesh retain independent poses/materials; file/mapping changes invalidate affected cache entries. |
| Modularity | DH generation and FK validation work with no visuals, no collision geometry, no inertia, and unavailable mesh paths; patch asset-loading calls to fail if the core tries to invoke them. Inertial extraction runs with the DH solver disabled. |
| State isolation | Renderer updates and layer toggles do not mutate kinematics, inertia, the automatic DH baseline, or accepted editor state. A legal DH-frame edit leaves physical link geometry fixed at the same `q`. |
| Warning behavior | Missing visual/collision assets produce structured link/path diagnostics and fallback while the valid chain still generates and validates DH. Fatal structural faults remain fatal. |
| Rendering | Numerically compare actor transforms with FK composed with each local origin at zero and nonzero configurations; verify color/alpha assignment and separate collision actors/toggles. Add visual smoke checks where feasible. |
| Persistence and packaging | Optional domain data round-trip with link identity and source references; missing sidecars/assets do not invalidate DH reload; source and executable resolution match for the Stage 23 cases. |

Use numerical transform assertions, not screenshots alone, for link attachment and FK motion. Test both revolute motion and supported prismatic translation. Preserve all existing geometric, state-machine, FK, and regression tests.

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
| Geometry attachment | Correct link and element transform at tested configurations | Numeric composition checks from §18.1 and §23.6 |
| Optional-data isolation | Identical DH/FK outputs with optional domains unavailable | Headless comparison and dependency tests |
| Asset handling | Correct resolution or actionable warning/fallback in all required cases | Source and packaged fixture matrix |

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

Also test missing/corrupt meshes, unknown package mappings, unsupported textures/formats, geometry attached to the wrong link, double-applied scale, incorrect visual/collision origins, unresolved material names, shared-cache mutation, and incomplete inertia. Report these by affected domain; optional-data failures must not mask or replace the kinematic validation result.

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

Cache reusable mesh geometry through the asset layer and update actor transforms without reopening files on each joint motion or editor preview. Keep asset I/O outside the DH computation path, record loading costs separately from FK/editor latency, and avoid duplicating large meshes for identical references unless per-instance data requires it.

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

The module completion gate also includes independent domain contracts and extraction, correctly attached primitive/mesh visuals with material support, distinct collision overlays and toggles, asset-warning fallback, and the modularity/transform checks in §23.6. Inertial extraction must work independently. This gate completes the kinematic module only. Dynamics and parameter identification are required later project stages. The initial Windows release must pass the external-asset cases in Stage 23, and the integrated final release must repeat those checks in Stage 31.

---

## 31.3 Full Project Definition of Done

The project is complete only when every Stage 0–31 completion gate is satisfied. Completion of the kinematic module or Stage 23 executable alone is not full project completion. Required evidence includes:

- verified kinematics and the independently usable standalone kinematic application;
- validated dynamic models and an implemented, benchmarked parameter-identification capability;
- constrained joint/Cartesian references, required controllers, and reproducible integrated simulation results;
- supervised experiments on at least one physical robot, with operating limits and calibrated, timestamped data;
- a quantitative simulation-to-real report and held-out evaluation of model/parameter updates;
- demonstrated live state synchronization, monitoring, and a validated model-update cycle with explicit latency, uncertainty, and disconnect behavior;
- a final integrated `URDF2DT.exe` release, regression results, datasets/configurations, documentation, and recorded acceptance.

Thresholds must be specified before evaluation and supported by recorded results (§35.3). Required stages must not be waived by relabeling them optional or by substituting simulated data for physical validation. Incomplete hardware access or unmet criteria must be recorded as unresolved project work. Advanced controller variants and the extensions in §36 are not all required.

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
20. **Modular URDF extraction layer** — independent kinematic, visual, collision, inertial, material, and joint-metadata responsibilities over a safe source snapshot.
21. **Typed reusable robot-description contracts** — domain collections with stable link/joint identifiers and explicit partial-data status.
22. **Visual geometry and material extraction** — meshes/primitives, origins, scales, inline/global colors, and neutral appearance fallback.
23. **Collision extraction and visualization** — separate reusable geometry records and an optional distinguishable overlay.
24. **Independent inertial extractor** — mass, COM/inertial origin, and full symmetric inertia tensor for the Stage 24 dynamics consumer; extraction alone is not dynamic validation.
25. **Mesh asset resolver and cache** — deterministic external-asset resolution and structured warnings in source and packaged execution.
26. **Layered realistic renderer** — URDF visual geometry, independent toggles, FK-following link attachments, and retained basic debugging/fallback rendering.
27. **Documented geometry demonstration and verification record** — example from §18.5, domain-routing explanation, missing-asset fallback, and Stage 23 external-asset results.
28. **Standalone Windows distribution** — `URDF2DT.exe` and release ZIP verified on a clean supported Windows system, including external robot assets.
29. **Validated dynamics module** — inverse/forward dynamics, independent parameter inputs, and verification reports.
30. **Parameter-identification module** — estimator, trusted-parameter mode, provenance, benchmark evidence, and physical-data evaluation.
31. **Trajectory generator** — joint/Cartesian references, motion constraints, and feasibility tests.
32. **Controller layer** — required baselines and model-based controller with configuration and regression evidence.
33. **Integrated simulator** — documented backend, reproducible scenarios, tracking metrics, and sensitivity analysis.
34. **Hardware interface and validation record** — calibrated measurements, operating limits, and physical experiment datasets.
35. **Simulation-to-real study** — matched comparisons, parameter/model updates, held-out evaluation, and residual uncertainty.
36. **Synchronized digital twin** — live state synchronization, monitoring, validated versioned model updates, and connection-failure handling.
37. **Final integrated release and handoff** — Stage 31 `URDF2DT.exe`, documented external dependencies, reproducibility package, and project acceptance.

---

# 33. Master Development Plan — Start to Finish

The following Stage 0–31 sequence is mandatory unless the advisor explicitly changes dependencies. Stages 0–23 deliver the initial kinematic application; Stages 24–31 are required to finish the full project. The earlier restriction that Stage 23 must be the final project stage is superseded by this expanded scope. No stage is claimed complete merely because it is listed here.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Define the independent domain contracts in §11.1, per-domain diagnostics, and extractor protocols; preserve the existing parser/solver adapter interfaces.
- Specify package mappings, asset resolution/scale policy, and optional-domain persistence without introducing mesh or inertia dependencies into DH.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Implement the core snapshot and structured fatal-error versus warning/domain-status contract in §12.5. Do not perform mandatory mesh loads in structural validation.
- Add valid-kinematics fixtures with missing visual/collision assets and missing inertia; confirm kinematic acceptance while optional availability is reported separately.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Integrate logically independent kinematic, visual, collision, inertial, material, and joint-metadata extractors over the safely parsed source snapshot (§4.2). Reuse existing contracts and utilities rather than duplicating the parser.
- Store multiple visual/collision elements with stable link association, origins, scales, materials, and independent inertial records.
- Prove that the DH solver requires only kinematics/configuration and produces the same result without mesh, collision, or inertia data; run inertial extraction without the solver.
- Add headless extraction fixtures and tests from §23.6 before renderer integration.

Suggested commit:

```text
stage5: parser and automatic DH integration
```

**Done when:** UR5 produces a reference-compatible immutable automatic DH model through the project interface. Independent extraction contracts and headless tests also pass, including inertial extraction without DH and DH generation without optional geometry or inertia.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Evolve StaticScene into the layered renderer in §18 while retaining triads, joint axes, DH frames, and basic link/bone debugging and fallback.
- Implement the dedicated asset resolver/cache and test relative paths, package mappings, absolute paths, supported-format loading, and missing/corrupt-asset diagnostics. Adapt the existing resolver into this owner.
- Render box, cylinder, sphere, and supported mesh visuals, including multiple visuals per link; apply mesh scale once and each visual origin before composing with link FK.
- Resolve inline/global URDF colors and neutral defaults; keep optional textures non-blocking.
- Add collision geometry as separate transparent/wireframe actors with independent origins/scales, and expose scene-layer visibility controls.
- Test correct owning-link attachment, joint-motion updates, unchanged physical geometry after DH-frame edits, and no mutation of shared model/cache data.
- Demonstrate the example in §18.5 and fallback with its mesh unavailable; record supported mesh formats and numerical transform checks.

Suggested commit:

```text
stage6: static URDF and DH scene renderer
```

**Done when:** the layered scene renders and existing numeric origin tests pass; visual geometry is attached to the correct link and follows joint FK, scales/origins and colors are applied correctly, collision and visual geometry remain distinguishable and independently toggled, and the editor can still operate with basic fallback when visual assets are unavailable.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Connect independent visual/collision/frame/axis toggles and optional link labels to the scene API; add package-directory selection and per-domain availability/warnings (§19.4).
- Verify missing assets do not block editing or falsely change validation status. Pass immutable scene snapshots through application orchestration rather than giving renderers direct session ownership.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Preserve optional domain data and asset provenance through the versioned contract in §21.5; keep the DH schema usable without asset availability.
- Test domain sidecar round-trips and relocated/missing-asset reload; log link/element/path warnings separately from kinematic validation failures.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Exercise the complete workflow with available URDF visuals/materials, the independent collision overlay, and layer toggles.
- Repeat with missing meshes and absent inertia; verify usable basic rendering, the same kinematic result, and successful validated export.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Add §23.6 regressions for link attachment, nonidentity origins, nonuniform scale, global/inline materials, multiple visual/collision elements, and partial inertial data.
- Enforce core execution without asset loading and inertia extraction without DH; verify renderer/toggle operations cannot mutate authoritative model state.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Audit extractor independence, no circular imports, single ownership of resolution/scaling, reusable domain types, and read-only scene interfaces.
- Check that fallback/asset code cannot modify DH/FK output, and document the tested mesh format boundary.

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

**Integrated advisor requirements (complete before this stage is closed):**

Demonstrate independent extraction and routing using §18.5 or an equivalent complete serial URDF with a locally supplied visual mesh, color, collision shape, and inertia. Show joint-motion attachment, separate geometry toggles, package/relative asset resolution, and missing-mesh fallback. Explain that inertial extraction is available to Stage 24 dynamics but does not itself implement it.

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

**Integrated advisor requirements (complete before this stage is closed):**

- Document the modular data-flow diagram, domain ownership, supported geometry/material formats, external asset resolution, collision-overlay limits, and warning/fallback behavior.
- Record extraction/modularity/transform evidence and the example's domain routing; keep the DH/editor/FK research contribution distinct from appearance quality.

Suggested commit:

```text
stage20: final research documentation
```

**Done when:** another researcher can understand what was built, why it is correct, how it was validated, and how to reproduce the results.

---

## Stage 21 — Final Integration and Release Candidate

1. Create a fresh virtual environment if practical.
2. Install project from repository.
3. Run full tests with zero unexplained skip/xfail.
4. Run type checker.
5. Run UR5 example notebook.
6. Run second-robot validation.
7. Generate final versioned validation reports.
8. Confirm exported schema/version.
9. Verify README setup instructions.
10. Verify CI green on the release-candidate commit.
11. Record the exact candidate commit, dependency-lock files, and verification results.
12. Prepare draft release notes.
13. Commit/push final documentation if needed.

**Done when:** a documented, tested release candidate is ready for standalone Windows packaging.

---

## Stage 22 — Handoff and Maintenance Plan

1. Write `HANDOFF.md` or equivalent section.
2. Document stable public interfaces.
3. Document output schema consumed by downstream modules.
4. Document known limitations.
5. Document unsupported robot topologies.
6. Document deferred improvements such as browser UI/CAD-mesh enhancements.
7. Document how to add a new robot fixture.
8. Document how to change tolerances safely.
9. Confirm advisor/team can access repository and release.
10. Record final acknowledgement/acceptance.
11. Commit and push final handoff state.

**Integrated advisor requirements (complete before this stage is closed):**

- Document reusable visual/collision/inertial contracts, resolver mapping configuration, supported mesh formats, and how to diagnose missing external robot assets.
- Distinguish current URDF mesh/color/collision-overlay support from deferred textures, advanced CAD features, and collision engines; dynamics is required in Stage 24.

Suggested commit:

```text
stage22: handoff and maintenance documentation
```

**Done when:** another developer can integrate and maintain the module without relying on undocumented chat history.

---

## Stage 23 — Windows Standalone Application and Distribution

Make the native desktop application the supported delivery path for the initial kinematic release. A user must
be able to use URDF2DT on a supported Windows computer without installing Python,
VS Code, Git, or project dependencies.

1. Finalize the GUI entry point so it opens an empty native window and lets the user select a URDF.
2. Provide a stable product name and executable: `URDF2DT.exe`.
3. Audit source code, configuration, documentation, and runtime setup for developer-specific absolute paths.
4. Make configuration, icons, example robots, schemas, and any required runtime assets discoverable both from source and from a packaged executable.
5. Freeze the exact production dependencies using the supported Windows lock file; record Python, PyInstaller, Qt, VTK, and graphics-runtime versions.
6. Create and maintain a PyInstaller build specification/script. Start with a one-folder build so runtime libraries and assets can be inspected and repaired; use one-file packaging only if it passes the same tests reliably.
7. Produce a release directory with `URDF2DT.exe`, required runtime files, bundled configuration/assets, a concise `README.txt`, licence/attribution material where required, and an example URDF.
8. Exercise the packaged application end to end: launch, choose an external URDF, validate it, generate automatic DH, render the 3-D scene, perform an edit, run FK validation, export results, and close/reopen the application.
9. Test the release directory on a clean supported Windows installation or virtual machine with no Python, repository checkout, VS Code, or project dependencies installed.
10. Diagnose and fix missing DLL, plugin, graphics, asset-resolution, write-permission, and file-dialog failures; add regressions or packaging checks for every issue found.
11. Create `URDF2DT-v1.0-Windows.zip`, including only the release directory and its user-facing documentation.
12. Tag the final verified commit, publish the ZIP and checksums to the GitHub Release, and verify the downloaded artifact independently.

**Integrated advisor requirements (complete before this stage is closed):**

- Package the supported mesh loaders and required runtime dependencies while keeping user URDFs and their meshes loadable outside the application directory; do not assume all assets are bundled into the executable.
- On the clean Windows test machine, run a URDF with local relative meshes, a robot package directory with explicit package mapping, a URDF with a missing mesh, and a primitive-only URDF. Also verify a readable local absolute mesh path.
- Compare source and packaged resolution behavior, scale/origin transforms, materials, collision overlays, toggles, and DH/FK results. Launch from a different working directory and move the external robot folder to expose accidental development-path dependencies.
- Confirm missing assets produce link/path warnings and basic fallback while editing, global validation, and export remain usable; package selection must not require a ROS or Python installation.
- Include supported-format and package-mapping instructions with the release, and record the external-asset matrix before tagging/publishing.

Suggested commit:

```text
stage23: package and verify standalone Windows application
```

**Done when:** on a clean supported Windows machine, a user can extract the release ZIP, double-click `URDF2DT.exe`, select a `.urdf`, and complete the full URDF-to-validated-DH-export workflow without installing development tools or dependencies. All required external-asset cases pass with source-mode-equivalent resolution and graceful missing-asset fallback. Stage 23 completes the initial standalone kinematic release milestone; full project completion requires Stages 24–31.

---

## Stage 24 — Dynamic Model Generation

**Required for project completion.** Dependencies: Stage 23 kinematic release and independent inertial contracts.

Extend the already validated robot kinematic chain into a dynamic model, preserving its link identities, joint ordering, joint types, coordinate conventions, and source-URDF provenance. Dynamics must build on this shared representation rather than introducing an unrelated second robot model. A simulator adapter may translate the representation, but its translation must be checked against the validated chain.

The v1 inertial extractor specified in §4.2 and §11.1 supplies independent `InertialModel` data alongside the validated `KinematicChain`. Its extraction/storage capability is within the revised v1 plan; constructing and validating dynamics is required in Stage 24. Missing inertia must never prevent current DH editing. Dynamics consumers must assess physical completeness and consistency before using extracted parameters.

The dynamic model must account for link mass, center of mass, inertia tensor, gravity, joint position, joint velocity, joint acceleration, actuator/joint torque, Coriolis effects, centrifugal effects, and friction where appropriate. For a fixed-base rigid serial manipulator without external contact forces, a standard model is:

```math
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) + F(\dot{q}) = \tau
```

Equivalently: `M(q) q_ddot + C(q, q_dot) q_dot + g(q) + F(q_dot) = tau`.

| Term | Meaning |
|---|---|
| `q` | Joint-position vector: angles for revolute joints and displacements for prismatic joints. |
| `q_dot`, `q_ddot` | Joint-velocity and joint-acceleration vectors. |
| `M(q)` | Configuration-dependent mass matrix; `M(q) q_ddot` is the generalized effort needed to accelerate the coupled robot. |
| `C(q, q_dot) q_dot` | Velocity-dependent generalized efforts accounting for Coriolis and centrifugal effects; the particular matrix representation of `C` is not unique. |
| `g(q)` | Generalized efforts required to balance gravity at configuration `q`, using the documented gravity direction and sign convention. |
| `F(q_dot)` | Modeled friction effort, for example viscous and Coulomb friction when supported by available data. |
| `tau` | Applied joint/actuator generalized effort: torque for revolute joints and force for prismatic joints, after any modeled transmission mapping. |

Where available, masses, centers of mass, and inertia tensors should come directly from URDF `<inertial>` data. Missing or implausible data should be reported explicitly. The implementation must check units and physical consistency and transform inertial quantities correctly between URDF inertial frames, link frames, and any chosen DH frames. In particular, changing a legal DH frame assignment must not change the physical mass distribution: rotate inertia tensors and apply the parallel-axis theorem when changing the reference point, as appropriate.

Required validation includes simple analytical examples, mass-matrix symmetry and positive-definiteness checks for a physically valid model, gravity checks, and inverse/forward-dynamics consistency. Independent dynamic reference comparisons are required in addition to the existing FK checks. External forces, payloads, compliance, or contact would require explicitly scoped extensions of the equation above.

### Required implementation and evidence

1. Implement inverse dynamics and forward dynamics for the supported fixed-base serial chain, using independent inertial data and consistent link-frame mappings.
2. Validate masses, centers of mass, tensor frames, gravity, joint effort conventions, and the selected friction model; reject dynamics runs with inadequate parameters without disabling kinematic editing.
3. Verify analytical examples, mass-matrix properties, gravity equilibrium, forward/inverse consistency, and agreement with an independent reference. Use nominal parameters initially; incorporate Stage 25 estimates before control experiments.
4. Export versioned dynamic-model parameters, assumptions, provenance, tests, and a reproducible validation report.

**Done when:** Inverse and forward dynamics run on the reference robot and analytical fixtures, all agreed numerical checks pass, and a reproducible model/report is available.

---

## Stage 25 — Dynamic Parameter Identification

**Required for project completion.** Dependencies: Stage 24 dynamics interface and benchmark data; later hardware evaluation is completed in Stage 30.

Implement an identification module to estimate dynamic parameters when URDF values are incomplete, inaccurate, or unavailable. Candidate parameters include link masses, center-of-mass locations, inertia parameters, joint friction, and motor/actuator parameters where suitable measurements and transmission information are available.

Data sources can include simulated trajectories and measured joint positions, velocities, accelerations, and torque/current data from real hardware. Current measurements would require an appropriate motor/transmission calibration before being treated as joint torque; derived velocities and accelerations would require documented filtering and uncertainty handling.

**The identification module and its validation are required.** A particular robot run may use trusted manufacturer parameters without refitting, but this does not waive Stage 25: demonstrate parameter recovery/predictive evaluation on an appropriate benchmark and evaluate calibration against hardware data in Stage 30. Synthetic data can test an identification method, but cannot by itself demonstrate physical-robot accuracy. Research should distinguish individually identifiable parameters from identifiable parameter combinations, use sufficiently informative excitation, enforce physical plausibility, and evaluate predictive accuracy on trajectories excluded from fitting. Estimated parameters and their provenance should remain associated with the validated kinematic chain.

### Required implementation and evidence

1. Implement a documented estimator for identifiable inertial parameter combinations and joint friction, with motor parameters included when calibrated data permits.
2. Support simulated and measured time-series datasets with recorded units, timestamps, filtering, excitation quality, and torque/current conversion assumptions.
3. Demonstrate recovery on known synthetic/reference parameters and compare baseline versus estimated-model prediction on held-out trajectories; report identifiability and uncertainty.
4. Enforce physical plausibility, retain trusted-manufacturer mode, and version every estimated parameter set and dataset. Stage 30 must revisit estimates using physical data.

**Done when:** The estimator, trusted-parameter mode, physical-consistency checks, and held-out benchmark evaluation are implemented and reproducible; simply supplying manufacturer values does not complete this stage.

---

## Stage 26 — Trajectory Generation

**Required for project completion.** Dependencies: Validated kinematics; the Stage 24–25 model for feasibility checks.

Add trajectory generation on top of the verified kinematic model. Implement capabilities for point-to-point joint motion, joint-space trajectories, Cartesian-space trajectories, cubic and quintic trajectories, velocity and acceleration constraints, waypoint interpolation, and end-effector path generation.

The generator must provide time-indexed controller reference states, such as desired joint positions, velocities, and accelerations, together with desired end-effector poses where applicable. Cartesian trajectories require suitable inverse kinematics or differential kinematics, with checks for reachability, singularities, joint limits, and continuity. Geometric path generation and time parameterization should be distinguished so that a valid path is not assumed to satisfy motion constraints automatically.

### Required implementation and evidence

1. Implement point-to-point joint motion, cubic and quintic time parameterization, waypoint interpolation, and joint velocity/acceleration constraints.
2. Implement at least one Cartesian end-effector path mapped to joint references using documented inverse/differential kinematics.
3. Detect infeasible limits, unreachable waypoints, singularities, and discontinuities; return actionable diagnostics instead of unsafe references.
4. Export timestamped desired position, velocity, acceleration, and applicable end-effector pose; test endpoints, continuity, constraints, and reproducibility.

**Done when:** Joint-space and Cartesian demonstrations produce valid controller reference states within configured limits, and infeasible requests are detected by tests.

---

## Stage 27 — Robot Controller Layer

**Required for project completion.** Dependencies: Validated Stage 24–25 dynamics and Stage 26 references.

Controller implementation must follow sufficient validation of the dynamic model. Initial candidates are P, PI where appropriate, PD, PID, and PD with gravity compensation. Selection should match the available actuation interface, sensing, sampling rate, and experiment; integral action would require attention to saturation and windup.

Progressive research opportunities include computed-torque control, inverse-dynamics control, Cartesian/task-space control, trajectory tracking control, impedance control, and model predictive control. **This list does not require every controller to be implemented.** A small, justified sequence of baselines and extensions would support more interpretable comparisons.

Controller performance depends on the accuracy of the preceding kinematic and dynamic models, including joint conventions, mass, inertia, center of mass, gravity, friction, and actuator behavior. Required experiments must evaluate tracking, robustness to parameter errors, and actuator limits before drawing conclusions about controller quality.

### Required implementation and evidence

1. Implement PD tracking and PD with gravity compensation as required baselines, plus one model-based controller such as computed-torque/inverse-dynamics control.
2. Define controller inputs/outputs, timing, saturation and effort limits, and recorded gains. PI/PID, task-space, impedance, and MPC remain optional alternatives beyond this baseline.
3. Unit-test feedback signs, reference handling, gravity terms, bounded outputs, and deterministic behavior on analytical/simple dynamic fixtures.
4. Prepare model-parameter sensitivity experiments and a reproducible controller configuration for comparative simulation in Stage 28.

**Done when:** Required controllers and their interfaces pass analytical/unit checks, honor effort limits, and are ready for the integrated tracking and robustness experiments in Stage 28.

---

## Stage 28 — Simulation Environment

**Required for project completion.** Dependencies: Stages 24–27 models, trajectories, and controllers.

Implement the integrated simulation workflow:

```text
Validated Robot Model
    → Dynamics
    → Desired Trajectory
    → Controller
    → Simulated Robot Response
    → Error Analysis
```

Candidate environments or integration components include Python-based simulation, ROS 2, Gazebo, MuJoCo, and PyBullet. ROS 2 could provide communication and integration around a simulator. **No platform is preselected; select and document one supported simulation backend with project/advisor confirmation during Stage 28**, based on the needs of the experiments.

Required evaluation includes joint position tracking error, joint velocity error, Cartesian position error, orientation error, trajectory RMSE, torque profiles, steady-state error, settling time, and overshoot. Runs should record model and controller versions, parameter values, initial conditions, integration settings, sampling rates, trajectories, and seeds where relevant. Simulation validation must assess numerical behavior and dynamic consistency; good tracking in a controller's own idealized model is not sufficient evidence of real-robot accuracy.

### Required implementation and evidence

1. Select and integrate one supported simulation backend; document model translation, integrator, timestep, controller rate, and initial conditions.
2. Run the complete dynamics–trajectory–controller loop with the required controllers on shared scenarios and parameter-perturbation cases.
3. Measure the metrics in §35.3; document timestep convergence and compare dynamic predictions with an independent reference rather than only the controller's own model.
4. Set experiment-specific acceptance thresholds before evaluation, run automated regressions, and publish reproducible datasets and reports.

**Done when:** The integrated simulator reproduces the agreed experiments, required tracking/stability/effort criteria pass, and numerical and model-sensitivity results are recorded.

---

## Stage 29 — Hardware Validation

**Required for project completion.** Dependencies: Stage 28 simulation evidence, a supported physical robot, and approved laboratory operating procedures.

Extend the system from simulation to real-world validation on at least one supported physical robot. **Physical robot access is required for full project completion.** Stage 23 can still deliver the v1 kinematic application independently, but missing hardware access leaves Stages 29–31 incomplete; simulation alone cannot satisfy their gates. Compare:

```text
Desired trajectory vs. Simulated trajectory vs. Measured real-robot trajectory
```

Hardware testing must include proper safety constraints, joint limits, emergency-stop procedures, velocity limits, and torque limits, using the robot's supported control interface and the laboratory's approved procedures. Experiments should begin with conservative motion and verified stop behavior. Record sensor calibration, timestamps, units, reference frames, controller configuration, and payload conditions so that comparisons are meaningful.

### Required implementation and evidence

1. Obtain physical robot access and implement the supported hardware/sensor interface with calibrated joint order, units, frames, timestamps, and effort/current interpretation.
2. Verify joint, velocity, and torque limits, emergency-stop behavior, communication-loss behavior, and conservative initial commands before experiments.
3. Execute an approved subset of the simulated trajectories on hardware; log desired, simulated, and measured motion and available effort signals under matched conditions.
4. Preserve raw logs, calibration, payload, controller settings, repeated-run results, and measurement uncertainty. Do not mark this stage complete using synthetic or simulator-only measurements.

**Done when:** Safe supervised experiments on at least one physical robot are complete, the approved limits and tracking gates pass, and synchronized, reproducible comparison datasets are available.

---

## Stage 30 — Simulation-to-Real Comparison

**Required for project completion.** Dependencies: Stage 29 physical measurements and matching Stage 28 simulation runs.

Add a research layer to quantify differences between simulated and physical robot behavior. Required comparisons include joint positions, velocities, accelerations, end-effector pose, torques, timing, and trajectory error under comparable commands and initial conditions.

Measurements and simulated outputs should be expressed in matching frames and units and aligned using documented timestamp/latency handling. Report timing discrepancies separately so that alignment does not conceal delays. Examine potential contributions from inertial errors, friction, actuator dynamics, sensor noise, compliance, and unmodeled loads.

Use the comparison to improve the dynamic model and parameter estimates, then repeat evaluation on held-out trajectories. This feedback loop should preserve versioned models and datasets so that improvements are measurable rather than inferred from fitting the same experiment repeatedly. This stage requires suitable physical measurements from Stage 29.

### Required implementation and evidence

1. Quantify joint position/velocity/acceleration, end-effector pose, timing, and available torque differences using explicit units, frames, alignment, and uncertainty estimates.
2. Compare desired, simulated, and measured trajectories; report latency independently instead of removing it silently through alignment.
3. Use physical data to evaluate/refine Stage 25 parameter estimates and model assumptions, then test on held-out physical trajectories.
4. Report baseline and updated simulation-to-real deviations, sensitivity, remaining limitations, and pass/fail against predefined criteria. Document non-improvement honestly rather than fitting the evaluation data.

**Done when:** A reproducible physical comparison and parameter/model-update study is complete, required metrics meet agreed limits, and residual gaps and uncertainty are explicitly documented.

---

## Stage 31 — Digital Twin Integration and Final Project Release

**Required for project completion.** Dependencies: Stages 24–30 completed, with a working physical robot/sensor connection.

Implement and demonstrate a reusable robot digital-twin workflow with this architecture:

```text
Robot URDF
    → Verified Kinematics
    → Dynamic Model
    → Parameter Identification (required capability; trusted-parameter mode supported)
    → Trajectory Generator
    → Controller
    → Simulator
    → Robot/Sensor Feedback
    → State Synchronization
    → Monitoring
    → Model Updating
    → Digital Twin
```

This is a conceptual flow, with robot/sensor feedback feeding synchronization and model updates back into the simulator and dynamic model. A true digital twin requires ongoing synchronization with a physical system, including an explicit mapping between measured robot state and virtual state. The implementation must define synchronization rate, latency, timestamp handling, uncertainty, stale-data behavior, and how model updates are validated before use.

**Do not claim a synchronized digital twin until Stage 31 evidence satisfies its completion gate.** A validated kinematic model or an offline dynamic simulator would provide useful foundations, but the digital-twin claim requires physical-system connectivity, demonstrated state synchronization, monitoring, and an evaluated model-updating process.

### Required implementation and evidence

1. Integrate state synchronization, monitoring, and versioned model updating around the verified kinematic/dynamic pipeline and physical feedback.
2. Define and test synchronization rate, latency, drift/error limits, stale data, disconnect/reconnect behavior, and timestamps; visibly distinguish live physical feedback from replay/simulation.
3. Demonstrate ongoing physical-to-virtual state synchronization and at least one evaluated, versioned model-update cycle with validation and rollback; do not apply unvalidated updates to active hardware control.
4. Run the full project regression/evaluation suite and document the complete URDF-to-digital-twin workflow, hardware setup, measured outcomes, and limitations.
5. Update the standalone Windows application and distribution using Stage 23 packaging checks so the final URDF2DT.exe exposes the completed workflow. Document any external simulator, robot driver, or service dependencies and verify installation on a clean supported Windows machine.
6. Produce the final versioned release, checksums, example configuration/datasets, updated architecture and handoff documentation, and obtain recorded project acceptance.

**Done when:** All Stage 0–31 gates are met; live physical synchronization, monitoring, and a validated model-update cycle are demonstrated within agreed limits; the integrated URDF2DT.exe release and reproducibility/handoff evidence are verified. This is the final project completion stage.

---

# 34. Final User Experience

The Stage 23 kinematic milestone should feel like this:

```text
1. User extracts the Windows release and double-clicks `URDF2DT.exe`.
2. User selects robot .urdf.
3. Program validates core URDF structure and reports fatal faults separately.
4. Program extracts kinematics and independent visual/collision/material/inertial domains.
5. Program automatically generates Standard-DH parameters.
6. Program resolves external meshes and colors, warns on missing optional assets,
   and renders available URDF geometry with basic link/bone fallback.
   User independently toggles visuals, collision overlay, URDF/DH frames, and axes.
   Joint motion moves attached geometry using source FK; DH edits affect DH overlays.
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

For the completed Stage 31 application, the workflow must continue from that validated model:

1. Load and validate independent inertial parameters; run identification or choose an already trusted parameter set.
2. Generate feasible joint-space or Cartesian references and select a validated controller configuration.
3. Simulate the motion and inspect tracking, effort, and model-validation results.
4. Connect the supported physical robot under the approved operating procedure and record the hardware experiment.
5. Compare desired, simulated, and measured trajectories and evaluate parameter/model updates.
6. Monitor the synchronized virtual and physical robot state, with visible connection, latency, and stale-data status.
7. Save the model versions, experiment configuration, measurements, and evaluation report for reproduction.

The kinematic workflow remains usable on its own when dynamics data or hardware are unavailable, but that partial operating mode does not satisfy the full project completion gate.

---

# 35. Final Research Success Criteria

The following ten criteria establish the kinematic contribution. Full project success additionally requires the Stage 24–31 gates: validated dynamics and identification, constrained trajectories and control, reproducible simulation, physical validation, quantified simulation-to-real comparison, and demonstrated synchronized digital-twin integration. Use §35.3 to define and evaluate the additional numerical acceptance criteria.

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

## 35.1 Full Project Architecture

Both portions below are required project scope. Stage 23 delivers the first standalone kinematic release; Stages 24–31 complete the integrated project. These are requirements, not claims of completed implementation.

```text
KINEMATIC APPLICATION MILESTONE — Stages 0–23

User URDF
    ↓
URDF Validation
    ↓
URDF Parser
    ↓
Kinematic Chain
    ↓
Automatic Standard-DH Generation
    ↓
3-D Visualization
    ↓
Constraint-Aware DH Editor
    ↓
Local DH Validation
    ↓
Global URDF-FK vs DH-FK Validation
    ↓
Validated Kinematic Model
    ↓
Export
    ↓
Standalone URDF2DT.exe — initial release at Stage 23

================ CONTINUE REQUIRED PROJECT WORK ================

REQUIRED DYNAMICS-TO-DIGITAL-TWIN IMPLEMENTATION — Stages 24–31

Validated Kinematic Model
    ↓
Dynamic Model Generation
    ↓
Dynamic Parameter Identification
    ↓
Trajectory Generation
    ↓
Controller
    ↓
Simulation
    ↓
Simulation Validation
    ↓
Hardware Validation (physical robot required)
    ↓
Simulation-to-Real Comparison
    ↓
Digital Twin Integration + Final URDF2DT.exe Release (Stage 31)
```

The executable packages the workflow; it is not a model-conversion step. The required downstream stages consume validated models and source-URDF data. Per-run trusted-parameter mode is allowed, but implementing and validating identification is mandatory. Physical robot access is a completion dependency for Stages 29–31; lack of access must be recorded as an unresolved blocker, not a completed stage.

The compact diagram shows the kinematic path only. The independent extraction branches in §4.2 supply visual/material assets to rendering, collision data to its optional overlay, and inertial data to Stage 24 dynamics; none of those branches is a prerequisite of automatic DH generation.

## 35.2 Project Research Questions

1. Can a validated URDF-derived kinematic representation be automatically extended into a reliable robot dynamic model?
2. How accurate are URDF inertial parameters for dynamic simulation?
3. How much can dynamic parameter identification improve model accuracy?
4. How sensitive are controller results to errors in mass, inertia, center of mass, and friction parameters?
5. Can controllers generated from the URDF2DT model accurately track joint and Cartesian trajectories?
6. How large is the simulation-to-real gap for models generated through this pipeline?
7. Can the workflow generalize across different serial manipulators?
8. Can URDF2DT become a reusable robot digital-twin creation framework?

## 35.3 Required Extended Evaluation Metrics

These metrics supplement §24 and are required evidence for Stages 24–31. Before each experiment, record numerical acceptance thresholds, datasets, baselines, and protocols with advisor agreement. Apply each metric where mathematically meaningful (for example, settling time for point-to-point tests), and document justified non-applicability or unavailable sensor quantities. Do not treat an unmeasured required acceptance criterion as passed.

| Metric | Possible evaluation method |
|---|---|
| Dynamics prediction RMSE | Compare predicted accelerations or state trajectories with an independent reference on held-out inputs; specify the quantity, prediction horizon, and units. |
| Torque prediction error | Compare inverse-dynamics torque predictions against calibrated measurements or an independent reference; report per-joint RMSE and peak error. Use force units for prismatic joints. |
| Identified-parameter error | Compare estimates with known reference values or identifiable parameter combinations; report uncertainty when individual physical parameters cannot be recovered uniquely. |
| Joint trajectory RMSE | Compare desired and simulated/measured joint positions over a defined time interval, reporting each joint in radians or meters as appropriate. |
| Joint velocity error | Compare reference and simulated/measured velocities per joint, with documented derivative estimation where needed. |
| Cartesian trajectory RMSE | Compare end-effector positions in the same reference frame, in meters. |
| Orientation error | Use the SO(3) geodesic angle from §20.5, in radians, with mean and maximum error or a documented RMS summary. |
| Controller steady-state error | Measure residual tracking error over a specified steady-state interval. |
| Controller settling time | Measure time to enter and remain within a predefined error band for a suitable step or point-to-point experiment. |
| Overshoot | Measure peak deviation beyond the target; state the normalization when reporting a percentage. |
| Torque profiles | Inspect peak and RMS effort, saturation duration, and effort variation along the trajectory. |
| Simulation-to-real deviation | Compare synchronized simulated and measured positions, velocities, accelerations, poses, torques, and timing under matched conditions. |
| Computational runtime | Record dynamics, trajectory, control, and simulation computation times with hardware, model size, and sampling settings. |

For a scalar quantity, RMSE is `sqrt(mean((prediction - reference)^2))` over the stated samples. Vector results should specify whether they are per component or based on a vector norm; angular and translational errors should not be combined without a declared normalization. Distinguish tracking error against the desired trajectory from prediction error against observed robot behavior, and retain separate fitting and evaluation datasets.

## 35.4 Full Project Objective

The required project evolves from the compact description:

```text
URDF → DH
```

to:

```text
URDF
    → Verified Kinematics
    → Validated Dynamics
    → Parameter Identification
    → Trajectory Generation
    → Control
    → Simulation
    → Hardware Validation
    → Digital Twin (with demonstrated physical-system synchronization)
```

The constrained, verifiable DH-frame editor remains the kinematic foundation. Full project completion additionally requires validated dynamics, parameter identification, trajectory generation, control, simulation, physical experiments, simulation-to-real analysis, and a synchronized digital twin. The mandatory implementation sequence is Stage 0 through Stage 31; claims of completion require recorded evidence for every stage.

---

# 36. Optional Extensions Beyond Required Project Completion

The following extensions remain optional beyond the required Stage 0–31 project scope:

Dynamics, identification, trajectories, control, simulation, hardware validation, simulation-to-real comparison, and digital-twin integration are mandatory Stages 24–31 in §33. This optional list does not waive those stages. Hardware-in-the-loop editing is a separate editor capability from the required physical validation and state synchronization.

- browser/web application frontend,
- advanced CAD/appearance features beyond the v1 URDF mesh/primitive layer, including optional textures and additional verified mesh formats,
- branching-tree support,
- closed-loop mechanisms,
- hardware-in-the-loop editing,
- additional dynamics backends or capabilities beyond the required validated model,
- multi-end-effector support,
- interactive comparison of Standard vs Modified DH conventions,
- session replay visualization,
- automatic report figures for publications.

Optional extensions must not replace or delay the required Stage 0–31 completion gates.

---

# 37. One-Sentence Project Summary

> **URDF2DT is specified to transform a robot URDF into verified, constraint-editable Standard-DH kinematics, validated dynamics, identified parameters, trajectories, control and simulation, then validate the results on physical hardware and deliver a synchronized digital twin through an integrated standalone application.**

