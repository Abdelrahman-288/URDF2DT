# URDF2DT

Constraint-aware Standard-DH frame editor for serial robots, with an immutable
automatic reference model and local geometry/global forward-kinematics validation.

Development is starting with Stage 0 (environment setup). The application is not
implemented yet. See [the project plan](docs/URDF2DT_Final_Plan.md) and
[environment progress](docs/stages/00_environment.md).

Repository: https://github.com/Abdelrahman-288/URDF2DT

Dependency imports, CasADi evaluation, and PyVista offscreen rendering have passed.
For the exact verified Windows/Python 3.12 environment, install
`requirements-windows-py312.lock.txt` instead of `requirements-dev.txt`.

## Development environment (Windows PowerShell)

With Python 3.10 or newer installed:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

Activation is optional; invoking the environment's Python directly avoids shell
execution-policy changes. Select `.venv\Scripts\python.exe` in your editor.

The first setup used the Python 3.12 runtime bundled with Codex to create `.venv`.
Recreate the environment with an independently installed Python if that runtime
is removed or relocated.

## Next dependencies

- Confirm whether the parent repository already implements the parser and DH solver.
- Supply the UR5 URDF, MATLAB reference files, DH technical report, and architecture
  reference before their associated integration/comparison stages.
- Confirm reference tolerances before implementing geometry-dependent behavior.
