# Stage 23: Standalone Windows application

The deliverable is a one-folder `URDF2DT.exe` application, including Python,
Qt/VTK and runtime dependencies. Keep `_internal` beside the executable. The
application opens an empty native window, offers Open URDF and Open example,
and stores logs/sessions in user locations rather than requiring a repository.

The project owner selected the MIT licence on 2026-09-16. Third-party libraries
retain their licences. The build gathers notices and a dependency inventory;
source-distribution obligations must be checked before a final public release.

## Build

On Windows x64 with Python 3.12, from a checkout root:

```powershell
py -3.12 -m venv .venv-build
.\.venv-build\Scripts\python.exe -m pip install -r requirements/windows-py312-build.lock
.\.venv-build\Scripts\python.exe -m pip install --no-deps --no-build-isolation -e .
.\.venv-build\Scripts\python.exe scripts/build_windows.py
```

The spec is `packaging/windows/URDF2DT.spec`. It explicitly includes lazy runtime
imports, examples, the SVG icon, configuration and session schema; unused Qt
modules are excluded. CasADi core DLL dependencies are discovered from its extension,
without requesting its unused optimization plugins. Build metadata records the
source commit; frozen provenance does not require Git. Build tools and exact
dependencies are pinned separately from the desktop runtime lock.

The output is `dist/URDF2DT/URDF2DT.exe`. Copy the entire folder to relocate it.
`build-info.json`, `third-party/inventory.json` and `SHA256SUMS` accompany it.
The package remains version 0.1.0 with session schema 1.0 pending final release.

## Verification and delivery gates

Run `URDF2DT.exe --verify-package <new-output-folder>` or double-click the
included `Verify URDF2DT.cmd`. Diagnostics launch an empty window, load external
copies of the included robots through the application callback, move joint
controls, edit frames, validate FK, save/reload, render both themes and reopen.
The automated test substitutes file-picker return values; the actual Windows
chooser must also be exercised in the manual clean-PC test.

Relocation testing on the development host, with Python/Git removed from PATH,
is useful evidence but does not substitute for a genuinely clean Windows PC.
`CLEAN_PC_TEST.txt` gives the recipient the manual steps and evidence to return.
The current host has Windows 11 Home; no separate clean Windows target has been
confirmed. Clean-PC acceptance is pending, as are the final verified release tag,
public stable release, and independent download verification. Do not describe a
successful local build as satisfying those remaining gates.

No Windows reset, reinstall or VM provisioning is required on the user's current
machine to produce the candidate. A separate Windows computer can supply the
remaining acceptance evidence when available.
