# URDF2DT project home

Your maintained project folder is:

`C:\Users\abdel\OneDrive\Desktop\URDF2DT`

## Run the application

Double-click **Launch URDF2DT.cmd** or **URDF2DT Desktop.lnk** in this folder.
Both open **Windows Application\URDF2DT.exe**. No Python installation is needed.
Keep the executable with its `_internal`, examples and other distribution files.
Do not copy the executable alone.

## Where everything belongs

| Location | Purpose |
| --- | --- |
| `Windows Application/` | Current installed Windows application and its build information. |
| `User Manual/URDF2DT_User_Manual.pdf` | Illustrated manual to read or send to your doctor. |
| `User Manual/URDF2DT_User_Manual.docx` | Editable manual. |
| `robots/` | Robot descriptions and body assets, including your local collection. |
| `Project Report/` | Research report material. |
| `urdf2dt/`, `tests/`, `scripts/`, `packaging/` | Maintained source, tests and build tooling. |
| `docs/` | Project plan, technical documentation and release records. |
| `outputs/sessions/` | Existing saved sessions; keep these as user data. |
| `outputs/tmp/` | Historical worktrees, build environments and temporary verification files. Not the maintained source. |

## Which version is current?

The installed application identifies its build in `Windows Application/build-info.json`.
The loading-reliability update is **d447b89**. It adds cancellable loading,
background validation, clearer load errors and preserved robot state on failure.
See `docs/releases/stage23-loading-reliability.md` for verification and limits.
The previous application is preserved in
`outputs/tmp/application-before-loading-d447b89`.

The root source now contains the retained Stage 23 improvements, including symbolic
kinematics and the updated desktop workspace. Stages 24 onward were not restored.
Future code changes require rebuilding and verifying the application before they
appear in the executable. Source and packaged application are separate deliverables.

## Send the application to someone

Send `Windows Application/URDF2DT-v1.0-Windows.zip`. Ask them to extract the entire
ZIP into a folder and run `URDF2DT.exe` inside it. Send the manual PDF alongside it.
Local robot collection assets are not automatically included in this ZIP; use a
portable project export when sharing a particular robot with its required assets.

## Older copies

Folders named `URDF2DT-*` in Downloads are earlier extracted deliveries and
verification workspaces. They are not the maintained project. Historical copies
and worktrees were preserved to avoid breaking references or losing local files.
Use the launcher in this project root for everyday use.

The consolidation backup is `outputs/tmp/location-consolidation-backup`.
Your local editor settings, notebook changes, robot collection, report and manual
were preserved. The root plan was updated to the newer retained plan, which includes
the previous local plan text plus subsequent Stage 23 additions.

## Future updates

Make source changes in this project root. Build and verify a candidate before
replacing `Windows Application/`. Keep a previous working package until the new
candidate passes checks. Update the build record and manual when behavior changes.
Do not make Downloads a second development location.
