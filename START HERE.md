# URDF2DT project home

Your maintained project folder is:

`URDF2DT`

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

Download the Windows application from the [latest GitHub release](https://github.com/Abdelrahman-288/URDF2DT/releases/latest).
Extract the full ZIP and run `URDF2DT.exe`; keep `_internal` beside it. The build
record identifies its source revision and verification status.
The root source now contains the retained Stage 23 improvements, including symbolic
kinematics and the updated desktop workspace. Stages 24 onward were not restored.
Future code changes require rebuilding and verifying the application before they
appear in the executable. Source and packaged application are separate deliverables.

## Try the curated robots

Open `robots/curated/index.html` for pictures, exercises and expected FK answers.
In the application, use **Open URDF** and choose a file in that collection. The
Windows distribution includes the same guide and robots at
`Windows Application/examples/curated/`. It contains a serial arm, SCARA and a
two-branch robot, all with self-contained solid bodies and MIT asset licences.

## Send the application to someone

Send `Windows Application/URDF2DT-v1.0-Windows.zip`. Ask them to extract the entire
ZIP into a folder and run `URDF2DT.exe` inside it. Send the manual PDF alongside it.
Local robot collection assets are not automatically included in this ZIP; use a
portable project export when sharing a particular robot with its required assets.

The bottom-right status distinguishes saved projects, unsaved changes and recovery
drafts. **File → Project and recovery status** shows their paths. Recovered drafts
require Save As; exporting a portable ZIP does not save the working project.
Portable exports check asset completeness and extracted-project model reopening
locally. A different-computer test is still required; recipients should extract
everything and open `RobotProject/project.json` in a compatible application.

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
