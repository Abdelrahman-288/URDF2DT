URDF2DT - Windows desktop candidate

1. Extract the whole ZIP to a folder you can write to.
2. Double-click URDF2DT.exe. Keep the _internal folder beside it.
3. Click Open example for the included SCARA, or Open URDF to choose your robot.
4. Move Joint motion sliders; orbit and zoom the robot with the mouse.
5. In DH editor, Unlock next, adjust enabled controls, then Accept frame.
6. Accept every frame, Validate FK, and Save session into a NEW folder.
7. Close and reopen the app. Load session by selecting its session.json file.
8. Use the toolbar to switch Light/Dark appearance.

No Python, Git or editor installation is required to run the bundled application.
This candidate has not yet passed the separate clean-Windows-PC acceptance test.
Use Windows x64 with a working graphics driver. An unsigned executable may trigger
Windows reputation checks; this build has no code-signing certificate.

The examples folder contains SCARA and a schematic UR5. Missing external UR5
meshes do not prevent kinematic viewing; visual notices identify missing assets.
For your own URDF, keep meshes with it or select Mesh package directory.

URDF2DT uses Standard DH. Joint pose changes and DH frame edits are different.
Only selected serial chains and revolute/continuous/prismatic joints are supported.
Sampled FK is not a proof over every possible pose; research tolerances remain
provisional. Convex collision parts are view-only and are not exported.

Save your sessions outside the extracted application directory. Logs are stored
under your Windows Local AppData URDF2DT directory. If the app cannot start, look
for startup-error.txt under Local AppData\URDF2DT\logs.

URDF2DT code: MIT licence (LICENSE.txt). Third-party code retains its own terms;
see third-party/README.md and inventory.json. Keep those notices with this folder.

Source, issue reporting and maintenance:
https://github.com/Abdelrahman-288/URDF2DT
See build-info.json for the build's source commit.
