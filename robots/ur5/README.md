# UR5 structural-validation fixtures

These are **classic UR5** descriptions, not the missing `universalUR5.urdf` from
the project's MATLAB workflow. Structural acceptance is not an FK comparison or
evidence that their frames match the Stage 3 DH fixture.

## Upstream source and license

`ur5_upstream.urdf` is an unmodified copy of
[Gepetto/example-robot-data's expanded UR5 description](https://github.com/Gepetto/example-robot-data/blob/6249cab1cdffa4fadb9a53dda964a50d79c5eaaf/robots/ur_description/urdf/ur5_robot.urdf).

- Pinned commit: `6249cab1cdffa4fadb9a53dda964a50d79c5eaaf`.
- Retrieved: 2026-09-13.
- SHA-256: `cede880fc2a987ef9dd23bbcda3e0ad1e7b1df8b58d638d100cfbff8ebc806cb`.
- The upstream package README attributes its source to ROS-Industrial's
  `universal_robot/ur_description` and identifies the license as BSD.
- Original notices are retained in `LICENSE.upstream` and `README.upstream.md`.
  That license applies to the copied/derived fixture files, not the entire project.

The full upstream description has fixed branches at `base_link` and
`wrist_3_link`. Stage 4 intentionally rejects it with `branching`; fixed branches
are still branches under the v1 contract. Mesh URIs and simulation extensions
are never fetched, loaded or executed by the validator.

## Explicit derived serial fixture

`ur5_serial.urdf` retains exactly the `world -> base_link -> ... -> wrist_3_link
-> tool0` path: 9 links, 8 joints, 6 movable revolute joints and 2 fixed joints.

Modifications from the original are explicit:

- Removed the `base` and `ee_link` auxiliary leaves and their fixed joints.
- Removed visual, collision, inertial, transmission, dynamics and Gazebo content.
- Kept every selected joint's name, type, parent, child, origin, axis and limits
  unchanged. No angle, offset, axis or joint-limit values were adjusted.
- Renamed the robot `ur5_serial` and formatted the resulting XML.

SHA-256: `2dc91dafb13f23beadb867f81880db1de4baa963973e768521ab7abbc5314882`.
Reproduce from the pinned local source with:

```powershell
python scripts/prepare_ur5_fixture.py
```

The preparation script first verifies the source checksum. It is a development
utility for this fixture; the production validator never selects paths or prunes
branches from user files automatically.
