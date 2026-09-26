# Curated robot examples

The collection lives in `robots/curated/`, with a self-contained illustrated HTML
guide, Markdown guide, per-example instructions, exact URDF files, MIT licence,
asset provenance and manifest. The Windows distribution includes it under
`examples/curated/`. Use the existing Open URDF action; no Open Example button was
reintroduced.

The serial arm (3R) and dual-branch robot (two 2R paths) are original schematic
teaching models. The RRPR SCARA is an unchanged copy of the existing original
project fixture. All visible bodies use embedded primitives; no unlicensed or
unavailable external meshes are copied from the user's local collection.

Each exercise includes joint coordinates, reference/target links, units and an
expected FK position checked independently against simple geometry. Images are
actual viewport captures with their demonstration poses documented. Reference and
tool links intentionally have no body. These examples supply neither collision
nor inertial models and do not represent calibrated hardware.

Verification:
- Four asset tests check the manifest, body counts, self-contained assets, licence,
  exact SCARA provenance, every serial path's sampled FK and exercise answer.
- Native Qt/VTK checks render every body and move every branch. The branched robot
  shows both arms while exactly two arm bodies move for each selected path.
- Native evidence and appearance captures: `outputs/tmp/curated-examples-native/`.
- Installed executable body/motion checks passed for all three files, loaded from
  `Windows Application/examples/curated/`. Evidence:
  `outputs/tmp/curated-frozen-serial_arm/`, `curated-frozen-scara/`, and
  `curated-frozen-branched_robot/`. These runs also retain the packaged symbolic
  output and validation-dashboard regression checks.
- Every local image and licence/guide link in the offline HTML resolves.

This is an asset/documentation update. The executable's code remains build
`5b40d36`; build-info.json is not relabelled. The packaging script now includes the
curated collection in future builds. The installed distribution and sharing ZIP
receive the collection without changing the tested executable.
