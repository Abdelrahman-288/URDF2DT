# Project location consolidation

The project root is now the maintained source location. It was fast-forwarded
from `9389a0c` to retained Stage 23 commit `3c072eb` on `codex/project-home`.
The previous branch and historical worktrees remain available.

`Launch URDF2DT.cmd` now launches `Windows Application/URDF2DT.exe`, matching
the existing desktop shortcut. Previously it launched the root Python environment
against older root code. `START HERE.md` and the README identify the application,
manual, robot assets, source, sharing ZIP and historical copies.

Existing Downloads deliveries received a location notice; they were not moved,
deleted or treated as maintained source. Build da8b32b remains installed unchanged.
No application rebuild is required for this launcher/documentation consolidation.
No Stage 24 onward code was introduced.

## Preservation and verification

- Pre-integration modified files and overlapping documentation were backed up in
  `outputs/tmp/location-consolidation-backup`.
- Editor settings and the modified notebook retain their exact pre-integration hashes.
- Overlapping UI plan/release notes match their preserved copies byte for byte.
- The newer project plan retains all previous local lines in order and adds later
  Stage 23 documentation.
- Root source, tests and packaging match the retained Stage 23 worktree.
- Full root source test suite: **375 passed** in 150.66 seconds using the retained
  Stage 23 build environment.
- Standalone `--verify-package` completed successfully from the root application;
  evidence is in `outputs/tmp/location-verification/verification.json`.

This is local organization work, not a new Windows release or clean-PC acceptance.
The installed build's recorded limitations remain applicable.
