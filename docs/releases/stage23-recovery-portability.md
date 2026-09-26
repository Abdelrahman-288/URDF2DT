# Recovery and portable project hardening

The permanent desktop status distinguishes unsaved work, explicitly saved projects,
reopened recovery drafts, unresolved previews and saved asset coverage. File →
Project and recovery status provides paths, available drafts and recovery failures.
Auto-recovery retains available assets in separate timestamped folders and never
clears dirty state. Opening a draft through either project or recovery actions
requires Save As. Pending previews postpone recovery; they are not silently saved.

New project metadata includes SHA256 hashes of the source, generated URDF and copied
asset files. Loading verifies them before changing the editor. Portable closure
checks require local existing mesh, texture and override paths, and traverse
OBJ/MTL/DAE dependencies without original-machine fallback. Unsupported dependency
syntax is rejected for review. Legacy projects remain readable without a manifest.

ZIP export stages a self-contained project, checks its dependencies and hashes,
creates an archive, verifies CRCs, extracts it, and prepares/revalidates its model
and presentation state. Only then is a new destination file published. Existing
ZIPs are preserved. Export does not clear unsaved changes. A recipient README and
PORTABILITY_CHECK.json distinguish this local evidence from second-computer testing.

Existing-project saves retain the previous checkpoint, replace core files via
temporary files, and restore checkpoint core files on ordinary write failures.
This is not a multi-file power-loss transaction: after interruption, integrity
checks may require opening the retained checkpoint or recovery draft and Save As.
Drafts include committed state only and are not automatically deleted. Large asset
copies and validation are synchronous; recovery storage can grow over time.

Verification covers hidden original source assets, portable extraction and actual
UI reopen with unchanged model/pose, draft identity, cancellation, pending previews,
existing archive preservation, corruption rejection, unresolved assets, simulated
save rollback and simulated recovery disk errors. Asset tests also cover dependency
closure, missing/tampered files, external references and legacy manifests.

No second computer was available for this work. A successful local relocation test
does not verify another Windows installation, graphics driver, permissions or
hardware. New checks do not constitute a hardware or collision safety certificate.

Source verification: 397 pytest tests passed (155.13 s); mypy passed for 77 source
files. Native acceptance: `outputs/tmp/recovery-portability-acceptance/`. The
existing user settings, notebook edits and robot collection were preserved.
