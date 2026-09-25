# Symbolic geometric Jacobian delivery

Delivered 25 September 2026 from clean source
`3a4607453e6b911fc25fb9316d5d1978ab1c5c20` on
`codex/stage23-desktop-improvements`.

The Symbolic Jacobian tab generates a 6×n geometric matrix at the selected
physical tip origin, expressed relative to and in the selected base. Signed
revolute/continuous and prismatic columns include fixed base/tool transforms.
The tab supports automatic/committed models, cancellable worker generation,
four-result caching, a full matrix view, text/LaTeX copy/export, live numerical
pose evaluation, and full-precision numerical JSON with frame/unit metadata.

Evidence:

- 362 pytest tests passed in 200.29 seconds; mypy passed for 56 source files.
- An exact planar analytic Jacobian, singular planar pose, UR5 and SCARA were
  checked, including central FK differences for both translation and rotation.
- Signed mixed joints, all-frame X/Z flips, terminal compensation, arbitrary
  base/tool offsets, tiny geometry constants, invalid inputs and size limits
  were covered. Symbolic substitution agrees with the live numeric construction.
- Native Jacobian and existing symbolic FK checks passed. Jacobian layout was
  rechecked at 1280×800 after compacting controls.
- Frozen package, Jacobian, symbolic FK, numeric formatting and studio checks
  passed outside the repo with Python and Git removed from PATH. The Jacobian
  check verifies event-loop responsiveness, cache, exports, pose changes,
  cancellation, pending-preview exclusion and stale-model/robot invalidation.
- The frozen full matrix capture was inspected. All 1,615 archive manifest files
  matched their checksums.

Launch: project-root `Windows Application/URDF2DT.exe`.
Backup: `outputs/tmp/windows-before-symbolic-jacobian-3a46074`.
Reports/captures: `C:/Users/abdel/Downloads/URDF2DT-Symbolic-Jacobian-3a46074`.

ZIP size: 208,637,018 bytes. SHA-256:

```text
c468f971f28267431668bde9e804a70357eb8920e548fc8c450a459f11d2569d
```

Limits: selected chain endpoints; numeric geometry constants; at most 12 movable
joints; 60-second worker limit. No RPY-rate Jacobian, inverse kinematics, arbitrary
point/frame selector or singularity diagnosis. No removed Stage 24+ work restored.
Independent clean-PC acceptance remains pending; this is development-host evidence.
See `docs/symbolic_jacobian.md` for conventions and use.
