# Symbolic forward kinematics delivery

Delivered 25 September 2026, built from clean source
`2d9e5fb96ad3f9ec0110478cbc20554907cb863b` on
`codex/stage23-desktop-improvements`.

The Symbolic FK tab generates selected-chain base-to-tip Standard-DH transforms
with original joint-coordinate symbols. Automatic and committed model choices,
joint-name/unit mapping, base/tool transforms, signed revolute/prismatic motion,
matrix-entry definitions, a larger snapshot view, and text/LaTeX copy/export are
included. Generation runs in a cancellable process, caches four results, and
invalidates stale robot/chain/model results. Pending previews are excluded.

Verification:

- 347 regression tests passed in 53.90 seconds; mypy passed for 52 source files.
- New mathematical tests cover an analytic planar mechanism, UR5 and SCARA,
  compensated terminal edits, axis flips, joint signs, arbitrary fixed alignment,
  mixed joints, invalid configurations, and the size limit.
- Source and frozen native checks passed: SCARA/UR5 generation, full matrix view,
  clipboard, text/LaTeX export, pose-independent cache reuse, cancellation, model
  choice, preview exclusion, committed-edit invalidation, and robot replacement.
- Frozen package and studio regression checks passed outside the repository with
  Python and Git removed from PATH. All 1,614 packaged manifest entries matched.
- The frozen dark-theme full matrix capture was visually inspected.

The usual launch point remains `Windows Application/URDF2DT.exe` in the project
root. The previous application is backed up under
`outputs/tmp/windows-before-symbolic-fk-2d9e5fb`. Delivery reports and captures are
in `C:/Users/abdel/Downloads/URDF2DT-Symbolic-FK-2d9e5fb`.

The application ZIP contains 208,618,785 bytes with SHA-256:

```text
1425b50a9d0de1cd8a59d6241f4218d28f37c57fbef2b3828ccfb47b9f8516a6
```

Limits: chain endpoints only, numeric geometry constants, up to 12 movable joints,
60-second generation limit, no global simplification or geometry rounding. Common
intermediate definitions must accompany exported matrices. No symbolic Jacobian
or additional requested UI changes are included. Clean-PC acceptance remains
pending; this is development-host verification, not a new CI or hardware result.
See `docs/symbolic_fk.md` for usage and mathematical conventions.
