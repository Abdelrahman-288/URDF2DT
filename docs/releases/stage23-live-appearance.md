# Stage 23 live appearance controls — 21 September 2026

Built source: `4897e66ab42edb67d2bf4bcd7a39e0039b958f6a`, clean checkout of
`codex/stage23-desktop-improvements`. This delivery retains the imported-body fixes.

Select a joint, link or geometry element in Robot Structure. Opacity is now a live
0–100% slider with a percentage label. Joint selections affect their child-link
body. Confirming the color dialog applies the color immediately; Cancel preserves
the current color. Visibility/style apply on change; aliases apply on editing
completion. The Apply Appearance button and Notes field are removed. Existing
project notes remain in metadata for compatibility. Edits change only the selected
property, retain per-component settings, and do not change accepted DH state.

Verification:

- 341 regression tests passed in 61.92 seconds; mypy passed for 49 source files.
- [CI passed](https://github.com/Abdelrahman-288/URDF2DT/actions/runs/35606361169).
- Source and frozen studio checks passed: live opacity and actor color, Cancel,
  selection changes without unintended edits, a single undo step for a slider
  drag, undo/redo, project persistence, and removal of both UI controls.
- Frozen package verification and five imported-robot body checks passed outside
  the repository with Python and Git removed from PATH. All 1,575 packaged manifest
  files matched their checksums. The frozen light-theme capture was inspected.

The project-root `Windows Application/URDF2DT.exe` is the launch point. The previous
version is retained at `outputs/tmp/windows-application-before-live-appearance-8b73e85`.
Original reports/captures are in `C:/Users/abdel/Downloads/URDF2DT-Live-Appearance-4897e66`.
The shareable `URDF2DT-v1.0-Windows.zip` contains 200,611,720 bytes, SHA-256:

```text
b2dd80d9f00a700dc08f5a178711f7d64f7b097d470740cebc7d019359861d68
```

Clean-PC and broader graphics/accessibility testing remain pending. Other rendering
and asset limitations from the imported-body delivery still apply. This is a
development-host verified research candidate within Stages 0–23.
