# Three-decimal display delivery

Delivered 25 September 2026 from clean source
`28643d0f4d71c1307b757dd88a87cf4beb6e7f21`.

Numeric desktop controls and readouts, DH tables, geometry fields and notebook DH
tables use three decimal places; rounded negative zero becomes `0.000`. Small
nonzero validation residuals use scientific notation. Calculations, project state,
JSON exports, original URDF source and symbolic expressions retain their precision.
Unchanged forms preserve their original values, including degree-based FK inputs.

Evidence:

- 354 pytest tests passed (61.65 seconds); mypy passed for 54 source files.
- Source precision and studio checks passed. The precision check additionally
  verified unchanged degree-based FK submission after the final UI adjustment.
- Frozen package, precision, studio and symbolic checks passed outside the repo
  with Python and Git removed from PATH. The frozen numeric screenshot was inspected.
- All 1,614 manifest entries matched in the extracted distribution.
- Reports: `C:/Users/abdel/Downloads/URDF2DT-Three-Decimals-28643d0`.
- Launch: project-root `Windows Application/URDF2DT.exe`.
- Previous application: `outputs/tmp/windows-before-three-decimals-28643d0`.

ZIP size: 208,625,827 bytes. SHA-256:

```text
0afbdef8c7960e123aa426fc92d3c98617da120595c25a6793293a3db3449979
```

Verification was on the development Windows host. Independent clean-PC acceptance
remains pending. No symbolic Jacobian or unrelated UI changes are included.
