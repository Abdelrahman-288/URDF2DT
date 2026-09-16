# Contributing

URDF2DT includes the serial parser/solver, constrained editor, native and notebook
interfaces, sampled FK validation, persistence and hosted CI. Start with
[HANDOFF.md](HANDOFF.md) for integration contracts, fixture development, dependency
maintenance and release procedures. Use the README for setup; stage notes retain
historical evidence and unresolved inputs.

## Changes

Keep changes focused on the current development stage. Include a description of
the problem, resulting behavior, relevant validation, and any remaining limits.
Update the stage record alongside code. Do not describe planned capabilities as
implemented or reference comparisons as passing without measured evidence.

## Architecture

- Keep geometry, editor state, and FK validation independent of notebook widgets.
- Preserve automatic DH output as an immutable baseline.
- Put robot-specific reference data in fixtures, with its source and license.
- Centralize numerical thresholds and record the values used in validation.
- Test meaningful mathematical and state-transition invariants, including failure
  behavior and preservation of accepted state after rejected edits.
- Validate URDF input with an XML parser that rejects external entities.

## Research evidence

Record reference provenance, joint/frame conventions, configuration, sampling
seed, and validation results. Resolve reference disagreements before dependent
work. Do not copy third-party implementations without checking their licenses.

## Current limitations

The source release candidate is verified; standalone Windows packaging, reference
reconciliation and advisor acceptance remain open. Global FK is sampled evidence,
not a formal proof. A top-level project licence has not been selected; public
repository visibility does not itself grant an open-source licence. Preserve
third-party fixture attribution and obtain the owner's licence decision before
distribution.
