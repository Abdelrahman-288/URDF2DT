# Contributing

URDF2DT has package scaffolding and Stage 3 data contracts. Use the development instructions in
the README and consult `docs/stages/` for verified progress and unresolved inputs.

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

The local configuration and immutable data APIs have tests. Parser/solver/editor
implementations, CI, and a project license have not yet been established.
The repository being public does not itself grant an open-source license.
