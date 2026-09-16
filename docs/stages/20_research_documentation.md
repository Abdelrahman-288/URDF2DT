# Stage 20: Final documentation and research record

The consolidated documentation describes the current implementation and measured
evidence without changing production code or rewriting historical experiments.

| Planned item | Final document |
|---|---|
| Architecture diagram and boundaries | [Architecture](../architecture.md) |
| Geometric cases, legal edits, local validation and session states | [Architecture](../architecture.md) |
| Configuration, FK tolerances, reproduction and failure modes | [Reproducibility](../reproducibility.md) |
| UR5, ablations, second robot and contribution paragraph | [Research record](../research_record.md) |
| Limitations and unverified acceptance items | All three documents, with evidence scope explicit |

The R1–R3 item is addressed by documenting actual implemented conditions and
explaining that their mapping to the external report is still unverified. No
missing report or MATLAB result is substituted with an invented claim. Defaults
were checked against configuration code, transition behavior against the session,
and geometry/FK descriptions against their implementations.

## Evidence checks

An audit loaded the original JSON files and checked both sets of 84 successful
UR5 sweeps, five unchanged threshold classifications per source, nine density
outcomes per source (including three expected negative-control failures), 24
boundary probes per source, the 200-pose SCARA oracle record and the 14-cell
notebook result. The numerical maxima in the research text match these files.
[The audit manifest](../../outputs/review/stage20_evidence_audit.json) retains
SHA-256 fingerprints of the original evidence and the extracted summary values.
No new numerical experiments were performed or attributed to this stage.

Local Markdown file targets were checked for existence, and the staged diff was
reviewed for whitespace and scope. The established hosted regression workflow
also runs on this branch; this documentation stage does not replace fresh release
verification. Historical test counts and environment claims remain tied to their
recorded stages rather than represented as new local runs.

Existing user plan changes, interactive-notebook edits, local sessions and added
robot inputs remain outside this documentation commit. The next planned stage is
**Stage 21: Final Integration and Release Candidate**. Clean-environment checks,
dependency locks and release-candidate evidence belong there; standalone Windows
packaging remains a later delivery stage in the locally revised plan.
