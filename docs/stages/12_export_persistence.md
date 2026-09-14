# Stage 12: Export, persistence and logging

Completed sessions can be saved as version 1.0 JSON-only archives, reloaded,
revalidated and edited again. No pickle, executable payloads or source-path reads
are involved. Exports require all frames accepted and no pending proposal, plus
fresh passing automatic and edited sampled FK checks under the saved configuration.

## Notebook and Python use

In the notebook, enter a new **Session folder** and click **Save session**. Saving
runs fresh FK validation. Existing folders are refused. **Load session** accepts
either that folder or its session.json file and rechecks it before replacing the
current UI/session. Failed loads preserve the existing session.

```python
from urdf2dt.export.persistence import save_session, load_session
path = save_session("outputs/sessions/my_run", run, session)
loaded = load_session(path)
session = loaded.session
run = loaded.run
```

Reproducible smoke test (choose a new output directory each run):

```powershell
.\.venv\Scripts\python.exe scripts/verify_stage12.py --output outputs/sessions/example_run
```

## Bundle contract

| File | Contents |
|---|---|
| session.json | Authoritative versioned archive: original URDF bytes, config, baseline/working state, events, restore bookkeeping, validation and git provenance |
| dh_model.json | Standard-DH convention, full working model with base/tool transforms, joint signs and provenance |
| edit_history.json | Accepted/rejected row records plus chronological full working-model snapshots, frame states and pending proposals |
| validation_report.json | Actual deterministic samples, prefix errors, summary, candidate identity and git provenance |
| validation_report.md | Human-readable metrics, diagnostics, tolerances and git provenance |
| configuration.json | Effective typed configuration snapshot |

The companion files are downstream views; session.json is the sole reload source.
The archive schema is `schemas/session-1.0.schema.json`, mirrored by
`urdf2dt.export.schemas.SESSION_SCHEMA`. Top-level version/shape checks use JSON
Schema; nested model/config/event/state decoding additionally enforces exact
dataclass fields, numeric and identity invariants. The older provisional 0.1
model envelope remains historical and is not accepted as a session archive.

Save builds a staging directory on the same filesystem and renames it into the
new destination only after every file is written. It does not overwrite exports.
Load caps input at 50 MiB, rejects duplicate keys/nonfinite JSON/unknown versions,
decodes the bounded source upload, and reconstructs the baseline from those exact
bytes. Stored source paths are provenance only. A regenerated baseline must match.
It then restores immutable state/audit records and recomputes sampled FK, comparing
the stored sample set and every metric with the fresh result. Changed runtime
numerics may require a new export rather than silently accepting an old result.

Full event snapshots retain compensation and restored values that earlier
row-only records could not represent. Adjacent invalidation is visible in each
event's frame states. Archives exported after upgrading can contain earlier events
without snapshots only if reconstructed first; reload rejects incomplete audit
snapshots. This format is auditable, not cryptographically signed: consistency
checks do not prove authorship or prevent coordinated rewriting of all fields.

## Logging and reproducibility

`configure_logging(level)` opts into timestamped `urdf2dt` logs without configuring
the root logger at import. Session proposal/accept/reject/restore actions, cascade
invalidation, structural validation, model generation, global validation and
export/reload emit useful events. Classification diagnostics use DEBUG.

Every newly generated FK report captures git commit and working-tree-clean status
for the implementation repository. Outside a git checkout both are null, never a
fabricated clean state. Archives also capture export-time provenance; reload
preserves saved provenance in the archive while returning a fresh validation
report for the current runtime. Historical Stage 5/7/11 files are not rewritten.

## Verification and remaining scope

265 tests passed, including nonzero geometric round trips, continuation/restore,
UI save/load synchronization, stale/tampered data rejection, duplicate JSON,
incomplete-session gating and overwrite refusal. Reference-specific acceptance
and continuous-space proof remain outside this stage. Exact downstream consumer
contracts still require their owning modules' agreement; this is the repository's
version 1.0 session contract.
