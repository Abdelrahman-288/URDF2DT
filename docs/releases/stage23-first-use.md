# Guided first-use workflow

Normal launch without a robot opens the start screen. Command-line robot loading
opens the workspace directly on success. Use Help → Start screen to return without
discarding your current robot; Return to workspace resumes it.

The start screen provides Open URDF, Open project, User manual and Start walkthrough.
Recent projects and URDF files are stored locally in application settings, limited
to 12 entries, deduplicated and updated after successful open/save. Unavailable
paths are labelled; removing history never deletes a file. Existing unsaved-change
confirmation applies when opening a recent item. Empty or malformed history is safe.

The optional bottom walkthrough follows Load → Inspect → Move → Calculate →
Validate → Save. Open step tool takes you to the relevant existing tool. Next and
Back navigate instructions, not model state. Finishing does not claim the model
was validated or saved. The walkthrough is dismissible and restartable through Help.

User manual opens the local PDF with the system PDF viewer. The Windows package
includes the PDF, so sharing the whole distribution preserves access. A missing
manual gives a readable message and directs users to the built-in walkthrough.
The PDF documents the earlier workspace edition; loading and first-use updates
are documented in the accompanying release notes.

## Verification

`--verify-welcome NEW_FOLDER` tests empty history, actual URDF loading, all step
destinations, unchanged model/pose during navigation, PDF URL availability,
recent project reopening/deduplication, unsaved-change cancellation, missing paths,
history removal and malformed stored history. It saves screenshots and JSON.
