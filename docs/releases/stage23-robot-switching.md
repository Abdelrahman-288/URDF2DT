# Robot switching and viewport selection delivery

Built from clean source ae92d32695fcd4729a693450d3c58a894ac85aab.
362 tests passed (66.30 s); mypy passed for 59 source files. Native and frozen
checks passed for ten repeated Open URDF cycles with interactions, stale workers,
parse and injected scene failures, body selection, background deselection and
camera drag preservation. Existing studio, numeric, symbolic FK and Jacobian
frozen checks also passed. Open Example was removed from the toolbar.

The universal third-open failure was not independently reproduced. Observer
suspension, rollback and immediate removal of obsolete controls address identified
replacement risks; repeated switching passed with SCARA, UR5 and a local ABB IRB52.

Installed at project-root Windows Application/URDF2DT.exe. Previous version:
outputs/tmp/windows-before-robot-switching-ae92d32. Reports are under
C:/Users/abdel/Downloads/URDF2DT-Robot-Switching-ae92d32.

ZIP: 208896087 bytes. SHA-256:
8f6a13aa10483d0bbb5e18617896571b1532bfe3c73f1b8bbebac53528bf8bc3
All 1617 manifest entries verified on extraction. Independent clean-PC testing
remains pending. Development-host standalone tests ran outside the repository
with Python and Git removed from PATH.
