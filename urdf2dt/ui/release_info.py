"""Visible build identity and bounded verification claims."""
import json
from pathlib import Path
import platform
import sys
from urdf2dt.runtime import asset_path

LIMITATIONS = (
    'Clean Windows without Python: not verified on a separate PC/VM. Sanitized PATH checks on a development PC are not equivalent.',
    'Display tests use Qt scaling overrides and actual window-size checks on this PC. They do not certify other monitors, graphics drivers or Windows DPI transitions.',
    'Small windows require scrolling and resizing/collapsing panels. Use View > Reset workspace layout if panels are hidden.',
    'Sampled FK PASS is not a continuous-space proof, hardware calibration, collision or safety certificate.',
    'Branched robots edit one active serial chain at a time; inactive branches stay at default pose.',
    'Symbolic generation has joint, time and expression-size limits. Publication LaTeX needs an external typesetter; PDF compilation is not bundled.',
    'Portable projects are checked locally after extraction; testing on the recipient computer remains necessary.',
    'Recovery excludes pending previews. Large asset saves are synchronous; a power loss may require a retained checkpoint. Recovery storage is not automatically pruned.',
)


def release_details():
    frozen = bool(getattr(sys, 'frozen', False))
    if frozen:
        try:
            data = json.loads(asset_path('build-info.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            data = {}
    else:
        from urdf2dt.logging_config import git_provenance
        data = git_provenance()
    commit = data.get('git_commit') or 'unavailable'
    lines = ['URDF2DT — release candidate', f'Build: {commit[:7]}', f'Source revision: {commit}',
             f'Runtime: {"standalone Windows executable" if frozen else "Python source checkout"}',
             f'Build time (UTC): {data.get("built_at_utc", "not recorded")}',
             f'Build working tree clean: {data.get("working_tree_clean", "unknown")}',
             f'Platform: {platform.platform()}', f'Application: {sys.executable}', '', 'KNOWN LIMITATIONS']
    lines.extend(f'• {item}' for item in LIMITATIONS)
    evidence = Path(sys.executable).parent / 'release-verification.json' if frozen else asset_path('outputs/tmp/release-verification.json')
    lines += ['', 'LOCAL RELEASE VERIFICATION']
    try:
        report = json.loads(evidence.read_text(encoding='utf-8'))
        if report.get('git_commit') != commit:
            raise ValueError('Evidence belongs to a different build')
        lines.append(json.dumps(report, indent=2, ensure_ascii=False))
    except (OSError, ValueError) as exc:
        lines.append(f'No matching verification record is available ({exc}). This is not a PASS.')
    return '\n'.join(lines)


def show_release(e):
    dialog = e.qt.QDialog(e.window)
    dialog.setWindowTitle('About URDF2DT — build, verification and limitations')
    dialog.resize(720, 560)
    screen = e.window.screen().availableGeometry()
    dialog.resize(min(720, screen.width()-40), min(560, screen.height()-60))
    layout = e.qt.QVBoxLayout(dialog)
    text = e.qt.QPlainTextEdit()
    text.setReadOnly(True)
    text.setPlainText(release_details())
    layout.addWidget(text)
    buttons = e.qt.QHBoxLayout(); layout.addLayout(buttons)
    e._button(buttons, 'Copy build and limitations', lambda: e.qt.QApplication.clipboard().setText(text.toPlainText()))
    e._button(buttons, 'Close', dialog.accept)
    dialog.exec()
