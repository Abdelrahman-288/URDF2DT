"""Check installed core imports in isolation with UI imports explicitly blocked."""

import subprocess
import sys


def test_core_imports_without_ui(tmp_path):
    code = """
import importlib.abc
import sys
class BlockUI(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'pyvista', 'ipywidgets', 'vtk', 'jupyterlab'}:
            raise ImportError('UI import forbidden in core: ' + fullname)
sys.meta_path.insert(0, BlockUI())
from urdf2dt.config import load_config
from urdf2dt.dh.types import DHModel, DHRow, EditorConfig
from urdf2dt.interfaces import URDFParser, DHSolver
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.visualization.scene import StaticScene
from urdf2dt.dh.classification import classify_axis_pair, get_editable_params
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.ui.dh_editor import DHEditor
from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.app import Application
assert load_config() == EditorConfig()
assert DHModel('test', (DHRow(0, 0, 0, 0, 'j'),), 'test').joint_names == ('j',)
"""
    result = subprocess.run([sys.executable, "-I", "-c", code], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
