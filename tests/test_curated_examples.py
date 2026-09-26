"""Self-contained teaching assets, every branch, and exercise FK answers."""
import json
from math import pi, cos, sin
from pathlib import Path

import numpy as np
import pytest

from urdf2dt.parser.robot_document import RobotDocument
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.kinematics import urdf_fk

ROOT = Path(__file__).resolve().parents[1] / 'robots/curated'
EXAMPLES = json.loads((ROOT / 'manifest.json').read_text())['examples']


@pytest.mark.parametrize('example', EXAMPLES, ids=lambda x: x['id'])
def test_curated_bodies_and_all_chains(example):
    document = RobotDocument.load(ROOT / example['urdf'])
    assert len(document.paths) == example['paths']
    assert len(document.root.findall('.//visual')) == example['visuals']
    assert not document.root.findall('.//mesh')  # no unresolved external assets
    assert not document.root.findall('.//texture')
    for i, path in enumerate(document.paths):
        run = generate_automatic_model(document.select(i))
        assert len(run.chain.joint_names) == example['joints_per_path'][i]
        assert validate_global_fk(run.chain, run.automatic_model, run.config).passed
        if example['id'] == 'serial_arm':
            q, expected = [pi/2, 0, 0], [0, .8, .18]
        elif example['id'] == 'scara':
            q, expected = [0, 0, .10, 0], [.1+.6*cos(.3), -.05+.6*sin(.3), .02]
        else:
            q = [0, pi/2]
            expected = [.28, .40, .40] if path[-1] == 'left_tool' else [-.28, -.40, .40]
        pose = urdf_fk(run.chain, q)
        np.testing.assert_allclose([pose[i][3] for i in range(3)], expected, atol=1e-12)


def test_asset_licence_and_scara_provenance():
    assert 'MIT License' in (ROOT / 'LICENSE').read_text()
    assert (ROOT / 'scara/scara_rrpr.urdf').read_bytes() == (ROOT.parent / 'scara/scara_rrpr.urdf').read_bytes()
