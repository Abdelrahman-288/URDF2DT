from types import SimpleNamespace
from urdf2dt.ui.result_help import fk_help, jacobian_help, joint_key


def test_mixed_joint_mapping_and_units():
    chain = SimpleNamespace(joints=[SimpleNamespace(name=n, joint_type=SimpleNamespace(value=k))
        for n, k in [('mount','fixed'), ('shoulder','revolute'), ('slide','prismatic')]])
    assert joint_key(chain, True) == 'q1 = shoulder [rad], q2 = slide [m]'
    text = jacobian_help('moving_mount', 'offset_tool', chain)
    assert 'moving_mount' in text and 'offset_tool' in text and 'moving reference' in text
    assert 'J2 = slide [m]' in text and 'not RPY rates' in text


def test_fk_explanation_tracks_units_and_frames():
    text = fk_help('base', 'named_tool', 'mm', 'deg')
    assert '"named_tool"' in text and '"base"' in text
    assert 'in mm' in text and 'use deg' in text and 'full-precision SI' in text
