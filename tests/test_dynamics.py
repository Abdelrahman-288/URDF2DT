"""Analytical dynamics, inertial isolation, serialization, and frame invariants."""

from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
import pytest

from urdf2dt.config import EditorConfig
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt.dh.types import IDENTITY
from urdf2dt.dynamics import DynamicModel, DynamicsConfig, JointFriction
from urdf2dt.dynamics.io import export_model, load_dynamic_model, load_model_archive
from urdf2dt.kinematics import dh_frame_transforms, urdf_link_transforms
from urdf2dt.parser.inertial_extractor import InertialProperties, extract_inertials
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import validate_urdf
from urdf2dt.pipeline import generate_automatic_model

ROOT = Path(__file__).resolve().parents[1]


def inertia(mass=2, com="0.5 0 0", rpy="0 0 0", xx=.02, yy=1/6, zz=1/6, xy=0, xz=0, yz=0):
    return (f'<inertial><mass value="{mass}"/><origin xyz="{com}" rpy="{rpy}"/>'
            f'<inertia ixx="{xx}" ixy="{xy}" ixz="{xz}" iyy="{yy}" iyz="{yz}" izz="{zz}"/></inertial>')


def source_xml(body=None, kind="continuous", axis="0 1 0", extra=""):
    body = inertia() if body is None else body
    limit = '<limit lower="-2" upper="2" effort="100" velocity="2"/>' if kind != "continuous" else ""
    xml = (f'<robot name="analytical"><link name="base"/><link name="body">{body}</link>'
           f'<joint name="j" type="{kind}"><parent link="base"/><child link="body"/>'
           f'<axis xyz="{axis}"/>{limit}</joint>{extra}</robot>')
    return URDFInput.from_upload("analytical.urdf", xml.encode())


def direct(source, config=DynamicsConfig()):
    chain = SerialURDFParser().parse(validate_urdf(source).require_valid(), EditorConfig())
    return DynamicModel(chain, extract_inertials(source), config)


@pytest.mark.parametrize("q", [-1.2, -.2, 0., .8, 2.])
def test_pendulum_closed_form(q):
    model = direct(source_xml())
    np.testing.assert_allclose(model.mass_matrix([q]), [[2/3]], atol=1e-14)
    np.testing.assert_allclose(model.gravity_effort([q]), [-9.81*np.cos(q)], atol=1e-13)
    np.testing.assert_allclose(model.coriolis([q], [.7]), [0], atol=1e-14)
    effort = (2/3)*.4 - 9.81*np.cos(q)
    np.testing.assert_allclose(model.inverse_dynamics([q], [.7], [.4]), [effort], atol=1e-13)
    np.testing.assert_allclose(model.forward_dynamics([q], [.7], [effort]), [.4], atol=1e-13)


def test_prismatic_includes_massive_fixed_tool():
    extra = (f'<link name="tool">{inertia(mass=3)}</link>'
             '<joint name="mount" type="fixed"><parent link="body"/><child link="tool"/>'
             '<origin xyz=".1 .2 .3" rpy=".2 .3 .4"/></joint>')
    model = direct(source_xml(kind="prismatic", axis="0 0 1", extra=extra))
    np.testing.assert_allclose(model.mass_matrix([.1]), [[5]], atol=1e-13)
    np.testing.assert_allclose(model.gravity_effort([.1]), [49.05], atol=1e-13)
    np.testing.assert_allclose(model.inverse_dynamics([.1], [.3], [2]), [59.05], atol=1e-13)


def test_two_link_coupling_and_coriolis_closed_form():
    first = inertia(mass=2, yy=.2, zz=.2)
    extra = (f'<link name="second">{inertia(mass=3, yy=.3, zz=.3)}</link>'
             '<joint name="j2" type="continuous"><parent link="body"/><child link="second"/>'
             '<origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>')
    model = direct(source_xml(first, axis="0 0 1", extra=extra), DynamicsConfig(gravity=(0, -9.81, 0)))
    for q in ([.4, -.7], [-.8, 1.3], [0, 0]):
        v = np.array([.8, -.3])
        coupling = 1.5*np.cos(q[1])
        mass = np.array([[.2+.3+.5+3*(1+.25)+2*coupling, .3+.75+coupling], [.3+.75+coupling, .3+.75]])
        h = 1.5*np.sin(q[1])
        c = np.array([-h*(2*v[0]*v[1]+v[1]**2), h*v[0]**2])
        g2 = 9.81*1.5*np.cos(sum(q))
        g = np.array([9.81*4*np.cos(q[0])+g2, g2])
        np.testing.assert_allclose(model.mass_matrix(q), mass, atol=1e-13)
        np.testing.assert_allclose(model.coriolis(q, v), c, atol=1e-13)
        np.testing.assert_allclose(model.gravity_effort(q), g, atol=1e-13)


@pytest.mark.parametrize("value,status", [
    ("", "missing"), ('<inertial><mass value="2"/></inertial>', "incomplete"),
    (inertia(mass=-1), "invalid"), (inertia(mass=float("nan")), "invalid"),
    (inertia(xx=1, yy=1, zz=3), "invalid"), (inertia(mass=0), "invalid"),
    (inertia(rpy="nan 0 0"), "invalid"), (inertia()+inertia(), "invalid"),
])
def test_bad_inertias_do_not_block_kinematics(value, status):
    source = source_xml(value)
    run = generate_automatic_model(source)
    record = extract_inertials(source).for_link("body")
    assert record.status == status
    with pytest.raises(ValueError, match="Dynamics unavailable"):
        DynamicModel(run.chain, extract_inertials(source))


def test_extractor_independent_of_topology_and_solver(monkeypatch):
    source = URDFInput.from_upload("only_links.urdf", ('<robot name="data"><link name="only">'+inertia()+'</link></robot>').encode())
    monkeypatch.setattr("urdf2dt.dh.dh_solver.StandardDHSolver.solve", lambda *a: pytest.fail("DH called"))
    assert extract_inertials(source).for_link("only").properties.mass == 2


def test_inertia_rotation_and_parallel_axis():
    body = extract_inertials(source_xml(inertia(xx=.1, yy=.2, zz=.25, xy=.02, xz=.01, yz=-.03, rpy=".2 -.4 .6"))).for_link("body").properties
    com, tensor, translated = body.in_frame(IDENTITY)
    rotation = np.asarray(body.origin)[:3, :3]
    np.testing.assert_allclose(tensor, rotation @ np.array(body.tensor) @ rotation.T)
    np.testing.assert_allclose(translated-tensor, np.diag([0, .5, .5]), atol=1e-14)
    np.testing.assert_allclose(com, [.5, 0, 0])


def test_friction_sign_dissipation_and_zero_velocity():
    model = direct(source_xml(), DynamicsConfig(friction=(JointFriction("j", .3, .7),)))
    for v in [-2., 0., 2.]:
        friction = .3*v + .7*np.sign(v)
        np.testing.assert_allclose(model.friction_effort([v]), [friction])
        assert model.friction_effort([v])[0]*v >= 0
        np.testing.assert_allclose(model.inverse_dynamics([0], [v], [0]), [-9.81+friction])
    with pytest.raises(ValueError):
        JointFriction("j", -1)


def test_urdf_friction_is_used_unless_config_explicitly_overrides():
    source = source_xml()
    supplied = URDFInput.from_upload(source.name, source.content.replace(b'</joint>', b'<dynamics damping=".2" friction=".1"/></joint>'))
    np.testing.assert_allclose(load_dynamic_model(supplied).friction_effort([2]), [.5])
    np.testing.assert_allclose(load_dynamic_model(supplied, DynamicsConfig()).friction_effort([2]), [0])


def test_dh_edit_inertia_mapping_preserves_world_distribution():
    source = URDFInput.from_path(ROOT / "robots/dynamics/ur5_dynamics.urdf")
    run = generate_automatic_model(source)
    model = direct(source)
    edited = recompute_model(run.automatic_model, 2, FrameEdit(.08))
    q = [.2, -.5, .6, .1, .8, -.3]
    link_pose = np.array(urdf_link_transforms(run.chain, q)[3])
    body = model.inertials.for_link("upper_arm_link").properties
    world_com, world_i, _ = body.in_frame(tuple(map(tuple, link_pose)))
    for dh in [run.automatic_model, edited]:
        frame = np.array(dh_frame_transforms(dh, q)[2])
        local_com, local_i, _ = body.in_frame(tuple(map(tuple, np.linalg.inv(frame) @ link_pose)))
        np.testing.assert_allclose(frame[:3,:3] @ local_com + frame[:3,3], world_com, atol=1e-12)
        np.testing.assert_allclose(frame[:3,:3] @ local_i @ frame[:3,:3].T, world_i, atol=1e-12)
    np.testing.assert_allclose(model.tip_transform(q), np.array(urdf_link_transforms(run.chain,q)[-1]), atol=1e-12)


def test_source_identity_and_singular_mechanism():
    source = source_xml()
    model = direct(source)
    with pytest.raises(ValueError, match="same URDF"):
        DynamicModel(model.chain, replace(model.inertials, source_sha256="a"*64))
    with pytest.raises(ValueError, match="positive definite"):
        direct(source_xml(inertia(mass=0, xx=0, yy=0, zz=0)))
    for values in ([0, 1], [float("nan")], [[1]]):
        with pytest.raises(ValueError, match="finite vector"):
            model.mass_matrix(values)


def test_energy_and_export_round_trip(tmp_path):
    source = URDFInput.from_path(ROOT / "robots/dynamics/scara_dynamics.urdf")
    model = load_dynamic_model(source)
    q, v, a = np.array([.3, -.5, .1, .2]), np.array([.2, -.3, .1, .4]), np.ones(4)
    eps = 1e-6
    mdot = (model.mass_matrix(q+eps*v)-model.mass_matrix(q-eps*v))/(2*eps)
    np.testing.assert_allclose(v @ model.coriolis(q, v), .5*v @ mdot @ v, atol=1e-10)
    path = export_model(model, source, tmp_path / "model.json")
    restored = load_model_archive(path)
    np.testing.assert_allclose(restored.inverse_dynamics(q,v,a), model.inverse_dynamics(q,v,a), atol=1e-12)
    with pytest.raises(FileExistsError):
        export_model(model, source, path)
    envelope = json.loads(path.read_text())
    envelope["payload"]["config"]["gravity"][2] = -2
    path.write_text(json.dumps(envelope))
    with pytest.raises(ValueError, match="checksum"):
        load_model_archive(path)
    envelope["payload"]["inertials"]["records"][2]["properties"]["mass"] = 100
    envelope["sha256"] = sha256(json.dumps(envelope["payload"], sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    path.write_text(json.dumps(envelope))
    with pytest.raises(ValueError, match="inertias differ"):
        load_model_archive(path)


def test_entity_input_is_rejected():
    source = URDFInput.from_upload("bad.urdf", b'<!DOCTYPE robot [<!ENTITY e SYSTEM "file:///etc/passwd">]><robot name="x">&e;</robot>')
    with pytest.raises(Exception):
        extract_inertials(source)


def test_application_dynamics_requires_accepted_fk_and_preserves_edits():
    from urdf2dt.app import Application
    app = Application(ROOT / "robots/dynamics/ur5_dynamics.urdf")
    with pytest.raises(ValueError, match="Accept all frames"):
        app.dynamic_model()
    for i in range(1, 7):
        app.session.unlock_next()
        app.session.propose_edit(i, FrameEdit(.04) if i == 2 else FrameEdit())
        assert app.session.accept().valid
    state = app.session.state
    model = app.dynamic_model()
    assert app.session.state == state
    assert model.chain == app.run.chain


def test_export_refuses_changed_chain_with_stale_source_hash(tmp_path):
    source = source_xml()
    model = direct(source)
    joint = replace(model.chain.joints[0], axis=(0., 0., 1.))
    altered = DynamicModel(replace(model.chain, joints=(joint,)), model.inertials)
    with pytest.raises(ValueError, match="chain differs"):
        export_model(altered, source, tmp_path / "bad.json")
