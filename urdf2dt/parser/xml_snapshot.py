"""Bounded, entity-free XML snapshot shared by optional domain extractors."""

from xml.etree.ElementTree import Element
from defusedxml.ElementTree import fromstring
from urdf2dt.parser.urdf_input import InputPolicy, URDFInput, _check_size


def read_snapshot(source: URDFInput, policy: InputPolicy = InputPolicy()) -> Element:
    """Parse owned bytes only, without fetching resources or running the DH solver."""
    _check_size(len(source.content), policy)
    root = fromstring(source.content, forbid_dtd=True, forbid_entities=True, forbid_external=True)
    stack = [(root, 1)]
    count = 0
    while stack:
        node, depth = stack.pop()
        count += 1
        if count > policy.max_elements or depth > policy.max_depth:
            raise ValueError("URDF exceeds structural resource limits")
        stack.extend((child, depth + 1) for child in node)
    if root.tag != "robot":
        raise ValueError("Expected a URDF robot element")
    return root
