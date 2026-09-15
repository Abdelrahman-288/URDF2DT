"""Headless URDF tree selection and local asset resolution.

Only a selected root-to-leaf path enters the serial DH editor. Other branches
are not silently fed to the serial solver. Stored selection bytes preserve visuals.
"""

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from xml.etree.ElementTree import Element, tostring

from defusedxml.ElementTree import fromstring

from urdf2dt.parser.urdf_input import URDFInput, InputPolicy


@dataclass(frozen=True)
class RobotDocument:
    """Validated rooted URDF tree with source bytes and selectable serial paths."""

    source: URDFInput
    root: Element
    paths: tuple[tuple[str, ...], ...]

    @classmethod
    def load(cls, path: str | Path) -> "RobotDocument":
        """Read bounded XML and reject invalid rooted-tree topology before path selection."""
        source = URDFInput.from_path(path)
        root = fromstring(
            source.content, forbid_dtd=True, forbid_entities=True, forbid_external=True
        )
        policy = InputPolicy()
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
        names = [e.get("name", "") for e in root.findall("link")]
        if not names or not all(names) or len(set(names)) != len(names):
            raise ValueError("Link names must be nonempty and unique")
        children: dict[str, list[str]] = {n: [] for n in names}
        parents: dict[str, str] = {}
        joints = set()
        for j in root.findall("joint"):
            name = j.get("name")
            p, c = j.find("parent"), j.find("child")
            if not name or name in joints or p is None or c is None:
                raise ValueError("Invalid or duplicate joint")
            joints.add(name)
            parent, child = p.get("link", ""), c.get("link", "")
            if parent not in children or child not in children or child in parents:
                raise ValueError("Invalid link reference or multiple parents")
            parents[child] = parent
            children[parent].append(child)
        roots = set(names) - set(parents)
        if len(roots) != 1:
            raise ValueError("Robot must be a rooted tree")
        paths = []
        visited = set()
        todo: list[tuple[str, tuple[str, ...]]] = [(next(iter(roots)), ())]
        while todo:
            name, prefix = todo.pop()
            if name in visited:
                raise ValueError("Cyclic robot topology")
            visited.add(name)
            current = prefix + (name,)
            if not children[name]:
                paths.append(current)
            todo.extend((c, current) for c in children[name])
        if visited != set(names):
            raise ValueError("Disconnected or cyclic robot topology")
        return cls(
            source,
            root,
            tuple(sorted(paths, key=lambda p: (-len(p), p[-1] != "tool0", p[-1]))),
        )

    def select(self, index: int) -> URDFInput:
        """Copy one zero-based root-to-leaf path, retaining visuals and source provenance."""
        if type(index) is not int or not 0 <= index < len(self.paths):
            raise ValueError(
                "chain index must be a zero-based index within the document"
            )
        path = self.paths[index]
        selected = deepcopy(self.root)
        edges = set(zip(path, path[1:]))
        for child in list(selected):
            if child.tag == "link" and child.get("name") in path:
                continue
            if child.tag == "material":
                continue
            if child.tag == "joint":
                p, c = child.find("parent"), child.find("child")
                if (
                    p is not None
                    and c is not None
                    and (p.get("link"), c.get("link")) in edges
                ):
                    continue
            selected.remove(child)
        return URDFInput(
            self.source.name,
            tostring(selected, encoding="utf-8"),
            self.source.source_path,
        )


def resolve_mesh(
    filename: str, urdf_path: Path, package_root: Path | None = None
) -> Path:
    """Resolve local assets; remote mesh URLs are not fetched. ROS roots are explicit."""
    if filename.startswith("package://"):
        relative = filename[len("package://") :]
        roots = [package_root] if package_root is not None else [urdf_path.parent]
        for root in roots:
            assert root is not None
            for candidate in (root / relative, root / relative.split("/", 1)[-1]):
                if candidate.is_file():
                    return candidate.resolve()
            # The supplied doctor repository bundles STL_Files rather than ROS
            # package directories. Use only this explicitly selected local folder.
            stem = Path(relative).stem.replace("_", "")
            bundled = root / (stem + ".stl")
            if root.name == "STL_Files" and bundled.is_file():
                return bundled.resolve()
    elif "://" not in filename:
        path = Path(filename)
        candidate = path if path.is_absolute() else urdf_path.parent / path
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        f"Mesh not found: {filename}. Select its ROS package directory."
    )
