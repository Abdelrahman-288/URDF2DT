"""Render a validated robot's static URDF/DH scene."""

import argparse

from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.visualization.scene import StaticScene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf")
    parser.add_argument("--output", help="Save an offscreen PNG instead of opening a window")
    parser.add_argument("--q", nargs="+", type=float, help="Movable joint coordinates in URDF order (rad/m)")
    parser.add_argument("--selected-frame", help="Highlight a DH frame, e.g. F2")
    parser.add_argument("--hide-urdf", action="store_true")
    parser.add_argument("--hide-dh", action="store_true")
    parser.add_argument("--hide-labels", action="store_true")
    args = parser.parse_args()
    run = generate_automatic_model(args.urdf)
    scene = StaticScene(run.chain, run.automatic_model, args.q, selected_frame=args.selected_frame)
    scene.render(args.output, urdf=not args.hide_urdf, dh=not args.hide_dh, labels=not args.hide_labels)
    if args.output:
        print(f"Rendered {args.output}")


if __name__ == "__main__":
    main()
