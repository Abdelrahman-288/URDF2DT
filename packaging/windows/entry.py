"""Windowed frozen entry point with a readable startup error."""

import multiprocessing
import traceback


if __name__ == "__main__":
    multiprocessing.freeze_support()
    try:
        from urdf2dt.ui.desktop import main
        raise SystemExit(main())
    except Exception:
        from pathlib import Path
        import os
        import ctypes
        import sys
        folder = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "URDF2DT" / "logs"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / "startup-error.txt"
        path.write_text(traceback.format_exc(), encoding="utf-8")
        if "--verify-package" not in sys.argv:
            ctypes.windll.user32.MessageBoxW(None, f"URDF2DT could not start.\nDetails: {path}", "URDF2DT", 0x10)
        raise SystemExit(1)
