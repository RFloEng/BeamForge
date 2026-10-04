"""The browser editor loads every module of the package (editor/app.js FILES)."""

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class TestEditorFiles(unittest.TestCase):
    def test_every_module_is_loaded(self):
        app = (ROOT / "editor" / "app.js").read_text(encoding="utf-8")
        block = re.search(r"const FILES = \[(.*?)\];", app, re.S).group(1)
        listed = set(re.findall(r"'(beamforge/[\w/]+\.py)'", block))
        modules = {f"beamforge/{p.name}" for p in (ROOT / "beamforge").glob("*.py")}
        self.assertEqual(modules - listed, set(), "modules the editor does not load (a missing one breaks its imports)")


if __name__ == "__main__":
    unittest.main()
