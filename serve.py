"""Serve the repo for the editor: python serve.py [port], then open http://localhost:8000/editor/

Like python -m http.server, but tells the browser not to cache, so an updated editor is always
loaded in full (the plain server lets the browser keep old copies of some files). Local only.
"""

import functools
import http.server
import sys
from pathlib import Path


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    handler = functools.partial(NoCache, directory=str(Path(__file__).resolve().parent))
    print(f"BeamForge editor: http://localhost:{port}/editor/  (Ctrl+C to stop)")
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
