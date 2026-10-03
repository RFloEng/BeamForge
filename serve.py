"""Serve the repo for the editor: python serve.py [port], then open http://localhost:8000/editor/

Like python -m http.server, but tells the browser not to cache, so an updated editor is always
loaded in full (the plain server lets the browser keep old copies of some files). Local only.
"""

import functools
import http.server
import socket
import sys
from pathlib import Path


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def in_use(port):
    """True when something already answers on the port (Windows lets two servers share one, and the
    old one keeps answering)."""
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    if in_use(port):
        sys.exit(f"Port {port} is already in use (an older server still running?). Stop it, or run: python serve.py {port + 1}")
    handler = functools.partial(NoCache, directory=str(Path(__file__).resolve().parent))
    print(f"BeamForge editor: http://localhost:{port}/editor/  (Ctrl+C to stop)")
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
