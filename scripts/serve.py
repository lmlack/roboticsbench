"""Static server for local work. Same as `python3 -m http.server`, plus
Cache-Control: no-cache, so the browser revalidates every file and an edited
stylesheet or module shows up on the next reload instead of a cached copy."""
import http.server
import sys

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

port = int(sys.argv[1]) if len(sys.argv) > 1 else 5180
http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
