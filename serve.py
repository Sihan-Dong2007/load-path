# Dev server for the web/ page. Same as `python3 -m http.server`, but tells the
# browser never to cache: with the stock server, Chrome keeps stale copies of
# edited ES modules, and a mix of old and new modules fails to import — the
# page then loads with a dead script (blank scene, empty labels, no dragging).
#
#   python3 serve.py [port]      then open http://localhost:<port>/web/index.html
import http.server
import sys


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    http.server.test(HandlerClass=NoCacheHandler, port=int(sys.argv[1]) if len(sys.argv) > 1 else 8080)
