# Dev server for the web/ page. Same as `python3 -m http.server`, but tells the
# browser never to cache: with the stock server, Chrome keeps stale copies of
# edited ES modules, and a mix of old and new modules fails to import — the
# page then loads with a dead script (blank scene, empty labels, no dragging).
#
#   python3 serve.py [port]          then open http://localhost:<port>/web/index.html
#   python3 serve.py [port] --dev    also swap in a fake Footron messaging client, so
#                                    dev/phone.html can drive the wall page from a
#                                    second tab (open /web/index.html?ftmsg=1)
import http.server
import sys

DEV = "--dev" in sys.argv
CLIENT_TAG = b'<script src="vendor/footron-messaging.min.js"></script>'


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        # In dev mode, follow the real messaging client with the fake one.
        if DEV and self.path.split("?")[0].endswith("/web/index.html"):
            with open("web/index.html", "rb") as f:
                body = f.read().replace(CLIENT_TAG, CLIENT_TAG + b'\n<script src="/dev/fake-messaging.js"></script>')
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    http.server.test(HandlerClass=NoCacheHandler, port=int(args[0]) if args else 8080)
