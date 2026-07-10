from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class MockCameraHandler(BaseHTTPRequestHandler):
    image_path: Path

    def do_GET(self) -> None:
        if self.path not in {"/api/frame", "/frame.jpg"}:
            self.send_error(404, "not found")
            return

        if not self.image_path.exists():
            self.send_error(404, f"image not found: {self.image_path}")
            return

        data = self.image_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args) -> None:
        print("%s - %s" % (self.address_string(), fmt % args))


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve one fixed image as a mock JetCar camera frame.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8100)
    parser.add_argument("--image", default="../yolov5-7.0/data/images/bus.jpg")
    args = parser.parse_args()

    MockCameraHandler.image_path = Path(args.image).resolve()
    server = ThreadingHTTPServer((args.host, args.port), MockCameraHandler)
    print(f"mock camera serving {MockCameraHandler.image_path}")
    print(f"GET http://{args.host}:{args.port}/api/frame")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
