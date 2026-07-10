from __future__ import annotations

import argparse
import base64
from pathlib import Path
from urllib.parse import urljoin

import requests


def encode_image(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(f"image not found: {path}")

    return {
        "encoding": "jpeg",
        "width": 1,
        "height": 1,
        "data": base64.b64encode(path.read_bytes()).decode("ascii"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload one mock camera image as JetCar reference image.")
    parser.add_argument("--cloud", default="http://127.0.0.1:8000", help="JetCarCloud base URL")
    parser.add_argument("--car-id", default="car_001")
    parser.add_argument(
        "--image",
        default="../yolov5-7.0/data/images/bus.jpg",
        help="Local image path used as the simulated camera frame",
    )
    args = parser.parse_args()

    image_path = Path(args.image).resolve()
    cloud = args.cloud.rstrip("/") + "/"
    health_url = urljoin(cloud, "health")
    upload_url = urljoin(cloud, "api/edge/reference")

    try:
        health = requests.get(health_url, timeout=(3, 5))
        health.raise_for_status()
        print(f"health ok: {health.text}")
    except requests.RequestException as exc:
        raise SystemExit(
            f"cloud health check failed: {health_url}\n"
            f"{exc}\n"
            "Check that JetCarCloud is running and that the URL is reachable from this machine."
        ) from exc

    payload = {
        "car_id": args.car_id,
        "image": encode_image(image_path),
    }
    try:
        response = requests.post(upload_url, json=payload, timeout=(3, 30))
        response.raise_for_status()
    except requests.RequestException as exc:
        raise SystemExit(f"reference upload failed: {upload_url}\n{exc}") from exc
    print(f"upload ok: {response.text}")


if __name__ == "__main__":
    main()
