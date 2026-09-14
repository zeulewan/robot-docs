#!/usr/bin/env python3
"""Export the mounted RealSense RGB-D calibration to a portable JSON file."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--serial", help="RealSense serial number; defaults to the first device")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    return parser.parse_args()


def intrinsics_dict(profile) -> dict[str, object]:
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    return {
        "width": intrinsics.width,
        "height": intrinsics.height,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "cx": intrinsics.ppx,
        "cy": intrinsics.ppy,
        "distortion_model": str(intrinsics.model),
        "distortion_coefficients": list(intrinsics.coeffs),
    }


def main() -> None:
    args = parse_args()
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit(
            "pyrealsense2 is required. Run this script on the G1 Jetson where "
            "the RealSense SDK is installed."
        ) from exc

    context = rs.context()
    devices = context.query_devices()
    if not devices:
        raise SystemExit("No RealSense device detected")

    device = None
    for candidate in devices:
        serial = candidate.get_info(rs.camera_info.serial_number)
        if args.serial is None or serial == args.serial:
            device = candidate
            break
    if device is None:
        raise SystemExit(f"RealSense serial {args.serial!r} was not found")

    serial = device.get_info(rs.camera_info.serial_number)
    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.rgb8,
        args.fps,
    )
    config.enable_stream(
        rs.stream.depth,
        args.width,
        args.height,
        rs.format.z16,
        args.fps,
    )

    profile = pipeline.start(config)
    try:
        color = profile.get_stream(rs.stream.color)
        depth = profile.get_stream(rs.stream.depth)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_to_color = depth.get_extrinsics_to(color)
        result = {
            "schema_version": 1,
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "device": {
                "name": device.get_info(rs.camera_info.name),
                "serial": serial,
                "firmware": device.get_info(rs.camera_info.firmware_version),
                "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
            },
            "streams": {
                "color": {
                    "format": "rgb8",
                    "fps": args.fps,
                    "intrinsics": intrinsics_dict(color),
                },
                "depth": {
                    "format": "z16",
                    "fps": args.fps,
                    "intrinsics": intrinsics_dict(depth),
                    "scale_m_per_unit": depth_sensor.get_depth_scale(),
                },
            },
            "extrinsics": {
                "depth_to_color": {
                    "rotation_row_major": list(depth_to_color.rotation),
                    "translation_m": list(depth_to_color.translation),
                }
            },
            "alignment": {
                "target_stream": "color",
                "runtime_operation": "rs.align(rs.stream.color)",
            },
        }
    finally:
        pipeline.stop()

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(output)


if __name__ == "__main__":
    main()
