#!/usr/bin/env python3
"""Lite3 camera primitive — registers with Atlas, provides snapshot via ffmpeg."""
from __future__ import annotations

import base64
import json
import os
import subprocess
import tempfile

from robonix_api import Primitive, Ok, Err, Deferred

lite3_cam = Primitive(id="lite3_camera", namespace="robonix/primitive/camera")

RTSP_URL = "rtsp://192.168.2.1:8554/test"

@lite3_cam.on_init
def init(cfg: dict):
    rtsp = cfg.get("rtsp_url", RTSP_URL)
    os.environ["LITE3_CAM_RTSP"] = rtsp
    # Declare capability (snapshot will be handled externally)
    lite3_cam.declare_ros2_topic("robonix/primitive/camera/rgb", "/camera/rgb", qos="reliable")
    print(f"[lite3_camera] init OK — RTSP: {rtsp}", flush=True)
    return Ok()

@lite3_cam.on_shutdown
def shutdown():
    print("[lite3_camera] shutdown", flush=True)
    return Ok()

@lite3_cam.on_activate
def activate():
    print("[lite3_camera] activated", flush=True)
    return Ok()

@lite3_cam.on_deactivate
def deactivate():
    print("[lite3_camera] deactivated", flush=True)
    return Ok()


def grab_snapshot():
    """Grab a single JPEG frame. Returns base64 string or error dict."""
    try:
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            tmp_path = tmp.name
        cmd = ['ffmpeg', '-rtsp_transport', 'tcp', '-i', 
               os.environ.get("LITE3_CAM_RTSP", RTSP_URL),
               '-vframes', '1', '-f', 'image2', '-vcodec', 'mjpeg', '-y', tmp_path]
        subprocess.run(cmd, capture_output=True, timeout=10.0)
        if not os.path.exists(tmp_path) or os.path.getsize(tmp_path) == 0:
            return {"error": "no frame"}
        with open(tmp_path, 'rb') as f:
            b64 = base64.b64encode(f.read()).decode()
        os.unlink(tmp_path)
        return {"image": b64, "format": "jpeg", "width": 1280, "height": 720}
    except FileNotFoundError:
        return {"error": "ffmpeg not found"}
    except subprocess.TimeoutExpired:
        return {"error": "timeout"}
    except Exception as e:
        return {"error": str(e)}

if __name__ == '__main__':
    print("[lite3_camera] starting...", flush=True)
    lite3_cam.run()
