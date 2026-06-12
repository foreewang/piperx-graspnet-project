from __future__ import annotations

from piperx_grasp.camera.rgbd_camera import RgbdCamera, RgbdCapture


def capture_scene(camera: RgbdCamera) -> RgbdCapture:
    return camera.capture()
