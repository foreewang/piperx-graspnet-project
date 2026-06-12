import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.camera.rgbd_camera import RgbdCamera


def main() -> int:
    cfg = load_config("configs/camera.yaml")
    camera = RgbdCamera.from_config(cfg)
    capture = camera.capture()
    print("rgb:", None if capture.rgb is None else capture.rgb.shape)
    print("depth:", None if capture.depth is None else capture.depth.shape)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
