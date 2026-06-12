from .config import load_config


def main() -> int:
    robot_cfg = load_config("configs/robot.yaml")
    print(robot_cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
