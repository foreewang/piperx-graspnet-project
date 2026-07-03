from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_request(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_result(path: str | Path, success: bool, message: str) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(
            {
                "success": success,
                "message": message,
                "trajectory": None,
                "metadata": {},
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", required=True)
    parser.add_argument("--result", required=True)
    args = parser.parse_args()

    request = load_request(args.request)
    waypoints = request.get("waypoints", [])
    print("planning_group:", request.get("planning_group"))
    print("base_frame:", request.get("base_frame"))
    print("end_effector_link:", request.get("end_effector_link"))
    print("waypoints:", [waypoint.get("name") for waypoint in waypoints])

    write_result(
        args.result,
        success=False,
        message=(
            "MoveIt planning is not wired yet. Convert request waypoints to "
            "geometry_msgs/Pose and call MoveIt here."
        ),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
