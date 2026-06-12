import argparse

import _bootstrap  # noqa: F401

from piperx_grasp.config import load_config
from piperx_grasp.grasp.grasp_selector import select_best_grasp
from piperx_grasp.grasp.graspnet_runner import GraspNetRunner


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="NPZ file with poses/scores/widths.")
    args = parser.parse_args()

    cfg = load_config("configs/graspnet.yaml")
    runner = GraspNetRunner.from_config(cfg)
    candidates = runner.load_candidates_npz(args.input)
    best = select_best_grasp(candidates, cfg.get("selection", {}))
    print(best)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
